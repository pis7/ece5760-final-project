// Fixed-point signed multiply

module FpMul #(
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits
) (
  input  logic signed [p_total_bits-1:0] a,
  input  logic signed [p_total_bits-1:0] b,
  output logic signed [p_total_bits-1:0] result
);

  logic signed [2*p_total_bits-1:0] full;

  always_comb begin
    full   = a * b;
    result = $signed({full[2*p_total_bits-1],
              full[2*(p_total_bits-1)-p_int_bits : p_frac_bits]});
  end

endmodule

// Wide fixed-point signed multiply. Returns a p_wide_bits-wide result
// with the frac-bits right shift applied. Used so products can
// accumulate without early truncation.

module FpMulWide #(
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_wide_bits  = 48
) (
  input  logic signed [p_total_bits-1:0] a,
  input  logic signed [p_total_bits-1:0] b,
  output logic signed [p_wide_bits-1:0]  result
);

  logic signed [2*p_total_bits-1:0] full;

  always_comb begin
    full   = a * b;
    result = p_wide_bits'(full >>> p_frac_bits);
  end

endmodule

// FpDiv wrapper: picks Newton-Raphson for p_total_bits <= 27 and a
// restoring shift-subtract divider for wider widths. The NR module's
// internals are hard-coded to 48-bit operands and 256x17 Q1.16 ROM, so
// it can't be cleanly stretched past 27-bit -- the SS module handles
// arbitrary widths at the cost of (p_wide_bits + p_frac_bits) iteration
// latency.
//
// CGTop wires p_wide_bits = p_acc_bits, and CGDpath derives
// p_acc_bits = 48 when p_total_bits <= 27 so the NR branch stays
// lossless.

module FpDiv #(
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_wide_bits  = 48
) (
  input  logic                           clk,
  input  logic                           rst,

  input  logic                           istream_val,
  output logic                           istream_rdy,
  input  logic signed [p_wide_bits-1:0]  istream_msg_a,
  input  logic signed [p_wide_bits-1:0]  istream_msg_b,

  output logic                           ostream_val,
  input  logic                           ostream_rdy,
  output logic signed [p_total_bits-1:0] ostream_msg_result
);

  generate
    if (p_total_bits <= 27) begin : g_div_nr
      FpDivNR #(
        .p_int_bits  (p_int_bits),
        .p_frac_bits (p_frac_bits),
        .p_total_bits(p_total_bits),
        .p_wide_bits (p_wide_bits)
      ) u_div (.*);
    end else begin : g_div_ss
      FpDivSS #(
        .p_int_bits  (p_int_bits),
        .p_frac_bits (p_frac_bits),
        .p_total_bits(p_total_bits),
        .p_wide_bits (p_wide_bits)
      ) u_div (.*);
    end
  endgenerate

endmodule

// Newton-Raphson reciprocal divider. val/rdy interface; ~10 cycles per
// divide. Hard-coded to 48-bit operands and Q1.16 normalization ROM --
// only correct for p_total_bits <= 27 with p_wide_bits == 48.
//
// Algorithm (per-divide, ~7 internal cycles):
//   1. Sign extract: sign = sign(a) ^ sign(b). Take |a|, |b|.
//   2. LZC normalize |b|. Let lzc = leading-zero count of |b| (0..47).
//      Define shifted = |b| << lzc (MSB at bit 47), b_norm = shifted[47:31].
//      b_norm is a Q1.16 value in [1.0, 2.0); MSB is always 1 for non-zero |b|.
//   3. ROM seed lookup: r0 = recip_rom[b_norm[15:8]]. r0 is Q1.16,
//      ~9-bit-accurate approximation of 1/b_norm_value.
//   4. NR iter: r1 = r0 * (2 - b_norm * r0). Roughly doubles accuracy.
//      Computed as fixed-point Q1.16:
//        m1 = b_norm * r0          (17*17 = 34-bit)
//        two_minus = 2'h2_0000 - m1[33:16]   (Q2.16, 18-bit)
//        m2 = r0 * two_minus       (17*18 = 35-bit)
//        r1 = m2[32:16]            (Q1.16, 17-bit)
//   5. Final multiply: m3 = |a| * r1 (48*17 = 65-bit unsigned).
//   6. Denormalize: result_unsigned = m3 >> (49 - lzc). Saturate if any bit
//      [64:26] is set. Apply sign.
//
// Numerical derivation of the shift count:
//   abs_b ~ B_n * 2^(31-lzc)         where B_n is b_norm raw int [2^16, 2^17)
//   1/abs_b ~= R1 / 2^32 / 2^(31-lzc) = R1 * 2^(lzc-63)
//   result = (|a| << p_frac_bits) * (1/abs_b)
//          = |a| * 2^14 * R1 * 2^(lzc-63)
//          = M3 * 2^(lzc - 49)
//   so the right shift on M3 is (49 - lzc), valid for lzc in [0, 47].
//
// Edge cases:
//   - b == 0: detected in S_NORM (lzc==48), result saturates to max
//     magnitude with sign of a.
//   - Quotient overflow (result needs more than p_total_bits-1 bits):
//     saturate to (1 << (p_total_bits-1)) - 1, then apply sign.

module FpDivNR #(
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_wide_bits  = 48
) (
  input  logic                           clk,
  input  logic                           rst,

  // istream (CGCtrl -> FpDiv)
  input  logic                           istream_val,
  output logic                           istream_rdy,
  input  logic signed [p_wide_bits-1:0]  istream_msg_a,
  input  logic signed [p_wide_bits-1:0]  istream_msg_b,

  // ostream (FpDiv -> CGCtrl)
  output logic                           ostream_val,
  input  logic                           ostream_rdy,
  output logic signed [p_total_bits-1:0] ostream_msg_result
);

  localparam ABS_W   = p_wide_bits;          // 48
  localparam BNORM_W = 17;                   // Q1.16
  localparam M3_W    = ABS_W + BNORM_W;      // 65
  localparam SAT_MAG = (1 << (p_total_bits - 1)) - 1;  // 0x3FFFFFF for 27-bit

  // Two NR iterations -- 1 iter gives ~17-bit accuracy in real arithmetic
  // but intermediate truncations to 17-bit Q1.16 limit the achievable
  // precision to a few LSBs; a 2nd iter washes out the truncation noise.
  // Each NR iteration is split into two states: _M1 latches m1 = b_norm *
  // nr_input into nr_m1_reg, _M2 then computes m2 = nr_input * (2 - m1)
  // from the registered m1. This keeps each cycle's combinational chain
  // to a single multiply plus a small subtract/select.
  typedef enum logic [3:0] {
    S_IDLE,
    S_NORM,
    S_ROM,
    S_NR1_M1,
    S_NR1_M2,
    S_NR2_M1,
    S_NR2_M2,
    S_QMUL,
    S_QFINISH,
    S_DONE
  } state_t;

  state_t state, next_state;

  // -- Latched / staged registers ---------------------------------------------
  logic [ABS_W-1:0]   abs_a;
  logic [ABS_W-1:0]   abs_b;
  logic               sign_q;
  logic               b_zero;

  logic [5:0]         lzc;                   // 0..48 (48 means b == 0)
  logic [ABS_W-1:0]   shifted;
  logic [BNORM_W-1:0] b_norm;
  logic [BNORM_W-1:0] r0;
  logic [BNORM_W-1:0] r1;
  logic [33:0]        nr_m1_reg;
  logic [M3_W-1:0]    m3;

  // -- Handshake wires --------------------------------------------------------
  wire input_handshake  = istream_val && istream_rdy;
  wire output_handshake = ostream_val && ostream_rdy;

  assign istream_rdy = (state == S_IDLE);
  assign ostream_val = (state == S_DONE);

  // -- 256x17 reciprocal seed ROM. Entry i = round(2^32 / bn_mid) where
  //    bn_mid = 0x10000 + (i << 8) + 0x80, the Q1.16 midpoint of bin i.
  //    Range: [0x08020, 0x10000].
  logic [BNORM_W-1:0] recip_rom [0:255];
  initial begin
    recip_rom[  0] = 17'h0ff80;
    recip_rom[  1] = 17'h0fe82;
    recip_rom[  2] = 17'h0fd86;
    recip_rom[  3] = 17'h0fc8c;
    recip_rom[  4] = 17'h0fb94;
    recip_rom[  5] = 17'h0fa9e;
    recip_rom[  6] = 17'h0f9a9;
    recip_rom[  7] = 17'h0f8b7;
    recip_rom[  8] = 17'h0f7c6;
    recip_rom[  9] = 17'h0f6d7;
    recip_rom[ 10] = 17'h0f5ea;
    recip_rom[ 11] = 17'h0f4ff;
    recip_rom[ 12] = 17'h0f415;
    recip_rom[ 13] = 17'h0f32d;
    recip_rom[ 14] = 17'h0f247;
    recip_rom[ 15] = 17'h0f163;
    recip_rom[ 16] = 17'h0f080;
    recip_rom[ 17] = 17'h0ef9f;
    recip_rom[ 18] = 17'h0eebf;
    recip_rom[ 19] = 17'h0ede1;
    recip_rom[ 20] = 17'h0ed05;
    recip_rom[ 21] = 17'h0ec2a;
    recip_rom[ 22] = 17'h0eb51;
    recip_rom[ 23] = 17'h0ea7a;
    recip_rom[ 24] = 17'h0e9a4;
    recip_rom[ 25] = 17'h0e8cf;
    recip_rom[ 26] = 17'h0e7fc;
    recip_rom[ 27] = 17'h0e72b;
    recip_rom[ 28] = 17'h0e65b;
    recip_rom[ 29] = 17'h0e58c;
    recip_rom[ 30] = 17'h0e4bf;
    recip_rom[ 31] = 17'h0e3f4;
    recip_rom[ 32] = 17'h0e329;
    recip_rom[ 33] = 17'h0e260;
    recip_rom[ 34] = 17'h0e199;
    recip_rom[ 35] = 17'h0e0d3;
    recip_rom[ 36] = 17'h0e00e;
    recip_rom[ 37] = 17'h0df4b;
    recip_rom[ 38] = 17'h0de88;
    recip_rom[ 39] = 17'h0ddc8;
    recip_rom[ 40] = 17'h0dd08;
    recip_rom[ 41] = 17'h0dc4a;
    recip_rom[ 42] = 17'h0db8d;
    recip_rom[ 43] = 17'h0dad1;
    recip_rom[ 44] = 17'h0da17;
    recip_rom[ 45] = 17'h0d95e;
    recip_rom[ 46] = 17'h0d8a6;
    recip_rom[ 47] = 17'h0d7ef;
    recip_rom[ 48] = 17'h0d73a;
    recip_rom[ 49] = 17'h0d685;
    recip_rom[ 50] = 17'h0d5d2;
    recip_rom[ 51] = 17'h0d520;
    recip_rom[ 52] = 17'h0d46f;
    recip_rom[ 53] = 17'h0d3bf;
    recip_rom[ 54] = 17'h0d311;
    recip_rom[ 55] = 17'h0d263;
    recip_rom[ 56] = 17'h0d1b7;
    recip_rom[ 57] = 17'h0d10c;
    recip_rom[ 58] = 17'h0d062;
    recip_rom[ 59] = 17'h0cfb9;
    recip_rom[ 60] = 17'h0cf11;
    recip_rom[ 61] = 17'h0ce6a;
    recip_rom[ 62] = 17'h0cdc4;
    recip_rom[ 63] = 17'h0cd1f;
    recip_rom[ 64] = 17'h0cc7b;
    recip_rom[ 65] = 17'h0cbd8;
    recip_rom[ 66] = 17'h0cb36;
    recip_rom[ 67] = 17'h0ca96;
    recip_rom[ 68] = 17'h0c9f6;
    recip_rom[ 69] = 17'h0c957;
    recip_rom[ 70] = 17'h0c8b9;
    recip_rom[ 71] = 17'h0c81c;
    recip_rom[ 72] = 17'h0c780;
    recip_rom[ 73] = 17'h0c6e5;
    recip_rom[ 74] = 17'h0c64b;
    recip_rom[ 75] = 17'h0c5b2;
    recip_rom[ 76] = 17'h0c51a;
    recip_rom[ 77] = 17'h0c482;
    recip_rom[ 78] = 17'h0c3ec;
    recip_rom[ 79] = 17'h0c357;
    recip_rom[ 80] = 17'h0c2c2;
    recip_rom[ 81] = 17'h0c22e;
    recip_rom[ 82] = 17'h0c19b;
    recip_rom[ 83] = 17'h0c109;
    recip_rom[ 84] = 17'h0c078;
    recip_rom[ 85] = 17'h0bfe8;
    recip_rom[ 86] = 17'h0bf59;
    recip_rom[ 87] = 17'h0beca;
    recip_rom[ 88] = 17'h0be3c;
    recip_rom[ 89] = 17'h0bdaf;
    recip_rom[ 90] = 17'h0bd23;
    recip_rom[ 91] = 17'h0bc98;
    recip_rom[ 92] = 17'h0bc0d;
    recip_rom[ 93] = 17'h0bb83;
    recip_rom[ 94] = 17'h0bafb;
    recip_rom[ 95] = 17'h0ba72;
    recip_rom[ 96] = 17'h0b9eb;
    recip_rom[ 97] = 17'h0b964;
    recip_rom[ 98] = 17'h0b8de;
    recip_rom[ 99] = 17'h0b859;
    recip_rom[100] = 17'h0b7d5;
    recip_rom[101] = 17'h0b751;
    recip_rom[102] = 17'h0b6ce;
    recip_rom[103] = 17'h0b64c;
    recip_rom[104] = 17'h0b5cb;
    recip_rom[105] = 17'h0b54a;
    recip_rom[106] = 17'h0b4ca;
    recip_rom[107] = 17'h0b44b;
    recip_rom[108] = 17'h0b3cc;
    recip_rom[109] = 17'h0b34e;
    recip_rom[110] = 17'h0b2d1;
    recip_rom[111] = 17'h0b254;
    recip_rom[112] = 17'h0b1d8;
    recip_rom[113] = 17'h0b15d;
    recip_rom[114] = 17'h0b0e3;
    recip_rom[115] = 17'h0b069;
    recip_rom[116] = 17'h0aff0;
    recip_rom[117] = 17'h0af77;
    recip_rom[118] = 17'h0aeff;
    recip_rom[119] = 17'h0ae88;
    recip_rom[120] = 17'h0ae11;
    recip_rom[121] = 17'h0ad9b;
    recip_rom[122] = 17'h0ad26;
    recip_rom[123] = 17'h0acb1;
    recip_rom[124] = 17'h0ac3d;
    recip_rom[125] = 17'h0abc9;
    recip_rom[126] = 17'h0ab56;
    recip_rom[127] = 17'h0aae4;
    recip_rom[128] = 17'h0aa72;
    recip_rom[129] = 17'h0aa01;
    recip_rom[130] = 17'h0a990;
    recip_rom[131] = 17'h0a920;
    recip_rom[132] = 17'h0a8b1;
    recip_rom[133] = 17'h0a842;
    recip_rom[134] = 17'h0a7d3;
    recip_rom[135] = 17'h0a766;
    recip_rom[136] = 17'h0a6f8;
    recip_rom[137] = 17'h0a68c;
    recip_rom[138] = 17'h0a620;
    recip_rom[139] = 17'h0a5b4;
    recip_rom[140] = 17'h0a549;
    recip_rom[141] = 17'h0a4df;
    recip_rom[142] = 17'h0a475;
    recip_rom[143] = 17'h0a40c;
    recip_rom[144] = 17'h0a3a3;
    recip_rom[145] = 17'h0a33a;
    recip_rom[146] = 17'h0a2d3;
    recip_rom[147] = 17'h0a26b;
    recip_rom[148] = 17'h0a204;
    recip_rom[149] = 17'h0a19e;
    recip_rom[150] = 17'h0a138;
    recip_rom[151] = 17'h0a0d3;
    recip_rom[152] = 17'h0a06e;
    recip_rom[153] = 17'h0a00a;
    recip_rom[154] = 17'h09fa6;
    recip_rom[155] = 17'h09f43;
    recip_rom[156] = 17'h09ee0;
    recip_rom[157] = 17'h09e7e;
    recip_rom[158] = 17'h09e1c;
    recip_rom[159] = 17'h09dba;
    recip_rom[160] = 17'h09d59;
    recip_rom[161] = 17'h09cf9;
    recip_rom[162] = 17'h09c99;
    recip_rom[163] = 17'h09c39;
    recip_rom[164] = 17'h09bda;
    recip_rom[165] = 17'h09b7c;
    recip_rom[166] = 17'h09b1d;
    recip_rom[167] = 17'h09ac0;
    recip_rom[168] = 17'h09a62;
    recip_rom[169] = 17'h09a05;
    recip_rom[170] = 17'h099a9;
    recip_rom[171] = 17'h0994d;
    recip_rom[172] = 17'h098f1;
    recip_rom[173] = 17'h09896;
    recip_rom[174] = 17'h0983b;
    recip_rom[175] = 17'h097e1;
    recip_rom[176] = 17'h09787;
    recip_rom[177] = 17'h0972e;
    recip_rom[178] = 17'h096d5;
    recip_rom[179] = 17'h0967c;
    recip_rom[180] = 17'h09624;
    recip_rom[181] = 17'h095cc;
    recip_rom[182] = 17'h09574;
    recip_rom[183] = 17'h0951d;
    recip_rom[184] = 17'h094c7;
    recip_rom[185] = 17'h09470;
    recip_rom[186] = 17'h0941b;
    recip_rom[187] = 17'h093c5;
    recip_rom[188] = 17'h09370;
    recip_rom[189] = 17'h0931b;
    recip_rom[190] = 17'h092c7;
    recip_rom[191] = 17'h09273;
    recip_rom[192] = 17'h0921f;
    recip_rom[193] = 17'h091cc;
    recip_rom[194] = 17'h09179;
    recip_rom[195] = 17'h09127;
    recip_rom[196] = 17'h090d5;
    recip_rom[197] = 17'h09083;
    recip_rom[198] = 17'h09032;
    recip_rom[199] = 17'h08fe1;
    recip_rom[200] = 17'h08f90;
    recip_rom[201] = 17'h08f40;
    recip_rom[202] = 17'h08ef0;
    recip_rom[203] = 17'h08ea0;
    recip_rom[204] = 17'h08e51;
    recip_rom[205] = 17'h08e02;
    recip_rom[206] = 17'h08db3;
    recip_rom[207] = 17'h08d65;
    recip_rom[208] = 17'h08d17;
    recip_rom[209] = 17'h08cc9;
    recip_rom[210] = 17'h08c7c;
    recip_rom[211] = 17'h08c2f;
    recip_rom[212] = 17'h08be2;
    recip_rom[213] = 17'h08b96;
    recip_rom[214] = 17'h08b4a;
    recip_rom[215] = 17'h08aff;
    recip_rom[216] = 17'h08ab3;
    recip_rom[217] = 17'h08a68;
    recip_rom[218] = 17'h08a1e;
    recip_rom[219] = 17'h089d3;
    recip_rom[220] = 17'h08989;
    recip_rom[221] = 17'h08940;
    recip_rom[222] = 17'h088f6;
    recip_rom[223] = 17'h088ad;
    recip_rom[224] = 17'h08864;
    recip_rom[225] = 17'h0881c;
    recip_rom[226] = 17'h087d3;
    recip_rom[227] = 17'h0878c;
    recip_rom[228] = 17'h08744;
    recip_rom[229] = 17'h086fd;
    recip_rom[230] = 17'h086b6;
    recip_rom[231] = 17'h0866f;
    recip_rom[232] = 17'h08628;
    recip_rom[233] = 17'h085e2;
    recip_rom[234] = 17'h0859c;
    recip_rom[235] = 17'h08557;
    recip_rom[236] = 17'h08511;
    recip_rom[237] = 17'h084cc;
    recip_rom[238] = 17'h08488;
    recip_rom[239] = 17'h08443;
    recip_rom[240] = 17'h083ff;
    recip_rom[241] = 17'h083bb;
    recip_rom[242] = 17'h08377;
    recip_rom[243] = 17'h08334;
    recip_rom[244] = 17'h082f1;
    recip_rom[245] = 17'h082ae;
    recip_rom[246] = 17'h0826b;
    recip_rom[247] = 17'h08229;
    recip_rom[248] = 17'h081e7;
    recip_rom[249] = 17'h081a5;
    recip_rom[250] = 17'h08164;
    recip_rom[251] = 17'h08123;
    recip_rom[252] = 17'h080e2;
    recip_rom[253] = 17'h080a1;
    recip_rom[254] = 17'h08060;
    recip_rom[255] = 17'h08020;
  end

  // -- Combinational helpers --------------------------------------------------

  // Operand abs (from istream), used only when input handshake fires.
  wire [ABS_W-1:0] abs_a_in =
    istream_msg_a[ABS_W-1] ? $unsigned(-istream_msg_a) : $unsigned(istream_msg_a);
  wire [ABS_W-1:0] abs_b_in =
    istream_msg_b[ABS_W-1] ? $unsigned(-istream_msg_b) : $unsigned(istream_msg_b);

  // 48-bit leading-zero count. Returns 48 when input is zero.
  function automatic logic [5:0] count_leading_zeros(input logic [47:0] x);
    logic [5:0] cnt;
    cnt = 6'd48;
    for (int k = 47; k >= 0; k--)
      if (x[k] && cnt == 6'd48) cnt = 6'd47 - 6'(k);
    return cnt;
  endfunction

  // NR step combinational expressions. The input is r0 in S_NR1_* (1st
  // iter) and r1 in S_NR2_* (2nd iter); the output is the new r1. The
  // first multiply's output is registered into nr_m1_reg in the _M1
  // phase so the second multiply (in _M2) sees a registered operand,
  // keeping the combinational chain to a single DSP per cycle.
  wire [BNORM_W-1:0] nr_input    =
      (state == S_NR2_M1 || state == S_NR2_M2) ? r1 : r0;
  wire [33:0]        nr_m1       = b_norm * nr_input;
  wire [17:0]        nr_two_minus = 18'h2_0000 - 18'(nr_m1_reg[33:16]);
  wire [34:0]        nr_m2       = nr_input * nr_two_minus;

  // Saturation / shift in S_QFINISH.
  wire [5:0]      shift_amt        = 6'd49 - lzc;          // 2..49 for non-zero b
  wire [M3_W-1:0] shifted_m3       = m3 >> shift_amt;
  wire            magnitude_overflow = |shifted_m3[M3_W-1:p_total_bits-1];
  wire            saturate         = b_zero || magnitude_overflow;

  // Build the result as a non-negative signed value (MSB=0), then either
  // pass it through or negate it based on sign_q. The {1'b0, mag_lo}
  // concatenation is `p_total_bits` wide with a leading zero, so when
  // assigned to a signed wire the value is guaranteed non-negative and
  // unary minus produces a representable signed result. Avoids inline
  // size-cast + $signed combinations that some synthesis tools dislike.
  wire [p_total_bits-2:0] result_mag_lo = saturate
      ? SAT_MAG[p_total_bits-2:0]
      : shifted_m3[p_total_bits-2:0];
  wire signed [p_total_bits-1:0] result_pos = {1'b0, result_mag_lo};
  wire signed [p_total_bits-1:0] result_neg = -result_pos;

  // -- State register ---------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) state <= S_IDLE;
    else     state <= next_state;
  end

  always_comb begin
    next_state = state;
    case (state)
      S_IDLE:    if (input_handshake)  next_state = S_NORM;
      S_NORM:                          next_state = S_ROM;
      S_ROM:                           next_state = S_NR1_M1;
      S_NR1_M1:                        next_state = S_NR1_M2;
      S_NR1_M2:                        next_state = S_NR2_M1;
      S_NR2_M1:                        next_state = S_NR2_M2;
      S_NR2_M2:                        next_state = S_QMUL;
      S_QMUL:                          next_state = S_QFINISH;
      S_QFINISH:                       next_state = S_DONE;
      S_DONE:    if (output_handshake) next_state = S_IDLE;
      default: ;
    endcase
  end

  // -- Datapath registers -----------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      abs_a              <= '0;
      abs_b              <= '0;
      sign_q             <= 1'b0;
      b_zero             <= 1'b0;
      lzc                <= '0;
      shifted            <= '0;
      b_norm             <= '0;
      r0                 <= '0;
      r1                 <= '0;
      nr_m1_reg          <= '0;
      m3                 <= '0;
      ostream_msg_result <= '0;
    end else begin
      case (state)
        S_IDLE: if (input_handshake) begin
          abs_a  <= abs_a_in;
          abs_b  <= abs_b_in;
          sign_q <= istream_msg_a[ABS_W-1] ^ istream_msg_b[ABS_W-1];
        end
        S_NORM: begin
          lzc     <= count_leading_zeros(abs_b);
          shifted <= abs_b << count_leading_zeros(abs_b);
          b_zero  <= (abs_b == '0);
        end
        S_ROM: begin
          b_norm <= shifted[47:31];
          r0     <= recip_rom[shifted[46:39]];  // = b_norm[15:8]
        end
        S_NR1_M1: begin
          // 1st NR iter, phase 1: m1 = b_norm * r0. One DSP this cycle.
          nr_m1_reg <= nr_m1;
        end
        S_NR1_M2: begin
          // 1st NR iter, phase 2: r1 = r0 * (2 - m1). One DSP this cycle.
          r1 <= nr_m2[32:16];
        end
        S_NR2_M1: begin
          // 2nd NR iter, phase 1: m1 = b_norm * r1. One DSP this cycle.
          nr_m1_reg <= nr_m1;
        end
        S_NR2_M2: begin
          // 2nd NR iter, phase 2: r1 = r1_prev * (2 - m1). One DSP this cycle.
          r1 <= nr_m2[32:16];
        end
        S_QMUL: begin
          m3 <= abs_a * r1;
        end
        S_QFINISH: begin
          ostream_msg_result <= sign_q ? result_neg : result_pos;
        end
        default: ;
      endcase
    end
  end

endmodule

// Sequential fixed-point signed divide (restoring shift-subtract) with
// val/rdy handshake. Fully parameterized on
// p_int_bits/p_frac_bits/p_total_bits/p_wide_bits, so this is what the
// FpDiv wrapper picks when p_total_bits > 27 (where the NR module's
// hard-coded 48-bit/Q1.16 internals can't be stretched).
//   istream_msg = {a, b}          sent together on one handshake
//   ostream_msg = quotient        one handshake per completed divide
// Internal latency: p_wide_bits + p_frac_bits iterations after the
// input handshake, then a FINISH cycle, then DONE holds until the
// output handshake.

module FpDivSS #(
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_wide_bits  = 48
) (
  input  logic                           clk,
  input  logic                           rst,

  // istream (CGCtrl -> FpDiv)
  input  logic                           istream_val,
  output logic                           istream_rdy,
  input  logic signed [p_wide_bits-1:0]  istream_msg_a,
  input  logic signed [p_wide_bits-1:0]  istream_msg_b,

  // ostream (FpDiv -> CGCtrl)
  output logic                           ostream_val,
  input  logic                           ostream_rdy,
  output logic signed [p_total_bits-1:0] ostream_msg_result
);

  localparam p_div_w  = p_wide_bits + p_frac_bits;
  localparam p_iter_w = $clog2(p_div_w + 1);

  typedef enum logic [1:0] {
    STATE_IDLE,
    STATE_RUN,
    STATE_FINISH,
    STATE_DONE
  } state_t;

  state_t state_reg, state_next;

  logic [p_div_w-1:0]     dividend;
  logic [p_wide_bits:0]   rem;
  logic [p_div_w-1:0]     quotient;
  logic [p_wide_bits-1:0] divisor;
  logic                   sign;
  logic [p_iter_w-1:0]    iter_cnt;

  // Handshake wires
  wire input_handshake;
  wire output_handshake;
  assign input_handshake  = istream_val && istream_rdy;
  assign output_handshake = ostream_val && ostream_rdy;

  assign istream_rdy = (state_reg == STATE_IDLE);
  assign ostream_val = (state_reg == STATE_DONE);

  // -- State register --------------------------------------------------------

  always_ff @(posedge clk) begin
    if (rst) state_reg <= STATE_IDLE;
    else     state_reg <= state_next;
  end

  // -- Next-state logic ------------------------------------------------------

  always_comb begin
    state_next = state_reg;
    case (state_reg)
      STATE_IDLE:    if (input_handshake)                          state_next = STATE_RUN;
      STATE_RUN:     if (iter_cnt == p_iter_w'(p_div_w))           state_next = STATE_FINISH;
      STATE_FINISH:                                                state_next = STATE_DONE;
      STATE_DONE:    if (output_handshake)                         state_next = STATE_IDLE;
      default: ;
    endcase
  end

  // -- Trial shift-subtract combinational helpers ---------------------------

  logic [p_wide_bits:0] new_rem_pre;
  logic [p_wide_bits:0] trial_sub;
  assign new_rem_pre = {rem[p_wide_bits-1:0], dividend[p_div_w-1]};
  assign trial_sub   = new_rem_pre - {1'b0, divisor};

  // Absolute value of operands latched on the input handshake
  logic [p_wide_bits-1:0] abs_a;
  logic [p_wide_bits-1:0] abs_b;
  assign abs_a = istream_msg_a[p_wide_bits-1] ? $unsigned(-istream_msg_a) : $unsigned(istream_msg_a);
  assign abs_b = istream_msg_b[p_wide_bits-1] ? $unsigned(-istream_msg_b) : $unsigned(istream_msg_b);

  // -- Sequential state updates ---------------------------------------------

  always_ff @(posedge clk) begin
    if (rst) begin
      rem                <= '0;
      dividend           <= '0;
      quotient           <= '0;
      divisor            <= '0;
      sign               <= 1'b0;
      iter_cnt           <= '0;
      ostream_msg_result <= '0;
    end else begin
      case (state_reg)
        STATE_IDLE: begin
          if (input_handshake) begin
            dividend <= {abs_a, {p_frac_bits{1'b0}}};
            divisor  <= abs_b;
            rem      <= '0;
            quotient <= '0;
            sign     <= istream_msg_a[p_wide_bits-1] ^ istream_msg_b[p_wide_bits-1];
            iter_cnt <= '0;
          end
        end
        STATE_RUN: begin
          if (iter_cnt < p_iter_w'(p_div_w)) begin
            if (!trial_sub[p_wide_bits]) begin
              rem      <= trial_sub;
              quotient <= {quotient[p_div_w-2:0], 1'b1};
            end else begin
              rem      <= new_rem_pre;
              quotient <= {quotient[p_div_w-2:0], 1'b0};
            end
            dividend <= {dividend[p_div_w-2:0], 1'b0};
            iter_cnt <= iter_cnt + 1;
          end
        end
        STATE_FINISH: begin
          if (sign)
            ostream_msg_result <= -$signed({1'b0, quotient[p_total_bits-2:0]});
          else
            ostream_msg_result <= $signed({1'b0, quotient[p_total_bits-2:0]});
        end
        default: ;
      endcase
    end
  end

endmodule
