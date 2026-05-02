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

// Sequential fixed-point signed divide (restoring shift-subtract) with
// val/rdy handshake
// convention.
//   istream_msg = {a, b}          sent together on one handshake
//   ostream_msg = quotient        one handshake per completed divide
// Internal latency: p_wide_bits + p_frac_bits iterations after the
// input handshake, then a FINISH cycle, then DONE holds until the
// output handshake.

module FpDiv #(
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
