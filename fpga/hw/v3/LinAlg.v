// v2 linalg kernels -- val/rdy (istream/ostream) handshake
//
// All three kernels (VecDot, AXPY, SPMV) follow the same convention:
//   istream_val / istream_rdy          upstream -> module handshake
//   istream_msg_*                      per-handshake operand payload
//   ostream_val / ostream_rdy          module -> downstream handshake
//   ostream_msg_*                      per-handshake result payload
//
// Each kernel is split into a {Kernel}Ctrl (FSM, val/rdy handling) and
// {Kernel}Dpath (multiplier + accumulator + register state) pair,
// wrapped in a {Kernel}_seq module for use by CGDpath.
//
// Semantics match fpga/hw/DotProduct.v and fpga/hw/AXPY.v, extended
// with p_lanes-wide SIMD on VecDot and AXPY:
//   - VecDot / AXPY: one group of p_lanes (a,b) pairs per istream
//     handshake. AXPY emits a group of p_lanes z outputs per ostream
//     handshake. VecDot emits one final scalar after num_groups input
//     handshakes. num_groups = ceil(n/p_lanes) is computed internally.
//     Out-of-range lanes (for the last partial group) are handled by
//     the caller driving zeros on rd_a/rd_b for those lanes and
//     masking we during writeback -- see CGCtrl/CGDpath.
//   - SPMV: single istream "start" handshake, then one row result per
//     ostream handshake. SPMV owns a memory-read port to the on-chip
//     SRAM and an external vec-RF read port. SPMV stays single-lane
//     because it is memory-bandwidth-bound on the single Avalon port.
//
// Every internal multiply uses FpMul / FpMulWide (DSP-mapped). DSP
// count: VecDot p_lanes + AXPY p_lanes + SPMV 1. For p_lanes=4 that
// is 9 DSPs total.

//======================================================================
// VecNegSub: result[i] = -(a[i] + b[i]).
// Per-handshake streaming, same val/rdy convention as AXPY.
//======================================================================

module VecNegSubCtrl (
  input  logic        clk,
  input  logic        rst,
  input  logic        istream_val,
  output logic        istream_rdy,
  output logic        ostream_val,
  input  logic        ostream_rdy,
  input  logic [31:0] n,
  input  logic [31:0] compute_counter,
  output logic        advance
);

  typedef enum logic [1:0] { INIT, COMPUTE, UPDATE, DONE } state_t;
  state_t state, next_state;

  wire input_handshake  = istream_val  && istream_rdy;
  wire output_handshake = ostream_val  && ostream_rdy;

  assign istream_rdy = (state == COMPUTE);
  assign ostream_val = (state == UPDATE);
  assign advance     = input_handshake;

  always_ff @(posedge clk) begin
    if (rst) state <= INIT;
    else     state <= next_state;
  end

  always_comb begin
    next_state = state;
    case (state)
      INIT:    next_state = COMPUTE;
      COMPUTE: if (input_handshake)                    next_state = UPDATE;
      UPDATE:  if (output_handshake) begin
                 if (compute_counter == n) next_state  = DONE;
                 else                      next_state  = COMPUTE;
               end
      DONE:                                            next_state  = COMPUTE;
      default:                                         next_state  = INIT;
    endcase
  end

endmodule


module VecNegSubDpath #(
  parameter p_total_bits = 27
) (
  input  logic                           clk,
  input  logic                           rst,
  input  logic                           advance,
  input  logic signed [p_total_bits-1:0] a,
  input  logic signed [p_total_bits-1:0] b,
  output logic signed [p_total_bits-1:0] z,
  output logic [31:0]                    compute_counter
);

  always_ff @(posedge clk) begin
    if (rst) begin
      z               <= '0;
      compute_counter <= '0;
    end else if (advance) begin
      z               <= -(a + b);
      compute_counter <= compute_counter + 1;
    end
  end

endmodule


module VecNegSub_seq #(
  parameter p_total_bits = 27
) (
  input  logic                           clk,
  input  logic                           rst,
  input  logic        [31:0]             n,

  input  logic                           istream_val,
  output logic                           istream_rdy,
  input  logic signed [p_total_bits-1:0] istream_msg_a,
  input  logic signed [p_total_bits-1:0] istream_msg_b,

  output logic                           ostream_val,
  input  logic                           ostream_rdy,
  output logic signed [p_total_bits-1:0] ostream_msg_z
);

  logic [31:0] compute_counter;
  logic        advance;

  VecNegSubCtrl u_ctrl (
    .clk, .rst,
    .istream_val, .istream_rdy,
    .ostream_val, .ostream_rdy,
    .n, .compute_counter,
    .advance (advance)
  );

  VecNegSubDpath #(.p_total_bits(p_total_bits)) u_dpath (
    .clk, .rst,
    .advance,
    .a (istream_msg_a),
    .b (istream_msg_b),
    .z (ostream_msg_z),
    .compute_counter
  );

endmodule


//======================================================================
// VecDot: result = sum_{i=0..n-1}(a[i] * b[i])
// Parallelized to p_lanes lanes: istream carries p_lanes (a,b) pairs
// per handshake, and the internal FSM runs for num_groups =
// ceil(n/p_lanes) handshakes. One FpMulWide per lane + a tree sum
// into the accumulator.
//======================================================================

module VecDotCtrl (
  input  logic        clk,
  input  logic        rst,
  input  logic        istream_val,
  output logic        istream_rdy,
  output logic        ostream_val,
  input  logic        ostream_rdy,
  input  logic [31:0] n_groups,
  input  logic [31:0] compute_counter,
  output logic        acc_en,
  output logic        acc_clear
);

  typedef enum logic [1:0] { INIT, COMPUTE, DONE } state_t;
  state_t state, next_state;

  wire input_handshake  = istream_val && istream_rdy;
  wire output_handshake = ostream_val && ostream_rdy;

  assign istream_rdy = (state == COMPUTE);
  assign ostream_val = (state == DONE);
  assign acc_en      = (state == COMPUTE) && input_handshake;
  // Clear the accumulator after the result has been consumed so the
  // next solve starts at zero.
  assign acc_clear   = (state == DONE) && output_handshake;

  always_ff @(posedge clk) begin
    if (rst) state <= INIT;
    else     state <= next_state;
  end

  always_comb begin
    next_state = state;
    case (state)
      INIT:    next_state = COMPUTE;
      COMPUTE: if (input_handshake && (compute_counter == n_groups - 1))
                                                       next_state = DONE;
      DONE:    if (output_handshake)                   next_state = COMPUTE;
      default:                                         next_state = INIT;
    endcase
  end

endmodule


module VecDotDpath #(
  parameter p_lanes      = 4,
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_acc_bits   = 48
) (
  input  logic                                   clk,
  input  logic                                   rst,
  input  logic                                   acc_en,
  input  logic                                   acc_clear,
  input  logic [p_lanes*p_total_bits-1:0]        a_packed,
  input  logic [p_lanes*p_total_bits-1:0]        b_packed,
  output logic signed [p_acc_bits-1:0]           result,
  output logic [31:0]                            compute_counter
);

  logic signed [p_total_bits-1:0] a_lane     [p_lanes];
  logic signed [p_total_bits-1:0] b_lane     [p_lanes];
  logic signed [p_acc_bits-1:0]   prod_lane  [p_lanes];

  genvar gi;
  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_mac
      assign a_lane[gi] = $signed(a_packed[(gi+1)*p_total_bits-1 -: p_total_bits]);
      assign b_lane[gi] = $signed(b_packed[(gi+1)*p_total_bits-1 -: p_total_bits]);

      FpMulWide #(
        .p_int_bits  (p_int_bits),
        .p_frac_bits (p_frac_bits),
        .p_total_bits(p_total_bits),
        .p_wide_bits (p_acc_bits)
      ) u_mul (
        .a     (a_lane[gi]),
        .b     (b_lane[gi]),
        .result(prod_lane[gi])
      );
    end
  endgenerate

  // Synthesizer will balance this into an adder tree of depth
  // ceil(log2(p_lanes)).
  logic signed [p_acc_bits-1:0] sum_prod;
  always_comb begin
    sum_prod = '0;
    for (int i = 0; i < p_lanes; i++)
      sum_prod = sum_prod + prod_lane[i];
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      result          <= '0;
      compute_counter <= '0;
    end else if (acc_clear) begin
      result          <= '0;
      compute_counter <= '0;
    end else if (acc_en) begin
      result          <= result + sum_prod;
      compute_counter <= compute_counter + 1;
    end
  end

endmodule


module VecDot_seq #(
  parameter p_lanes      = 4,
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_acc_bits   = 48
) (
  input  logic                                   clk,
  input  logic                                   rst,
  input  logic        [31:0]                     n,

  input  logic                                   istream_val,
  output logic                                   istream_rdy,
  input  logic [p_lanes*p_total_bits-1:0]        istream_msg_a,
  input  logic [p_lanes*p_total_bits-1:0]        istream_msg_b,

  output logic                                   ostream_val,
  input  logic                                   ostream_rdy,
  output logic signed [p_acc_bits-1:0]           ostream_msg_result
);

  // $clog2(1) returns 0, so this handles p_lanes==1 degenerately.
  localparam CLOG2_LANES = $clog2(p_lanes);

  logic        acc_en, acc_clear;
  logic [31:0] compute_counter;
  logic [31:0] n_groups;

  // num_groups = ceil(n / p_lanes). Shift works for any p_lanes >= 1
  // because we only ever instantiate with power-of-two p_lanes.
  assign n_groups = (n + p_lanes - 1) >> CLOG2_LANES;

  VecDotCtrl u_ctrl (
    .clk, .rst,
    .istream_val, .istream_rdy,
    .ostream_val, .ostream_rdy,
    .n_groups        (n_groups),
    .compute_counter,
    .acc_en, .acc_clear
  );

  VecDotDpath #(
    .p_lanes     (p_lanes),
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits),
    .p_acc_bits  (p_acc_bits)
  ) u_dpath (
    .clk, .rst,
    .acc_en, .acc_clear,
    .a_packed        (istream_msg_a),
    .b_packed        (istream_msg_b),
    .result          (ostream_msg_result),
    .compute_counter
  );

endmodule


//======================================================================
// AXPY: z[i] = a[i] +/- coef * b[i] for i = 0..n-1
// Parallelized to p_lanes lanes. istream carries p_lanes (a,b) pairs;
// ostream carries p_lanes z outputs. Internal FSM runs for
// num_groups = ceil(n/p_lanes) handshakes. coef and mode held
// quasi-static by the caller across the whole operation.
//======================================================================

module AXPYCtrl (
  input  logic        clk,
  input  logic        rst,
  input  logic        istream_val,
  output logic        istream_rdy,
  output logic        ostream_val,
  input  logic        ostream_rdy,
  input  logic [31:0] n_groups,
  input  logic [31:0] compute_counter,
  output logic        advance
);

  typedef enum logic [1:0] { INIT, COMPUTE, UPDATE, DONE } state_t;
  state_t state, next_state;

  wire input_handshake  = istream_val && istream_rdy;
  wire output_handshake = ostream_val && ostream_rdy;

  assign istream_rdy = (state == COMPUTE);
  assign ostream_val = (state == UPDATE);
  // advance fires on the input handshake (same cycle) so the dpath
  // latches z and bumps the counter in sync with the state move.
  assign advance     = input_handshake;

  always_ff @(posedge clk) begin
    if (rst) state <= INIT;
    else     state <= next_state;
  end

  always_comb begin
    next_state = state;
    case (state)
      INIT:    next_state = COMPUTE;
      COMPUTE: if (input_handshake)                         next_state = UPDATE;
      UPDATE:  if (output_handshake) begin
                 if (compute_counter == n_groups) next_state = DONE;
                 else                             next_state = COMPUTE;
               end
      DONE:                                                 next_state = COMPUTE;
      default:                                              next_state = INIT;
    endcase
  end

endmodule


module AXPYDpath #(
  parameter p_lanes      = 4,
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits
) (
  input  logic                                   clk,
  input  logic                                   rst,
  input  logic                                   advance,
  input  logic                                   mode,    // 0 = add, 1 = sub
  input  logic [p_lanes*p_total_bits-1:0]        a_packed,
  input  logic [p_lanes*p_total_bits-1:0]        b_packed,
  input  logic signed [p_total_bits-1:0]         coef,
  output logic [p_lanes*p_total_bits-1:0]        z_packed,
  output logic [31:0]                            compute_counter
);

  logic signed [p_total_bits-1:0] a_lane   [p_lanes];
  logic signed [p_total_bits-1:0] b_lane   [p_lanes];
  logic signed [p_total_bits-1:0] prod     [p_lanes];
  logic signed [p_total_bits-1:0] combined [p_lanes];
  logic signed [p_total_bits-1:0] z_reg    [p_lanes];

  genvar gi;
  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_mac
      assign a_lane[gi] = $signed(a_packed[(gi+1)*p_total_bits-1 -: p_total_bits]);
      assign b_lane[gi] = $signed(b_packed[(gi+1)*p_total_bits-1 -: p_total_bits]);

      FpMul #(
        .p_int_bits  (p_int_bits),
        .p_frac_bits (p_frac_bits),
        .p_total_bits(p_total_bits)
      ) u_mul (
        .a     (coef),
        .b     (b_lane[gi]),
        .result(prod[gi])
      );

      assign combined[gi] = (mode == 1'b0) ? (a_lane[gi] + prod[gi])
                                           : (a_lane[gi] - prod[gi]);

      assign z_packed[(gi+1)*p_total_bits-1 -: p_total_bits] = $unsigned(z_reg[gi]);
    end
  endgenerate

  always_ff @(posedge clk) begin
    if (rst) begin
      compute_counter <= '0;
      for (int i = 0; i < p_lanes; i++) z_reg[i] <= '0;
    end else if (advance) begin
      compute_counter <= compute_counter + 1;
      for (int i = 0; i < p_lanes; i++) z_reg[i] <= combined[i];
    end
  end

endmodule


module AXPY_seq #(
  parameter p_lanes      = 4,
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits
) (
  input  logic                                   clk,
  input  logic                                   rst,
  input  logic        [31:0]                     n,

  input  logic                                   mode,    // 0 = add, 1 = sub
  input  logic signed [p_total_bits-1:0]         coef,

  input  logic                                   istream_val,
  output logic                                   istream_rdy,
  input  logic [p_lanes*p_total_bits-1:0]        istream_msg_a,
  input  logic [p_lanes*p_total_bits-1:0]        istream_msg_b,

  output logic                                   ostream_val,
  input  logic                                   ostream_rdy,
  output logic [p_lanes*p_total_bits-1:0]        ostream_msg_z
);

  localparam CLOG2_LANES = $clog2(p_lanes);

  logic        advance;
  logic [31:0] compute_counter;
  logic [31:0] n_groups;

  assign n_groups = (n + p_lanes[31:0] - 32'd1) >> CLOG2_LANES;

  AXPYCtrl u_ctrl (
    .clk, .rst,
    .istream_val, .istream_rdy,
    .ostream_val, .ostream_rdy,
    .n_groups        (n_groups),
    .compute_counter,
    .advance
  );

  AXPYDpath #(
    .p_lanes     (p_lanes),
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits)
  ) u_dpath (
    .clk, .rst,
    .advance,
    .mode,
    .a_packed        (istream_msg_a),
    .b_packed        (istream_msg_b),
    .coef            (coef),
    .z_packed        (ostream_msg_z),
    .compute_counter
  );

endmodule


//======================================================================
// SPMV: result = Q * vec using CSR from on-chip SRAM.
// Single istream handshake to start. One ostream handshake per output
// row carries (row_idx, row_val). SPMV owns a memory bus and a vec-RF
// read port exposed to the caller.
//
// Per-row inner loop reads vals[j] and col_idx[j] from SRAM
// sequentially, looks up vec[col] via a combinational RF read port,
// and MACs into a wide accumulator via one FpMulWide.
//======================================================================

module SPMVCtrl #(
  parameter p_m10k_addr_bits  = 32
) (
  input  logic                           clk,
  input  logic                           rst,

  // val/rdy handshakes
  input  logic                           istream_val,
  output logic                           istream_rdy,
  output logic                           ostream_val,
  input  logic                           ostream_rdy,

  // Loop bounds + CSR layout (quasi-static from caller)
  input  logic [31:0]                    n,
  input  logic [p_m10k_addr_bits-1:0]    q_val_base,
  input  logic [p_m10k_addr_bits-1:0]    q_col_base,
  input  logic [p_m10k_addr_bits-1:0]    q_rowp_base,

  // From dpath
  input  logic [31:0]                    row_idx,
  input  logic [31:0]                    j_addr_idx,          // next nnz to issue addrs for
  input  logic [31:0]                    j_mac_idx,           // next nnz to MAC
  input  logic                           pipe_phase,          // 0=val cyc, 1=col cyc
  input  logic [1:0]                     pipe_warm,           // 0/1 prologue, 2 steady
  input  logic [31:0]                    rp_lo,
  input  logic [31:0]                    rp_hi,

  // To dpath
  output logic                           d_begin_op,          // reset row state
  output logic                           d_capture_rplo,
  output logic                           d_capture_rphi,
  output logic                           d_capture_rphi_next, // carry rp_hi -> rp_lo
  output logic                           d_init_row,          // acc<-0, pipe state <- 0
  output logic                           d_pipe_step,         // advance pipe_phase/warm/j_addr_idx
  output logic                           d_capture_val,
  output logic                           d_capture_col,
  output logic                           d_acc_en,            // MAC step
  output logic                           d_bump_row,          // advance row after ostream handshake

  // Memory bus
  output logic [p_m10k_addr_bits-1:0]    mem_addr,
  output logic                           mem_rd_en
);

  // Two-cycle reads for prologue and inner-loop states: ADDR drives mem_addr;
  // CAPT latches mem_rdata one cycle later.
  //
  // rp_lo / rp_hi collapse: only row 0 reads both rp_ptr[0] and rp_ptr[1].
  // For rows 1..n-1, the previous row's rp_hi == this row's rp_lo, so we
  // carry rp_hi -> rp_lo and read only the new rp_ptr[row+1] into rp_hi.
  // States S_RPHI_ADDR_NEXT / S_RPHI_CAPT_NEXT replace the rp_lo prologue
  // for non-first rows; saves 2 cycles per row except row 0.
  //
  // Inner loop is a 2-cyc/nnz overlapped pipeline (S_PIPE). On each cycle
  // it alternates issuing val[k] and col[k] addresses while concurrently
  // capturing the previous read into val_reg / col_reg and firing the MAC
  // for the (k-1)th nnz. Per-row cost is 2*nnz + 2 cycles.
  //
  // History: two earlier 2-cyc pipelined attempts regressed tiny3 HPWL
  // end-to-end while passing all 16 DPI tests bit-exact. Root cause was
  // an off-by-one in the C++ M10K shim in cg_hw_driver.h: it sampled
  // `on_chip_ram_address` AFTER the rising-edge eval, so it used the
  // NEW state's decode instead of the address driven during the cycle
  // that just ended. With serial 5-cyc/nnz the address held stable for
  // 2 cycles so the bug was invisible; with single-cycle addresses the
  // shim returned data for the wrong cycle and val_reg captured col[]
  // bits. The shim has been fixed (sample pre-edge); this comment is
  // here so a future debugger can find the explanation if a similar
  // regression ever shows up again.

  typedef enum logic [3:0] {
    S_IDLE,
    S_RPLO_ADDR,        S_RPLO_CAPT,
    S_RPHI_ADDR,        S_RPHI_CAPT,
    S_RPHI_ADDR_NEXT,   S_RPHI_CAPT_NEXT,
    S_ROW_INIT,
    S_PIPE,
    S_EMIT,
    S_DONE
  } state_t;

  state_t state, next_state;

  wire input_handshake  = istream_val && istream_rdy;
  wire output_handshake = ostream_val && ostream_rdy;

  assign istream_rdy = (state == S_IDLE);
  assign ostream_val = (state == S_EMIT);

  always_ff @(posedge clk) begin
    if (rst) state <= S_IDLE;
    else     state <= next_state;
  end

  // ---- Memory address + rd_en -------------------------------------------
  always_comb begin
    mem_addr  = '0;
    mem_rd_en = 1'b0;
    case (state)
      S_RPLO_ADDR, S_RPLO_CAPT: begin
        mem_addr  = q_rowp_base + p_m10k_addr_bits'(row_idx);
        mem_rd_en = 1'b1;
      end
      S_RPHI_ADDR, S_RPHI_CAPT: begin
        mem_addr  = q_rowp_base + p_m10k_addr_bits'(row_idx) + p_m10k_addr_bits'(1);
        mem_rd_en = 1'b1;
      end
      S_RPHI_ADDR_NEXT, S_RPHI_CAPT_NEXT: begin
        mem_addr  = q_rowp_base + p_m10k_addr_bits'(row_idx) + p_m10k_addr_bits'(1);
        mem_rd_en = 1'b1;
      end
      S_PIPE: begin
        // Each cycle alternates between issuing val[k] and col[k] until
        // we've issued addresses for every nnz in this row; after that
        // the bus idles while the pipeline drains.
        if (j_addr_idx < rp_hi) begin
          mem_rd_en = 1'b1;
          if (pipe_phase == 1'b0)
            mem_addr = q_val_base + p_m10k_addr_bits'(j_addr_idx);
          else
            mem_addr = q_col_base + p_m10k_addr_bits'(j_addr_idx);
        end
      end
      default: ;
    endcase
  end

  // ---- Next-state -------------------------------------------------------
  always_comb begin
    next_state = state;
    case (state)
      S_IDLE:      if (input_handshake)             next_state = S_RPLO_ADDR;
      S_RPLO_ADDR:                                  next_state = S_RPLO_CAPT;
      S_RPLO_CAPT:                                  next_state = S_RPHI_ADDR;
      S_RPHI_ADDR:                                  next_state = S_RPHI_CAPT;
      S_RPHI_CAPT:                                  next_state = S_ROW_INIT;
      S_RPHI_ADDR_NEXT:                             next_state = S_RPHI_CAPT_NEXT;
      S_RPHI_CAPT_NEXT:                             next_state = S_ROW_INIT;
      S_ROW_INIT:  if (rp_lo == rp_hi)              next_state = S_EMIT;
                   else                             next_state = S_PIPE;
      S_PIPE:      if (d_acc_en && (j_mac_idx + 1 == rp_hi))
                                                    next_state = S_EMIT;
      S_EMIT:      if (output_handshake) begin
                     if (row_idx + 1 == n)          next_state = S_DONE;
                     else                           next_state = S_RPHI_ADDR_NEXT;
                   end
      S_DONE:                                       next_state = S_IDLE;
      default:                                      next_state = S_IDLE;
    endcase
  end

  // ---- Control outputs to dpath -----------------------------------------
  // In S_PIPE the pipeline runs every cycle. The capture/MAC strobes are a
  // pure function of pipe_phase, pipe_warm, and j_mac_idx -- they fire
  // automatically as soon as the pipeline is warm enough for each stage.
  //
  // Timing (entering S_PIPE with phase=0, warm=0):
  //   cyc 0: phase=0, warm=0 -- issue val[0]
  //   cyc 1: phase=1, warm=1 -- issue col[0]; capture val_reg<=val[0]
  //   cyc 2: phase=0, warm=2 -- issue val[1]; capture col_reg<=col[0]
  //   cyc 3: phase=1, warm=2 -- issue col[1]; val_reg<=val[1]; MAC nnz 0
  //   ... (steady state 2 cyc/nnz; per-nnz row cost is 2*nnz + 2)
  //
  // val_reg is read by the MAC in the same cycle it gets overwritten with
  // the next val[]. NBA semantics make this safe: the RHS of `acc <= ...`
  // reads val_reg's pre-edge value, while `val_reg <= mem_rdata` writes
  // its post-edge value. Both writes live in the same always_ff in
  // SPMVDpath -- splitting them across multiple always_ff blocks would
  // create an IEEE-1364 race that has bitten earlier pipelining attempts.
  always_comb begin
    d_begin_op          = 1'b0;
    d_capture_rplo      = 1'b0;
    d_capture_rphi      = 1'b0;
    d_capture_rphi_next = 1'b0;
    d_init_row          = 1'b0;
    d_pipe_step         = 1'b0;
    d_capture_val       = 1'b0;
    d_capture_col       = 1'b0;
    d_acc_en            = 1'b0;
    d_bump_row          = 1'b0;

    case (state)
      S_IDLE:           d_begin_op          = input_handshake;
      S_RPLO_CAPT:      d_capture_rplo      = 1'b1;
      S_RPHI_CAPT:      d_capture_rphi      = 1'b1;
      S_RPHI_CAPT_NEXT: d_capture_rphi_next = 1'b1;
      S_ROW_INIT:       d_init_row          = 1'b1;
      S_PIPE: begin
        d_pipe_step   = 1'b1;
        d_capture_val = (pipe_phase == 1'b1) && (pipe_warm >= 2'd1);
        d_capture_col = (pipe_phase == 1'b0) && (pipe_warm >= 2'd2);
        d_acc_en      = (pipe_phase == 1'b1) && (pipe_warm >= 2'd2)
                        && (j_mac_idx < rp_hi);
      end
      S_EMIT:           d_bump_row          = output_handshake;
      default: ;
    endcase
  end

endmodule


module SPMVDpath #(
  parameter p_int_bits   = 13,
  parameter p_frac_bits  = 14,
  parameter p_total_bits = p_int_bits + p_frac_bits,
  parameter p_acc_bits   = 48,
  parameter p_max_n      = 50
) (
  input  logic                           clk,
  input  logic                           rst,

  // From memory
  input  logic [31:0]                    mem_rdata,

  // From RF read port (combinational lookup of vec[col_reg])
  output logic [$clog2(p_max_n)-1:0]     vec_rd_idx,
  input  logic signed [p_total_bits-1:0] vec_rd_data,

  // Ctrl signals
  input  logic                           d_begin_op,
  input  logic                           d_capture_rplo,
  input  logic                           d_capture_rphi,
  input  logic                           d_capture_rphi_next,
  input  logic                           d_init_row,
  input  logic                           d_pipe_step,
  input  logic                           d_capture_val,
  input  logic                           d_capture_col,
  input  logic                           d_acc_en,
  input  logic                           d_bump_row,

  // Exposed to Ctrl
  output logic [31:0]                    row_idx,
  output logic [31:0]                    j_addr_idx,
  output logic [31:0]                    j_mac_idx,
  output logic                           pipe_phase,
  output logic [1:0]                     pipe_warm,
  output logic [31:0]                    rp_lo,
  output logic [31:0]                    rp_hi,

  // ostream payload
  output logic [31:0]                    ostream_msg_row_idx,
  output logic signed [p_total_bits-1:0] ostream_msg_row_val
);

  logic signed [p_total_bits-1:0] val_reg;
  logic signed [p_total_bits-1:0] col_reg;
  logic signed [p_acc_bits-1:0]   acc;

  // Combinational MAC: product is full-precision, accumulator adds it.
  logic signed [p_acc_bits-1:0] product_wide;

  FpMulWide #(
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits),
    .p_wide_bits (p_acc_bits)
  ) u_mul (
    .a     (val_reg),
    .b     (vec_rd_data),
    .result(product_wide)
  );

  // Drive vec RF lookup with the currently captured col.
  assign vec_rd_idx = col_reg[$clog2(p_max_n)-1:0];

  // Output payload: emit row_idx and row_val (truncated from acc).
  assign ostream_msg_row_idx = row_idx;
  assign ostream_msg_row_val = p_total_bits'(acc);

  // ALL pipeline-related state lives in this single always_ff block. Do
  // NOT split val_reg / col_reg / acc / counters across multiple always_ff
  // blocks: that would create an IEEE-1364 race when one block reads a
  // signal another block writes on the same edge. Verilator may schedule
  // such code consistently across simple tests yet diverge in larger
  // workloads (this is one of the things that bit prior pipeline attempts).
  always_ff @(posedge clk) begin
    if (rst) begin
      row_idx    <= '0;
      j_addr_idx <= '0;
      j_mac_idx  <= '0;
      pipe_phase <= 1'b0;
      pipe_warm  <= 2'd0;
      rp_lo      <= '0;
      rp_hi      <= '0;
      val_reg    <= '0;
      col_reg    <= '0;
      acc        <= '0;
    end else begin
      if (d_begin_op) begin
        row_idx <= '0;
      end
      if (d_capture_rplo) rp_lo <= mem_rdata;
      if (d_capture_rphi) rp_hi <= mem_rdata;
      // Non-first row prologue: previous row's rp_hi becomes new row's rp_lo,
      // and the new rp_ptr[row+1] read from memory becomes the new rp_hi.
      // Nonblocking semantics: rp_lo gets old rp_hi (start-of-cycle value),
      // rp_hi gets mem_rdata (end-of-cycle value). Both fire same cycle.
      if (d_capture_rphi_next) begin
        rp_lo <= rp_hi;
        rp_hi <= mem_rdata;
      end
      if (d_init_row) begin
        j_addr_idx <= rp_lo;
        j_mac_idx  <= rp_lo;
        pipe_phase <= 1'b0;
        pipe_warm  <= 2'd0;
        acc        <= '0;
      end
      if (d_pipe_step) begin
        pipe_phase <= ~pipe_phase;
        if (pipe_warm < 2'd2) pipe_warm <= pipe_warm + 2'd1;
        // j_addr_idx advances after issuing both val and col for the
        // current nnz, i.e. after the cycle where phase==1.
        if (pipe_phase == 1'b1 && j_addr_idx < rp_hi)
          j_addr_idx <= j_addr_idx + 32'd1;
      end
      if (d_capture_val) val_reg <= p_total_bits'($signed(mem_rdata));
      if (d_capture_col) col_reg <= p_total_bits'($signed(mem_rdata));
      if (d_acc_en) begin
        // Reading val_reg here grabs its pre-edge value, even when
        // d_capture_val is also true this cycle (NBA semantics).
        acc       <= acc + product_wide;
        j_mac_idx <= j_mac_idx + 32'd1;
      end
      if (d_bump_row) begin
        row_idx <= row_idx + 1;
      end
    end
  end

endmodule


module SPMV_seq #(
  parameter p_int_bits        = 13,
  parameter p_frac_bits       = 14,
  parameter p_total_bits      = p_int_bits + p_frac_bits,
  parameter p_acc_bits        = 48,
  parameter p_max_n           = 50,
  parameter p_m10k_addr_bits  = 32
) (
  input  logic                           clk,
  input  logic                           rst,

  // istream: single handshake starts the whole SPMV
  input  logic                           istream_val,
  output logic                           istream_rdy,

  // ostream: one handshake per output row
  output logic                           ostream_val,
  input  logic                           ostream_rdy,
  output logic [31:0]                    ostream_msg_row_idx,
  output logic signed [p_total_bits-1:0] ostream_msg_row_val,

  // Quasi-static (held by caller across the operation)
  input  logic [31:0]                    n,
  input  logic [p_m10k_addr_bits-1:0]    q_val_base,
  input  logic [p_m10k_addr_bits-1:0]    q_col_base,
  input  logic [p_m10k_addr_bits-1:0]    q_rowp_base,

  // Memory bus (SPMV drives addr; mem returns rdata 1 cycle later)
  output logic [p_m10k_addr_bits-1:0]    mem_addr,
  output logic                           mem_rd_en,
  input  logic [31:0]                    mem_rdata,

  // RF read port for vec[col]
  output logic [$clog2(p_max_n)-1:0]     vec_rd_idx,
  input  logic signed [p_total_bits-1:0] vec_rd_data
);

  // Ctrl <-> Dpath connections
  logic        d_begin_op, d_capture_rplo, d_capture_rphi, d_capture_rphi_next;
  logic        d_init_row, d_pipe_step;
  logic        d_capture_val, d_capture_col, d_acc_en, d_bump_row;
  logic [31:0] row_idx, j_addr_idx, j_mac_idx, rp_lo, rp_hi;
  logic        pipe_phase;
  logic [1:0]  pipe_warm;

  SPMVCtrl #(
    .p_m10k_addr_bits (p_m10k_addr_bits)
  ) u_ctrl (
    .clk, .rst,
    .istream_val, .istream_rdy,
    .ostream_val, .ostream_rdy,
    .n, .q_val_base, .q_col_base, .q_rowp_base,
    .row_idx, .j_addr_idx, .j_mac_idx, .pipe_phase, .pipe_warm, .rp_lo, .rp_hi,
    .d_begin_op, .d_capture_rplo, .d_capture_rphi, .d_capture_rphi_next,
    .d_init_row, .d_pipe_step,
    .d_capture_val, .d_capture_col, .d_acc_en, .d_bump_row,
    .mem_addr, .mem_rd_en
  );

  SPMVDpath #(
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits),
    .p_acc_bits  (p_acc_bits),
    .p_max_n     (p_max_n)
  ) u_dpath (
    .clk, .rst,
    .mem_rdata,
    .vec_rd_idx, .vec_rd_data,
    .d_begin_op, .d_capture_rplo, .d_capture_rphi, .d_capture_rphi_next,
    .d_init_row, .d_pipe_step,
    .d_capture_val, .d_capture_col, .d_acc_en, .d_bump_row,
    .row_idx, .j_addr_idx, .j_mac_idx, .pipe_phase, .pipe_warm, .rp_lo, .rp_hi,
    .ostream_msg_row_idx, .ostream_msg_row_val
  );

endmodule
