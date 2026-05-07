// v3 CGCtrl: monolithic FSM. CGDpath is a pure datapath (no FSM of
// its own); CGCtrl drives every mux select, latch enable, and val/rdy
// handshake signal.
//
// A single `sel_y` register picks between x/y base addresses in the
// inner sub-states so the same FSM runs both x and y solves.
//
// p_lanes parallelism:
//   - The streaming phases (VDOT feeds, AXPY feeds, VNS, COPY_D) use
//     stream_idx as a *group* counter, 0..num_groups-1 where
//     num_groups = ceil(n/p_lanes). Each cycle/handshake covers
//     p_lanes elements.
//   - The per-element phases (LD, WB, SPMV COLLECT) still use
//     stream_idx as an element counter, 0..n-1.
//   - For group phases CGCtrl drives the p_lanes-wide RF ports via
//     group_idx_packed/group_in_valid (input side) and
//     group_out_packed/group_out_valid (AXPY output side). Out-of-n
//     lanes have valid=0 so CGDpath returns zero on reads and we[k]
//     is gated off for writes.
//   - For element phases CGCtrl uses lane 0 only (other lanes masked).

module CGCtrl #(
  parameter p_lanes            = 4,
  parameter p_max_n            = 50,
  parameter p_int_bits         = 13,
  parameter p_frac_bits        = 14,
  parameter p_total_bits       = p_int_bits + p_frac_bits,
  parameter p_acc_bits         = (p_total_bits <= 27)
      ? 48
      : (2*p_total_bits - p_frac_bits + $clog2(p_max_n+1) + 4),
  parameter p_m10k_addr_bits   = 32,
  parameter p_word_bits        = (p_total_bits <= 32) ? 32 : 64,
  parameter p_cx_x_base_addr   = 2 * p_max_n * p_max_n + p_max_n + 1,
  parameter p_cx_y_base_addr   = 2 * p_max_n * p_max_n + 2 * p_max_n + 1,
  parameter p_x_base_addr      = 2 * p_max_n * p_max_n + 3 * p_max_n + 1,
  parameter p_y_base_addr      = 2 * p_max_n * p_max_n + 4 * p_max_n + 1
) (
  input  logic clk,
  input  logic rst,

  // ARM handshake
  input  logic sw_go,
  output logic sw_done,
  input  logic sw_done_ack,

  // Solve parameters
  input  logic [31:0] n,
  input  logic [31:0] max_iter,
  input  logic [31:0] eps_sq,

  // Observability from CGDpath
  input  logic [31:0]                  iter,
  input  logic signed [p_acc_bits-1:0] rr_new,
  input  logic signed [p_acc_bits-1:0] rr_old,

  // -- Control outputs to CGDpath ------------------------------------------

  // Memory bus driven by CGCtrl during LD / WB
  output logic [p_m10k_addr_bits-1:0] ctrl_mem_addr,
  output logic                        ctrl_mem_wr_en,
  output logic [p_word_bits-1:0]      ctrl_mem_wdata,
  output logic                        ctrl_mem_src_spmv,

  // RF read ports (p_lanes-wide)
  output logic [2:0]                              rd_a_sel,
  output logic [p_lanes*$clog2(p_max_n)-1:0]      rd_a_idx_packed,
  output logic [p_lanes-1:0]                      rd_a_valid,
  input  logic [p_lanes*p_total_bits-1:0]         rd_a_data_packed,

  output logic [2:0]                              rd_b_sel,
  output logic [p_lanes*$clog2(p_max_n)-1:0]      rd_b_idx_packed,
  output logic [p_lanes-1:0]                      rd_b_valid,
  input  logic [p_lanes*p_total_bits-1:0]         rd_b_data_packed,

  output logic [2:0]                              rd_vec_sel,

  // RF write port (p_lanes-wide)
  output logic [2:0]                              wr_sel,
  output logic [p_lanes*$clog2(p_max_n)-1:0]      wr_idx_ctrl_packed,
  output logic                                    wr_idx_src_spmv,
  output logic [p_lanes-1:0]                      we,
  output logic [2:0]                              wdata_src,

  // Scalar latches
  output logic                               latch_dq,
  output logic                               latch_rr_new,
  output logic                               latch_alpha,
  output logic                               latch_beta,
  output logic                               refresh_rr_reg,
  output logic                               bump_iter,
  output logic                               reset_iter,

  // Submodule handshakes
  output logic                               vdot_istream_val,
  input  logic                               vdot_istream_rdy,
  input  logic                               vdot_ostream_val,
  output logic                               vdot_ostream_rdy,

  output logic                               axpy_istream_val,
  input  logic                               axpy_istream_rdy,
  input  logic                               axpy_ostream_val,
  output logic                               axpy_ostream_rdy,
  output logic                               axpy_mode,
  output logic                               axpy_coef_src_beta,

  output logic                               spmv_istream_val,
  input  logic                               spmv_istream_rdy,
  input  logic                               spmv_ostream_val,
  output logic                               spmv_ostream_rdy,

  output logic                               fpdiv_istream_val,
  input  logic                               fpdiv_istream_rdy,
  input  logic                               fpdiv_ostream_val,
  output logic                               fpdiv_ostream_rdy,
  output logic                               fpdiv_a_src_rrnew,
  output logic                               fpdiv_b_src_rr
);

  localparam IDX_W       = $clog2(p_max_n);
  localparam CLOG2_LANES = $clog2(p_lanes);
  // Narrow width for n, n-1, num_groups, num_groups-1, stream/out indices
  // and e_in/e_out. n is in [0..p_max_n], so $clog2(p_max_n+1) bits
  // suffice (e.g. 6 bits for p_max_n=50). Also the natural bound for
  // stream_elem_base/e_in since (num_groups-1)*p_lanes + p_lanes-1 <
  // p_max_n + p_lanes fits in N_W for typical configs (50+4=54 -> 6).
  localparam N_W         = $clog2(p_max_n + 1);

  //----------------------------------------------------------------------
  // RF select encodings (match CGDpath's rf_read function)
  //----------------------------------------------------------------------
  localparam [2:0] RF_D_REG     = 3'd0;
  localparam [2:0] RF_R_REG     = 3'd1;
  localparam [2:0] RF_X_VEC_REG = 3'd2;
  localparam [2:0] RF_CX_REG    = 3'd3;
  localparam [2:0] RF_Q_BUF     = 3'd4;

  localparam [2:0] WD_MEM  = 3'd0;
  localparam [2:0] WD_AXPY = 3'd1;
  localparam [2:0] WD_SPMV = 3'd2;
  localparam [2:0] WD_RDA  = 3'd3;
  localparam [2:0] WD_VNS  = 3'd4;

  //----------------------------------------------------------------------
  // State enum
  //----------------------------------------------------------------------
  typedef enum logic [5:0] {
    S_IDLE,
    S_PREP,

    S_LD_X_ADDR,
    S_LD_X_CAPT,
    S_LD_CX_ADDR,
    S_LD_CX_CAPT,

    S_SPMV_INIT_FIRE,
    S_SPMV_INIT_COLLECT,

    S_VNS_R,
    S_COPY_D,

    S_VDOT_INIT_FEED,
    S_RR_REG_COPY,

    S_SPMV_RUN_FIRE,
    S_SPMV_RUN_COLLECT,

    S_VDOT_DQ_FEED,

    S_DIV_A_SEND,
    S_DIV_A_RECV,

    S_AXPY_X_FEED,
    S_AXPY_R_FEED,

    S_VDOT_RR_FEED,

    S_DIV_B_SEND,
    S_DIV_B_RECV,

    S_AXPY_D_FEED,

    S_RUN_CHECK,

    S_WB_WRITE,

    S_CG_DONE
  } state_t;

  state_t state, next_state;

  //----------------------------------------------------------------------
  // Auxiliary registers
  //----------------------------------------------------------------------
  logic                               sel_y;
  logic [$clog2(p_max_n+1)-1:0]       stream_idx;
  logic [$clog2(p_max_n+1)-1:0]       out_idx;

  //----------------------------------------------------------------------
  // Handshake wires
  //----------------------------------------------------------------------
  wire vdot_in_hs   = vdot_istream_val  && vdot_istream_rdy;
  wire vdot_out_hs  = vdot_ostream_val  && vdot_ostream_rdy;
  wire axpy_in_hs   = axpy_istream_val  && axpy_istream_rdy;
  wire axpy_out_hs  = axpy_ostream_val  && axpy_ostream_rdy;
  wire spmv_in_hs   = spmv_istream_val  && spmv_istream_rdy;
  wire spmv_out_hs  = spmv_ostream_val  && spmv_ostream_rdy;
  wire fpdiv_in_hs  = fpdiv_istream_val && fpdiv_istream_rdy;
  wire fpdiv_out_hs = fpdiv_ostream_val && fpdiv_ostream_rdy;

  //----------------------------------------------------------------------
  // Registered narrow forms of n, n-1, num_groups, num_groups-1.
  // Capturing n once at S_IDLE -> S_PREP and reusing the N_W-bit forms
  // breaks the combinational chain off the PIO and shrinks every
  // dependent comparator from 32 bits to N_W bits.
  //----------------------------------------------------------------------
  logic [N_W-1:0] n_narrow;
  logic [N_W-1:0] num_groups_calc;
  assign n_narrow        = n[N_W-1:0];
  // Round-up add in 32 bits, then truncate. n_narrow + (p_lanes - 1) can
  // exceed N_W bits (e.g. n=50, p_lanes=16, N_W=6 -> 65 wraps to 1).
  assign num_groups_calc = N_W'((n + unsigned'(p_lanes - 1)) >> CLOG2_LANES);

  logic [N_W-1:0] n_reg;
  logic [N_W-1:0] n_minus_1_reg;
  logic [N_W-1:0] num_groups_reg;
  logic [N_W-1:0] num_groups_minus_1_reg;

  always_ff @(posedge clk) begin
    if (rst) begin
      n_reg                  <= '0;
      n_minus_1_reg          <= '0;
      num_groups_reg         <= '0;
      num_groups_minus_1_reg <= '0;
    end else if (state == S_IDLE && sw_go) begin
      n_reg                  <= n_narrow;
      n_minus_1_reg          <= n_narrow        - N_W'(1);
      num_groups_reg         <= num_groups_calc;
      num_groups_minus_1_reg <= num_groups_calc - N_W'(1);
    end
  end

  //----------------------------------------------------------------------
  // Convergence test
  //----------------------------------------------------------------------
  logic signed [p_acc_bits-1:0] eps_sq_wide;
  // eps_sq is a small positive fixed-point threshold from the ARM/PIO;
  // sign-extending the whole 32-bit word is safe at any p_total_bits in
  // [2, 64] -- callers pre-clamp the value so the upper bits are zero.
  assign eps_sq_wide = p_acc_bits'($signed(eps_sq));

  logic run_converged;
  assign run_converged = (iter >= max_iter)
                      || (rr_new <= eps_sq_wide)
                      || (iter > 32'd1 && rr_new >= rr_old);

  //----------------------------------------------------------------------
  // Base-address selection via sel_y
  //----------------------------------------------------------------------
  logic [p_m10k_addr_bits-1:0] x_base_sel;
  logic [p_m10k_addr_bits-1:0] cx_base_sel;
  assign x_base_sel  = sel_y ? p_m10k_addr_bits'(p_y_base_addr)
                             : p_m10k_addr_bits'(p_x_base_addr);
  assign cx_base_sel = sel_y ? p_m10k_addr_bits'(p_cx_y_base_addr)
                             : p_m10k_addr_bits'(p_cx_x_base_addr);

  //----------------------------------------------------------------------
  // Per-lane index / valid helpers.
  //  group_* are driven from stream_idx (input side) and out_idx (AXPY
  //  output side) with elem = (group << CLOG2_LANES) + k.
  //  single_* are lane-0-only forms for LD / WB / SPMV-COLLECT which
  //  use stream_idx directly as an element index.
  //----------------------------------------------------------------------
  logic [p_lanes*IDX_W-1:0] group_in_idx_packed;
  logic [p_lanes-1:0]       group_in_valid;
  logic [p_lanes*IDX_W-1:0] group_out_idx_packed;
  logic [p_lanes-1:0]       group_out_valid;
  logic [p_lanes*IDX_W-1:0] single_idx_packed;
  logic [p_lanes-1:0]       single_lane0_mask;

  logic [N_W-1:0] stream_elem_base;
  logic [N_W-1:0] out_elem_base;
  assign stream_elem_base = stream_idx << CLOG2_LANES;
  assign out_elem_base    = out_idx    << CLOG2_LANES;

  always_comb begin
    group_in_idx_packed  = '0;
    group_in_valid       = '0;
    group_out_idx_packed = '0;
    group_out_valid      = '0;

    for (int k = 0; k < p_lanes; k++) begin
      automatic logic [N_W-1:0] e_in;
      automatic logic [N_W-1:0] e_out;
      e_in  = stream_elem_base + N_W'(k);
      e_out = out_elem_base    + N_W'(k);
      if (e_in < n_reg) begin
        group_in_valid[k] = 1'b1;
        group_in_idx_packed[(k+1)*IDX_W-1 -: IDX_W] = e_in[IDX_W-1:0];
      end
      if (e_out < n_reg) begin
        group_out_valid[k] = 1'b1;
        group_out_idx_packed[(k+1)*IDX_W-1 -: IDX_W] = e_out[IDX_W-1:0];
      end
    end
  end

  // Single-element packed form: lane 0 holds stream_idx, others are don't-care.
  always_comb begin
    single_idx_packed             = '0;
    single_idx_packed[IDX_W-1:0]  = stream_idx[IDX_W-1:0];
    single_lane0_mask             = '0;
    single_lane0_mask[0]          = 1'b1;
  end

  // Lane-0 extract of rd_a_data_packed for writing x_vec to SRAM in WB.
  logic signed [p_total_bits-1:0] rd_a_data_lane0;
  assign rd_a_data_lane0 = $signed(rd_a_data_packed[p_total_bits-1:0]);

  //----------------------------------------------------------------------
  // State register
  //----------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) state <= S_IDLE;
    else     state <= next_state;
  end

  //----------------------------------------------------------------------
  // Next-state logic
  //----------------------------------------------------------------------
  always_comb begin
    next_state = state;
    case (state)
      S_IDLE:            if (sw_go)                                           next_state = S_PREP;
      S_PREP:                                                                 next_state = S_LD_X_ADDR;

      // LD x vector (1 elem / cycle)
      S_LD_X_ADDR:                                                            next_state = S_LD_X_CAPT;
      S_LD_X_CAPT:       if (stream_idx == n_minus_1_reg)                         next_state = S_LD_CX_ADDR;
                         else                                                 next_state = S_LD_X_ADDR;

      // LD cx vector (1 elem / cycle)
      S_LD_CX_ADDR:                                                           next_state = S_LD_CX_CAPT;
      S_LD_CX_CAPT:      if (stream_idx == n_minus_1_reg)                         next_state = S_SPMV_INIT_FIRE;
                         else                                                 next_state = S_LD_CX_ADDR;

      // INIT SPMV (1 row / handshake)
      S_SPMV_INIT_FIRE:  if (spmv_in_hs)                                      next_state = S_SPMV_INIT_COLLECT;
      S_SPMV_INIT_COLLECT:
                         if (spmv_out_hs && stream_idx == n_minus_1_reg)          next_state = S_VNS_R;

      // r_reg[i] = -(cx[i] + q_buf[i]); then copy r_reg -> d_reg
      // (p_lanes/cycle, num_groups cycles)
      S_VNS_R:           if (stream_idx == num_groups_minus_1_reg)                next_state = S_COPY_D;
      S_COPY_D:          if (stream_idx == num_groups_minus_1_reg)                next_state = S_VDOT_INIT_FEED;

      // VDOT r.r -> rr_new_latched
      S_VDOT_INIT_FEED:  if (vdot_out_hs)                                     next_state = S_RR_REG_COPY;

      // rr_reg <= rr_new_latched (initial rr)
      S_RR_REG_COPY:                                                          next_state = S_SPMV_RUN_FIRE;

      // RUN: SPMV (Q * d_reg) -> q_buf (1 row / handshake)
      S_SPMV_RUN_FIRE:   if (spmv_in_hs)                                      next_state = S_SPMV_RUN_COLLECT;
      S_SPMV_RUN_COLLECT:
                         if (spmv_out_hs && stream_idx == n_minus_1_reg)          next_state = S_VDOT_DQ_FEED;

      // dq = d . q_buf
      S_VDOT_DQ_FEED:    if (vdot_out_hs)                                     next_state = S_DIV_A_SEND;

      // alpha = rr_reg / dq
      S_DIV_A_SEND:      if (fpdiv_in_hs)                                     next_state = S_DIV_A_RECV;
      S_DIV_A_RECV:      if (fpdiv_out_hs)                                    next_state = S_AXPY_X_FEED;

      // x_vec_reg[i] += alpha * d_reg[i]
      S_AXPY_X_FEED:     if (axpy_out_hs && out_idx == num_groups_minus_1_reg)    next_state = S_AXPY_R_FEED;

      // r_reg[i] -= alpha * q_buf[i]
      S_AXPY_R_FEED:     if (axpy_out_hs && out_idx == num_groups_minus_1_reg)    next_state = S_VDOT_RR_FEED;

      // rr_new = r_new . r_new
      S_VDOT_RR_FEED:    if (vdot_out_hs)                                     next_state = S_DIV_B_SEND;

      // beta = rr_new / rr_reg
      S_DIV_B_SEND:      if (fpdiv_in_hs)                                     next_state = S_DIV_B_RECV;
      S_DIV_B_RECV:      if (fpdiv_out_hs)                                    next_state = S_AXPY_D_FEED;

      // d_reg[i] = r_reg[i] + beta * d_reg[i]
      S_AXPY_D_FEED:     if (axpy_out_hs && out_idx == num_groups_minus_1_reg)    next_state = S_RUN_CHECK;

      // Convergence check + iter bump
      S_RUN_CHECK:       if (run_converged)                                   next_state = S_WB_WRITE;
                         else                                                 next_state = S_SPMV_RUN_FIRE;

      // WB phase: stream x_vec_reg -> cg_mem[x_base..] (1 elem / cycle)
      S_WB_WRITE: begin
        if (stream_idx == n_minus_1_reg) begin
          if (sel_y)   next_state = S_CG_DONE;
          else         next_state = S_LD_X_ADDR;  // swap to Y phase
        end
      end

      S_CG_DONE:         if (sw_done_ack)                                     next_state = S_IDLE;
      default:                                                                next_state = S_IDLE;
    endcase
  end

  //----------------------------------------------------------------------
  // sel_y, stream_idx, out_idx updates
  //----------------------------------------------------------------------
  // A "phase" groups states that share the same streaming counter.
  function automatic logic [4:0] phase_of(state_t s);
    case (s)
      S_LD_X_ADDR,         S_LD_X_CAPT:          phase_of = 5'd1;
      S_LD_CX_ADDR,        S_LD_CX_CAPT:         phase_of = 5'd2;
      S_SPMV_INIT_FIRE,    S_SPMV_INIT_COLLECT:  phase_of = 5'd3;
      S_VNS_R:                                   phase_of = 5'd4;
      S_COPY_D:                                  phase_of = 5'd5;
      S_VDOT_INIT_FEED:                          phase_of = 5'd6;
      S_RR_REG_COPY:                             phase_of = 5'd7;
      S_SPMV_RUN_FIRE,     S_SPMV_RUN_COLLECT:   phase_of = 5'd8;
      S_VDOT_DQ_FEED:                            phase_of = 5'd9;
      S_DIV_A_SEND,        S_DIV_A_RECV:         phase_of = 5'd10;
      S_AXPY_X_FEED:                             phase_of = 5'd11;
      S_AXPY_R_FEED:                             phase_of = 5'd12;
      S_VDOT_RR_FEED:                            phase_of = 5'd13;
      S_DIV_B_SEND,        S_DIV_B_RECV:         phase_of = 5'd14;
      S_AXPY_D_FEED:                             phase_of = 5'd15;
      S_RUN_CHECK:                               phase_of = 5'd16;
      S_WB_WRITE:                                phase_of = 5'd17;
      default:                                   phase_of = 5'd0;
    endcase
  endfunction

  logic reset_counters;
  assign reset_counters = (phase_of(state) != phase_of(next_state));

  always_ff @(posedge clk) begin
    if (rst) begin
      sel_y      <= 1'b0;
      stream_idx <= '0;
      out_idx    <= '0;
    end else begin
      if (state == S_PREP)
        sel_y <= 1'b0;
      else if (state == S_WB_WRITE && stream_idx == n_minus_1_reg && !sel_y)
        sel_y <= 1'b1;

      if (reset_counters)
        stream_idx <= '0;
      else begin
        case (state)
          // Per-element phases: increment every cycle
          S_LD_X_CAPT, S_LD_CX_CAPT, S_WB_WRITE:
            stream_idx <= stream_idx + 1;
          // SPMV collect: increment per row handshake
          S_SPMV_INIT_COLLECT, S_SPMV_RUN_COLLECT:
            if (spmv_out_hs) stream_idx <= stream_idx + 1;
          // Group phases: increment every cycle (VNS/COPY_D -- comb op)
          S_VNS_R, S_COPY_D:
            stream_idx <= stream_idx + 1;
          // Group phases: increment per VDOT input handshake
          S_VDOT_INIT_FEED, S_VDOT_DQ_FEED, S_VDOT_RR_FEED:
            if (vdot_in_hs) stream_idx <= stream_idx + 1;
          // Group phases: increment per AXPY input handshake
          S_AXPY_X_FEED, S_AXPY_R_FEED, S_AXPY_D_FEED:
            if (axpy_in_hs) stream_idx <= stream_idx + 1;
          default: ;
        endcase
      end

      if (reset_counters)
        out_idx <= '0;
      else if (axpy_out_hs &&
               (state == S_AXPY_X_FEED || state == S_AXPY_R_FEED || state == S_AXPY_D_FEED))
        out_idx <= out_idx + 1;
    end
  end

  //----------------------------------------------------------------------
  // Output defaults + per-state drive
  //----------------------------------------------------------------------
  always_comb begin
    // ARM
    sw_done = 1'b0;

    // Memory bus (CGCtrl side)
    ctrl_mem_addr     = '0;
    ctrl_mem_wr_en    = 1'b0;
    ctrl_mem_wdata    = '0;
    ctrl_mem_src_spmv = 1'b0;

    // RF read/write
    rd_a_sel            = '0;
    rd_a_idx_packed     = '0;
    rd_a_valid          = '0;
    rd_b_sel            = '0;
    rd_b_idx_packed     = '0;
    rd_b_valid          = '0;
    rd_vec_sel          = RF_D_REG;
    wr_sel              = '0;
    wr_idx_ctrl_packed  = '0;
    wr_idx_src_spmv     = 1'b0;
    we                  = '0;
    wdata_src           = '0;

    // Scalar latches
    latch_dq          = 1'b0;
    latch_rr_new      = 1'b0;
    latch_alpha       = 1'b0;
    latch_beta        = 1'b0;
    refresh_rr_reg    = 1'b0;
    bump_iter         = 1'b0;
    reset_iter        = 1'b0;

    // Submodule handshakes
    vdot_istream_val      = 1'b0;
    vdot_ostream_rdy      = 1'b0;
    axpy_istream_val      = 1'b0;
    axpy_ostream_rdy      = 1'b0;
    axpy_mode             = 1'b0;
    axpy_coef_src_beta    = 1'b0;
    spmv_istream_val      = 1'b0;
    spmv_ostream_rdy      = 1'b0;
    fpdiv_istream_val     = 1'b0;
    fpdiv_ostream_rdy     = 1'b0;
    fpdiv_a_src_rrnew     = 1'b0;
    fpdiv_b_src_rr        = 1'b0;

    case (state)
      S_IDLE:            ;
      S_PREP:            reset_iter = 1'b1;

      // LD x (lane 0 only): ADDR drives mem_addr, CAPT captures mem_rdata
      S_LD_X_ADDR: begin
        ctrl_mem_addr = x_base_sel + p_m10k_addr_bits'(stream_idx);
      end
      S_LD_X_CAPT: begin
        ctrl_mem_addr      = x_base_sel + p_m10k_addr_bits'(stream_idx);
        wr_sel             = RF_X_VEC_REG;
        wr_idx_ctrl_packed = single_idx_packed;
        we                 = single_lane0_mask;
        wdata_src          = WD_MEM;
      end

      S_LD_CX_ADDR: begin
        ctrl_mem_addr = cx_base_sel + p_m10k_addr_bits'(stream_idx);
      end
      S_LD_CX_CAPT: begin
        ctrl_mem_addr      = cx_base_sel + p_m10k_addr_bits'(stream_idx);
        wr_sel             = RF_CX_REG;
        wr_idx_ctrl_packed = single_idx_packed;
        we                 = single_lane0_mask;
        wdata_src          = WD_MEM;
      end

      // SPMV INIT (vec = x_vec_reg): q_buf[row] = row_val
      S_SPMV_INIT_FIRE: begin
        ctrl_mem_src_spmv = 1'b1;
        spmv_istream_val  = 1'b1;
        rd_vec_sel        = RF_X_VEC_REG;
      end
      S_SPMV_INIT_COLLECT: begin
        ctrl_mem_src_spmv = 1'b1;
        spmv_ostream_rdy  = 1'b1;
        rd_vec_sel        = RF_X_VEC_REG;
        wr_sel            = RF_Q_BUF;
        wr_idx_src_spmv   = 1'b1;
        wdata_src         = WD_SPMV;
        we[0]             = spmv_out_hs;
      end

      // VNS: r_reg[i] = -(cx_reg[i] + q_buf[i]) (p_lanes/cycle)
      S_VNS_R: begin
        rd_a_sel           = RF_CX_REG;
        rd_a_idx_packed    = group_in_idx_packed;
        rd_a_valid         = group_in_valid;
        rd_b_sel           = RF_Q_BUF;
        rd_b_idx_packed    = group_in_idx_packed;
        rd_b_valid         = group_in_valid;
        wr_sel             = RF_R_REG;
        wr_idx_ctrl_packed = group_in_idx_packed;
        we                 = group_in_valid;
        wdata_src          = WD_VNS;
      end

      // Copy r_reg -> d_reg (p_lanes/cycle)
      S_COPY_D: begin
        rd_a_sel           = RF_R_REG;
        rd_a_idx_packed    = group_in_idx_packed;
        rd_a_valid         = group_in_valid;
        wr_sel             = RF_D_REG;
        wr_idx_ctrl_packed = group_in_idx_packed;
        we                 = group_in_valid;
        wdata_src          = WD_RDA;
      end

      // VDOT r.r for INIT
      S_VDOT_INIT_FEED: begin
        rd_a_sel         = RF_R_REG;
        rd_a_idx_packed  = group_in_idx_packed;
        rd_a_valid       = group_in_valid;
        rd_b_sel         = RF_R_REG;
        rd_b_idx_packed  = group_in_idx_packed;
        rd_b_valid       = group_in_valid;
        vdot_istream_val = (stream_idx < num_groups_reg);
        vdot_ostream_rdy = 1'b1;
        latch_rr_new     = vdot_out_hs;
      end

      // rr_reg <= rr_new_latched (initial rr)
      S_RR_REG_COPY: refresh_rr_reg = 1'b1;

      // SPMV RUN (vec = d_reg)
      S_SPMV_RUN_FIRE: begin
        ctrl_mem_src_spmv = 1'b1;
        spmv_istream_val  = 1'b1;
        rd_vec_sel        = RF_D_REG;
      end
      S_SPMV_RUN_COLLECT: begin
        ctrl_mem_src_spmv = 1'b1;
        spmv_ostream_rdy  = 1'b1;
        rd_vec_sel        = RF_D_REG;
        wr_sel            = RF_Q_BUF;
        wr_idx_src_spmv   = 1'b1;
        wdata_src         = WD_SPMV;
        we[0]             = spmv_out_hs;
      end

      // VDOT d . q -> dq_latched
      S_VDOT_DQ_FEED: begin
        rd_a_sel         = RF_D_REG;
        rd_a_idx_packed  = group_in_idx_packed;
        rd_a_valid       = group_in_valid;
        rd_b_sel         = RF_Q_BUF;
        rd_b_idx_packed  = group_in_idx_packed;
        rd_b_valid       = group_in_valid;
        vdot_istream_val = (stream_idx < num_groups_reg);
        vdot_ostream_rdy = 1'b1;
        latch_dq         = vdot_out_hs;
      end

      // alpha = rr_reg / dq
      S_DIV_A_SEND: begin
        fpdiv_istream_val = 1'b1;
        fpdiv_a_src_rrnew = 1'b0;  // rr_reg
        fpdiv_b_src_rr    = 1'b0;  // dq_latched
      end
      S_DIV_A_RECV: begin
        fpdiv_ostream_rdy = 1'b1;
        latch_alpha       = fpdiv_out_hs;
      end

      // AXPY X: x_vec[i] += alpha * d[i]
      S_AXPY_X_FEED: begin
        rd_a_sel           = RF_X_VEC_REG;
        rd_a_idx_packed    = group_in_idx_packed;
        rd_a_valid         = group_in_valid;
        rd_b_sel           = RF_D_REG;
        rd_b_idx_packed    = group_in_idx_packed;
        rd_b_valid         = group_in_valid;
        axpy_istream_val   = (stream_idx < num_groups_reg);
        axpy_ostream_rdy   = 1'b1;
        axpy_mode          = 1'b0;
        axpy_coef_src_beta = 1'b0;
        wr_sel             = RF_X_VEC_REG;
        wr_idx_ctrl_packed = group_out_idx_packed;
        we                 = axpy_out_hs ? group_out_valid : '0;
        wdata_src          = WD_AXPY;
      end

      // AXPY R: r[i] -= alpha * q[i]
      S_AXPY_R_FEED: begin
        rd_a_sel           = RF_R_REG;
        rd_a_idx_packed    = group_in_idx_packed;
        rd_a_valid         = group_in_valid;
        rd_b_sel           = RF_Q_BUF;
        rd_b_idx_packed    = group_in_idx_packed;
        rd_b_valid         = group_in_valid;
        axpy_istream_val   = (stream_idx < num_groups_reg);
        axpy_ostream_rdy   = 1'b1;
        axpy_mode          = 1'b1;
        axpy_coef_src_beta = 1'b0;
        wr_sel             = RF_R_REG;
        wr_idx_ctrl_packed = group_out_idx_packed;
        we                 = axpy_out_hs ? group_out_valid : '0;
        wdata_src          = WD_AXPY;
      end

      // VDOT r_new . r_new -> rr_new_latched
      S_VDOT_RR_FEED: begin
        rd_a_sel         = RF_R_REG;
        rd_a_idx_packed  = group_in_idx_packed;
        rd_a_valid       = group_in_valid;
        rd_b_sel         = RF_R_REG;
        rd_b_idx_packed  = group_in_idx_packed;
        rd_b_valid       = group_in_valid;
        vdot_istream_val = (stream_idx < num_groups_reg);
        vdot_ostream_rdy = 1'b1;
        latch_rr_new     = vdot_out_hs;
      end

      // beta = rr_new / rr_reg
      S_DIV_B_SEND: begin
        fpdiv_istream_val = 1'b1;
        fpdiv_a_src_rrnew = 1'b1;
        fpdiv_b_src_rr    = 1'b1;
      end
      S_DIV_B_RECV: begin
        fpdiv_ostream_rdy = 1'b1;
        latch_beta        = fpdiv_out_hs;
      end

      // AXPY D: d[i] = r[i] + beta * d[i]
      S_AXPY_D_FEED: begin
        rd_a_sel           = RF_R_REG;
        rd_a_idx_packed    = group_in_idx_packed;
        rd_a_valid         = group_in_valid;
        rd_b_sel           = RF_D_REG;
        rd_b_idx_packed    = group_in_idx_packed;
        rd_b_valid         = group_in_valid;
        axpy_istream_val   = (stream_idx < num_groups_reg);
        axpy_ostream_rdy   = 1'b1;
        axpy_mode          = 1'b0;
        axpy_coef_src_beta = 1'b1;
        wr_sel             = RF_D_REG;
        wr_idx_ctrl_packed = group_out_idx_packed;
        we                 = axpy_out_hs ? group_out_valid : '0;
        wdata_src          = WD_AXPY;
      end

      // Convergence check: always bump iter + refresh rr_reg.
      // If the check fires, next state is S_WB_WRITE; else S_SPMV_RUN_FIRE.
      S_RUN_CHECK: begin
        bump_iter      = 1'b1;
        refresh_rr_reg = 1'b1;
      end

      // WB: single-cycle write. Drive addr + wdata + wr_en from lane 0.
      S_WB_WRITE: begin
        ctrl_mem_addr   = x_base_sel + p_m10k_addr_bits'(stream_idx);
        ctrl_mem_wr_en  = 1'b1;
        rd_a_sel        = RF_X_VEC_REG;
        rd_a_idx_packed = single_idx_packed;
        rd_a_valid      = single_lane0_mask;
        ctrl_mem_wdata  = p_word_bits'($signed(rd_a_data_lane0));
        if (stream_idx == n_minus_1_reg && !sel_y)
          reset_iter = 1'b1;
      end

      S_CG_DONE: sw_done = 1'b1;

      default: ;
    endcase
  end

endmodule
