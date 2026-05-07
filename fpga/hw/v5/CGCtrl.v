// v5 CGCtrl: sequential x-then-y CG solver. sel_y picks dimension.

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
  parameter p_word_bits        = (p_total_bits <= 32) ? 32 : 64
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

  // -- x/y load + writeback bus (CGDpath routes to x_ram or y_ram by sel_y).
  // Read-back into the X_VEC_REG RF in S_LD_X_CAPT goes through CGDpath's
  // WD_MEM mux, so we don't need ctrl_xy_rdata here.
  output logic [p_m10k_addr_bits-1:0] ctrl_xy_addr,
  output logic                        ctrl_xy_wr_en,
  output logic [p_word_bits-1:0]      ctrl_xy_wdata,

  // -- VNS_R cx serial-read port (CGDpath routes to cx_ram or cy_ram).
  // The capture is consumed by CGDpath's WD_VNS_SCALAR mux; CGCtrl
  // only drives addr/rd_en.
  output logic [p_m10k_addr_bits-1:0] vns_cx_addr,
  output logic                        vns_cx_rd_en,

  // sel_y is exposed so CGDpath/CGTop can route x_ram vs y_ram and
  // cx_ram vs cy_ram. Held by CGCtrl across the whole second-dimension
  // pass.
  output logic                        sel_y,

  // RF read ports (p_lanes-wide). rd_a/rd_b are shared across all states;
  // rd_c/rd_d are only used in S_AXPY_XR_FEED to feed u_axpy_r.
  output logic [2:0]                              rd_a_sel,
  output logic [p_lanes*$clog2(p_max_n)-1:0]      rd_a_idx_packed,
  output logic [p_lanes-1:0]                      rd_a_valid,
  input  logic [p_lanes*p_total_bits-1:0]         rd_a_data_packed,

  output logic [2:0]                              rd_b_sel,
  output logic [p_lanes*$clog2(p_max_n)-1:0]      rd_b_idx_packed,
  output logic [p_lanes-1:0]                      rd_b_valid,
  input  logic [p_lanes*p_total_bits-1:0]         rd_b_data_packed,

  output logic [2:0]                              rd_c_sel,
  output logic [p_lanes*$clog2(p_max_n)-1:0]      rd_c_idx_packed,
  output logic [p_lanes-1:0]                      rd_c_valid,
  input  logic [p_lanes*p_total_bits-1:0]         rd_c_data_packed,

  output logic [2:0]                              rd_d_sel,
  output logic [p_lanes*$clog2(p_max_n)-1:0]      rd_d_idx_packed,
  output logic [p_lanes-1:0]                      rd_d_valid,
  input  logic [p_lanes*p_total_bits-1:0]         rd_d_data_packed,

  output logic [2:0]                              rd_vec_sel,

  // Primary RF write port (p_lanes-wide)
  output logic [2:0]                              wr_sel,
  output logic [p_lanes*$clog2(p_max_n)-1:0]      wr_idx_ctrl_packed,
  output logic                                    wr_idx_src_spmv,
  output logic [p_lanes-1:0]                      we,
  output logic [2:0]                              wdata_src,

  // Secondary RF write port (p_lanes-wide). Shares the primary
  // wr_idx_ctrl_packed (callers always retire the same group on both ports).
  // Used in S_AXPY_XR_FEED (writes axpy_r_z_lane to r_reg) and in
  // S_VNS_R (writes -(cx+q) to d_reg in parallel with the primary r_reg
  // write).
  output logic [2:0]                              wr_sel_sec,
  output logic [2:0]                              wdata_src_sec,
  output logic [p_lanes-1:0]                      we_sec,

  // Scalar latches
  output logic                               latch_dq,
  output logic                               latch_rr_new,
  output logic                               latch_alpha,
  output logic                               latch_beta,
  // refresh_rr_reg copies rr_new_latched -> rr_reg after RUN_CHECK.
  // init_rr_reg bypasses rr_new_latched and writes rr_reg directly
  // from vdot_result during S_VDOT_INIT_FEED so the initial rr lands
  // in rr_reg the same cycle vdot finishes.
  output logic                               refresh_rr_reg,
  output logic                               init_rr_reg,
  output logic                               bump_iter,
  output logic                               reset_iter,

  // Submodule handshakes
  output logic                               vdot_istream_val,
  input  logic                               vdot_istream_rdy,
  input  logic                               vdot_ostream_val,
  output logic                               vdot_ostream_rdy,

  // AXPY x: handles x update (S_AXPY_XR_FEED) and d update (S_AXPY_D_FEED).
  // Mode is hard-wired to ADD inside CGDpath; coef source is shared.
  output logic                               axpy_x_istream_val,
  input  logic                               axpy_x_istream_rdy,
  input  logic                               axpy_x_ostream_val,
  output logic                               axpy_x_ostream_rdy,

  // AXPY r: handles r update (S_AXPY_XR_FEED only). Mode is hard-wired to SUB.
  output logic                               axpy_r_istream_val,
  input  logic                               axpy_r_istream_rdy,
  input  logic                               axpy_r_ostream_val,
  output logic                               axpy_r_ostream_rdy,

  // Shared AXPY coef select (both u_axpy_x and u_axpy_r see the same coef).
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
  localparam N_W         = $clog2(p_max_n + 1);
  // BANK_SEL_W = max(CLOG2_LANES, 1) so we can name a lane id even
  // when p_lanes == 1 (CLOG2_LANES = 0). Mirrors CGDpath.
  localparam BANK_SEL_W  = (CLOG2_LANES == 0) ? 1 : CLOG2_LANES;

  //----------------------------------------------------------------------
  // RF select encodings (match CGDpath's rf_read function). Four RFs:
  // D, R, X_VEC, Q_BUF -- cx lives in cx_ram (M10K), not in the central RF.
  //----------------------------------------------------------------------
  localparam [2:0] RF_D_REG     = 3'd0;
  localparam [2:0] RF_R_REG     = 3'd1;
  localparam [2:0] RF_X_VEC_REG = 3'd2;
  localparam [2:0] RF_Q_BUF     = 3'd4;

  localparam [2:0] WD_MEM         = 3'd0;
  localparam [2:0] WD_AXPY        = 3'd1;
  localparam [2:0] WD_SPMV        = 3'd2;
  // WD_VNS_SCALAR: wr_data[k] = -(vns_cx_rdata + rd_b_data[k]). Single-lane
  // writeback during the serialized S_VNS_R_CAPT.
  localparam [2:0] WD_VNS_SCALAR  = 3'd3;
  localparam [2:0] WD_AXPY_R      = 3'd5;

  //----------------------------------------------------------------------
  // State enum
  //----------------------------------------------------------------------
  typedef enum logic [5:0] {
    S_IDLE,
    S_PREP,

    S_LD_X_ADDR,
    S_LD_X_CAPT,

    S_SPMV_INIT_FIRE,
    S_SPMV_INIT_COLLECT,

    // Serialized read of cx[stream_idx] from M10K (1-cycle latency);
    // CAPT computes -(cx + q_buf[stream_idx]) and writes back to r_reg
    // (primary) + d_reg (secondary) at the single active lane.
    S_VNS_R_ADDR,
    S_VNS_R_CAPT,

    // init_rr_reg fires on vdot_out_hs so the initial rr lands in
    // rr_reg the same cycle vdot finishes.
    S_VDOT_INIT_FEED,

    S_SPMV_RUN_FIRE,
    S_SPMV_RUN_COLLECT,

    S_VDOT_DQ_FEED,

    S_DIV_A_SEND,
    S_DIV_A_RECV,

    // Merged x and r AXPY: x += alpha*d || r -= alpha*q (lockstep).
    S_AXPY_XR_FEED,

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
  logic                               sel_y_reg;
  logic [$clog2(p_max_n+1)-1:0]       stream_idx;
  logic [$clog2(p_max_n+1)-1:0]       out_idx;

  assign sel_y = sel_y_reg;

  //----------------------------------------------------------------------
  // Handshake wires. We use u_axpy_x's handshakes as canonical for FSM
  // and counter control: both AXPY units run lockstep (same n, p_lanes,
  // identical inputs, same handshake driver) so axpy_r_*_hs are
  // equivalent by construction.
  //----------------------------------------------------------------------
  wire vdot_in_hs    = vdot_istream_val   && vdot_istream_rdy;
  wire vdot_out_hs   = vdot_ostream_val   && vdot_ostream_rdy;
  wire axpy_x_in_hs  = axpy_x_istream_val && axpy_x_istream_rdy;
  wire axpy_x_out_hs = axpy_x_ostream_val && axpy_x_ostream_rdy;
  wire spmv_in_hs    = spmv_istream_val   && spmv_istream_rdy;
  wire spmv_out_hs   = spmv_ostream_val   && spmv_ostream_rdy;
  wire fpdiv_in_hs   = fpdiv_istream_val  && fpdiv_istream_rdy;
  wire fpdiv_out_hs  = fpdiv_ostream_val  && fpdiv_ostream_rdy;

  //----------------------------------------------------------------------
  // Registered narrow forms of n, n-1, num_groups, and num_groups-1.
  //----------------------------------------------------------------------
  logic [N_W-1:0] n_narrow;
  logic [N_W-1:0] num_groups_calc;
  assign n_narrow        = n[N_W-1:0];
  assign num_groups_calc = N_W'((n + unsigned'(p_lanes - 1)) / unsigned'(p_lanes));

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
  // eps_sq is the 32-bit ARM/PIO input (a small positive fixed-point
  // threshold); sign-extending the whole word to p_acc_bits is safe at
  // any p_total_bits in [2, 64] -- callers pre-clamp the value so the
  // upper bits are zero.
  assign eps_sq_wide = p_acc_bits'($signed(eps_sq));

  logic run_converged;
  assign run_converged = (iter >= max_iter)
                      || (rr_new <= eps_sq_wide)
                      || (iter > 32'd1 && rr_new >= rr_old);

  //----------------------------------------------------------------------
  // Per-lane index / valid helpers
  //----------------------------------------------------------------------
  logic [p_lanes*IDX_W-1:0] group_in_idx_packed;
  logic [p_lanes-1:0]       group_in_valid;
  logic [p_lanes*IDX_W-1:0] group_out_idx_packed;
  logic [p_lanes-1:0]       group_out_valid;
  logic [p_lanes*IDX_W-1:0] single_idx_packed;
  logic [p_lanes-1:0]       single_active_mask;
  logic [BANK_SEL_W-1:0]    single_active_lane;

  logic [N_W-1:0] stream_elem_base;
  logic [N_W-1:0] out_elem_base;
  logic [N_W-1:0] p_lanes_narrow;
  assign p_lanes_narrow = N_W'(unsigned'(p_lanes));
  assign stream_elem_base = stream_idx * p_lanes_narrow;
  assign out_elem_base    = out_idx    * p_lanes_narrow;

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

  assign single_active_lane = BANK_SEL_W'(stream_idx % p_lanes_narrow);

  always_comb begin
    single_idx_packed  = '0;
    single_active_mask = '0;
    for (int k = 0; k < p_lanes; k++) begin
      single_idx_packed[(k+1)*IDX_W-1 -: IDX_W] = stream_idx[IDX_W-1:0];
      if (BANK_SEL_W'(unsigned'(k)) == single_active_lane)
        single_active_mask[k] = 1'b1;
    end
  end

  // Active-lane extract of rd_a_data_packed for writing x_vec to SRAM in WB.
  logic signed [p_total_bits-1:0] rd_a_data_active;
  always_comb begin
    rd_a_data_active = '0;
    for (int k = 0; k < p_lanes; k++)
      if (BANK_SEL_W'(unsigned'(k)) == single_active_lane)
        rd_a_data_active =
          $signed(rd_a_data_packed[(k+1)*p_total_bits-1 -: p_total_bits]);
  end

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

      // LD x vector (1 elem / cycle), then jump straight to SPMV_INIT
      // -- cx is read directly from cx_ram during S_VNS_R, no separate
      // load phase.
      S_LD_X_ADDR:                                                            next_state = S_LD_X_CAPT;
      S_LD_X_CAPT:       if (stream_idx == n_minus_1_reg)                    next_state = S_SPMV_INIT_FIRE;
                         else                                                 next_state = S_LD_X_ADDR;

      // INIT SPMV (1 row / handshake)
      S_SPMV_INIT_FIRE:  if (spmv_in_hs)                                      next_state = S_SPMV_INIT_COLLECT;
      S_SPMV_INIT_COLLECT:
                         if (spmv_out_hs && stream_idx == n_minus_1_reg)     next_state = S_VNS_R_ADDR;

      // r_reg[i] = d_reg[i] = -(cx[i] + q_buf[i]). Serialized: 1 element
      // per (ADDR, CAPT) pair. cx comes from M10K.
      S_VNS_R_ADDR:                                                           next_state = S_VNS_R_CAPT;
      S_VNS_R_CAPT:      if (stream_idx == n_minus_1_reg)                    next_state = S_VDOT_INIT_FEED;
                         else                                                 next_state = S_VNS_R_ADDR;

      // VDOT r.r -> rr_new_latched, and (via init_rr_reg) -> rr_reg
      S_VDOT_INIT_FEED:  if (vdot_out_hs)                                     next_state = S_SPMV_RUN_FIRE;

      // RUN: SPMV (Q * d_reg) -> q_buf (1 row / handshake)
      S_SPMV_RUN_FIRE:   if (spmv_in_hs)                                      next_state = S_SPMV_RUN_COLLECT;
      S_SPMV_RUN_COLLECT:
                         if (spmv_out_hs && stream_idx == n_minus_1_reg)     next_state = S_VDOT_DQ_FEED;

      // dq = d . q_buf
      S_VDOT_DQ_FEED:    if (vdot_out_hs)                                     next_state = S_DIV_A_SEND;

      // alpha = rr_reg / dq
      S_DIV_A_SEND:      if (fpdiv_in_hs)                                     next_state = S_DIV_A_RECV;
      S_DIV_A_RECV:      if (fpdiv_out_hs)                                    next_state = S_AXPY_XR_FEED;

      // Parallel: x_vec[i] += alpha*d[i] || r[i] -= alpha*q[i]
      S_AXPY_XR_FEED:    if (axpy_x_out_hs && out_idx == num_groups_minus_1_reg)  next_state = S_VDOT_RR_FEED;

      // rr_new = r_new . r_new
      S_VDOT_RR_FEED:    if (vdot_out_hs)                                     next_state = S_DIV_B_SEND;

      // beta = rr_new / rr_reg
      S_DIV_B_SEND:      if (fpdiv_in_hs)                                     next_state = S_DIV_B_RECV;
      S_DIV_B_RECV:      if (fpdiv_out_hs)                                    next_state = S_AXPY_D_FEED;

      // d_reg[i] = r_reg[i] + beta * d_reg[i]
      S_AXPY_D_FEED:     if (axpy_x_out_hs && out_idx == num_groups_minus_1_reg)  next_state = S_RUN_CHECK;

      S_RUN_CHECK:       if (run_converged)                                   next_state = S_WB_WRITE;
                         else                                                 next_state = S_SPMV_RUN_FIRE;

      S_WB_WRITE: begin
        if (stream_idx == n_minus_1_reg) begin
          if (sel_y_reg) next_state = S_CG_DONE;
          else           next_state = S_LD_X_ADDR;
        end
      end

      S_CG_DONE:         if (sw_done_ack)                                     next_state = S_IDLE;
      default:                                                                next_state = S_IDLE;
    endcase
  end

  //----------------------------------------------------------------------
  // sel_y, stream_idx, out_idx updates
  //----------------------------------------------------------------------
  function automatic logic [4:0] phase_of(state_t s);
    case (s)
      S_LD_X_ADDR,         S_LD_X_CAPT:          phase_of = 5'd1;
      S_SPMV_INIT_FIRE,    S_SPMV_INIT_COLLECT:  phase_of = 5'd3;
      S_VNS_R_ADDR,        S_VNS_R_CAPT:         phase_of = 5'd4;
      S_VDOT_INIT_FEED:                          phase_of = 5'd6;
      S_SPMV_RUN_FIRE,     S_SPMV_RUN_COLLECT:   phase_of = 5'd8;
      S_VDOT_DQ_FEED:                            phase_of = 5'd9;
      S_DIV_A_SEND,        S_DIV_A_RECV:         phase_of = 5'd10;
      S_AXPY_XR_FEED:                            phase_of = 5'd11;
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
      sel_y_reg  <= 1'b0;
      stream_idx <= '0;
      out_idx    <= '0;
    end else begin
      if (state == S_PREP)
        sel_y_reg <= 1'b0;
      else if (state == S_WB_WRITE && stream_idx == n_minus_1_reg && !sel_y_reg)
        sel_y_reg <= 1'b1;

      if (reset_counters)
        stream_idx <= '0;
      else begin
        case (state)
          S_LD_X_CAPT, S_WB_WRITE:
            stream_idx <= stream_idx + 1;
          S_SPMV_INIT_COLLECT, S_SPMV_RUN_COLLECT:
            if (spmv_out_hs) stream_idx <= stream_idx + 1;
          S_VNS_R_CAPT:
            stream_idx <= stream_idx + 1;
          S_VDOT_INIT_FEED, S_VDOT_DQ_FEED, S_VDOT_RR_FEED:
            if (vdot_in_hs) stream_idx <= stream_idx + 1;
          S_AXPY_XR_FEED, S_AXPY_D_FEED:
            if (axpy_x_in_hs) stream_idx <= stream_idx + 1;
          default: ;
        endcase
      end

      if (reset_counters)
        out_idx <= '0;
      else if (axpy_x_out_hs &&
               (state == S_AXPY_XR_FEED || state == S_AXPY_D_FEED))
        out_idx <= out_idx + 1;
    end
  end

  //----------------------------------------------------------------------
  // Output defaults + per-state drive
  //----------------------------------------------------------------------
  always_comb begin
    sw_done = 1'b0;

    ctrl_xy_addr      = '0;
    ctrl_xy_wr_en     = 1'b0;
    ctrl_xy_wdata     = '0;

    vns_cx_addr       = '0;
    vns_cx_rd_en      = 1'b0;

    rd_a_sel            = '0;
    rd_a_idx_packed     = '0;
    rd_a_valid          = '0;
    rd_b_sel            = '0;
    rd_b_idx_packed     = '0;
    rd_b_valid          = '0;
    rd_c_sel            = '0;
    rd_c_idx_packed     = '0;
    rd_c_valid          = '0;
    rd_d_sel            = '0;
    rd_d_idx_packed     = '0;
    rd_d_valid          = '0;
    rd_vec_sel          = RF_D_REG;

    wr_sel              = '0;
    wr_idx_ctrl_packed  = '0;
    wr_idx_src_spmv     = 1'b0;
    we                  = '0;
    wdata_src           = '0;
    wr_sel_sec          = '0;
    wdata_src_sec       = '0;
    we_sec              = '0;

    latch_dq          = 1'b0;
    latch_rr_new      = 1'b0;
    latch_alpha       = 1'b0;
    latch_beta        = 1'b0;
    refresh_rr_reg    = 1'b0;
    init_rr_reg       = 1'b0;
    bump_iter         = 1'b0;
    reset_iter        = 1'b0;

    vdot_istream_val      = 1'b0;
    vdot_ostream_rdy      = 1'b0;
    axpy_x_istream_val    = 1'b0;
    axpy_x_ostream_rdy    = 1'b0;
    axpy_r_istream_val    = 1'b0;
    axpy_r_ostream_rdy    = 1'b0;
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

      S_LD_X_ADDR: begin
        ctrl_xy_addr = p_m10k_addr_bits'(stream_idx);
      end
      S_LD_X_CAPT: begin
        ctrl_xy_addr       = p_m10k_addr_bits'(stream_idx);
        wr_sel             = RF_X_VEC_REG;
        wr_idx_ctrl_packed = single_idx_packed;
        we                 = single_active_mask;
        wdata_src          = WD_MEM;
      end

      S_SPMV_INIT_FIRE: begin
        spmv_istream_val  = 1'b1;
        rd_vec_sel        = RF_X_VEC_REG;
      end
      S_SPMV_INIT_COLLECT: begin
        spmv_ostream_rdy  = 1'b1;
        rd_vec_sel        = RF_X_VEC_REG;
        wr_sel            = RF_Q_BUF;
        wr_idx_src_spmv   = 1'b1;
        wdata_src         = WD_SPMV;
        we[0]             = spmv_out_hs;
      end

      // Serialized VNS_R: ADDR drives cx_addr; CAPT latches the writeback.
      S_VNS_R_ADDR: begin
        vns_cx_addr  = p_m10k_addr_bits'(stream_idx);
        vns_cx_rd_en = 1'b1;
      end
      S_VNS_R_CAPT: begin
        vns_cx_addr        = p_m10k_addr_bits'(stream_idx);
        vns_cx_rd_en       = 1'b1;
        // q_buf read at stream_idx via single_active_lane.
        rd_b_sel           = RF_Q_BUF;
        rd_b_idx_packed    = single_idx_packed;
        rd_b_valid         = single_active_mask;
        // Primary writes r_reg, secondary writes d_reg, both using the
        // WD_VNS_SCALAR mux (-(vns_cx_rdata + rd_b)).
        wr_sel             = RF_R_REG;
        wr_idx_ctrl_packed = single_idx_packed;
        we                 = single_active_mask;
        wdata_src          = WD_VNS_SCALAR;
        wr_sel_sec         = RF_D_REG;
        wdata_src_sec      = WD_VNS_SCALAR;
        we_sec             = single_active_mask;
      end

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
        init_rr_reg      = vdot_out_hs;
      end

      S_SPMV_RUN_FIRE: begin
        spmv_istream_val  = 1'b1;
        rd_vec_sel        = RF_D_REG;
      end
      S_SPMV_RUN_COLLECT: begin
        spmv_ostream_rdy  = 1'b1;
        rd_vec_sel        = RF_D_REG;
        wr_sel            = RF_Q_BUF;
        wr_idx_src_spmv   = 1'b1;
        wdata_src         = WD_SPMV;
        we[0]             = spmv_out_hs;
      end

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

      S_DIV_A_SEND: begin
        fpdiv_istream_val = 1'b1;
        fpdiv_a_src_rrnew = 1'b0;
        fpdiv_b_src_rr    = 1'b0;
      end
      S_DIV_A_RECV: begin
        fpdiv_ostream_rdy = 1'b1;
        latch_alpha       = fpdiv_out_hs;
      end

      S_AXPY_XR_FEED: begin
        rd_a_sel           = RF_X_VEC_REG;
        rd_a_idx_packed    = group_in_idx_packed;
        rd_a_valid         = group_in_valid;
        rd_b_sel           = RF_D_REG;
        rd_b_idx_packed    = group_in_idx_packed;
        rd_b_valid         = group_in_valid;
        rd_c_sel           = RF_R_REG;
        rd_c_idx_packed    = group_in_idx_packed;
        rd_c_valid         = group_in_valid;
        rd_d_sel           = RF_Q_BUF;
        rd_d_idx_packed    = group_in_idx_packed;
        rd_d_valid         = group_in_valid;
        axpy_x_istream_val = (stream_idx < num_groups_reg);
        axpy_x_ostream_rdy = 1'b1;
        axpy_r_istream_val = (stream_idx < num_groups_reg);
        axpy_r_ostream_rdy = 1'b1;
        axpy_coef_src_beta = 1'b0;
        wr_sel             = RF_X_VEC_REG;
        wr_idx_ctrl_packed = group_out_idx_packed;
        we                 = axpy_x_out_hs ? group_out_valid : '0;
        wdata_src          = WD_AXPY;
        wr_sel_sec         = RF_R_REG;
        wdata_src_sec      = WD_AXPY_R;
        we_sec             = axpy_x_out_hs ? group_out_valid : '0;
      end

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

      S_DIV_B_SEND: begin
        fpdiv_istream_val = 1'b1;
        fpdiv_a_src_rrnew = 1'b1;
        fpdiv_b_src_rr    = 1'b1;
      end
      S_DIV_B_RECV: begin
        fpdiv_ostream_rdy = 1'b1;
        latch_beta        = fpdiv_out_hs;
      end

      S_AXPY_D_FEED: begin
        rd_a_sel           = RF_R_REG;
        rd_a_idx_packed    = group_in_idx_packed;
        rd_a_valid         = group_in_valid;
        rd_b_sel           = RF_D_REG;
        rd_b_idx_packed    = group_in_idx_packed;
        rd_b_valid         = group_in_valid;
        axpy_x_istream_val = (stream_idx < num_groups_reg);
        axpy_x_ostream_rdy = 1'b1;
        axpy_coef_src_beta = 1'b1;
        wr_sel             = RF_D_REG;
        wr_idx_ctrl_packed = group_out_idx_packed;
        we                 = axpy_x_out_hs ? group_out_valid : '0;
        wdata_src          = WD_AXPY;
      end

      S_RUN_CHECK: begin
        bump_iter      = 1'b1;
        refresh_rr_reg = 1'b1;
      end

      S_WB_WRITE: begin
        ctrl_xy_addr    = p_m10k_addr_bits'(stream_idx);
        ctrl_xy_wr_en   = 1'b1;
        rd_a_sel        = RF_X_VEC_REG;
        rd_a_idx_packed = single_idx_packed;
        rd_a_valid      = single_active_mask;
        ctrl_xy_wdata   = p_word_bits'($signed(rd_a_data_active));
        if (stream_idx == n_minus_1_reg && !sel_y_reg)
          reset_iter = 1'b1;
      end

      S_CG_DONE: sw_done = 1'b1;

      default: ;
    endcase
  end

endmodule
