// v4 datapath -- two AXPY units run in lockstep during the merged
// S_AXPY_XR_FEED state. u_axpy_x (ADD) reads rd_a/rd_b and writes the
// primary port; u_axpy_r (SUB) reads rd_c/rd_d and writes the
// secondary port. The secondary port is also reused by S_VNS_R for
// the d_reg writeback, paired with the primary r_reg writeback.

module CGDpath #(
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
  parameter p_q_val_base_addr  = 0,
  parameter p_q_col_base_addr  = p_max_n * p_max_n,
  parameter p_q_rowp_base_addr = 2 * p_max_n * p_max_n
) (
  input  logic clk,
  input  logic rst,

  input  logic [31:0] n,

  output logic [p_m10k_addr_bits-1:0] mem_addr,
  output logic                        mem_wr_en,
  output logic [p_word_bits-1:0]      mem_wdata,
  input  logic [p_word_bits-1:0]      mem_rdata,

  input  logic [p_m10k_addr_bits-1:0] ctrl_mem_addr,
  input  logic                        ctrl_mem_wr_en,
  input  logic [p_word_bits-1:0]      ctrl_mem_wdata,
  input  logic                        ctrl_mem_src_spmv,

  // --- Addressed RF read ports (combinational) ---------------------------
  // 5 RFs: 0=d_reg, 1=r_reg, 2=x_vec_reg, 3=cx_reg, 4=q_buf
  // rd_a/rd_b are general-purpose; rd_c/rd_d feed u_axpy_r in S_AXPY_XR_FEED.
  input  logic [2:0]                              rd_a_sel,
  input  logic [p_lanes*$clog2(p_max_n)-1:0]      rd_a_idx_packed,
  input  logic [p_lanes-1:0]                      rd_a_valid,
  output logic [p_lanes*p_total_bits-1:0]         rd_a_data_packed,

  input  logic [2:0]                              rd_b_sel,
  input  logic [p_lanes*$clog2(p_max_n)-1:0]      rd_b_idx_packed,
  input  logic [p_lanes-1:0]                      rd_b_valid,
  output logic [p_lanes*p_total_bits-1:0]         rd_b_data_packed,

  input  logic [2:0]                              rd_c_sel,
  input  logic [p_lanes*$clog2(p_max_n)-1:0]      rd_c_idx_packed,
  input  logic [p_lanes-1:0]                      rd_c_valid,
  output logic [p_lanes*p_total_bits-1:0]         rd_c_data_packed,

  input  logic [2:0]                              rd_d_sel,
  input  logic [p_lanes*$clog2(p_max_n)-1:0]      rd_d_idx_packed,
  input  logic [p_lanes-1:0]                      rd_d_valid,
  output logic [p_lanes*p_total_bits-1:0]         rd_d_data_packed,

  input  logic [2:0]                              rd_vec_sel,

  // --- Primary RF write port (p_lanes-wide) ------------------------------
  input  logic [2:0]                              wr_sel,
  input  logic [p_lanes*$clog2(p_max_n)-1:0]      wr_idx_ctrl_packed,
  input  logic                                    wr_idx_src_spmv,
  input  logic [p_lanes-1:0]                      we,
  input  logic [2:0]                              wdata_src,

  // --- Secondary RF write port (p_lanes-wide). Index reuses primary -------
  // wr_idx_ctrl_packed (callers always retire the same group on both
  // ports). Data source is selected by wdata_src_sec:
  //   WD_AXPY_R: axpy_r_z_lane (S_AXPY_XR_FEED)
  //   WD_VNS:    -(rd_a + rd_b) (S_VNS_R, fused with primary's r_reg write)
  input  logic [2:0]                              wr_sel_sec,
  input  logic [2:0]                              wdata_src_sec,
  input  logic [p_lanes-1:0]                      we_sec,

  // --- Scalar latch enables ----------------------------------------------
  input  logic                             latch_dq,
  input  logic                             latch_rr_new,
  input  logic                             latch_alpha,
  input  logic                             latch_beta,
  input  logic                             refresh_rr_reg,
  input  logic                             init_rr_reg,
  input  logic                             bump_iter,
  input  logic                             reset_iter,

  // --- Submodule handshake plumbing --------------------------------------
  input  logic                             vdot_istream_val,
  output logic                             vdot_istream_rdy,
  output logic                             vdot_ostream_val,
  input  logic                             vdot_ostream_rdy,

  // AXPY x: mode hard-wired to ADD; coef shared with u_axpy_r.
  input  logic                             axpy_x_istream_val,
  output logic                             axpy_x_istream_rdy,
  output logic                             axpy_x_ostream_val,
  input  logic                             axpy_x_ostream_rdy,

  // AXPY r: mode hard-wired to SUB; same coef as u_axpy_x.
  input  logic                             axpy_r_istream_val,
  output logic                             axpy_r_istream_rdy,
  output logic                             axpy_r_ostream_val,
  input  logic                             axpy_r_ostream_rdy,

  input  logic                             axpy_coef_src_beta, // 0=alpha, 1=beta

  input  logic                             spmv_istream_val,
  output logic                             spmv_istream_rdy,
  output logic                             spmv_ostream_val,
  input  logic                             spmv_ostream_rdy,

  input  logic                             fpdiv_istream_val,
  output logic                             fpdiv_istream_rdy,
  output logic                             fpdiv_ostream_val,
  input  logic                             fpdiv_ostream_rdy,
  input  logic                             fpdiv_a_src_rrnew,
  input  logic                             fpdiv_b_src_rr,

  output logic [31:0]                      iter,
  output logic signed [p_acc_bits-1:0]     rr_new,
  output logic signed [p_acc_bits-1:0]     rr_old
);

  localparam IDX_W       = $clog2(p_max_n);
  localparam CLOG2_LANES = $clog2(p_lanes);
  // BANK_DEPTH = ceil(p_max_n / p_lanes). Each RF is split into p_lanes
  // banks of BANK_DEPTH entries; element idx lives in bank (idx mod p_lanes)
  // at bank-internal address (idx / p_lanes). p_lanes may be any positive
  // integer; div/mod synthesize as constant divides since p_lanes is a
  // compile-time parameter.
  localparam BANK_DEPTH  = (p_max_n + p_lanes - 1) / p_lanes;
  localparam BANK_ADDR_W = $clog2(BANK_DEPTH);
  // BANK_SEL_W = max(CLOG2_LANES, 1) so we can declare bank-id signals
  // without [-1:0] slices when p_lanes==1. CLOG2_LANES is wide enough to
  // hold a value in [0, p_lanes-1] for any p_lanes >= 2.
  localparam BANK_SEL_W  = (CLOG2_LANES == 0) ? 1 : CLOG2_LANES;

  //----------------------------------------------------------------------
  // RF select encodings (must match CGCtrl.v)
  //----------------------------------------------------------------------
  localparam [2:0] RF_D_REG     = 3'd0;
  localparam [2:0] RF_R_REG     = 3'd1;
  localparam [2:0] RF_X_VEC_REG = 3'd2;
  localparam [2:0] RF_CX_REG    = 3'd3;
  localparam [2:0] RF_Q_BUF     = 3'd4;

  localparam [2:0] WD_MEM    = 3'd0;
  localparam [2:0] WD_AXPY   = 3'd1;
  localparam [2:0] WD_SPMV   = 3'd2;
  localparam [2:0] WD_VNS    = 3'd4;
  localparam [2:0] WD_AXPY_R = 3'd5;

  //----------------------------------------------------------------------
  // Register files -- banked by lane id. Bank b holds elements with
  // (idx mod p_lanes) == b. Lane k always reads/writes bank k, so each
  // bank's flops only fan out to lane k's per-alias muxes (4 reads) plus
  // the SPMV vec-read crossbar.
  //----------------------------------------------------------------------

  logic signed [p_total_bits-1:0] d_reg     [p_lanes][BANK_DEPTH];
  logic signed [p_total_bits-1:0] r_reg     [p_lanes][BANK_DEPTH];
  logic signed [p_total_bits-1:0] x_vec_reg [p_lanes][BANK_DEPTH];
  logic signed [p_total_bits-1:0] cx_reg    [p_lanes][BANK_DEPTH];
  logic signed [p_total_bits-1:0] q_buf     [p_lanes][BANK_DEPTH];

  //----------------------------------------------------------------------
  // Scalars
  //----------------------------------------------------------------------

  logic signed [p_acc_bits-1:0]   rr_reg;
  logic signed [p_acc_bits-1:0]   rr_new_latched;
  logic signed [p_acc_bits-1:0]   dq_latched;
  logic signed [p_total_bits-1:0] alpha;
  logic signed [p_total_bits-1:0] beta;

  assign rr_new = rr_new_latched;
  assign rr_old = rr_reg;

  //----------------------------------------------------------------------
  // RF read muxes (per-lane, gated by valid mask).
  //
  // Banking contract: lane k always reads bank k. CGCtrl drives idx[k]
  // such that (idx[k] mod p_lanes) == k whenever rd_*_valid[k] is high
  // (or the read result is masked to 0). Bank-internal address is
  // (idx[k] / p_lanes). Because the bank index is the constant genvar gi,
  // each bank's flops only fan out to lane gi's read muxes.
  //----------------------------------------------------------------------

  function automatic logic signed [p_total_bits-1:0] rf_read_bank(
    input logic [2:0]                sel,
    input integer                    bank,
    input logic [BANK_ADDR_W-1:0]    bank_addr
  );
    case (sel)
      RF_D_REG:     rf_read_bank = d_reg     [bank][bank_addr];
      RF_R_REG:     rf_read_bank = r_reg     [bank][bank_addr];
      RF_X_VEC_REG: rf_read_bank = x_vec_reg [bank][bank_addr];
      RF_CX_REG:    rf_read_bank = cx_reg    [bank][bank_addr];
      RF_Q_BUF:     rf_read_bank = q_buf    [bank][bank_addr];
      default:      rf_read_bank = '0;
    endcase
  endfunction

  logic [IDX_W-1:0]               rd_a_idx       [p_lanes];
  logic [IDX_W-1:0]               rd_b_idx       [p_lanes];
  logic [IDX_W-1:0]               rd_c_idx       [p_lanes];
  logic [IDX_W-1:0]               rd_d_idx       [p_lanes];
  logic [BANK_ADDR_W-1:0]         rd_a_bank_addr [p_lanes];
  logic [BANK_ADDR_W-1:0]         rd_b_bank_addr [p_lanes];
  logic [BANK_ADDR_W-1:0]         rd_c_bank_addr [p_lanes];
  logic [BANK_ADDR_W-1:0]         rd_d_bank_addr [p_lanes];
  logic signed [p_total_bits-1:0] rd_a_data      [p_lanes];
  logic signed [p_total_bits-1:0] rd_b_data      [p_lanes];
  logic signed [p_total_bits-1:0] rd_c_data      [p_lanes];
  logic signed [p_total_bits-1:0] rd_d_data      [p_lanes];

  genvar gi;
  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_rd
      assign rd_a_idx[gi] = rd_a_idx_packed[(gi+1)*IDX_W-1 -: IDX_W];
      assign rd_b_idx[gi] = rd_b_idx_packed[(gi+1)*IDX_W-1 -: IDX_W];
      assign rd_c_idx[gi] = rd_c_idx_packed[(gi+1)*IDX_W-1 -: IDX_W];
      assign rd_d_idx[gi] = rd_d_idx_packed[(gi+1)*IDX_W-1 -: IDX_W];

      assign rd_a_bank_addr[gi] = BANK_ADDR_W'(rd_a_idx[gi] / IDX_W'(unsigned'(p_lanes)));
      assign rd_b_bank_addr[gi] = BANK_ADDR_W'(rd_b_idx[gi] / IDX_W'(unsigned'(p_lanes)));
      assign rd_c_bank_addr[gi] = BANK_ADDR_W'(rd_c_idx[gi] / IDX_W'(unsigned'(p_lanes)));
      assign rd_d_bank_addr[gi] = BANK_ADDR_W'(rd_d_idx[gi] / IDX_W'(unsigned'(p_lanes)));

      assign rd_a_data[gi] = rd_a_valid[gi]
                             ? rf_read_bank(rd_a_sel, gi, rd_a_bank_addr[gi])
                             : p_total_bits'(0);
      assign rd_b_data[gi] = rd_b_valid[gi]
                             ? rf_read_bank(rd_b_sel, gi, rd_b_bank_addr[gi])
                             : p_total_bits'(0);
      assign rd_c_data[gi] = rd_c_valid[gi]
                             ? rf_read_bank(rd_c_sel, gi, rd_c_bank_addr[gi])
                             : p_total_bits'(0);
      assign rd_d_data[gi] = rd_d_valid[gi]
                             ? rf_read_bank(rd_d_sel, gi, rd_d_bank_addr[gi])
                             : p_total_bits'(0);
      assign rd_a_data_packed[(gi+1)*p_total_bits-1 -: p_total_bits] =
        $unsigned(rd_a_data[gi]);
      assign rd_b_data_packed[(gi+1)*p_total_bits-1 -: p_total_bits] =
        $unsigned(rd_b_data[gi]);
      assign rd_c_data_packed[(gi+1)*p_total_bits-1 -: p_total_bits] =
        $unsigned(rd_c_data[gi]);
      assign rd_d_data_packed[(gi+1)*p_total_bits-1 -: p_total_bits] =
        $unsigned(rd_d_data[gi]);
    end
  endgenerate

  //----------------------------------------------------------------------
  // Linalg submodule instances
  //----------------------------------------------------------------------

  // --- VecDot -----------------------------------------------------------
  logic signed [p_acc_bits-1:0] vdot_result;

  VecDot_seq #(
    .p_lanes     (p_lanes),
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits),
    .p_acc_bits  (p_acc_bits)
  ) u_vdot (
    .clk, .rst,
    .n,
    .istream_val        (vdot_istream_val),
    .istream_rdy        (vdot_istream_rdy),
    .istream_msg_a      (rd_a_data_packed),
    .istream_msg_b      (rd_b_data_packed),
    .ostream_val        (vdot_ostream_val),
    .ostream_rdy        (vdot_ostream_rdy),
    .ostream_msg_result (vdot_result)
  );

  // --- AXPY x (ADD) -----------------------------------------------------
  logic signed [p_total_bits-1:0]  axpy_coef;
  logic [p_lanes*p_total_bits-1:0] axpy_x_z_packed;
  logic signed [p_total_bits-1:0]  axpy_x_z_lane [p_lanes];

  assign axpy_coef = axpy_coef_src_beta ? beta : alpha;

  AXPY_seq #(
    .p_lanes     (p_lanes),
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits)
  ) u_axpy_x (
    .clk, .rst,
    .n,
    .mode          (1'b0),               // hard-wired ADD (used by x and d updates)
    .coef          (axpy_coef),
    .istream_val   (axpy_x_istream_val),
    .istream_rdy   (axpy_x_istream_rdy),
    .istream_msg_a (rd_a_data_packed),
    .istream_msg_b (rd_b_data_packed),
    .ostream_val   (axpy_x_ostream_val),
    .ostream_rdy   (axpy_x_ostream_rdy),
    .ostream_msg_z (axpy_x_z_packed)
  );

  // --- AXPY r (SUB) -----------------------------------------------------
  logic [p_lanes*p_total_bits-1:0] axpy_r_z_packed;
  logic signed [p_total_bits-1:0]  axpy_r_z_lane [p_lanes];

  AXPY_seq #(
    .p_lanes     (p_lanes),
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits)
  ) u_axpy_r (
    .clk, .rst,
    .n,
    .mode          (1'b1),               // hard-wired SUB (only used in S_AXPY_XR_FEED)
    .coef          (axpy_coef),
    .istream_val   (axpy_r_istream_val),
    .istream_rdy   (axpy_r_istream_rdy),
    .istream_msg_a (rd_c_data_packed),
    .istream_msg_b (rd_d_data_packed),
    .ostream_val   (axpy_r_ostream_val),
    .ostream_rdy   (axpy_r_ostream_rdy),
    .ostream_msg_z (axpy_r_z_packed)
  );

  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_axpy_unpack
      assign axpy_x_z_lane[gi] =
        $signed(axpy_x_z_packed[(gi+1)*p_total_bits-1 -: p_total_bits]);
      assign axpy_r_z_lane[gi] =
        $signed(axpy_r_z_packed[(gi+1)*p_total_bits-1 -: p_total_bits]);
    end
  endgenerate

  // --- SPMV -------------------------------------------------------------
  logic [p_m10k_addr_bits-1:0]    spmv_mem_addr;
  logic                            spmv_mem_rd_en;
  logic [$clog2(p_max_n)-1:0]      spmv_vec_rd_idx;
  logic signed [p_total_bits-1:0]  spmv_vec_rd_data;
  logic [31:0]                     spmv_row_idx;
  logic signed [p_total_bits-1:0]  spmv_row_val;

  // SPMV vec read is the only arbitrary-index read. We add a small bank
  // crossbar: fan out each bank's read at spmv_vec_bank_addr, then mux
  // by spmv_vec_bank. Costs one extra read alias per bank.
  logic [BANK_SEL_W-1:0]           spmv_vec_bank;
  logic [BANK_ADDR_W-1:0]          spmv_vec_bank_addr;
  logic signed [p_total_bits-1:0]  spmv_vec_per_bank [p_lanes];

  assign spmv_vec_bank      = BANK_SEL_W'(spmv_vec_rd_idx % IDX_W'(unsigned'(p_lanes)));
  assign spmv_vec_bank_addr = BANK_ADDR_W'(spmv_vec_rd_idx / IDX_W'(unsigned'(p_lanes)));

  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_spmv_vec_rd
      assign spmv_vec_per_bank[gi] =
        rf_read_bank(rd_vec_sel, gi, spmv_vec_bank_addr);
    end
  endgenerate

  assign spmv_vec_rd_data = spmv_vec_per_bank[spmv_vec_bank];

  SPMV_seq #(
    .p_int_bits        (p_int_bits),
    .p_frac_bits       (p_frac_bits),
    .p_total_bits      (p_total_bits),
    .p_acc_bits        (p_acc_bits),
    .p_max_n           (p_max_n),
    .p_m10k_addr_bits  (p_m10k_addr_bits),
    .p_word_bits       (p_word_bits)
  ) u_spmv (
    .clk, .rst,
    .istream_val        (spmv_istream_val),
    .istream_rdy        (spmv_istream_rdy),
    .ostream_val        (spmv_ostream_val),
    .ostream_rdy        (spmv_ostream_rdy),
    .ostream_msg_row_idx(spmv_row_idx),
    .ostream_msg_row_val(spmv_row_val),
    .n,
    .q_val_base         (p_m10k_addr_bits'(p_q_val_base_addr)),
    .q_col_base         (p_m10k_addr_bits'(p_q_col_base_addr)),
    .q_rowp_base        (p_m10k_addr_bits'(p_q_rowp_base_addr)),
    .mem_addr           (spmv_mem_addr),
    .mem_rd_en          (spmv_mem_rd_en),
    .mem_rdata          (mem_rdata),
    .vec_rd_idx         (spmv_vec_rd_idx),
    .vec_rd_data        (spmv_vec_rd_data)
  );

  // --- FpDiv ------------------------------------------------------------
  logic signed [p_acc_bits-1:0]   fpdiv_a;
  logic signed [p_acc_bits-1:0]   fpdiv_b;
  logic signed [p_total_bits-1:0] fpdiv_result;

  assign fpdiv_a = fpdiv_a_src_rrnew ? rr_new_latched : rr_reg;
  assign fpdiv_b = fpdiv_b_src_rr    ? rr_reg         : dq_latched;

  FpDiv #(
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits),
    .p_wide_bits (p_acc_bits)
  ) u_fpdiv (
    .clk, .rst,
    .istream_val        (fpdiv_istream_val),
    .istream_rdy        (fpdiv_istream_rdy),
    .istream_msg_a      (fpdiv_a),
    .istream_msg_b      (fpdiv_b),
    .ostream_val        (fpdiv_ostream_val),
    .ostream_rdy        (fpdiv_ostream_rdy),
    .ostream_msg_result (fpdiv_result)
  );

  //----------------------------------------------------------------------
  // Memory bus ownership mux
  //----------------------------------------------------------------------

  assign mem_addr  = ctrl_mem_src_spmv ? spmv_mem_addr  : ctrl_mem_addr;
  assign mem_wr_en = ctrl_mem_src_spmv ? 1'b0           : ctrl_mem_wr_en;
  assign mem_wdata = ctrl_mem_src_spmv ? '0             : ctrl_mem_wdata;

  //----------------------------------------------------------------------
  // RF write port (per-lane). With banking, lane k targets bank k:
  //   primary:   wdata_src   -> wr_data     -> wr_sel    (we_eff)
  //   secondary: wdata_src_sec -> wr_data_sec -> wr_sel_sec (we_sec)
  //
  // Banking contract: when we[k] is high, CGCtrl drives wr_idx_ctrl[k]
  // such that (wr_idx_ctrl[k] mod p_lanes) == k. SIMD writes naturally
  // satisfy this (lane k targets group_idx*p_lanes + k); CGCtrl's
  // single-lane writes drive the active lane (= linear_idx mod p_lanes).
  //
  // SPMV writeback is the one path where the target bank is determined
  // by data the FSM doesn't see (spmv_row_idx). When wr_idx_src_spmv is
  // high, CGDpath redirects the lane-0 write enable from CGCtrl onto
  // bank (spmv_row_idx mod p_lanes) instead. WD_MEM and WD_SPMV
  // replicate their scalar value across every wr_data[k] so whichever
  // lane has we_eff=1 picks it up.
  //----------------------------------------------------------------------

  logic [IDX_W-1:0]                wr_idx_ctrl    [p_lanes];
  logic [BANK_ADDR_W-1:0]          wr_bank_addr   [p_lanes];
  logic [p_lanes-1:0]              we_eff;
  logic signed [p_total_bits-1:0]  wr_data        [p_lanes];
  logic signed [p_total_bits-1:0]  wr_data_sec    [p_lanes];

  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_wr_idx
      assign wr_idx_ctrl[gi] = wr_idx_ctrl_packed[(gi+1)*IDX_W-1 -: IDX_W];
    end
  endgenerate

  // SPMV-writeback bank/addr. spmv_row_idx is computed inside SPMV_seq;
  // CGCtrl signals "do the write this cycle" via we[0].
  logic [BANK_SEL_W-1:0]   spmv_wb_bank;
  logic [BANK_ADDR_W-1:0]  spmv_wb_addr;
  assign spmv_wb_bank = BANK_SEL_W'(spmv_row_idx % unsigned'(p_lanes));
  assign spmv_wb_addr = BANK_ADDR_W'(spmv_row_idx / unsigned'(p_lanes));

  always_comb begin
    if (wr_idx_src_spmv) begin
      // Route the SPMV result to the bank that owns spmv_row_idx. Other
      // banks idle this cycle.
      for (int k = 0; k < p_lanes; k++) begin
        if (BANK_SEL_W'(unsigned'(k)) == spmv_wb_bank) begin
          we_eff[k]       = we[0];
          wr_bank_addr[k] = spmv_wb_addr;
        end else begin
          we_eff[k]       = 1'b0;
          wr_bank_addr[k] = '0;
        end
      end
    end else begin
      // SIMD or CGCtrl-scalar path: lane k's wr_idx_ctrl[k] satisfies
      // (wr_idx_ctrl[k] mod p_lanes) == k by contract; bank-internal
      // address is wr_idx_ctrl[k] / p_lanes.
      for (int k = 0; k < p_lanes; k++) begin
        wr_bank_addr[k] = BANK_ADDR_W'(wr_idx_ctrl[k] / IDX_W'(unsigned'(p_lanes)));
        we_eff[k]       = we[k];
      end
    end
  end

  // Secondary write only fires for SIMD (S_AXPY_XR_FEED, S_VNS_R), so we
  // can use wr_idx_ctrl directly without the SPMV override.
  logic [BANK_ADDR_W-1:0] wr_bank_addr_sec [p_lanes];
  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_wr_addr_sec
      assign wr_bank_addr_sec[gi] = BANK_ADDR_W'(wr_idx_ctrl[gi] / IDX_W'(unsigned'(p_lanes)));
    end
  endgenerate

  always_comb begin
    for (int k = 0; k < p_lanes; k++) wr_data[k] = '0;
    case (wdata_src)
      WD_MEM: begin
        // Replicate so the SPMV-redirected or single-lane-active bank
        // can latch it via we_eff.
        for (int k = 0; k < p_lanes; k++)
          wr_data[k] = p_total_bits'($signed(mem_rdata));
      end
      WD_AXPY: begin
        for (int k = 0; k < p_lanes; k++) wr_data[k] = axpy_x_z_lane[k];
      end
      WD_SPMV: begin
        for (int k = 0; k < p_lanes; k++) wr_data[k] = spmv_row_val;
      end
      WD_VNS: begin
        for (int k = 0; k < p_lanes; k++)
          wr_data[k] = -(rd_a_data[k] + rd_b_data[k]);
      end
      default: ;
    endcase
  end

  // Secondary write data mux. Only the codes the secondary path
  // actually consumes are decoded; everything else stays at zero.
  always_comb begin
    for (int k = 0; k < p_lanes; k++) wr_data_sec[k] = '0;
    case (wdata_src_sec)
      WD_AXPY_R: begin
        for (int k = 0; k < p_lanes; k++) wr_data_sec[k] = axpy_r_z_lane[k];
      end
      WD_VNS: begin
        for (int k = 0; k < p_lanes; k++)
          wr_data_sec[k] = -(rd_a_data[k] + rd_b_data[k]);
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      for (int b = 0; b < p_lanes; b++) begin
        for (int i = 0; i < BANK_DEPTH; i++) begin
          d_reg[b][i]     <= '0;
          r_reg[b][i]     <= '0;
          x_vec_reg[b][i] <= '0;
          cx_reg[b][i]    <= '0;
          q_buf[b][i]     <= '0;
        end
      end
    end else begin
      // Primary write path. Lane k -> bank k (constant), bank-internal
      // addr from wr_bank_addr[k]. SPMV writeback is handled by the
      // we_eff/wr_bank_addr override above.
      for (int k = 0; k < p_lanes; k++) begin
        if (we_eff[k]) begin
          case (wr_sel)
            RF_D_REG:     d_reg    [k][wr_bank_addr[k]] <= wr_data[k];
            RF_R_REG:     r_reg    [k][wr_bank_addr[k]] <= wr_data[k];
            RF_X_VEC_REG: x_vec_reg[k][wr_bank_addr[k]] <= wr_data[k];
            RF_CX_REG:    cx_reg   [k][wr_bank_addr[k]] <= wr_data[k];
            RF_Q_BUF:     q_buf    [k][wr_bank_addr[k]] <= wr_data[k];
            default: ;
          endcase
        end
      end
      // Secondary write path. Always targets a different RF than the
      // primary on the same cycle (r_reg vs x_vec_reg in S_AXPY_XR_FEED;
      // d_reg vs r_reg in S_VNS_R), so concurrent writes never collide
      // on the same flop. Outside those states CGCtrl drives we_sec='0.
      for (int k = 0; k < p_lanes; k++) begin
        if (we_sec[k]) begin
          case (wr_sel_sec)
            RF_D_REG:     d_reg    [k][wr_bank_addr_sec[k]] <= wr_data_sec[k];
            RF_R_REG:     r_reg    [k][wr_bank_addr_sec[k]] <= wr_data_sec[k];
            RF_X_VEC_REG: x_vec_reg[k][wr_bank_addr_sec[k]] <= wr_data_sec[k];
            RF_CX_REG:    cx_reg   [k][wr_bank_addr_sec[k]] <= wr_data_sec[k];
            RF_Q_BUF:     q_buf    [k][wr_bank_addr_sec[k]] <= wr_data_sec[k];
            default: ;
          endcase
        end
      end
    end
  end

  //----------------------------------------------------------------------
  // Scalar latches
  //----------------------------------------------------------------------

  always_ff @(posedge clk) begin
    if (rst) begin
      rr_reg         <= '0;
      rr_new_latched <= '0;
      dq_latched     <= '0;
      alpha          <= '0;
      beta           <= '0;
      iter           <= '0;
    end else begin
      if (latch_dq)        dq_latched     <= vdot_result;
      if (latch_rr_new)    rr_new_latched <= vdot_result;
      if (latch_alpha)     alpha          <= fpdiv_result;
      if (latch_beta)      beta           <= fpdiv_result;
      // init_rr_reg takes priority: bypasses rr_new_latched and grabs
      // vdot_result on the same cycle vdot finishes (S_VDOT_INIT_FEED).
      if (init_rr_reg)         rr_reg     <= vdot_result;
      else if (refresh_rr_reg) rr_reg     <= rr_new_latched;
      if (reset_iter)      iter           <= '0;
      else if (bump_iter)  iter           <= iter + 1;
    end
  end

endmodule
