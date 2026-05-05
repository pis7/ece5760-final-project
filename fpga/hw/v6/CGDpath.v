// v6 datapath -- single dimension; CGTop instantiates one CGDpath per
// engine.
//
// SPMV owns three independent read ports (q_val, q_col, q_rowp), wired
// directly to this engine's dedicated Qsys M10K slaves. CGCtrl owns a
// private load+writeback bus to this engine's x_ram (or y_ram) and a
// separate serial cx-read port for S_VNS_R that goes to this engine's
// cx_ram (or cy_ram). Every slave port has exactly one consumer; no
// shared bus mux.
//
// Central RF: 4 banked flop arrays (d_reg, r_reg, x_vec_reg, q_buf).
// cx is read directly from M10K via WD_VNS_SCALAR during S_VNS_R_CAPT
// (single-lane writeback computing -(vns_cx_rdata + rd_b)).
//
// DSP count: VecDot p_lanes + AXPY x p_lanes + AXPY r p_lanes + SPMV 1.

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
  parameter p_word_bits        = (p_total_bits <= 32) ? 32 : 64
) (
  input  logic clk,
  input  logic rst,

  input  logic [31:0] n,

  // -- SPMV-owned Q read ports (3 independent M10K slaves) ----------------
  // q_val carries a fixed-point value (p_word_bits wide); q_col / q_rowp
  // carry plain integer indices and stay 32-bit.
  output logic [p_m10k_addr_bits-1:0] spmv_q_val_addr,
  input  logic [p_word_bits-1:0]      spmv_q_val_rdata,
  output logic [p_m10k_addr_bits-1:0] spmv_q_col_addr,
  input  logic [31:0]                 spmv_q_col_rdata,
  output logic [p_m10k_addr_bits-1:0] spmv_q_rowp_addr,
  input  logic [31:0]                 spmv_q_rowp_rdata,

  // -- M10K read-data inputs from this engine's dimension-private slaves --
  // CGTop wires CGCtrl's ctrl_xy_* outputs out to the engine's x_ram
  // (or y_ram) and feeds the readdata back as ctrl_xy_rdata for WD_MEM.
  // Same for cx_ram (or cy_ram) -> vns_cx_rdata for WD_VNS_SCALAR.
  input  logic [p_word_bits-1:0]      ctrl_xy_rdata,
  input  logic [p_word_bits-1:0]      vns_cx_rdata,

  // --- Addressed RF read ports (combinational) ---------------------------
  // 4 RFs: 0=d_reg, 1=r_reg, 2=x_vec_reg, 4=q_buf. rd_a/rd_b are
  // general-purpose; rd_c/rd_d feed u_axpy_r in S_AXPY_XR_FEED.
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
  //   WD_AXPY_R:     axpy_r_z_lane (S_AXPY_XR_FEED)
  //   WD_VNS_SCALAR: -(vns_cx_rdata + rd_b) (S_VNS_R_CAPT, fused with primary)
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

  input  logic                             axpy_x_istream_val,
  output logic                             axpy_x_istream_rdy,
  output logic                             axpy_x_ostream_val,
  input  logic                             axpy_x_ostream_rdy,

  input  logic                             axpy_r_istream_val,
  output logic                             axpy_r_istream_rdy,
  output logic                             axpy_r_ostream_val,
  input  logic                             axpy_r_ostream_rdy,

  input  logic                             axpy_coef_src_beta,

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
  localparam BANK_DEPTH  = (p_max_n + p_lanes - 1) / p_lanes;
  localparam BANK_ADDR_W = $clog2(BANK_DEPTH);
  localparam BANK_SEL_W  = (CLOG2_LANES == 0) ? 1 : CLOG2_LANES;

  //----------------------------------------------------------------------
  // RF select encodings (must match CGCtrl.v).
  //----------------------------------------------------------------------
  localparam [2:0] RF_D_REG     = 3'd0;
  localparam [2:0] RF_R_REG     = 3'd1;
  localparam [2:0] RF_X_VEC_REG = 3'd2;
  localparam [2:0] RF_Q_BUF     = 3'd4;

  localparam [2:0] WD_MEM         = 3'd0;
  localparam [2:0] WD_AXPY        = 3'd1;
  localparam [2:0] WD_SPMV        = 3'd2;
  localparam [2:0] WD_VNS_SCALAR  = 3'd3;
  localparam [2:0] WD_AXPY_R      = 3'd5;

  //----------------------------------------------------------------------
  // Register files -- banked by lane id. Bank b holds elements with
  // (idx mod p_lanes) == b.
  //----------------------------------------------------------------------

  logic signed [p_total_bits-1:0] d_reg     [p_lanes][BANK_DEPTH];
  logic signed [p_total_bits-1:0] r_reg     [p_lanes][BANK_DEPTH];
  logic signed [p_total_bits-1:0] x_vec_reg [p_lanes][BANK_DEPTH];
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
      RF_Q_BUF:     rf_read_bank = q_buf     [bank][bank_addr];
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
    .mode          (1'b0),
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
    .mode          (1'b1),
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
  logic [$clog2(p_max_n)-1:0]      spmv_vec_rd_idx;
  logic signed [p_total_bits-1:0]  spmv_vec_rd_data;
  logic [31:0]                     spmv_row_idx;
  logic signed [p_total_bits-1:0]  spmv_row_val;

  // SPMV vec read crossbar (vec lives in x_vec_reg or d_reg flop arrays).
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
    // Three independent Q read ports (local-base 0 in each slave).
    .q_val_addr         (spmv_q_val_addr),
    .q_val_rdata        (spmv_q_val_rdata),
    .q_col_addr         (spmv_q_col_addr),
    .q_col_rdata        (spmv_q_col_rdata),
    .q_rowp_addr        (spmv_q_rowp_addr),
    .q_rowp_rdata       (spmv_q_rowp_rdata),
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
  // RF write port (per-lane).
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
      for (int k = 0; k < p_lanes; k++) begin
        wr_bank_addr[k] = BANK_ADDR_W'(wr_idx_ctrl[k] / IDX_W'(unsigned'(p_lanes)));
        we_eff[k]       = we[k];
      end
    end
  end

  logic [BANK_ADDR_W-1:0] wr_bank_addr_sec [p_lanes];
  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_wr_addr_sec
      assign wr_bank_addr_sec[gi] = BANK_ADDR_W'(wr_idx_ctrl[gi] / IDX_W'(unsigned'(p_lanes)));
    end
  endgenerate

  // Sign-extended cx scalar from M10K, used by WD_VNS_SCALAR.
  logic signed [p_total_bits-1:0] vns_cx_signed;
  assign vns_cx_signed = p_total_bits'($signed(vns_cx_rdata));

  always_comb begin
    for (int k = 0; k < p_lanes; k++) wr_data[k] = '0;
    case (wdata_src)
      WD_MEM: begin
        // Loaded from this engine's x_ram (or y_ram) via ctrl_xy_rdata. Replicate so
        // whichever lane has we_eff=1 picks it up.
        for (int k = 0; k < p_lanes; k++)
          wr_data[k] = p_total_bits'($signed(ctrl_xy_rdata));
      end
      WD_AXPY: begin
        for (int k = 0; k < p_lanes; k++) wr_data[k] = axpy_x_z_lane[k];
      end
      WD_SPMV: begin
        for (int k = 0; k < p_lanes; k++) wr_data[k] = spmv_row_val;
      end
      WD_VNS_SCALAR: begin
        // -(cx[stream_idx] + q_buf[stream_idx]). Single-lane writeback,
        // but we replicate so the active lane picks the right value.
        for (int k = 0; k < p_lanes; k++)
          wr_data[k] = -(vns_cx_signed + rd_b_data[k]);
      end
      default: ;
    endcase
  end

  always_comb begin
    for (int k = 0; k < p_lanes; k++) wr_data_sec[k] = '0;
    case (wdata_src_sec)
      WD_AXPY_R: begin
        for (int k = 0; k < p_lanes; k++) wr_data_sec[k] = axpy_r_z_lane[k];
      end
      WD_VNS_SCALAR: begin
        for (int k = 0; k < p_lanes; k++)
          wr_data_sec[k] = -(vns_cx_signed + rd_b_data[k]);
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
          q_buf[b][i]     <= '0;
        end
      end
    end else begin
      for (int k = 0; k < p_lanes; k++) begin
        if (we_eff[k]) begin
          case (wr_sel)
            RF_D_REG:     d_reg    [k][wr_bank_addr[k]] <= wr_data[k];
            RF_R_REG:     r_reg    [k][wr_bank_addr[k]] <= wr_data[k];
            RF_X_VEC_REG: x_vec_reg[k][wr_bank_addr[k]] <= wr_data[k];
            RF_Q_BUF:     q_buf    [k][wr_bank_addr[k]] <= wr_data[k];
            default: ;
          endcase
        end
      end
      for (int k = 0; k < p_lanes; k++) begin
        if (we_sec[k]) begin
          case (wr_sel_sec)
            RF_D_REG:     d_reg    [k][wr_bank_addr_sec[k]] <= wr_data_sec[k];
            RF_R_REG:     r_reg    [k][wr_bank_addr_sec[k]] <= wr_data_sec[k];
            RF_X_VEC_REG: x_vec_reg[k][wr_bank_addr_sec[k]] <= wr_data_sec[k];
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
      if (init_rr_reg)         rr_reg     <= vdot_result;
      else if (refresh_rr_reg) rr_reg     <= rr_new_latched;
      if (reset_iter)      iter           <= '0;
      else if (bump_iter)  iter           <= iter + 1;
    end
  end

endmodule
