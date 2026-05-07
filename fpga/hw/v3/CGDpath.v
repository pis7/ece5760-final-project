// v3 datapath -- pure datapath, no FSM.
//
// Owns all register files, scalar registers, linalg submodule
// instances (each with its own Ctrl+Dpath internally), and a small
// set of muxes. CGCtrl drives every enable / select / handshake
// signal; this module has no state machine of its own.
//
// p_lanes parallelism:
//   - VecDot and AXPY each take p_lanes (a,b) pairs per handshake.
//   - Two p_lanes-wide addressed RF read ports (rd_a, rd_b) plus
//     one single-lane vec read port for SPMV (memory-bound).
//   - One p_lanes-wide RF write port. AXPY writeback uses all lanes;
//     single-element writes (LD, SPMV result, VNS, COPY) use lane 0
//     and gate we[k>=1] to 0.
//   - Per-lane read valid mask: out-of-range lanes (for the final
//     partial group) read as zero, so a*b contributes 0 to VecDot
//     and AXPY's computed-but-unwritten z is harmless.
//
// DSP count: VecDot p_lanes + AXPY p_lanes + SPMV 1.

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

  // --- Runtime parameters passed through from CGTop ----------------------
  input  logic [31:0] n,

  // --- Memory bus: one of CGCtrl (for LD/WB) or SPMV (for CSR) owns it ---
  output logic [p_m10k_addr_bits-1:0] mem_addr,
  output logic                        mem_wr_en,
  output logic [p_word_bits-1:0]      mem_wdata,
  input  logic [p_word_bits-1:0]      mem_rdata,

  // CGCtrl's memory bus (for LD / WB phases)
  input  logic [p_m10k_addr_bits-1:0] ctrl_mem_addr,
  input  logic                        ctrl_mem_wr_en,
  input  logic [p_word_bits-1:0]      ctrl_mem_wdata,
  input  logic                        ctrl_mem_src_spmv,     // 0 = CGCtrl, 1 = SPMV

  // --- Addressed RF read ports (combinational) ---------------------------
  // 5 RFs: 0=d_reg, 1=r_reg, 2=x_vec_reg, 3=cx_reg, 4=q_buf
  input  logic [2:0]                              rd_a_sel,
  input  logic [p_lanes*$clog2(p_max_n)-1:0]      rd_a_idx_packed,
  input  logic [p_lanes-1:0]                      rd_a_valid,
  output logic [p_lanes*p_total_bits-1:0]         rd_a_data_packed,

  input  logic [2:0]                              rd_b_sel,
  input  logic [p_lanes*$clog2(p_max_n)-1:0]      rd_b_idx_packed,
  input  logic [p_lanes-1:0]                      rd_b_valid,
  output logic [p_lanes*p_total_bits-1:0]         rd_b_data_packed,

  input  logic [2:0]                              rd_vec_sel,

  // --- Addressed RF write port (p_lanes-wide) ----------------------------
  // wdata_src:  0=mem_rdata(lane0), 1=axpy ostream(all lanes),
  //             2=spmv ostream(lane0), 3=rd_a_data(all lanes),
  //             4=VNS -(rd_a+rd_b)(all lanes)
  input  logic [2:0]                              wr_sel,
  input  logic [p_lanes*$clog2(p_max_n)-1:0]      wr_idx_ctrl_packed,
  input  logic                                    wr_idx_src_spmv,   // lane0 idx from SPMV row_idx
  input  logic [p_lanes-1:0]                      we,
  input  logic [2:0]                              wdata_src,

  // --- Scalar latch enables ----------------------------------------------
  input  logic                             latch_dq,
  input  logic                             latch_rr_new,
  input  logic                             latch_alpha,
  input  logic                             latch_beta,
  input  logic                             refresh_rr_reg,
  input  logic                             bump_iter,
  input  logic                             reset_iter,

  // --- Submodule handshake plumbing --------------------------------------
  // VecDot
  input  logic                             vdot_istream_val,
  output logic                             vdot_istream_rdy,
  output logic                             vdot_ostream_val,
  input  logic                             vdot_ostream_rdy,

  // AXPY: mode + coef source selected here
  input  logic                             axpy_istream_val,
  output logic                             axpy_istream_rdy,
  output logic                             axpy_ostream_val,
  input  logic                             axpy_ostream_rdy,
  input  logic                             axpy_mode,          // 0=add, 1=sub
  input  logic                             axpy_coef_src_beta, // 0=alpha, 1=beta

  // SPMV
  input  logic                             spmv_istream_val,
  output logic                             spmv_istream_rdy,
  output logic                             spmv_ostream_val,
  input  logic                             spmv_ostream_rdy,

  // FpDiv: istream {a, b} source selects; ostream result goes to alpha/beta via latch
  input  logic                             fpdiv_istream_val,
  output logic                             fpdiv_istream_rdy,
  output logic                             fpdiv_ostream_val,
  input  logic                             fpdiv_ostream_rdy,
  input  logic                             fpdiv_a_src_rrnew,  // 0=rr_reg, 1=rr_new_latched
  input  logic                             fpdiv_b_src_rr,     // 0=dq_latched, 1=rr_reg

  // --- Observability to CGCtrl -------------------------------------------
  output logic [31:0]                      iter,
  output logic signed [p_acc_bits-1:0]     rr_new,
  output logic signed [p_acc_bits-1:0]     rr_old
);

  localparam IDX_W = $clog2(p_max_n);

  //----------------------------------------------------------------------
  // RF select encodings (must match CGCtrl.v)
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
  // Register files (flip-flop unpacked arrays)
  //----------------------------------------------------------------------

  logic signed [p_total_bits-1:0] d_reg     [p_max_n];
  logic signed [p_total_bits-1:0] r_reg     [p_max_n];
  logic signed [p_total_bits-1:0] x_vec_reg [p_max_n];
  logic signed [p_total_bits-1:0] cx_reg    [p_max_n];
  logic signed [p_total_bits-1:0] q_buf     [p_max_n];

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
  // RF read muxes (per-lane, gated by valid mask)
  //----------------------------------------------------------------------

  function automatic logic signed [p_total_bits-1:0] rf_read(
    input logic [2:0]          sel,
    input logic [IDX_W-1:0]    idx
  );
    case (sel)
      RF_D_REG:     rf_read = d_reg[idx];
      RF_R_REG:     rf_read = r_reg[idx];
      RF_X_VEC_REG: rf_read = x_vec_reg[idx];
      RF_CX_REG:    rf_read = cx_reg[idx];
      RF_Q_BUF:     rf_read = q_buf[idx];
      default:      rf_read = '0;
    endcase
  endfunction

  logic [IDX_W-1:0]               rd_a_idx [p_lanes];
  logic [IDX_W-1:0]               rd_b_idx [p_lanes];
  logic signed [p_total_bits-1:0] rd_a_data [p_lanes];
  logic signed [p_total_bits-1:0] rd_b_data [p_lanes];

  genvar gi;
  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_rd
      assign rd_a_idx[gi] = rd_a_idx_packed[(gi+1)*IDX_W-1 -: IDX_W];
      assign rd_b_idx[gi] = rd_b_idx_packed[(gi+1)*IDX_W-1 -: IDX_W];
      assign rd_a_data[gi] = rd_a_valid[gi]
                             ? rf_read(rd_a_sel, rd_a_idx[gi])
                             : p_total_bits'(0);
      assign rd_b_data[gi] = rd_b_valid[gi]
                             ? rf_read(rd_b_sel, rd_b_idx[gi])
                             : p_total_bits'(0);
      assign rd_a_data_packed[(gi+1)*p_total_bits-1 -: p_total_bits] =
        $unsigned(rd_a_data[gi]);
      assign rd_b_data_packed[(gi+1)*p_total_bits-1 -: p_total_bits] =
        $unsigned(rd_b_data[gi]);
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

  // --- AXPY -------------------------------------------------------------
  logic signed [p_total_bits-1:0]          axpy_coef;
  logic [p_lanes*p_total_bits-1:0]         axpy_z_packed;
  logic signed [p_total_bits-1:0]          axpy_z_lane [p_lanes];

  assign axpy_coef = axpy_coef_src_beta ? beta : alpha;

  AXPY_seq #(
    .p_lanes     (p_lanes),
    .p_int_bits  (p_int_bits),
    .p_frac_bits (p_frac_bits),
    .p_total_bits(p_total_bits)
  ) u_axpy (
    .clk, .rst,
    .n,
    .mode          (axpy_mode),
    .coef          (axpy_coef),
    .istream_val   (axpy_istream_val),
    .istream_rdy   (axpy_istream_rdy),
    .istream_msg_a (rd_a_data_packed),
    .istream_msg_b (rd_b_data_packed),
    .ostream_val   (axpy_ostream_val),
    .ostream_rdy   (axpy_ostream_rdy),
    .ostream_msg_z (axpy_z_packed)
  );

  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_axpy_unpack
      assign axpy_z_lane[gi] =
        $signed(axpy_z_packed[(gi+1)*p_total_bits-1 -: p_total_bits]);
    end
  endgenerate

  // --- SPMV -------------------------------------------------------------
  logic [p_m10k_addr_bits-1:0]    spmv_mem_addr;
  logic                            spmv_mem_rd_en;
  logic [$clog2(p_max_n)-1:0]      spmv_vec_rd_idx;
  logic signed [p_total_bits-1:0]  spmv_vec_rd_data;
  logic [31:0]                     spmv_row_idx;
  logic signed [p_total_bits-1:0]  spmv_row_val;

  // vec lookup for SPMV: SPMV drives idx, CGCtrl picks which RF via rd_vec_sel
  assign spmv_vec_rd_data = rf_read(rd_vec_sel, spmv_vec_rd_idx);

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
  // RF write port (per-lane)
  //----------------------------------------------------------------------

  logic [IDX_W-1:0]                wr_idx_ctrl [p_lanes];
  logic [IDX_W-1:0]                wr_idx      [p_lanes];
  logic signed [p_total_bits-1:0]  wr_data     [p_lanes];

  generate
    for (gi = 0; gi < p_lanes; gi = gi + 1) begin : g_wr_idx
      assign wr_idx_ctrl[gi] = wr_idx_ctrl_packed[(gi+1)*IDX_W-1 -: IDX_W];
    end
  endgenerate

  // Lane 0 idx may come from SPMV; other lanes always take the per-lane ctrl idx.
  always_comb begin
    wr_idx[0] = wr_idx_src_spmv ? spmv_row_idx[IDX_W-1:0]
                                : wr_idx_ctrl[0];
    for (int k = 1; k < p_lanes; k++) wr_idx[k] = wr_idx_ctrl[k];
  end

  // Per-lane write data. Only WD_AXPY, WD_RDA, WD_VNS produce distinct
  // values across lanes; the others only drive lane 0 (other lanes'
  // we are gated off by the caller).
  always_comb begin
    for (int k = 0; k < p_lanes; k++) wr_data[k] = '0;
    case (wdata_src)
      WD_MEM:  wr_data[0] = p_total_bits'($signed(mem_rdata));
      WD_AXPY: begin
        for (int k = 0; k < p_lanes; k++) wr_data[k] = axpy_z_lane[k];
      end
      WD_SPMV: wr_data[0] = spmv_row_val;
      WD_RDA: begin
        for (int k = 0; k < p_lanes; k++) wr_data[k] = rd_a_data[k];
      end
      WD_VNS: begin
        for (int k = 0; k < p_lanes; k++)
          wr_data[k] = -(rd_a_data[k] + rd_b_data[k]);
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      for (int i = 0; i < p_max_n; i++) begin
        d_reg[i]     <= '0;
        r_reg[i]     <= '0;
        x_vec_reg[i] <= '0;
        cx_reg[i]    <= '0;
        q_buf[i]     <= '0;
      end
    end else begin
      for (int k = 0; k < p_lanes; k++) begin
        if (we[k]) begin
          case (wr_sel)
            RF_D_REG:     d_reg    [wr_idx[k]] <= wr_data[k];
            RF_R_REG:     r_reg    [wr_idx[k]] <= wr_data[k];
            RF_X_VEC_REG: x_vec_reg[wr_idx[k]] <= wr_data[k];
            RF_CX_REG:    cx_reg   [wr_idx[k]] <= wr_data[k];
            RF_Q_BUF:     q_buf    [wr_idx[k]] <= wr_data[k];
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
      if (refresh_rr_reg)  rr_reg         <= rr_new_latched;
      if (reset_iter)      iter           <= '0;
      else if (bump_iter)  iter           <= iter + 1;
    end
  end

endmodule
