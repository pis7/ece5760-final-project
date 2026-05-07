// Toplevel v5 Verilog: synthesizable CG solver for DE1-SoC.
// Seven dedicated Qsys on-chip RAM slaves: q_val_ram, q_col_ram,
// q_rowp_ram (SPMV-owned), cx_ram, cy_ram (S_VNS_R serial reads),
// x_ram, y_ram (CGCtrl load + writeback). sel_y picks x vs y pass.

module CGTop #(
  parameter p_lanes            = 8,
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

  // ARM control
  input  logic sw_go,
  output logic sw_done,
  input  logic sw_done_ack,

  // -- Avalon slave: q_val_ram (SPMV read-only) ---------------------------
  output logic [p_m10k_addr_bits-1:0] q_val_ram_address,
  output logic                        q_val_ram_chipselect,
  output logic                        q_val_ram_clken,
  output logic                        q_val_ram_write,
  input  logic [p_word_bits-1:0]      q_val_ram_readdata,
  output logic [p_word_bits-1:0]      q_val_ram_writedata,
  output logic [3:0]                  q_val_ram_byteenable,

  // -- Avalon slave: q_col_ram (SPMV read-only) ---------------------------
  output logic [p_m10k_addr_bits-1:0] q_col_ram_address,
  output logic                        q_col_ram_chipselect,
  output logic                        q_col_ram_clken,
  output logic                        q_col_ram_write,
  input  logic [31:0]                 q_col_ram_readdata,
  output logic [31:0]                 q_col_ram_writedata,
  output logic [3:0]                  q_col_ram_byteenable,

  // -- Avalon slave: q_rowp_ram (SPMV read-only) --------------------------
  output logic [p_m10k_addr_bits-1:0] q_rowp_ram_address,
  output logic                        q_rowp_ram_chipselect,
  output logic                        q_rowp_ram_clken,
  output logic                        q_rowp_ram_write,
  input  logic [31:0]                 q_rowp_ram_readdata,
  output logic [31:0]                 q_rowp_ram_writedata,
  output logic [3:0]                  q_rowp_ram_byteenable,

  // -- Avalon slave: cx_ram (CGCtrl S_VNS_R read-only) --------------------
  output logic [p_m10k_addr_bits-1:0] cx_ram_address,
  output logic                        cx_ram_chipselect,
  output logic                        cx_ram_clken,
  output logic                        cx_ram_write,
  input  logic [p_word_bits-1:0]      cx_ram_readdata,
  output logic [p_word_bits-1:0]      cx_ram_writedata,
  output logic [3:0]                  cx_ram_byteenable,

  // -- Avalon slave: cy_ram (CGCtrl S_VNS_R read-only) --------------------
  output logic [p_m10k_addr_bits-1:0] cy_ram_address,
  output logic                        cy_ram_chipselect,
  output logic                        cy_ram_clken,
  output logic                        cy_ram_write,
  input  logic [p_word_bits-1:0]      cy_ram_readdata,
  output logic [p_word_bits-1:0]      cy_ram_writedata,
  output logic [3:0]                  cy_ram_byteenable,

  // -- Avalon slave: x_ram (CGCtrl load + writeback) ----------------------
  output logic [p_m10k_addr_bits-1:0] x_ram_address,
  output logic                        x_ram_chipselect,
  output logic                        x_ram_clken,
  output logic                        x_ram_write,
  input  logic [p_word_bits-1:0]      x_ram_readdata,
  output logic [p_word_bits-1:0]      x_ram_writedata,
  output logic [3:0]                  x_ram_byteenable,

  // -- Avalon slave: y_ram (CGCtrl load + writeback) ----------------------
  output logic [p_m10k_addr_bits-1:0] y_ram_address,
  output logic                        y_ram_chipselect,
  output logic                        y_ram_clken,
  output logic                        y_ram_write,
  input  logic [p_word_bits-1:0]      y_ram_readdata,
  output logic [p_word_bits-1:0]      y_ram_writedata,
  output logic [3:0]                  y_ram_byteenable,

  // CG solve parameters
  input [31:0] max_iter,
  input [31:0] eps_sq,
  input [31:0] n
);

  //----------------------------------------------------------------------
  // PIO input registration
  //----------------------------------------------------------------------
  logic        sw_go_q;
  logic        sw_done_ack_q;
  logic        rst_q;
  logic [31:0] n_q;
  logic [31:0] max_iter_q;
  logic [31:0] eps_sq_q;

  always_ff @(posedge clk) begin
    sw_go_q       <= sw_go;
    sw_done_ack_q <= sw_done_ack;
    rst_q         <= rst;
    n_q           <= n;
    max_iter_q    <= max_iter;
    eps_sq_q      <= eps_sq;
  end

  //----------------------------------------------------------------------
  // CGCtrl <-> CGDpath wires
  //----------------------------------------------------------------------

  logic [31:0]                  iter;
  logic signed [p_acc_bits-1:0] rr_new;
  logic signed [p_acc_bits-1:0] rr_old;

  // CGCtrl-driven x/y load/writeback bus (CGDpath routes to x_ram or
  // y_ram based on sel_y).
  logic [p_m10k_addr_bits-1:0] ctrl_xy_addr;
  logic                        ctrl_xy_wr_en;
  logic [p_word_bits-1:0]      ctrl_xy_wdata;
  logic                        sel_y;

  // CGCtrl S_VNS_R cx serial-read port (CGDpath routes to cx_ram or
  // cy_ram based on sel_y).
  logic [p_m10k_addr_bits-1:0] vns_cx_addr;
  logic                        vns_cx_rd_en;
  logic [p_word_bits-1:0]      vns_cx_rdata;

  // RF read ports (4 x p_lanes-wide)
  logic [2:0]                                 rd_a_sel;
  logic [p_lanes*$clog2(p_max_n)-1:0]         rd_a_idx_packed;
  logic [p_lanes-1:0]                         rd_a_valid;
  logic [p_lanes*p_total_bits-1:0]            rd_a_data_packed;

  logic [2:0]                                 rd_b_sel;
  logic [p_lanes*$clog2(p_max_n)-1:0]         rd_b_idx_packed;
  logic [p_lanes-1:0]                         rd_b_valid;
  logic [p_lanes*p_total_bits-1:0]            rd_b_data_packed;

  logic [2:0]                                 rd_c_sel;
  logic [p_lanes*$clog2(p_max_n)-1:0]         rd_c_idx_packed;
  logic [p_lanes-1:0]                         rd_c_valid;
  logic [p_lanes*p_total_bits-1:0]            rd_c_data_packed;

  logic [2:0]                                 rd_d_sel;
  logic [p_lanes*$clog2(p_max_n)-1:0]         rd_d_idx_packed;
  logic [p_lanes-1:0]                         rd_d_valid;
  logic [p_lanes*p_total_bits-1:0]            rd_d_data_packed;

  logic [2:0]                                 rd_vec_sel;

  // RF write ports (primary + secondary)
  logic [2:0]                                 wr_sel;
  logic [p_lanes*$clog2(p_max_n)-1:0]         wr_idx_ctrl_packed;
  logic                                       wr_idx_src_spmv;
  logic [p_lanes-1:0]                         we;
  logic [2:0]                                 wdata_src;
  logic [2:0]                                 wr_sel_sec;
  logic [2:0]                                 wdata_src_sec;
  logic [p_lanes-1:0]                         we_sec;

  // Scalar latches
  logic                               latch_dq, latch_rr_new;
  logic                               latch_alpha, latch_beta;
  logic                               refresh_rr_reg, init_rr_reg;
  logic                               bump_iter, reset_iter;

  // Submodule handshakes
  logic vdot_istream_val,  vdot_istream_rdy;
  logic vdot_ostream_val,  vdot_ostream_rdy;

  logic axpy_x_istream_val, axpy_x_istream_rdy;
  logic axpy_x_ostream_val, axpy_x_ostream_rdy;
  logic axpy_r_istream_val, axpy_r_istream_rdy;
  logic axpy_r_ostream_val, axpy_r_ostream_rdy;
  logic axpy_coef_src_beta;

  logic spmv_istream_val,  spmv_istream_rdy;
  logic spmv_ostream_val,  spmv_ostream_rdy;

  logic fpdiv_istream_val, fpdiv_istream_rdy;
  logic fpdiv_ostream_val, fpdiv_ostream_rdy;
  logic fpdiv_a_src_rrnew, fpdiv_b_src_rr;

  //----------------------------------------------------------------------
  // Avalon slave constant tie-offs
  //----------------------------------------------------------------------

  // SPMV-owned slaves are read-only; tie write/wdata low.
  assign q_val_ram_chipselect  = 1'b1;
  assign q_val_ram_clken       = 1'b1;
  assign q_val_ram_write       = 1'b0;
  assign q_val_ram_writedata   = '0;
  assign q_val_ram_byteenable  = 4'b1111;

  assign q_col_ram_chipselect  = 1'b1;
  assign q_col_ram_clken       = 1'b1;
  assign q_col_ram_write       = 1'b0;
  assign q_col_ram_writedata   = 32'd0;
  assign q_col_ram_byteenable  = 4'b1111;

  assign q_rowp_ram_chipselect = 1'b1;
  assign q_rowp_ram_clken      = 1'b1;
  assign q_rowp_ram_write      = 1'b0;
  assign q_rowp_ram_writedata  = 32'd0;
  assign q_rowp_ram_byteenable = 4'b1111;

  // cx/cy slaves are read-only from RTL (ARM writes via h2f bridge port).
  assign cx_ram_chipselect     = 1'b1;
  assign cx_ram_clken          = 1'b1;
  assign cx_ram_write          = 1'b0;
  assign cx_ram_writedata      = '0;
  assign cx_ram_byteenable     = 4'b1111;

  assign cy_ram_chipselect     = 1'b1;
  assign cy_ram_clken          = 1'b1;
  assign cy_ram_write          = 1'b0;
  assign cy_ram_writedata      = '0;
  assign cy_ram_byteenable     = 4'b1111;

  // x/y slaves see read+write from CGCtrl (load + writeback). ARM also
  // accesses via the h2f bridge port; the dual-port nature of Qsys
  // on-chip RAM keeps both sides isolated.
  assign x_ram_chipselect      = 1'b1;
  assign x_ram_clken           = 1'b1;
  assign x_ram_byteenable      = 4'b1111;

  assign y_ram_chipselect      = 1'b1;
  assign y_ram_clken           = 1'b1;
  assign y_ram_byteenable      = 4'b1111;

  // sel_y picks which slave's port the ctrl_xy bus drives.
  assign x_ram_address         = ctrl_xy_addr;
  assign x_ram_write           = ctrl_xy_wr_en & ~sel_y;
  assign x_ram_writedata       = ctrl_xy_wdata;

  assign y_ram_address         = ctrl_xy_addr;
  assign y_ram_write           = ctrl_xy_wr_en & sel_y;
  assign y_ram_writedata       = ctrl_xy_wdata;

  // Read-back into CGCtrl during S_LD_X_CAPT: pick x_ram or y_ram based
  // on sel_y. Combinational mux of the registered Avalon readdata.
  logic [p_word_bits-1:0] ctrl_xy_rdata;
  assign ctrl_xy_rdata = sel_y ? y_ram_readdata : x_ram_readdata;

  // SPMV port plumbing (3 independent reads).
  logic [p_m10k_addr_bits-1:0] spmv_q_val_addr,  spmv_q_col_addr,  spmv_q_rowp_addr;
  assign q_val_ram_address  = spmv_q_val_addr;
  assign q_col_ram_address  = spmv_q_col_addr;
  assign q_rowp_ram_address = spmv_q_rowp_addr;

  // S_VNS_R cx port: route to cx_ram or cy_ram based on sel_y.
  assign cx_ram_address = vns_cx_addr;
  assign cy_ram_address = vns_cx_addr;
  assign vns_cx_rdata   = sel_y ? cy_ram_readdata : cx_ram_readdata;

  //----------------------------------------------------------------------
  // Control Unit
  //----------------------------------------------------------------------

  CGCtrl #(
    .p_lanes          (p_lanes),
    .p_max_n          (p_max_n),
    .p_int_bits       (p_int_bits),
    .p_frac_bits      (p_frac_bits),
    .p_total_bits     (p_total_bits),
    .p_acc_bits       (p_acc_bits),
    .p_m10k_addr_bits (p_m10k_addr_bits),
    .p_word_bits      (p_word_bits)
  ) ctrl (
    .clk,
    .rst         (rst_q),
    .sw_go       (sw_go_q),
    .sw_done,
    .sw_done_ack (sw_done_ack_q),
    .n           (n_q),
    .max_iter    (max_iter_q),
    .eps_sq      (eps_sq_q),
    .iter, .rr_new, .rr_old,
    // x/y load+writeback bus (rdata is consumed by CGDpath, not here)
    .ctrl_xy_addr, .ctrl_xy_wr_en, .ctrl_xy_wdata,
    .sel_y,
    // VNS_R cx serial-read port (rdata is consumed by CGDpath)
    .vns_cx_addr, .vns_cx_rd_en,
    .rd_a_sel, .rd_a_idx_packed, .rd_a_valid, .rd_a_data_packed,
    .rd_b_sel, .rd_b_idx_packed, .rd_b_valid, .rd_b_data_packed,
    .rd_c_sel, .rd_c_idx_packed, .rd_c_valid, .rd_c_data_packed,
    .rd_d_sel, .rd_d_idx_packed, .rd_d_valid, .rd_d_data_packed,
    .rd_vec_sel,
    .wr_sel, .wr_idx_ctrl_packed, .wr_idx_src_spmv, .we, .wdata_src,
    .wr_sel_sec, .wdata_src_sec, .we_sec,
    .latch_dq, .latch_rr_new, .latch_alpha, .latch_beta,
    .refresh_rr_reg, .init_rr_reg, .bump_iter, .reset_iter,
    .vdot_istream_val, .vdot_istream_rdy,
    .vdot_ostream_val, .vdot_ostream_rdy,
    .axpy_x_istream_val, .axpy_x_istream_rdy,
    .axpy_x_ostream_val, .axpy_x_ostream_rdy,
    .axpy_r_istream_val, .axpy_r_istream_rdy,
    .axpy_r_ostream_val, .axpy_r_ostream_rdy,
    .axpy_coef_src_beta,
    .spmv_istream_val, .spmv_istream_rdy,
    .spmv_ostream_val, .spmv_ostream_rdy,
    .fpdiv_istream_val, .fpdiv_istream_rdy,
    .fpdiv_ostream_val, .fpdiv_ostream_rdy,
    .fpdiv_a_src_rrnew, .fpdiv_b_src_rr
  );

  //----------------------------------------------------------------------
  // Datapath
  //----------------------------------------------------------------------

  CGDpath #(
    .p_lanes            (p_lanes),
    .p_max_n            (p_max_n),
    .p_int_bits         (p_int_bits),
    .p_frac_bits        (p_frac_bits),
    .p_total_bits       (p_total_bits),
    .p_acc_bits         (p_acc_bits),
    .p_m10k_addr_bits   (p_m10k_addr_bits),
    .p_word_bits        (p_word_bits)
  ) dpath (
    .clk,
    .rst (rst_q),
    .n   (n_q),
    // SPMV-owned Q ports
    .spmv_q_val_addr,   .spmv_q_val_rdata  (q_val_ram_readdata),
    .spmv_q_col_addr,   .spmv_q_col_rdata  (q_col_ram_readdata),
    .spmv_q_rowp_addr,  .spmv_q_rowp_rdata (q_rowp_ram_readdata),
    // M10K rdata muxed by sel_y (driven up at the CGTop level).
    .ctrl_xy_rdata,
    .vns_cx_rdata,
    .rd_a_sel, .rd_a_idx_packed, .rd_a_valid, .rd_a_data_packed,
    .rd_b_sel, .rd_b_idx_packed, .rd_b_valid, .rd_b_data_packed,
    .rd_c_sel, .rd_c_idx_packed, .rd_c_valid, .rd_c_data_packed,
    .rd_d_sel, .rd_d_idx_packed, .rd_d_valid, .rd_d_data_packed,
    .rd_vec_sel,
    .wr_sel, .wr_idx_ctrl_packed, .wr_idx_src_spmv, .we, .wdata_src,
    .wr_sel_sec, .wdata_src_sec, .we_sec,
    .latch_dq, .latch_rr_new, .latch_alpha, .latch_beta,
    .refresh_rr_reg, .init_rr_reg, .bump_iter, .reset_iter,
    .vdot_istream_val, .vdot_istream_rdy,
    .vdot_ostream_val, .vdot_ostream_rdy,
    .axpy_x_istream_val, .axpy_x_istream_rdy,
    .axpy_x_ostream_val, .axpy_x_ostream_rdy,
    .axpy_r_istream_val, .axpy_r_istream_rdy,
    .axpy_r_ostream_val, .axpy_r_ostream_rdy,
    .axpy_coef_src_beta,
    .spmv_istream_val, .spmv_istream_rdy,
    .spmv_ostream_val, .spmv_ostream_rdy,
    .fpdiv_istream_val, .fpdiv_istream_rdy,
    .fpdiv_ostream_val, .fpdiv_ostream_rdy,
    .fpdiv_a_src_rrnew, .fpdiv_b_src_rr,
    .iter, .rr_new, .rr_old
  );

endmodule
