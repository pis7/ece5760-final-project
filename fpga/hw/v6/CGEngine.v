// v6 CGEngine: a single-dimension CG solver. Wraps CGCtrl + CGDpath
// and exposes one dimension's worth of Avalon-MM slave ports (3
// read-only Q ports + 1 read-only c port + 1 read+write xy port).
// CGTop instantiates this twice (one per dimension), each with its
// own Q slaves -- no contention, no arbiter.

module CGEngine #(
  parameter p_lanes            = 4,
  parameter p_max_n            = 50,
  parameter p_int_bits         = 13,
  parameter p_frac_bits        = 14,
  parameter p_total_bits       = p_int_bits + p_frac_bits,
  parameter p_acc_bits         = 48,
  parameter p_m10k_addr_bits   = 32
) (
  input  logic clk,
  input  logic rst,

  // ARM control (per-engine; CGTop fans one ARM go-pulse to both
  // engines and ANDs their two dones into one sw_done).
  input  logic sw_go,
  output logic sw_done,
  input  logic sw_done_ack,

  // CG solve parameters
  input  logic [31:0] n,
  input  logic [31:0] max_iter,
  input  logic [31:0] eps_sq,

  // -- Engine's Q-CSR Avalon slaves (read-only) --------------------------
  output logic [p_m10k_addr_bits-1:0] q_val_ram_address,
  output logic                        q_val_ram_chipselect,
  output logic                        q_val_ram_clken,
  output logic                        q_val_ram_write,
  input  logic [31:0]                 q_val_ram_readdata,
  output logic [31:0]                 q_val_ram_writedata,
  output logic [3:0]                  q_val_ram_byteenable,

  output logic [p_m10k_addr_bits-1:0] q_col_ram_address,
  output logic                        q_col_ram_chipselect,
  output logic                        q_col_ram_clken,
  output logic                        q_col_ram_write,
  input  logic [31:0]                 q_col_ram_readdata,
  output logic [31:0]                 q_col_ram_writedata,
  output logic [3:0]                  q_col_ram_byteenable,

  output logic [p_m10k_addr_bits-1:0] q_rowp_ram_address,
  output logic                        q_rowp_ram_chipselect,
  output logic                        q_rowp_ram_clken,
  output logic                        q_rowp_ram_write,
  input  logic [31:0]                 q_rowp_ram_readdata,
  output logic [31:0]                 q_rowp_ram_writedata,
  output logic [3:0]                  q_rowp_ram_byteenable,

  // -- Engine's c-vector Avalon slave (read-only) ------------------------
  // cx_ram for the x engine, cy_ram for the y engine.
  output logic [p_m10k_addr_bits-1:0] c_ram_address,
  output logic                        c_ram_chipselect,
  output logic                        c_ram_clken,
  output logic                        c_ram_write,
  input  logic [31:0]                 c_ram_readdata,
  output logic [31:0]                 c_ram_writedata,
  output logic [3:0]                  c_ram_byteenable,

  // -- Engine's xy Avalon slave (load + writeback) -----------------------
  // x_ram for the x engine, y_ram for the y engine.
  output logic [p_m10k_addr_bits-1:0] xy_ram_address,
  output logic                        xy_ram_chipselect,
  output logic                        xy_ram_clken,
  output logic                        xy_ram_write,
  input  logic [31:0]                 xy_ram_readdata,
  output logic [31:0]                 xy_ram_writedata,
  output logic [3:0]                  xy_ram_byteenable
);

  //----------------------------------------------------------------------
  // CGCtrl <-> CGDpath wires
  //----------------------------------------------------------------------
  logic [31:0]                  iter;
  logic signed [p_acc_bits-1:0] rr_new;
  logic signed [p_acc_bits-1:0] rr_old;

  // CGCtrl-driven xy load/writeback bus.
  logic [p_m10k_addr_bits-1:0] ctrl_xy_addr;
  logic                        ctrl_xy_wr_en;
  logic [31:0]                 ctrl_xy_wdata;

  // CGCtrl S_VNS_R c-vector serial-read port.
  logic [p_m10k_addr_bits-1:0] vns_cx_addr;
  logic                        vns_cx_rd_en;

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
  logic latch_dq, latch_rr_new;
  logic latch_alpha, latch_beta;
  logic refresh_rr_reg, init_rr_reg;
  logic bump_iter, reset_iter;

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
  // Avalon slave constant tie-offs and direct routing
  //----------------------------------------------------------------------
  // Q ports: read-only.
  assign q_val_ram_chipselect  = 1'b1;
  assign q_val_ram_clken       = 1'b1;
  assign q_val_ram_write       = 1'b0;
  assign q_val_ram_writedata   = 32'd0;
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

  // c port: read-only from RTL (ARM writes via h2f).
  assign c_ram_chipselect      = 1'b1;
  assign c_ram_clken           = 1'b1;
  assign c_ram_write           = 1'b0;
  assign c_ram_writedata       = 32'd0;
  assign c_ram_byteenable      = 4'b1111;
  assign c_ram_address         = vns_cx_addr;
  logic [31:0] vns_cx_rdata;
  assign vns_cx_rdata          = c_ram_readdata;

  // xy port: read + write.
  assign xy_ram_chipselect     = 1'b1;
  assign xy_ram_clken          = 1'b1;
  assign xy_ram_byteenable     = 4'b1111;
  assign xy_ram_address        = ctrl_xy_addr;
  assign xy_ram_write          = ctrl_xy_wr_en;
  assign xy_ram_writedata      = ctrl_xy_wdata;
  logic [31:0] ctrl_xy_rdata;
  assign ctrl_xy_rdata         = xy_ram_readdata;

  // SPMV ports: just plug through.
  logic [p_m10k_addr_bits-1:0] spmv_q_val_addr;
  logic [p_m10k_addr_bits-1:0] spmv_q_col_addr;
  logic [p_m10k_addr_bits-1:0] spmv_q_rowp_addr;
  assign q_val_ram_address     = spmv_q_val_addr;
  assign q_col_ram_address     = spmv_q_col_addr;
  assign q_rowp_ram_address    = spmv_q_rowp_addr;

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
    .p_m10k_addr_bits (p_m10k_addr_bits)
  ) ctrl (
    .clk,
    .rst,
    .sw_go,
    .sw_done,
    .sw_done_ack,
    .n,
    .max_iter,
    .eps_sq,
    .iter, .rr_new, .rr_old,
    .ctrl_xy_addr, .ctrl_xy_wr_en, .ctrl_xy_wdata,
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
    .p_m10k_addr_bits   (p_m10k_addr_bits)
  ) dpath (
    .clk,
    .rst,
    .n,
    .spmv_q_val_addr,   .spmv_q_val_rdata  (q_val_ram_readdata),
    .spmv_q_col_addr,   .spmv_q_col_rdata  (q_col_ram_readdata),
    .spmv_q_rowp_addr,  .spmv_q_rowp_rdata (q_rowp_ram_readdata),
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
