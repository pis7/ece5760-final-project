// Toplevel v4 Verilog: synthesizable CG solver for DE1-SoC.
//
// v4 == v3 + parallel x/r AXPY. Two AXPY units inside CGDpath fire in
// lockstep during the merged S_AXPY_XR_FEED state, halving the cycles
// spent on x/r updates per CG iteration. Cycle savings per iter:
// num_groups = ceil(n/p_lanes). Cost: +p_lanes DSPs.
//
// External (FPGATop-facing) ports are unchanged from v3.

module CGTop #(
  parameter p_lanes            = 8,
  parameter p_max_n            = 50,
  parameter p_int_bits         = 13,
  parameter p_frac_bits        = 14,
  parameter p_total_bits       = p_int_bits + p_frac_bits,
  parameter p_acc_bits         = 48,
  parameter p_m10k_addr_bits   = 32,
  parameter p_q_val_base_addr  = 0,
  parameter p_q_col_base_addr  = p_max_n * p_max_n,
  parameter p_q_rowp_base_addr = 2 * p_max_n * p_max_n,
  parameter p_cx_x_base_addr   = 2 * p_max_n * p_max_n + p_max_n + 1,
  parameter p_cx_y_base_addr   = 2 * p_max_n * p_max_n + 2 * p_max_n + 1,
  parameter p_x_base_addr      = 2 * p_max_n * p_max_n + 3 * p_max_n + 1,
  parameter p_y_base_addr      = 2 * p_max_n * p_max_n + 4 * p_max_n + 1,
  parameter p_total_words      = 2 * p_max_n * p_max_n + 5 * p_max_n + 1
) (
  input  logic clk,
  input  logic rst,

  // ARM control
  input  logic sw_go,
  output logic sw_done,
  input  logic sw_done_ack,

  // Avalon slave interface to Qsys on-chip SRAM
  output logic [p_m10k_addr_bits-1:0] on_chip_ram_address,
  output logic                        on_chip_ram_chipselect,
  output logic                        on_chip_ram_clken,
  output logic                        on_chip_ram_write,
  input  logic [31:0]                 on_chip_ram_readdata,
  output logic [31:0]                 on_chip_ram_writedata,
  output logic [3:0]                  on_chip_ram_byteenable,

  // CG solve parameters
  input [31:0] max_iter,
  input [31:0] eps_sq,
  input [31:0] n
);

  //----------------------------------------------------------------------
  // PIO input registration (same as v3)
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

  logic [p_m10k_addr_bits-1:0] ctrl_mem_addr;
  logic                        ctrl_mem_wr_en;
  logic [31:0]                 ctrl_mem_wdata;
  logic                        ctrl_mem_src_spmv;

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
  // Avalon slave pass-through
  //----------------------------------------------------------------------

  logic [p_m10k_addr_bits-1:0] dp_mem_addr;
  logic                        dp_mem_wr_en;
  logic [31:0]                 dp_mem_wdata;

  assign on_chip_ram_address    = dp_mem_addr;
  assign on_chip_ram_chipselect = 1'b1;
  assign on_chip_ram_clken      = 1'b1;
  assign on_chip_ram_write      = dp_mem_wr_en;
  assign on_chip_ram_writedata  = dp_mem_wdata;
  assign on_chip_ram_byteenable = 4'b1111;

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
    .p_cx_x_base_addr (p_cx_x_base_addr),
    .p_cx_y_base_addr (p_cx_y_base_addr),
    .p_x_base_addr    (p_x_base_addr),
    .p_y_base_addr    (p_y_base_addr)
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
    .ctrl_mem_addr, .ctrl_mem_wr_en, .ctrl_mem_wdata, .ctrl_mem_src_spmv,
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
    .p_q_val_base_addr  (p_q_val_base_addr),
    .p_q_col_base_addr  (p_q_col_base_addr),
    .p_q_rowp_base_addr (p_q_rowp_base_addr)
  ) dpath (
    .clk,
    .rst (rst_q),
    .n   (n_q),
    .mem_addr (dp_mem_addr),
    .mem_wr_en(dp_mem_wr_en),
    .mem_wdata(dp_mem_wdata),
    .mem_rdata(on_chip_ram_readdata),
    .ctrl_mem_addr, .ctrl_mem_wr_en, .ctrl_mem_wdata, .ctrl_mem_src_spmv,
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
