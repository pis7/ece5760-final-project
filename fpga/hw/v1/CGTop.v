// Toplevel V1 Verilog

module CGTop #(
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

  // M10K interface
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
  // Registers and Local Signals
  //----------------------------------------------------------------------

  // M10K read/write go
  logic m10k_rd_go, m10k_wr_go;
  logic m10k_loader_done;

  // Registers
  logic signed [p_total_bits-1:0] cg_data [p_total_words];

  // Wires between loader and register array
  logic        [p_m10k_addr_bits-1:0] loader_reg_addr;
  logic                               loader_reg_wr_en;
  logic signed [p_m10k_addr_bits-1:0] loader_reg_wr_data;
  logic signed [p_m10k_addr_bits-1:0] loader_reg_rd_data;

  logic        [31:0]            iter;
  logic signed [p_acc_bits-1:0]  rr_new;
  logic signed [p_acc_bits-1:0]  rr_old;
  logic do_init, do_run, sel_y;

  // Datapath outputs
  logic signed [p_total_bits-1:0] x_new [p_max_n];
  logic signed [p_acc_bits-1:0]   dq;

  assign loader_reg_rd_data = p_m10k_addr_bits'(cg_data[loader_reg_addr]);
  always_ff @(posedge clk) begin
    if (loader_reg_wr_en)
      cg_data[loader_reg_addr] <= p_total_bits'(loader_reg_wr_data);
    else if( do_run && dq != 0 ) begin
      for( int i = 0; i < p_max_n; i++ ) begin
        if( sel_y )
          cg_data[p_y_base_addr + i] <= x_new[i];
        else
          cg_data[p_x_base_addr + i] <= x_new[i];
      end
    end
  end

  //----------------------------------------------------------------------
  // Control Unit
  //----------------------------------------------------------------------

  CGCtrl #(
    .p_int_bits   (p_int_bits),
    .p_frac_bits  (p_frac_bits),
    .p_total_bits (p_total_bits),
    .p_acc_bits   (p_acc_bits)
  ) ctrl (
    .clk              (clk),
    .rst              (rst),
    .sw_go            (sw_go),
    .sw_done          (sw_done),
    .sw_done_ack      (sw_done_ack),
    .m10k_loader_done (m10k_loader_done),
    .m10k_rd_go       (m10k_rd_go),
    .m10k_wr_go       (m10k_wr_go),
    .iter             (iter),
    .max_iter         (max_iter),
    .rr_new           (rr_new),
    .rr_old           (rr_old),
    .eps_sq           (eps_sq),
    .do_init          (do_init),
    .do_run           (do_run),
    .sel_y            (sel_y)
  );

  M10KLoader #(
    .NUM_WORDS  (p_total_words),
    .DATA_WIDTH (32),
    .ADDR_WIDTH (p_m10k_addr_bits)
  ) loader (
    .clk                    (clk),
    .rst                    (rst),
    .go_read                (m10k_rd_go),
    .go_write               (m10k_wr_go),
    .done                   (m10k_loader_done),
    .on_chip_ram_address    (on_chip_ram_address),
    .on_chip_ram_chipselect (on_chip_ram_chipselect),
    .on_chip_ram_clken      (on_chip_ram_clken),
    .on_chip_ram_write      (on_chip_ram_write),
    .on_chip_ram_readdata   (on_chip_ram_readdata),
    .on_chip_ram_writedata  (on_chip_ram_writedata),
    .on_chip_ram_byteenable (on_chip_ram_byteenable),
    .reg_addr               (loader_reg_addr),
    .reg_wr_en              (loader_reg_wr_en),
    .reg_wr_data            (loader_reg_wr_data),
    .reg_rd_data            (loader_reg_rd_data)
  );

  //----------------------------------------------------------------------
  // Datapath
  //----------------------------------------------------------------------

  CGDpath #(
    .p_max_n            (p_max_n),
    .p_int_bits         (p_int_bits),
    .p_frac_bits        (p_frac_bits),
    .p_total_bits       (p_total_bits),
    .p_acc_bits         (p_acc_bits),
    .p_q_val_base_addr  (p_q_val_base_addr),
    .p_q_col_base_addr  (p_q_col_base_addr),
    .p_q_rowp_base_addr (p_q_rowp_base_addr),
    .p_cx_x_base_addr   (p_cx_x_base_addr),
    .p_cx_y_base_addr   (p_cx_y_base_addr),
    .p_x_base_addr      (p_x_base_addr),
    .p_y_base_addr      (p_y_base_addr),
    .p_total_words      (p_total_words)
  ) dpath (
    .clk     (clk),
    .rst     (rst),
    .do_init (do_init),
    .do_run  (do_run),
    .sel_y   (sel_y),
    .n       (n),
    .cg_data (cg_data),
    .iter    (iter),
    .rr_new  (rr_new),
    .rr_old  (rr_old),
    .x_new   (x_new),
    .dq      (dq)
  );

endmodule
