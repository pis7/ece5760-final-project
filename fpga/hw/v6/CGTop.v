// Toplevel v6 Verilog: parallel-x/y CG solver for DE1-SoC. Two
// CGEngine instances run in lockstep, each owning a private {q_val,
// q_col, q_rowp, c, xy} slave trio. Q is duplicated across two M10K
// trios so the SPMVs run with zero contention. One sw_go pulse fires
// both engines; sw_done = engine_x.done & engine_y.done.

module CGTop #(
  parameter p_lanes            = 8,
  parameter p_max_n            = 50,
  parameter p_int_bits         = 13,
  parameter p_frac_bits        = 14,
  parameter p_total_bits       = p_int_bits + p_frac_bits,
  // 48 keeps the Q13.14 build bit-identical and matches the NR FpDiv's
  // hardcoded 48-bit internals; the wider branch sizes the wide
  // accumulator to fit a full SPMV/dot-product without overflow.
  parameter p_acc_bits         = (p_total_bits <= 27)
      ? 48
      : (2*p_total_bits - p_frac_bits + $clog2(p_max_n+1) + 4),
  parameter p_m10k_addr_bits   = 32,
  // Avalon data-port width for value-carrying slaves (q_val/c/xy). 32
  // keeps the FPGA build identical; widen to 64 in verilated mode when
  // a single Q value no longer fits.
  parameter p_word_bits        = (p_total_bits <= 32) ? 32 : 64
) (
  input  logic clk,
  input  logic rst,

  // ARM control (single go/done/ack pair drives both engines).
  input  logic sw_go,
  output logic sw_done,
  input  logic sw_done_ack,

  // -- x engine's Q-CSR Avalon slaves (read-only) ------------------------
  // q_val carries fixed-point values (p_word_bits wide); q_col / q_rowp
  // carry plain integer indices and stay 32-bit.
  output logic [p_m10k_addr_bits-1:0] q_val_x_ram_address,
  output logic                        q_val_x_ram_chipselect,
  output logic                        q_val_x_ram_clken,
  output logic                        q_val_x_ram_write,
  input  logic [p_word_bits-1:0]      q_val_x_ram_readdata,
  output logic [p_word_bits-1:0]      q_val_x_ram_writedata,
  output logic [3:0]                  q_val_x_ram_byteenable,

  output logic [p_m10k_addr_bits-1:0] q_col_x_ram_address,
  output logic                        q_col_x_ram_chipselect,
  output logic                        q_col_x_ram_clken,
  output logic                        q_col_x_ram_write,
  input  logic [31:0]                 q_col_x_ram_readdata,
  output logic [31:0]                 q_col_x_ram_writedata,
  output logic [3:0]                  q_col_x_ram_byteenable,

  output logic [p_m10k_addr_bits-1:0] q_rowp_x_ram_address,
  output logic                        q_rowp_x_ram_chipselect,
  output logic                        q_rowp_x_ram_clken,
  output logic                        q_rowp_x_ram_write,
  input  logic [31:0]                 q_rowp_x_ram_readdata,
  output logic [31:0]                 q_rowp_x_ram_writedata,
  output logic [3:0]                  q_rowp_x_ram_byteenable,

  // -- y engine's Q-CSR Avalon slaves (read-only) ------------------------
  output logic [p_m10k_addr_bits-1:0] q_val_y_ram_address,
  output logic                        q_val_y_ram_chipselect,
  output logic                        q_val_y_ram_clken,
  output logic                        q_val_y_ram_write,
  input  logic [p_word_bits-1:0]      q_val_y_ram_readdata,
  output logic [p_word_bits-1:0]      q_val_y_ram_writedata,
  output logic [3:0]                  q_val_y_ram_byteenable,

  output logic [p_m10k_addr_bits-1:0] q_col_y_ram_address,
  output logic                        q_col_y_ram_chipselect,
  output logic                        q_col_y_ram_clken,
  output logic                        q_col_y_ram_write,
  input  logic [31:0]                 q_col_y_ram_readdata,
  output logic [31:0]                 q_col_y_ram_writedata,
  output logic [3:0]                  q_col_y_ram_byteenable,

  output logic [p_m10k_addr_bits-1:0] q_rowp_y_ram_address,
  output logic                        q_rowp_y_ram_chipselect,
  output logic                        q_rowp_y_ram_clken,
  output logic                        q_rowp_y_ram_write,
  input  logic [31:0]                 q_rowp_y_ram_readdata,
  output logic [31:0]                 q_rowp_y_ram_writedata,
  output logic [3:0]                  q_rowp_y_ram_byteenable,

  // -- cx_ram (x engine's c-vector, read-only) ---------------------------
  output logic [p_m10k_addr_bits-1:0] cx_ram_address,
  output logic                        cx_ram_chipselect,
  output logic                        cx_ram_clken,
  output logic                        cx_ram_write,
  input  logic [p_word_bits-1:0]      cx_ram_readdata,
  output logic [p_word_bits-1:0]      cx_ram_writedata,
  output logic [3:0]                  cx_ram_byteenable,

  // -- cy_ram (y engine's c-vector, read-only) ---------------------------
  output logic [p_m10k_addr_bits-1:0] cy_ram_address,
  output logic                        cy_ram_chipselect,
  output logic                        cy_ram_clken,
  output logic                        cy_ram_write,
  input  logic [p_word_bits-1:0]      cy_ram_readdata,
  output logic [p_word_bits-1:0]      cy_ram_writedata,
  output logic [3:0]                  cy_ram_byteenable,

  // -- x_ram (x engine's load + writeback) -------------------------------
  output logic [p_m10k_addr_bits-1:0] x_ram_address,
  output logic                        x_ram_chipselect,
  output logic                        x_ram_clken,
  output logic                        x_ram_write,
  input  logic [p_word_bits-1:0]      x_ram_readdata,
  output logic [p_word_bits-1:0]      x_ram_writedata,
  output logic [3:0]                  x_ram_byteenable,

  // -- y_ram (y engine's load + writeback) -------------------------------
  output logic [p_m10k_addr_bits-1:0] y_ram_address,
  output logic                        y_ram_chipselect,
  output logic                        y_ram_clken,
  output logic                        y_ram_write,
  input  logic [p_word_bits-1:0]      y_ram_readdata,
  output logic [p_word_bits-1:0]      y_ram_writedata,
  output logic [3:0]                  y_ram_byteenable,

  // CG solve parameters (broadcast to both engines).
  input  logic [31:0] max_iter,
  input  logic [31:0] eps_sq,
  input  logic [31:0] n
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
  // Per-engine done signals (combined into the single sw_done output).
  //----------------------------------------------------------------------
  logic engine_x_done;
  logic engine_y_done;
  assign sw_done = engine_x_done & engine_y_done;

  //----------------------------------------------------------------------
  // x engine: solves Q*x = -cx, owns q_*_x_ram, cx_ram, x_ram.
  //----------------------------------------------------------------------
  CGEngine #(
    .p_lanes          (p_lanes),
    .p_max_n          (p_max_n),
    .p_int_bits       (p_int_bits),
    .p_frac_bits      (p_frac_bits),
    .p_total_bits     (p_total_bits),
    .p_acc_bits       (p_acc_bits),
    .p_m10k_addr_bits (p_m10k_addr_bits),
    .p_word_bits      (p_word_bits)
  ) engine_x (
    .clk,
    .rst         (rst_q),
    .sw_go       (sw_go_q),
    .sw_done     (engine_x_done),
    .sw_done_ack (sw_done_ack_q),
    .n           (n_q),
    .max_iter    (max_iter_q),
    .eps_sq      (eps_sq_q),

    .q_val_ram_address    (q_val_x_ram_address),
    .q_val_ram_chipselect (q_val_x_ram_chipselect),
    .q_val_ram_clken      (q_val_x_ram_clken),
    .q_val_ram_write      (q_val_x_ram_write),
    .q_val_ram_readdata   (q_val_x_ram_readdata),
    .q_val_ram_writedata  (q_val_x_ram_writedata),
    .q_val_ram_byteenable (q_val_x_ram_byteenable),

    .q_col_ram_address    (q_col_x_ram_address),
    .q_col_ram_chipselect (q_col_x_ram_chipselect),
    .q_col_ram_clken      (q_col_x_ram_clken),
    .q_col_ram_write      (q_col_x_ram_write),
    .q_col_ram_readdata   (q_col_x_ram_readdata),
    .q_col_ram_writedata  (q_col_x_ram_writedata),
    .q_col_ram_byteenable (q_col_x_ram_byteenable),

    .q_rowp_ram_address    (q_rowp_x_ram_address),
    .q_rowp_ram_chipselect (q_rowp_x_ram_chipselect),
    .q_rowp_ram_clken      (q_rowp_x_ram_clken),
    .q_rowp_ram_write      (q_rowp_x_ram_write),
    .q_rowp_ram_readdata   (q_rowp_x_ram_readdata),
    .q_rowp_ram_writedata  (q_rowp_x_ram_writedata),
    .q_rowp_ram_byteenable (q_rowp_x_ram_byteenable),

    .c_ram_address    (cx_ram_address),
    .c_ram_chipselect (cx_ram_chipselect),
    .c_ram_clken      (cx_ram_clken),
    .c_ram_write      (cx_ram_write),
    .c_ram_readdata   (cx_ram_readdata),
    .c_ram_writedata  (cx_ram_writedata),
    .c_ram_byteenable (cx_ram_byteenable),

    .xy_ram_address    (x_ram_address),
    .xy_ram_chipselect (x_ram_chipselect),
    .xy_ram_clken      (x_ram_clken),
    .xy_ram_write      (x_ram_write),
    .xy_ram_readdata   (x_ram_readdata),
    .xy_ram_writedata  (x_ram_writedata),
    .xy_ram_byteenable (x_ram_byteenable)
  );

  //----------------------------------------------------------------------
  // y engine: solves Q*y = -cy, owns q_*_y_ram, cy_ram, y_ram.
  //----------------------------------------------------------------------
  CGEngine #(
    .p_lanes          (p_lanes),
    .p_max_n          (p_max_n),
    .p_int_bits       (p_int_bits),
    .p_frac_bits      (p_frac_bits),
    .p_total_bits     (p_total_bits),
    .p_acc_bits       (p_acc_bits),
    .p_m10k_addr_bits (p_m10k_addr_bits),
    .p_word_bits      (p_word_bits)
  ) engine_y (
    .clk,
    .rst         (rst_q),
    .sw_go       (sw_go_q),
    .sw_done     (engine_y_done),
    .sw_done_ack (sw_done_ack_q),
    .n           (n_q),
    .max_iter    (max_iter_q),
    .eps_sq      (eps_sq_q),

    .q_val_ram_address    (q_val_y_ram_address),
    .q_val_ram_chipselect (q_val_y_ram_chipselect),
    .q_val_ram_clken      (q_val_y_ram_clken),
    .q_val_ram_write      (q_val_y_ram_write),
    .q_val_ram_readdata   (q_val_y_ram_readdata),
    .q_val_ram_writedata  (q_val_y_ram_writedata),
    .q_val_ram_byteenable (q_val_y_ram_byteenable),

    .q_col_ram_address    (q_col_y_ram_address),
    .q_col_ram_chipselect (q_col_y_ram_chipselect),
    .q_col_ram_clken      (q_col_y_ram_clken),
    .q_col_ram_write      (q_col_y_ram_write),
    .q_col_ram_readdata   (q_col_y_ram_readdata),
    .q_col_ram_writedata  (q_col_y_ram_writedata),
    .q_col_ram_byteenable (q_col_y_ram_byteenable),

    .q_rowp_ram_address    (q_rowp_y_ram_address),
    .q_rowp_ram_chipselect (q_rowp_y_ram_chipselect),
    .q_rowp_ram_clken      (q_rowp_y_ram_clken),
    .q_rowp_ram_write      (q_rowp_y_ram_write),
    .q_rowp_ram_readdata   (q_rowp_y_ram_readdata),
    .q_rowp_ram_writedata  (q_rowp_y_ram_writedata),
    .q_rowp_ram_byteenable (q_rowp_y_ram_byteenable),

    .c_ram_address    (cy_ram_address),
    .c_ram_chipselect (cy_ram_chipselect),
    .c_ram_clken      (cy_ram_clken),
    .c_ram_write      (cy_ram_write),
    .c_ram_readdata   (cy_ram_readdata),
    .c_ram_writedata  (cy_ram_writedata),
    .c_ram_byteenable (cy_ram_byteenable),

    .xy_ram_address    (y_ram_address),
    .xy_ram_chipselect (y_ram_chipselect),
    .xy_ram_clken      (y_ram_clken),
    .xy_ram_write      (y_ram_write),
    .xy_ram_readdata   (y_ram_readdata),
    .xy_ram_writedata  (y_ram_writedata),
    .xy_ram_byteenable (y_ram_byteenable)
  );

endmodule
