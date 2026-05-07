// Testbench for v5/v5_deep CGTop (multi-block on-chip RAM topology --
// 7 dedicated slaves: q_val, q_col, q_rowp, cx, cy, x, y).
//
// p_int_bits / p_frac_bits / p_max_n are top-level parameters,
// overridable via -Gp_int_bits / -Gp_frac_bits / -Gp_max_n at simulator
// build time. WORD_BITS up to 64 is supported; the FPGA build stays
// 27-bit.

module CGTop_tb #(
  parameter int p_int_bits  = 13,
  parameter int p_frac_bits = 14,
  parameter int p_max_n     = 50
);

  localparam MAX_N       = p_max_n;
  localparam INT_BITS    = p_int_bits;
  localparam FRAC_BITS   = p_frac_bits;
  localparam TOTAL_BITS  = INT_BITS + FRAC_BITS;
  localparam WORD_BITS   = (TOTAL_BITS <= 32) ? 32 : 64;
  localparam ADDR_BITS   = 32;
  localparam longint FRAC_SCALE = longint'(1) << FRAC_BITS;

  localparam Q_VAL_BASE  = 0;
  localparam Q_COL_BASE  = MAX_N * MAX_N;
  localparam Q_ROWP_BASE = 2 * MAX_N * MAX_N;
  localparam CX_X_BASE   = 2 * MAX_N * MAX_N + MAX_N + 1;
  localparam CX_Y_BASE   = 2 * MAX_N * MAX_N + 2 * MAX_N + 1;
  localparam X_BASE      = 2 * MAX_N * MAX_N + 3 * MAX_N + 1;
  localparam Y_BASE      = 2 * MAX_N * MAX_N + 4 * MAX_N + 1;
  localparam TOTAL_WORDS = 2 * MAX_N * MAX_N + 5 * MAX_N + 1;

  import "DPI-C" function void dpi_cg_solve(
    inout longint mem [],
    input int     n,
    input int     max_n,
    input int     max_iter,
    input longint eps_sq
  );

  logic clk, rst;
  initial clk = 0;
  always #5 clk = ~clk;

  initial begin
    string vcd_file;
    if ($value$plusargs("dump-vcd=%s", vcd_file)) begin
      $display("VCD: dumping to %s", vcd_file);
      $dumpfile(vcd_file);
      $dumpvars(0, CGTop_tb);
    end
  end

  logic        sw_go, sw_done, sw_done_ack;
  logic [31:0] max_iter, eps_sq, n;

  logic [ADDR_BITS-1:0] q_val_addr,  q_col_addr,  q_rowp_addr;
  logic [ADDR_BITS-1:0] cx_addr,     cy_addr;
  logic [ADDR_BITS-1:0] x_addr,      y_addr;
  logic                 q_val_cs,    q_col_cs,    q_rowp_cs;
  logic                 cx_cs,       cy_cs;
  logic                 x_cs,        y_cs;
  logic                 q_val_clken, q_col_clken, q_rowp_clken;
  logic                 cx_clken,    cy_clken;
  logic                 x_clken,     y_clken;
  logic                 q_val_we,    q_col_we,    q_rowp_we;
  logic                 cx_we,       cy_we;
  logic                 x_we,        y_we;
  logic [WORD_BITS-1:0] q_val_rdata;
  logic [31:0]          q_col_rdata, q_rowp_rdata;
  logic [WORD_BITS-1:0] cx_rdata,    cy_rdata;
  logic [WORD_BITS-1:0] x_rdata,     y_rdata;
  logic [WORD_BITS-1:0] q_val_wdata;
  logic [31:0]          q_col_wdata, q_rowp_wdata;
  logic [WORD_BITS-1:0] cx_wdata,    cy_wdata;
  logic [WORD_BITS-1:0] x_wdata,     y_wdata;
  logic [3:0]           q_val_be,    q_col_be,    q_rowp_be;
  logic [3:0]           cx_be,       cy_be;
  logic [3:0]           x_be,        y_be;

  logic signed [WORD_BITS-1:0] q_val_mem  [MAX_N * MAX_N];
  int                          q_col_mem  [MAX_N * MAX_N];
  int                          q_rowp_mem [MAX_N + 1];
  logic signed [WORD_BITS-1:0] cx_mem     [MAX_N];
  logic signed [WORD_BITS-1:0] cy_mem     [MAX_N];
  logic signed [WORD_BITS-1:0] x_mem      [MAX_N];
  logic signed [WORD_BITS-1:0] y_mem      [MAX_N];

  always_ff @(posedge clk) begin
    if (q_val_cs && q_val_clken) begin
      if (q_val_we) q_val_mem[q_val_addr] <= q_val_wdata;
      else          q_val_rdata <= q_val_mem[q_val_addr];
    end
    if (q_col_cs && q_col_clken) begin
      if (q_col_we) q_col_mem[q_col_addr] <= q_col_wdata;
      else          q_col_rdata <= q_col_mem[q_col_addr];
    end
    if (q_rowp_cs && q_rowp_clken) begin
      if (q_rowp_we) q_rowp_mem[q_rowp_addr] <= q_rowp_wdata;
      else           q_rowp_rdata <= q_rowp_mem[q_rowp_addr];
    end
    if (cx_cs && cx_clken) begin
      if (cx_we) cx_mem[cx_addr] <= cx_wdata;
      else       cx_rdata <= cx_mem[cx_addr];
    end
    if (cy_cs && cy_clken) begin
      if (cy_we) cy_mem[cy_addr] <= cy_wdata;
      else       cy_rdata <= cy_mem[cy_addr];
    end
    if (x_cs && x_clken) begin
      if (x_we) x_mem[x_addr] <= x_wdata;
      else      x_rdata <= x_mem[x_addr];
    end
    if (y_cs && y_clken) begin
      if (y_we) y_mem[y_addr] <= y_wdata;
      else      y_rdata <= y_mem[y_addr];
    end
  end

  CGTop #(
    .p_max_n          (MAX_N),
    .p_int_bits       (INT_BITS),
    .p_frac_bits      (FRAC_BITS),
    .p_m10k_addr_bits (ADDR_BITS)
  ) dut (
    .clk         (clk),
    .rst         (rst),
    .sw_go       (sw_go),
    .sw_done     (sw_done),
    .sw_done_ack (sw_done_ack),

    .q_val_ram_address     (q_val_addr),
    .q_val_ram_chipselect  (q_val_cs),
    .q_val_ram_clken       (q_val_clken),
    .q_val_ram_write       (q_val_we),
    .q_val_ram_readdata    (q_val_rdata),
    .q_val_ram_writedata   (q_val_wdata),
    .q_val_ram_byteenable  (q_val_be),

    .q_col_ram_address     (q_col_addr),
    .q_col_ram_chipselect  (q_col_cs),
    .q_col_ram_clken       (q_col_clken),
    .q_col_ram_write       (q_col_we),
    .q_col_ram_readdata    (q_col_rdata),
    .q_col_ram_writedata   (q_col_wdata),
    .q_col_ram_byteenable  (q_col_be),

    .q_rowp_ram_address    (q_rowp_addr),
    .q_rowp_ram_chipselect (q_rowp_cs),
    .q_rowp_ram_clken      (q_rowp_clken),
    .q_rowp_ram_write      (q_rowp_we),
    .q_rowp_ram_readdata   (q_rowp_rdata),
    .q_rowp_ram_writedata  (q_rowp_wdata),
    .q_rowp_ram_byteenable (q_rowp_be),

    .cx_ram_address        (cx_addr),
    .cx_ram_chipselect     (cx_cs),
    .cx_ram_clken          (cx_clken),
    .cx_ram_write          (cx_we),
    .cx_ram_readdata       (cx_rdata),
    .cx_ram_writedata      (cx_wdata),
    .cx_ram_byteenable     (cx_be),

    .cy_ram_address        (cy_addr),
    .cy_ram_chipselect     (cy_cs),
    .cy_ram_clken          (cy_clken),
    .cy_ram_write          (cy_we),
    .cy_ram_readdata       (cy_rdata),
    .cy_ram_writedata      (cy_wdata),
    .cy_ram_byteenable     (cy_be),

    .x_ram_address         (x_addr),
    .x_ram_chipselect      (x_cs),
    .x_ram_clken           (x_clken),
    .x_ram_write           (x_we),
    .x_ram_readdata        (x_rdata),
    .x_ram_writedata       (x_wdata),
    .x_ram_byteenable      (x_be),

    .y_ram_address         (y_addr),
    .y_ram_chipselect      (y_cs),
    .y_ram_clken           (y_clken),
    .y_ram_write           (y_we),
    .y_ram_readdata        (y_rdata),
    .y_ram_writedata       (y_wdata),
    .y_ram_byteenable      (y_be),

    .max_iter (max_iter),
    .eps_sq   (eps_sq),
    .n        (n)
  );

  longint golden_mem [TOTAL_WORDS];

  int test_num;
  int fail_count;

  function automatic longint to_fp(real v);
    return longint'(v * real'(FRAC_SCALE));
  endfunction

  task automatic load_test(
    input int     t_n,
    input int     nnz,
    input int     row_ptr [],
    input int     col_idx [],
    input longint vals    [],
    input longint cx      [],
    input longint x0      [],
    input longint cy      [],
    input longint y0      []
  );
    for (int i = 0; i < MAX_N * MAX_N; i++) begin
      q_val_mem[i] = 0;
      q_col_mem[i] = 0;
    end
    for (int i = 0; i < MAX_N + 1; i++) q_rowp_mem[i] = 0;
    for (int i = 0; i < MAX_N; i++) begin
      cx_mem[i] = 0;
      cy_mem[i] = 0;
      x_mem [i] = 0;
      y_mem [i] = 0;
    end
    for (int i = 0; i < TOTAL_WORDS; i++) golden_mem[i] = 0;

    for (int j = 0; j < nnz; j++) begin
      q_val_mem[j] = vals[j];
      q_col_mem[j] = col_idx[j];
    end
    for (int i = 0; i <= t_n; i++) q_rowp_mem[i] = row_ptr[i];
    for (int i = 0; i < t_n; i++) begin
      cx_mem[i] = cx[i];
      cy_mem[i] = cy[i];
      x_mem [i] = x0[i];
      y_mem [i] = y0[i];
    end

    for (int j = 0; j < nnz; j++) begin
      golden_mem[Q_VAL_BASE + j] = vals[j];
      golden_mem[Q_COL_BASE + j] = col_idx[j];
    end
    for (int i = 0; i <= t_n; i++) golden_mem[Q_ROWP_BASE + i] = row_ptr[i];
    for (int i = 0; i < t_n; i++) begin
      golden_mem[CX_X_BASE + i] = cx[i];
      golden_mem[CX_Y_BASE + i] = cy[i];
      golden_mem[X_BASE    + i] = x0[i];
      golden_mem[Y_BASE    + i] = y0[i];
    end
  endtask

  task automatic run_dut();
    int timeout_cnt;
    @(posedge clk); sw_go = 1;
    @(posedge clk); sw_go = 0;
    timeout_cnt = 0;
    while (!sw_done) begin
      @(posedge clk);
      timeout_cnt++;
      if (timeout_cnt > 2000000) begin
        $display("  TIMEOUT: DUT did not complete");
        $finish;
      end
    end
    @(posedge clk); sw_done_ack = 1;
    @(posedge clk); sw_done_ack = 0;
    repeat(2) @(posedge clk);
  endtask

  task automatic check_vec(
    input int t_n,
    input int gold_base,
    input bit pick_y,
    input string vec_name,
    input string test_name,
    inout int mismatch
  );
    longint dut_v, gold_v;
    real dut_real, gold_real;
    for (int i = 0; i < t_n; i++) begin
      dut_v  = pick_y ? y_mem[i] : x_mem[i];
      gold_v = golden_mem[gold_base + i];
      dut_real  = real'(dut_v)  / real'(FRAC_SCALE);
      gold_real = real'(gold_v) / real'(FRAC_SCALE);
      if (dut_v !== gold_v) begin
        $display("  FAIL %s: %s[%0d] dut=%.6f gold=%.6f (raw dut=%0d gold=%0d diff=%0d)",
                 test_name, vec_name, i, dut_real, gold_real, dut_v, gold_v, dut_v - gold_v);
        mismatch++;
      end else begin
        $display("       %s: %s[%0d] = %.6f (match)", test_name, vec_name, i, dut_real);
      end
    end
  endtask

  task automatic check_results(input int t_n, input string name);
    int mismatch;
    mismatch = 0;
    check_vec(t_n, X_BASE, 1'b0, "x", name, mismatch);
    check_vec(t_n, Y_BASE, 1'b1, "y", name, mismatch);
    if (mismatch == 0)
      $display("  PASS %s: all %0d x+y elements match", name, 2 * t_n);
    else begin
      $display("  FAIL %s: %0d mismatches out of %0d", name, mismatch, 2 * t_n);
      fail_count++;
    end
  endtask

  task automatic run_test(
    input string  name,
    input int     t_n,
    input int     nnz,
    input int     row_ptr [],
    input int     col_idx [],
    input longint vals    [],
    input longint cx      [],
    input longint cy      [],
    input longint x0      [],
    input longint y0      []
  );
    test_num++;
    n = t_n;
    $display("Test %0d: %s", test_num, name);
    load_test(t_n, nnz, row_ptr, col_idx, vals, cx, x0, cy, y0);
    dpi_cg_solve(golden_mem, t_n, MAX_N, max_iter, eps_sq);
    run_dut();
    check_results(t_n, name);
  endtask

  task automatic build_uniform_tridiag(
    input  int     t_n,
    input  longint diag_fp,
    input  longint off_fp,
    output int     row_ptr [],
    output int     col_idx [],
    output longint vals    [],
    output int     nnz
  );
    int idx;
    nnz = 3 * t_n - 2;
    row_ptr = new[t_n + 1];
    col_idx = new[nnz];
    vals    = new[nnz];
    idx = 0;
    for (int i = 0; i < t_n; i++) begin
      row_ptr[i] = idx;
      if (i > 0) begin
        col_idx[idx] = i - 1; vals[idx] = off_fp; idx++;
      end
      col_idx[idx] = i; vals[idx] = diag_fp; idx++;
      if (i < t_n - 1) begin
        col_idx[idx] = i + 1; vals[idx] = off_fp; idx++;
      end
    end
    row_ptr[t_n] = idx;
  endtask

  initial begin
    rst         = 1;
    sw_go       = 0;
    sw_done_ack = 0;
    max_iter    = 100;
    eps_sq      = to_fp(1e-6);
    n           = 0;
    test_num    = 0;
    fail_count  = 0;

    repeat(5) @(posedge clk);
    rst = 0;
    repeat(2) @(posedge clk);

    `include "test_cases.v"

    $display("");
    if (fail_count == 0)
      $display("ALL %0d TESTS PASSED", test_num);
    else
      $display("FAILED: %0d out of %0d tests", fail_count, test_num);

    $finish;
  end

endmodule
