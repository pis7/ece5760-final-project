// Testbench for CGTop
// - M10K RAM shim responds to CGTop's M10K interface
// - DPI call to golden-reference C++ CG solver for comparison

module CGTop_tb;

  // Parameters (must match CGTop defaults)
  localparam MAX_N       = 50;
  localparam INT_BITS    = 13;
  localparam FRAC_BITS   = 14;
  localparam ADDR_BITS   = 32;
  localparam FRAC_SCALE  = 1 << FRAC_BITS;

  // Memory layout
  localparam Q_VAL_BASE  = 0;
  localparam Q_COL_BASE  = MAX_N * MAX_N;
  localparam Q_ROWP_BASE = 2 * MAX_N * MAX_N;
  localparam CX_X_BASE   = 2 * MAX_N * MAX_N + MAX_N + 1;
  localparam CX_Y_BASE   = 2 * MAX_N * MAX_N + 2 * MAX_N + 1;
  localparam X_BASE      = 2 * MAX_N * MAX_N + 3 * MAX_N + 1;
  localparam Y_BASE      = 2 * MAX_N * MAX_N + 4 * MAX_N + 1;
  localparam TOTAL_WORDS = 2 * MAX_N * MAX_N + 5 * MAX_N + 1;

  // DPI import
  import "DPI-C" function void dpi_cg_solve(
    inout int mem [],
    input int n,
    input int max_n,
    input int max_iter,
    input int eps_sq
  );

  // Clock and reset
  logic clk, rst;
  initial clk = 0;
  always #5 clk = ~clk;

  // Optional VCD dump. Pass `+vcd=<path>` on the simulator command line
  // to enable; with no plusarg, no waveform is written. Verilator must
  // be built with --trace (set in fpga/hw/test/CMakeLists.txt) for this
  // to do anything.
  initial begin
    string vcd_file;
    if ($value$plusargs("dump-vcd=%s", vcd_file)) begin
      $display("VCD: dumping to %s", vcd_file);
      $dumpfile(vcd_file);
      $dumpvars(0, CGTop_tb);
    end
  end

  // DUT signals
  logic        sw_go, sw_done, sw_done_ack;
  logic [31:0] max_iter, eps_sq, n;

  logic [ADDR_BITS-1:0] ram_address;
  logic                 ram_chipselect, ram_clken, ram_write;
  logic [31:0]          ram_readdata, ram_writedata;
  logic [3:0]           ram_byteenable;

  // M10K RAM shim
  int m10k_mem [TOTAL_WORDS];
  always_ff @(posedge clk) begin
    if (ram_chipselect && ram_clken) begin
      if (ram_write)
        m10k_mem[ram_address] <= ram_writedata;
      else
        ram_readdata <= m10k_mem[ram_address];
    end
  end

  // DUT
  CGTop #(
    .p_max_n          (MAX_N),
    .p_int_bits       (INT_BITS),
    .p_frac_bits      (FRAC_BITS),
    .p_m10k_addr_bits (ADDR_BITS)
  ) dut (
    .clk                    (clk),
    .rst                    (rst),
    .sw_go                  (sw_go),
    .sw_done                (sw_done),
    .sw_done_ack            (sw_done_ack),
    .on_chip_ram_address    (ram_address),
    .on_chip_ram_chipselect (ram_chipselect),
    .on_chip_ram_clken      (ram_clken),
    .on_chip_ram_write      (ram_write),
    .on_chip_ram_readdata   (ram_readdata),
    .on_chip_ram_writedata  (ram_writedata),
    .on_chip_ram_byteenable (ram_byteenable),
    .max_iter               (max_iter),
    .eps_sq                 (eps_sq),
    .n                      (n)
  );

  // Golden reference memory (separate copy)
  int golden_mem [TOTAL_WORDS];

  // Test infrastructure
  int test_num;
  int fail_count;

  // ----------------------------------------------------------------------
  // Helpers
  // ----------------------------------------------------------------------

  // Fixed-point conversion
  function int to_fp(real v);
    return int'(v * real'(FRAC_SCALE));
  endfunction

  // Load a CSR matrix + vectors into both m10k_mem and golden_mem.
  task automatic load_test(
    input int t_n,
    input int nnz,
    input int row_ptr [],
    input int col_idx [],
    input int vals    [],
    input int cx      [],
    input int x0      [],
    input int cy      [],
    input int y0      []
  );
    for (int i = 0; i < TOTAL_WORDS; i++) begin
      m10k_mem[i]   = 0;
      golden_mem[i] = 0;
    end
    for (int j = 0; j < nnz; j++) begin
      m10k_mem  [Q_VAL_BASE + j] = vals[j];
      golden_mem[Q_VAL_BASE + j] = vals[j];
      m10k_mem  [Q_COL_BASE + j] = col_idx[j];
      golden_mem[Q_COL_BASE + j] = col_idx[j];
    end
    for (int i = 0; i <= t_n; i++) begin
      m10k_mem  [Q_ROWP_BASE + i] = row_ptr[i];
      golden_mem[Q_ROWP_BASE + i] = row_ptr[i];
    end
    for (int i = 0; i < t_n; i++) begin
      m10k_mem  [CX_X_BASE + i] = cx[i];
      golden_mem[CX_X_BASE + i] = cx[i];
      m10k_mem  [CX_Y_BASE + i] = cy[i];
      golden_mem[CX_Y_BASE + i] = cy[i];
      m10k_mem  [X_BASE    + i] = x0[i];
      golden_mem[X_BASE    + i] = x0[i];
      m10k_mem  [Y_BASE    + i] = y0[i];
      golden_mem[Y_BASE    + i] = y0[i];
    end
  endtask

  // Drive go, wait for done (with 2M-cycle timeout), ack, and return to IDLE.
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

  // Compare a single solution vector (x or y) in DUT vs golden memory.
  task automatic check_vec(
    input int t_n,
    input int base,
    input string vec_name,
    input string test_name,
    inout int mismatch
  );
    int dut_v, gold_v;
    real dut_real, gold_real;
    for (int i = 0; i < t_n; i++) begin
      dut_v  = m10k_mem  [base + i];
      gold_v = golden_mem[base + i];
      dut_real  = $itor(dut_v)  / real'(FRAC_SCALE);
      gold_real = $itor(gold_v) / real'(FRAC_SCALE);
      if (dut_v !== gold_v) begin
        $display("  FAIL %s: %s[%0d] dut=%.6f gold=%.6f (raw dut=%0d gold=%0d diff=%0d)",
                 test_name, vec_name, i, dut_real, gold_real, dut_v, gold_v, dut_v - gold_v);
        mismatch++;
      end else begin
        $display("       %s: %s[%0d] = %.6f (match)", test_name, vec_name, i, dut_real);
      end
    end
  endtask

  // Compare DUT solution (m10k_mem) vs golden (golden_mem) for both dims.
  task automatic check_results(input int t_n, input string name);
    int mismatch;
    mismatch = 0;
    check_vec(t_n, X_BASE, "x", name, mismatch);
    check_vec(t_n, Y_BASE, "y", name, mismatch);
    if (mismatch == 0)
      $display("  PASS %s: all %0d x+y elements match", name, 2 * t_n);
    else begin
      $display("  FAIL %s: %0d mismatches out of %0d", name, mismatch, 2 * t_n);
      fail_count++;
    end
  endtask

  // Run a full test: load matrix+vectors, invoke golden, drive DUT, compare.
  task automatic run_test(
    input string name,
    input int    t_n,
    input int    nnz,
    input int    row_ptr [],
    input int    col_idx [],
    input int    vals    [],
    input int    cx      [],
    input int    cy      [],
    input int    x0      [],
    input int    y0      []
  );
    test_num++;
    n = t_n;
    $display("Test %0d: %s", test_num, name);
    load_test(t_n, nnz, row_ptr, col_idx, vals, cx, x0, cy, y0);
    dpi_cg_solve(golden_mem, t_n, MAX_N, max_iter, eps_sq);
    run_dut();
    check_results(t_n, name);
  endtask

  // Build a uniform tridiagonal CSR: diag_fp on the main diagonal, off_fp on
  // both first off-diagonals. Allocates and writes the row_ptr/col_idx/vals
  // dynamic arrays and returns nnz.
  task automatic build_uniform_tridiag(
    input  int t_n,
    input  int diag_fp,
    input  int off_fp,
    output int row_ptr [],
    output int col_idx [],
    output int vals    [],
    output int nnz
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

  // -----------------------------------------------------------------------
  // Test cases
  // -----------------------------------------------------------------------

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

    // Summary
    $display("");
    if (fail_count == 0)
      $display("ALL %0d TESTS PASSED", test_num);
    else
      $display("FAILED: %0d out of %0d tests", fail_count, test_num);

    $finish;
  end

endmodule
