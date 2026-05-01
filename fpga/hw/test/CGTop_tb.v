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

    // ==== Test 1: 2x2 ====
    // Q = [[4,1],[1,3]], cx = [1,2], cy = [2,1], x0/y0 = [0,0]
    begin
      int row_ptr[] = '{0, 2, 4};
      int col_idx[] = '{0, 1, 0, 1};
      int vals[]    = '{to_fp(4.0), to_fp(1.0), to_fp(1.0), to_fp(3.0)};
      int cx[]      = '{to_fp(1.0), to_fp(2.0)};
      int cy[]      = '{to_fp(2.0), to_fp(1.0)};
      int x0[]      = '{0, 0};
      int y0[]      = '{0, 0};
      run_test("2x2", 2, 4, row_ptr, col_idx, vals, cx, cy, x0, y0);
    end

    // ==== Test 2: 3x3 ====
    // Q = [[2,1,0],[1,3,1],[0,1,2]], cx = [1,1,1], cy = [2,0,1]
    begin
      int row_ptr[] = '{0, 2, 5, 7};
      int col_idx[] = '{0, 1, 0, 1, 2, 1, 2};
      int vals[]    = '{to_fp(2.0), to_fp(1.0),
                        to_fp(1.0), to_fp(3.0), to_fp(1.0),
                        to_fp(1.0), to_fp(2.0)};
      int cx[]      = '{to_fp(1.0), to_fp(1.0), to_fp(1.0)};
      int cy[]      = '{to_fp(2.0), to_fp(0.0), to_fp(1.0)};
      int x0[]      = '{0, 0, 0};
      int y0[]      = '{0, 0, 0};
      run_test("3x3", 3, 7, row_ptr, col_idx, vals, cx, cy, x0, y0);
    end

    // ==== Test 3: 2x2 diagonal ====
    // Q = [[5,0],[0,3]], cx = [10,6], cy = [5,9]
    begin
      int row_ptr[] = '{0, 1, 2};
      int col_idx[] = '{0, 1};
      int vals[]    = '{to_fp(5.0), to_fp(3.0)};
      int cx[]      = '{to_fp(10.0), to_fp(6.0)};
      int cy[]      = '{to_fp(5.0), to_fp(9.0)};
      int x0[]      = '{0, 0};
      int y0[]      = '{0, 0};
      run_test("2x2_diag", 2, 2, row_ptr, col_idx, vals, cx, cy, x0, y0);
    end

    // ==== Test 4: 4x4 tridiagonal ====
    // Q = standard 1-D Laplacian, cx = [1,0,0,1], cy = [0,1,1,0]
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx[]      = '{to_fp(1.0), to_fp(0.0), to_fp(0.0), to_fp(1.0)};
      int cy[]      = '{to_fp(0.0), to_fp(1.0), to_fp(1.0), to_fp(0.0)};
      int x0[]      = '{0, 0, 0, 0};
      int y0[]      = '{0, 0, 0, 0};
      t_n = 4;
      build_uniform_tridiag(t_n, to_fp(2.0), to_fp(-1.0), row_ptr, col_idx, vals, nnz);
      run_test("4x4_tridiag", t_n, nnz, row_ptr, col_idx, vals, cx, cy, x0, y0);
    end

    // ==== Test 5: 8x8 tridiagonal ====
    // Same matrix as Test 4, larger n; cx/cy place sources at the ends.
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx[]      = '{to_fp(1.0), to_fp(0.0), to_fp(0.0), to_fp(0.0),
                         to_fp(0.0), to_fp(0.0), to_fp(0.0), to_fp(1.0)};
      int cy[]      = '{to_fp(0.0), to_fp(1.0), to_fp(0.0), to_fp(0.0),
                         to_fp(0.0), to_fp(0.0), to_fp(1.0), to_fp(0.0)};
      int x0[]      = '{0, 0, 0, 0, 0, 0, 0, 0};
      int y0[]      = '{0, 0, 0, 0, 0, 0, 0, 0};
      t_n = 8;
      build_uniform_tridiag(t_n, to_fp(2.0), to_fp(-1.0), row_ptr, col_idx, vals, nnz);
      run_test("8x8_tridiag", t_n, nnz, row_ptr, col_idx, vals, cx, cy, x0, y0);
    end

    // ==== Test 6: 10x10 dense diagonally-dominant ====
    // Q = diag(10..19) + 0.1 off-diagonal everywhere else.
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      t_n = 10;
      nnz = t_n * t_n;
      row_ptr = new[t_n + 1];
      col_idx = new[nnz];
      vals    = new[nnz];
      cx_arr  = new[t_n];
      cy_arr  = new[t_n];
      x0_arr  = new[t_n];
      y0_arr  = new[t_n];
      for (int i = 0; i <= t_n; i++) row_ptr[i] = i * t_n;
      for (int i = 0; i < t_n; i++) begin
        for (int jj = 0; jj < t_n; jj++) begin
          col_idx[i * t_n + jj] = jj;
          vals[i * t_n + jj] = (i == jj) ? to_fp(10.0 + real'(i)) : to_fp(0.1);
        end
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(2.0);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      run_test("10x10_dense", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
    end

    // ==== Test 7: 20x20 tridiagonal ====
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      t_n = 20;
      build_uniform_tridiag(t_n, to_fp(2.0), to_fp(-1.0), row_ptr, col_idx, vals, nnz);
      cx_arr = new[t_n]; cy_arr = new[t_n]; x0_arr = new[t_n]; y0_arr = new[t_n];
      for (int i = 0; i < t_n; i++) begin
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(0.5 + real'(i) * 0.1);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      run_test("20x20_tridiag", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
    end

    // ==== Test 8: 50x50 tridiagonal ====
    // Hits MAX_N. Needs more iterations to converge.
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      t_n = 50;
      build_uniform_tridiag(t_n, to_fp(2.0), to_fp(-1.0), row_ptr, col_idx, vals, nnz);
      cx_arr = new[t_n]; cy_arr = new[t_n]; x0_arr = new[t_n]; y0_arr = new[t_n];
      for (int i = 0; i < t_n; i++) begin
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(0.5);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      max_iter = 200;
      run_test("50x50_tridiag", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
      max_iter = 100;
    end

    // ==== Test 9: n=1 (degenerate single-element) ====
    // Q = [[5.0]], cx = [10], cy = [3]. Solution: x = -2, y = -0.6.
    // Exercises n-1=0 boundary in CGCtrl streaming index logic.
    begin
      int row_ptr[] = '{0, 1};
      int col_idx[] = '{0};
      int vals[]    = '{to_fp(5.0)};
      int cx[]      = '{to_fp(10.0)};
      int cy[]      = '{to_fp(3.0)};
      int x0[]      = '{0};
      int y0[]      = '{0};
      run_test("n1", 1, 1, row_ptr, col_idx, vals, cx, cy, x0, y0);
    end

    // ==== Test 10: 49x49 tridiagonal (one below p_max_n) ====
    // Last group with p_lanes=4 has only 1 valid lane. Tests lane-mask logic.
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      t_n = 49;
      build_uniform_tridiag(t_n, to_fp(2.0), to_fp(-1.0), row_ptr, col_idx, vals, nnz);
      cx_arr = new[t_n]; cy_arr = new[t_n]; x0_arr = new[t_n]; y0_arr = new[t_n];
      for (int i = 0; i < t_n; i++) begin
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(0.5);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      max_iter = 200;
      run_test("49x49_tridiag", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
      max_iter = 100;
    end

    // ==== Test 11: 10x10 diagonal-only (single-nnz per row) ====
    // Maximum sparsity. Stresses SPMV row-boundary transitions on
    // minimum-length rows.
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      t_n = 10;
      nnz = t_n;
      row_ptr = new[t_n + 1];
      col_idx = new[nnz];
      vals    = new[nnz];
      cx_arr  = new[t_n];
      cy_arr  = new[t_n];
      x0_arr  = new[t_n];
      y0_arr  = new[t_n];
      for (int i = 0; i < t_n; i++) begin
        row_ptr[i] = i;
        col_idx[i] = i;
        vals[i]    = to_fp(2.0 + real'(i));
        cx_arr[i]  = to_fp(1.0);
        cy_arr[i]  = to_fp(real'(i + 1) * 0.5);
        x0_arr[i]  = 0;
        y0_arr[i]  = 0;
      end
      row_ptr[t_n] = nnz;
      run_test("10x10_diag_only", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
    end

    // ==== Test 12: 10x10 arrow matrix ====
    // Row 0 has 10 nnz (full); rows 1..9 have 2 nnz each (diag + col 0).
    // Mixes a max-density row with min-density rows. Diagonally dominant -> SPD.
    begin
      int t_n, nnz, idx;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      t_n = 10;
      nnz = t_n + 2 * (t_n - 1);
      row_ptr = new[t_n + 1];
      col_idx = new[nnz];
      vals    = new[nnz];
      cx_arr  = new[t_n];
      cy_arr  = new[t_n];
      x0_arr  = new[t_n];
      y0_arr  = new[t_n];
      idx = 0;
      // Row 0: [20, 1, 1, ..., 1]
      row_ptr[0] = idx;
      col_idx[idx] = 0; vals[idx] = to_fp(20.0); idx++;
      for (int j = 1; j < t_n; j++) begin
        col_idx[idx] = j; vals[idx] = to_fp(1.0); idx++;
      end
      // Rows 1..n-1: [1 in col 0, diag = 5]
      for (int i = 1; i < t_n; i++) begin
        row_ptr[i] = idx;
        col_idx[idx] = 0; vals[idx] = to_fp(1.0); idx++;
        col_idx[idx] = i; vals[idx] = to_fp(5.0); idx++;
      end
      row_ptr[t_n] = idx;
      for (int i = 0; i < t_n; i++) begin
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(real'(i) * 0.25);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      run_test("10x10_arrow", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
    end

    // ==== Test 13: 20x20 random-coefficient tridiagonal ====
    // Tridiag with deterministic per-index varying coefficients.
    // Strictly diagonally dominant -> SPD. Cannot use build_uniform_tridiag
    // because diag/off vary per row.
    begin
      int t_n, nnz, idx;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      real diag_v, off_v;
      t_n = 20;
      nnz = 3 * t_n - 2;
      row_ptr = new[t_n + 1];
      col_idx = new[nnz];
      vals    = new[nnz];
      cx_arr  = new[t_n];
      cy_arr  = new[t_n];
      x0_arr  = new[t_n];
      y0_arr  = new[t_n];
      idx = 0;
      for (int i = 0; i < t_n; i++) begin
        diag_v = 10.0 + real'(i % 5);                 // 10..14
        off_v  = -(1.0 + real'(i % 3) * 0.125);       // ~-1.0..-1.25
        row_ptr[i] = idx;
        if (i > 0) begin
          col_idx[idx] = i - 1; vals[idx] = to_fp(off_v); idx++;
        end
        col_idx[idx] = i; vals[idx] = to_fp(diag_v); idx++;
        if (i < t_n - 1) begin
          col_idx[idx] = i + 1; vals[idx] = to_fp(off_v); idx++;
        end
        cx_arr[i] = to_fp(real'(((i * 7) % 11)) * 0.25 - 1.0);
        cy_arr[i] = to_fp(real'(((i * 13) % 7)) * 0.5 - 1.5);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      row_ptr[t_n] = idx;
      run_test("20x20_rand_tridiag", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
    end

    // ==== Test 14: 8x8 tridiagonal with nonzero x0/y0 ====
    // Q has diag=4, off=-1 (different from Test 5). Starts from a non-zero
    // initial guess. Exercises the SPMV-INIT phase computing Q*x0 for non-
    // trivial x0.
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      t_n = 8;
      build_uniform_tridiag(t_n, to_fp(4.0), to_fp(-1.0), row_ptr, col_idx, vals, nnz);
      cx_arr = new[t_n]; cy_arr = new[t_n]; x0_arr = new[t_n]; y0_arr = new[t_n];
      for (int i = 0; i < t_n; i++) begin
        cx_arr[i] = to_fp(real'(i + 1));
        cy_arr[i] = to_fp(-(real'(i + 1)));
        x0_arr[i] = to_fp(real'(i + 1) * 0.1);
        y0_arr[i] = to_fp(-(real'(i + 1)) * 0.1);
      end
      run_test("8x8_nonzero_x0", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
    end

    // ==== Test 15: 2x2 with x0 near solution ====
    // Q = [[4,1],[1,3]], cx = [1,2]. Exact solution x = -[1/11, 7/11].
    // Start x0 close to solution; should converge in 1-2 iterations.
    begin
      int row_ptr[] = '{0, 2, 4};
      int col_idx[] = '{0, 1, 0, 1};
      int vals[]    = '{to_fp(4.0), to_fp(1.0), to_fp(1.0), to_fp(3.0)};
      int cx[]      = '{to_fp(1.0), to_fp(2.0)};
      int cy[]      = '{to_fp(2.0), to_fp(1.0)};
      int x0[]      = '{to_fp(-0.090), to_fp(-0.640)};
      int y0[]      = '{to_fp(-0.450), to_fp(-0.180)};
      run_test("2x2_near", 2, 4, row_ptr, col_idx, vals, cx, cy, x0, y0);
    end

    // ==== Test 16: 50x50 dense SPD (max density at p_max_n boundary) ====
    // Strictly diagonally dominant: Q[i][i]=100+i, off-diag=0.01.
    // Exercises full-density SPMV inner loop at maximum n.
    begin
      int t_n, nnz;
      int row_ptr [], col_idx [], vals [];
      int cx_arr [], cy_arr [], x0_arr [], y0_arr [];
      t_n = 50;
      nnz = t_n * t_n;
      row_ptr = new[t_n + 1];
      col_idx = new[nnz];
      vals    = new[nnz];
      cx_arr  = new[t_n];
      cy_arr  = new[t_n];
      x0_arr  = new[t_n];
      y0_arr  = new[t_n];
      for (int i = 0; i <= t_n; i++) row_ptr[i] = i * t_n;
      for (int i = 0; i < t_n; i++) begin
        for (int jj = 0; jj < t_n; jj++) begin
          col_idx[i * t_n + jj] = jj;
          vals[i * t_n + jj] = (i == jj) ? to_fp(100.0 + real'(i)) : to_fp(0.01);
        end
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(real'(i) * 0.05 - 0.5);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      max_iter = 200;
      run_test("50x50_dense", t_n, nnz, row_ptr, col_idx, vals, cx_arr, cy_arr, x0_arr, y0_arr);
      max_iter = 100;
    end

    // Summary
    $display("");
    if (fail_count == 0)
      $display("ALL %0d TESTS PASSED", test_num);
    else
      $display("FAILED: %0d out of %0d tests", fail_count, test_num);

    $finish;
  end

endmodule
