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

  // Helper: load a CSR matrix + vectors into both m10k_mem and golden_mem
  task load_test(
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
    // Clear both memories
    for (int i = 0; i < TOTAL_WORDS; i++) begin
      m10k_mem[i]   = 0;
      golden_mem[i] = 0;
    end

    // Q vals
    for (int j = 0; j < nnz; j++) begin
      m10k_mem  [Q_VAL_BASE + j] = vals[j];
      golden_mem[Q_VAL_BASE + j] = vals[j];
    end

    // Q col_idx
    for (int j = 0; j < nnz; j++) begin
      m10k_mem  [Q_COL_BASE + j] = col_idx[j];
      golden_mem[Q_COL_BASE + j] = col_idx[j];
    end

    // Q row_ptr
    for (int i = 0; i <= t_n; i++) begin
      m10k_mem  [Q_ROWP_BASE + i] = row_ptr[i];
      golden_mem[Q_ROWP_BASE + i] = row_ptr[i];
    end

    // cx_x
    for (int i = 0; i < t_n; i++) begin
      m10k_mem  [CX_X_BASE + i] = cx[i];
      golden_mem[CX_X_BASE + i] = cx[i];
    end

    // cx_y
    for (int i = 0; i < t_n; i++) begin
      m10k_mem  [CX_Y_BASE + i] = cy[i];
      golden_mem[CX_Y_BASE + i] = cy[i];
    end

    // x0
    for (int i = 0; i < t_n; i++) begin
      m10k_mem  [X_BASE + i] = x0[i];
      golden_mem[X_BASE + i] = x0[i];
    end

    // y0
    for (int i = 0; i < t_n; i++) begin
      m10k_mem  [Y_BASE + i] = y0[i];
      golden_mem[Y_BASE + i] = y0[i];
    end
  endtask

  // Helper: run DUT to completion
  task run_dut();
    // Assert go
    @(posedge clk);
    sw_go = 1;
    @(posedge clk);
    sw_go = 0;

    // Wait for done with timeout
    begin
      int timeout_cnt;
      timeout_cnt = 0;
      while (!sw_done) begin
        @(posedge clk);
        timeout_cnt++;
        if (timeout_cnt > 2000000) begin
          $display("  TIMEOUT: DUT did not complete");
          $finish;
        end
      end
    end

    // Acknowledge done
    @(posedge clk);
    sw_done_ack = 1;
    @(posedge clk);
    sw_done_ack = 0;

    // Let state machine return to IDLE
    repeat(2) @(posedge clk);
  endtask

  // Helper: compare a single vector (x or y) in DUT vs golden
  task check_vec(
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

  // Helper: compare DUT result (in m10k_mem) vs golden (in golden_mem)
  task check_results(input int t_n, input string name);
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

  // -----------------------------------------------------------------------
  // Test cases
  // -----------------------------------------------------------------------

  // Fixed-point conversion helper
  function int to_fp(real v);
    return int'(v * real'(FRAC_SCALE));
  endfunction

  initial begin
    // Init
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

      test_num++;
      n = 2;
      $display("Test %0d: 2x2 system", test_num);

      load_test(2, 4, row_ptr, col_idx, vals, cx, x0, cy, y0);

      dpi_cg_solve(golden_mem, 2, MAX_N, max_iter, eps_sq);

      run_dut();

      check_results(2, "2x2");
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

      test_num++;
      n = 3;
      $display("Test %0d: 3x3 system", test_num);

      load_test(3, 7, row_ptr, col_idx, vals, cx, x0, cy, y0);

      dpi_cg_solve(golden_mem, 3, MAX_N, max_iter, eps_sq);

      run_dut();

      check_results(3, "3x3");
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

      test_num++;
      n = 2;
      $display("Test %0d: 2x2 diagonal", test_num);

      load_test(2, 2, row_ptr, col_idx, vals, cx, x0, cy, y0);

      dpi_cg_solve(golden_mem, 2, MAX_N, max_iter, eps_sq);

      run_dut();

      check_results(2, "2x2_diag");
    end

    // ==== Test 4: 4x4 tridiagonal ====
    // Q = [[2,-1,0,0],[-1,2,-1,0],[0,-1,2,-1],[0,0,-1,2]]
    // cx = [1,0,0,1], cy = [0,1,1,0]
    begin
      int row_ptr[] = '{0, 2, 5, 8, 10};
      int col_idx[] = '{0, 1, 0, 1, 2, 1, 2, 3, 2, 3};
      int vals[]    = '{to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0)};
      int cx[]      = '{to_fp(1.0), to_fp(0.0), to_fp(0.0), to_fp(1.0)};
      int cy[]      = '{to_fp(0.0), to_fp(1.0), to_fp(1.0), to_fp(0.0)};
      int x0[]      = '{0, 0, 0, 0};
      int y0[]      = '{0, 0, 0, 0};

      test_num++;
      n = 4;
      $display("Test %0d: 4x4 tridiagonal", test_num);

      load_test(4, 10, row_ptr, col_idx, vals, cx, x0, cy, y0);

      dpi_cg_solve(golden_mem, 4, MAX_N, max_iter, eps_sq);

      run_dut();

      check_results(4, "4x4_tridiag");
    end

    // ==== Test 5: 8x8 tridiagonal ====
    begin
      int row_ptr[] = '{0, 2, 5, 8, 11, 14, 17, 20, 22};
      int col_idx[] = '{0,1, 0,1,2, 1,2,3, 2,3,4, 3,4,5, 4,5,6, 5,6,7, 6,7};
      int vals[]    = '{to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0), to_fp(-1.0),
                        to_fp(-1.0), to_fp(2.0)};
      int cx[]      = '{to_fp(1.0), to_fp(0.0), to_fp(0.0), to_fp(0.0),
                         to_fp(0.0), to_fp(0.0), to_fp(0.0), to_fp(1.0)};
      int cy[]      = '{to_fp(0.0), to_fp(1.0), to_fp(0.0), to_fp(0.0),
                         to_fp(0.0), to_fp(0.0), to_fp(1.0), to_fp(0.0)};
      int x0[]      = '{0, 0, 0, 0, 0, 0, 0, 0};
      int y0[]      = '{0, 0, 0, 0, 0, 0, 0, 0};

      test_num++;
      n = 8;
      $display("Test %0d: 8x8 tridiagonal", test_num);

      load_test(8, 22, row_ptr, col_idx, vals, cx, x0, cy, y0);

      dpi_cg_solve(golden_mem, 8, MAX_N, max_iter, eps_sq);

      run_dut();

      check_results(8, "8x8_tridiag");
    end

    // ==== Test 6: 10x10 dense SPD ====
    // Q = diag(10..19) + 0.1 off-diagonal
    begin
      int t_n, nnz;
      int row_ptr [];
      int col_idx [];
      int vals    [];
      int cx_arr  [];
      int cy_arr  [];
      int x0_arr  [];
      int y0_arr  [];

      t_n = 10;
      nnz = t_n * t_n;
      row_ptr = new[t_n + 1];
      col_idx = new[nnz];
      vals    = new[nnz];
      cx_arr  = new[t_n];
      cy_arr  = new[t_n];
      x0_arr  = new[t_n];
      y0_arr  = new[t_n];

      for (int i = 0; i <= t_n; i++)
        row_ptr[i] = i * t_n;

      for (int i = 0; i < t_n; i++) begin
        for (int jj = 0; jj < t_n; jj++) begin
          col_idx[i * t_n + jj] = jj;
          if (i == jj)
            vals[i * t_n + jj] = to_fp(10.0 + real'(i));
          else
            vals[i * t_n + jj] = to_fp(0.1);
        end
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(2.0);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end

      test_num++;
      n = t_n;
      $display("Test %0d: 10x10 dense diag-dominant", test_num);

      load_test(t_n, nnz, row_ptr, col_idx, vals, cx_arr, x0_arr, cy_arr, y0_arr);

      dpi_cg_solve(golden_mem, t_n, MAX_N, max_iter, eps_sq);

      run_dut();

      check_results(t_n, "10x10_dense");
    end

    // ==== Test 7: 20x20 tridiagonal ====
    begin
      int t_n, nnz, idx;
      int row_ptr [];
      int col_idx [];
      int vals    [];
      int cx_arr  [];
      int cy_arr  [];
      int x0_arr  [];
      int y0_arr  [];

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
        row_ptr[i] = idx;
        if (i > 0) begin
          col_idx[idx] = i - 1;
          vals[idx] = to_fp(-1.0);
          idx++;
        end
        col_idx[idx] = i;
        vals[idx] = to_fp(2.0);
        idx++;
        if (i < t_n - 1) begin
          col_idx[idx] = i + 1;
          vals[idx] = to_fp(-1.0);
          idx++;
        end
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(0.5 + real'(i) * 0.1);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      row_ptr[t_n] = idx;

      test_num++;
      n = t_n;
      $display("Test %0d: 20x20 tridiagonal", test_num);

      load_test(t_n, nnz, row_ptr, col_idx, vals, cx_arr, x0_arr, cy_arr, y0_arr);

      dpi_cg_solve(golden_mem, t_n, MAX_N, max_iter, eps_sq);

      run_dut();

      check_results(t_n, "20x20_tridiag");
    end

    // ==== Test 8: 50x50 tridiagonal ====
    begin
      int t_n, nnz, idx;
      int row_ptr [];
      int col_idx [];
      int vals    [];
      int cx_arr  [];
      int cy_arr  [];
      int x0_arr  [];
      int y0_arr  [];

      t_n = 50;
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
        row_ptr[i] = idx;
        if (i > 0) begin
          col_idx[idx] = i - 1;
          vals[idx] = to_fp(-1.0);
          idx++;
        end
        col_idx[idx] = i;
        vals[idx] = to_fp(2.0);
        idx++;
        if (i < t_n - 1) begin
          col_idx[idx] = i + 1;
          vals[idx] = to_fp(-1.0);
          idx++;
        end
        cx_arr[i] = to_fp(1.0);
        cy_arr[i] = to_fp(0.5);
        x0_arr[i] = 0;
        y0_arr[i] = 0;
      end
      row_ptr[t_n] = idx;

      test_num++;
      n = t_n;
      max_iter = 200;
      $display("Test %0d: 50x50 tridiagonal", test_num);

      load_test(t_n, nnz, row_ptr, col_idx, vals, cx_arr, x0_arr, cy_arr, y0_arr);

      dpi_cg_solve(golden_mem, t_n, MAX_N, max_iter, eps_sq);

      run_dut();

      check_results(t_n, "50x50_tridiag");

      max_iter = 100;  // restore default
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
