// -----------------------------------------------------------------------
// Test cases
// -----------------------------------------------------------------------

    // ==== Test 1: 2x2 ====
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

    // ==== Test 9: n=1 ====
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

    // ==== Test 10: 49x49 tridiagonal ====
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

    // ==== Test 11: 10x10 diagonal-only ====
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
      row_ptr[0] = idx;
      col_idx[idx] = 0; vals[idx] = to_fp(20.0); idx++;
      for (int j = 1; j < t_n; j++) begin
        col_idx[idx] = j; vals[idx] = to_fp(1.0); idx++;
      end
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
        diag_v = 10.0 + real'(i % 5);
        off_v  = -(1.0 + real'(i % 3) * 0.125);
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

    // ==== Test 16: 50x50 dense SPD ====
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
