// DPI wrapper around shared CG golden model for Verilator testbench.
// Unpacks the flat M10K memory layout, runs cg_solve, writes results back.

#include <cstdint>
#include <vector>
#include "svdpi.h"
#include "cg_golden.h"

// Solve for a single dimension (cx_base / sol_base select x or y)
static void dpi_cg_solve_dim(int32_t* mp, int n, int max_n,
                              int max_iter, int32_t eps_sq,
                              int cx_base, int sol_base) {
    int q_val_base  = 0;
    int q_col_base  = max_n * max_n;
    int q_rowp_base = 2 * max_n * max_n;

    // Unpack CSR
    CGGolden::CSR Q;
    Q.n = n;
    Q.row_ptr.resize(n + 1);
    for (int i = 0; i <= n; ++i)
        Q.row_ptr[i] = mp[q_rowp_base + i];

    int nnz = Q.row_ptr[n];
    Q.col_idx.resize(nnz);
    Q.vals.resize(nnz);
    for (int j = 0; j < nnz; ++j) {
        Q.col_idx[j] = mp[q_col_base + j];
        Q.vals[j]    = mp[q_val_base + j];
    }

    // Unpack cx and solution vector
    std::vector<int32_t> cx(n), x(n);
    for (int i = 0; i < n; ++i) {
        cx[i] = mp[cx_base + i];
        x[i]  = mp[sol_base + i];
    }

    // Run golden CG
    CGGolden::cg_solve(Q, cx, x, max_iter, eps_sq);

    // Write solution back
    for (int i = 0; i < n; ++i)
        mp[sol_base + i] = x[i];
}

extern "C" void dpi_cg_solve(
    const svOpenArrayHandle mem,  // int32_t mem[total_words]
    int n,
    int max_n,
    int max_iter,
    int eps_sq
) {
    int32_t* mp = (int32_t*)svGetArrayPtr(mem);
    int cx_x_base = 2 * max_n * max_n + max_n + 1;
    int cx_y_base = 2 * max_n * max_n + 2 * max_n + 1;
    int x_base    = 2 * max_n * max_n + 3 * max_n + 1;
    int y_base    = 2 * max_n * max_n + 4 * max_n + 1;

    // Solve for x, then y (matches DUT sequencing)
    dpi_cg_solve_dim(mp, n, max_n, max_iter, eps_sq, cx_x_base, x_base);
    dpi_cg_solve_dim(mp, n, max_n, max_iter, eps_sq, cx_y_base, y_base);
}
