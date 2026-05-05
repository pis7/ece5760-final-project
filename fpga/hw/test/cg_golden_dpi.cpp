// DPI wrapper around shared CG golden model for older Verilator
// testbenches (v1..v5_deep). The testbench passes a 32-bit `int mem[]`
// because their RTL is locked to Q13.14; the int64-based golden model
// is bridged via narrow/widen conversions at the DPI boundary.
//
// v6 has its own DPI (cg_golden_dpi_v6.cpp) that takes longint mem[]
// for the wider value words.

#include <cstdint>
#include <vector>
#include "svdpi.h"
#include "cg_golden_model.h"

// Solve for a single dimension (cx_base / sol_base select x or y)
static void dpi_cg_solve_dim(int32_t* mp, int n, int max_n,
                              int max_iter, int32_t eps_sq,
                              int cx_base, int sol_base) {
    int q_val_base  = 0;
    int q_col_base  = max_n * max_n;
    int q_rowp_base = 2 * max_n * max_n;

    // Unpack CSR. Values widen 32->64 (sign-extension preserves the
    // existing low-bit value).
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
        Q.vals[j]    = static_cast<int64_t>(mp[q_val_base + j]);
    }

    // Unpack cx and solution vector (32 -> 64).
    std::vector<int64_t> cx(n), x(n);
    for (int i = 0; i < n; ++i) {
        cx[i] = static_cast<int64_t>(mp[cx_base + i]);
        x[i]  = static_cast<int64_t>(mp[sol_base + i]);
    }

    // Run golden CG
    CGGolden::cg_solve(Q, cx, x, max_iter, static_cast<int64_t>(eps_sq));

    // Write solution back (64 -> 32).
    for (int i = 0; i < n; ++i)
        mp[sol_base + i] = static_cast<int32_t>(x[i]);
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
