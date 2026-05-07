// DPI wrapper around shared CG golden model. Used by every Verilator
// testbench (v1..v6). Memory is the v4-layout contiguous flat array,
// stored as longint so it carries up to 64-bit fixed-point values
// uniformly; q_col / q_rowp slots store plain integers in the low 32
// bits of their longint cell.

#include <cstdint>
#include <vector>
#include "svdpi.h"
#include "cg_golden_model.h"

static void dpi_cg_solve_dim(int64_t* mp, int n, int max_n,
                              int max_iter, int64_t eps_sq,
                              int cx_base, int sol_base) {
    int q_val_base  = 0;
    int q_col_base  = max_n * max_n;
    int q_rowp_base = 2 * max_n * max_n;

    CGGolden::CSR Q;
    Q.n = n;
    Q.row_ptr.resize(n + 1);
    for (int i = 0; i <= n; ++i)
        Q.row_ptr[i] = static_cast<int32_t>(mp[q_rowp_base + i]);

    int nnz = Q.row_ptr[n];
    Q.col_idx.resize(nnz);
    Q.vals.resize(nnz);
    for (int j = 0; j < nnz; ++j) {
        Q.col_idx[j] = static_cast<int32_t>(mp[q_col_base + j]);
        Q.vals[j]    = mp[q_val_base + j];
    }

    std::vector<int64_t> cx(n), x(n);
    for (int i = 0; i < n; ++i) {
        cx[i] = mp[cx_base + i];
        x[i]  = mp[sol_base + i];
    }

    CGGolden::cg_solve(Q, cx, x, max_iter, eps_sq);

    for (int i = 0; i < n; ++i)
        mp[sol_base + i] = x[i];
}

extern "C" void dpi_cg_solve(
    const svOpenArrayHandle mem,
    int n,
    int max_n,
    int max_iter,
    long long eps_sq
) {
    int64_t* mp = (int64_t*)svGetArrayPtr(mem);
    int cx_x_base = 2 * max_n * max_n + max_n + 1;
    int cx_y_base = 2 * max_n * max_n + 2 * max_n + 1;
    int x_base    = 2 * max_n * max_n + 3 * max_n + 1;
    int y_base    = 2 * max_n * max_n + 4 * max_n + 1;

    dpi_cg_solve_dim(mp, n, max_n, max_iter, eps_sq, cx_x_base, x_base);
    dpi_cg_solve_dim(mp, n, max_n, max_iter, eps_sq, cx_y_base, y_base);
}
