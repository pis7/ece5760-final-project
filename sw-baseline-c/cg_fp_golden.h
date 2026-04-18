// Placer wrapper around CG golden model — converts double ↔ fixed-point
// and provides the same CGHwDriver interface as cg_hw_driver.h (Verilator).

#pragma once

#include <cassert>
#include <cstdint>
#include <vector>

#include "cg_golden.h"

class CGHwDriver {
public:
    static constexpr int MAX_N = 50;

    void solve(const CSRMatrix& Q,
               const std::vector<double>& cx,
               const std::vector<double>& cy,
               std::vector<double>& x,
               std::vector<double>& y,
               int max_iter, double eps) {
        int n = Q.n;
        assert(n <= MAX_N);
        int32_t eps_sq = CGGolden::sign_extend(
            static_cast<int32_t>(eps * eps * CGGolden::FRAC_SCALE));

        // Build int32_t CSR from double CSR
        CGGolden::CSR Q_fp;
        Q_fp.n = n;
        Q_fp.row_ptr = Q.row_ptr;
        Q_fp.col_idx = Q.col_idx;
        Q_fp.vals.resize(Q.vals.size());
        for (size_t j = 0; j < Q.vals.size(); ++j)
            Q_fp.vals[j] = d2fp(Q.vals[j]);

        solve_dim(Q_fp, cx, x, n, max_iter, eps_sq);
        solve_dim(Q_fp, cy, y, n, max_iter, eps_sq);
    }

private:
    static int32_t d2fp(double v) {
        return CGGolden::sign_extend(
            static_cast<int32_t>(v * CGGolden::FRAC_SCALE));
    }

    static double fp2d(int32_t v) {
        return static_cast<double>(CGGolden::sign_extend(v)) /
               CGGolden::FRAC_SCALE;
    }

    static void solve_dim(const CGGolden::CSR& Q_fp,
                          const std::vector<double>& c_d,
                          std::vector<double>& x_d,
                          int n, int max_iter, int32_t eps_sq) {
        std::vector<int32_t> c_fp(n), x_fp(n);
        for (int i = 0; i < n; ++i) {
            c_fp[i] = d2fp(c_d[i]);
            x_fp[i] = d2fp(x_d[i]);
        }

        CGGolden::cg_solve(Q_fp, c_fp, x_fp, max_iter, eps_sq);

        for (int i = 0; i < n; ++i)
            x_d[i] = fp2d(x_fp[i]);
    }
};
