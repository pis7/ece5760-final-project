// Placer-facing wrapper around the bit-exact golden CG model. Converts
// double <-> fixed-point and exposes the same CGHwDriver interface as
// cg_verilator_driver.h and cg_fpga_mmap_driver.h, so the placer can swap
// backends via cmake macros (see placer.cpp's include block).

#pragma once

#include <cassert>
#include <cstdint>
#include <vector>

#include "cg_golden_model.h"

class CGHwDriver {
public:
    static constexpr int MAX_N = 50;

    // Public mirrors -- canonical storage for the placer's working
    // state. Same accessor surface as the verilator / mmap drivers so
    // placer.cpp can be backend-agnostic. The golden model has no
    // SRAM, so the setters just update the mirror.
    std::vector<double> c_x, c_y, x_pos, y_pos;
    CSRMatrix Q;

    void resize_n(int n) {
        c_x.assign(n, 0.0);
        c_y.assign(n, 0.0);
        x_pos.assign(n, 0.0);
        y_pos.assign(n, 0.0);
    }

    void set_cx(int i, double v) { c_x[i] = v; }
    void set_cy(int i, double v) { c_y[i] = v; }
    void set_x (int i, double v) { x_pos[i] = v; }
    void set_y (int i, double v) { y_pos[i] = v; }

    // Same surface as the SRAM-backed drivers. Caller must ensure src
    // has a diagonal entry in every row (csr_add_diagonal with
    // alpha=0 covers it).
    void load_q_initial(const CSRMatrix& src) {
        Q = src;
        q_diag_pos_.assign(Q.n, -1);
        for (int i = 0; i < Q.n; ++i) {
            for (int j = Q.row_ptr[i]; j < Q.row_ptr[i + 1]; ++j) {
                if (Q.col_idx[j] == i) {
                    q_diag_pos_[i] = j;
                    break;
                }
            }
        }
    }

    void set_q_diag(int i, double v) {
        int j = q_diag_pos_[i];
        assert(j >= 0);
        Q.vals[j] = v;
    }

    void solve(int max_iter, double eps) {
        int n = Q.n;
        assert(n <= MAX_N);
        int32_t eps_sq = CGGolden::sign_extend(
            static_cast<int32_t>(eps * eps * CGGolden::FRAC_SCALE));

        // Build int32_t CSR from the placer-set double CSR
        CGGolden::CSR Q_fp;
        Q_fp.n = n;
        Q_fp.row_ptr = Q.row_ptr;
        Q_fp.col_idx = Q.col_idx;
        Q_fp.vals.resize(Q.vals.size());
        for (size_t j = 0; j < Q.vals.size(); ++j)
            Q_fp.vals[j] = d2fp(Q.vals[j]);

        solve_dim(Q_fp, c_x, x_pos, n, max_iter, eps_sq);
        solve_dim(Q_fp, c_y, y_pos, n, max_iter, eps_sq);
    }

private:
    std::vector<int> q_diag_pos_;

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
