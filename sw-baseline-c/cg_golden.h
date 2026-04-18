// Fixed-point CG golden model — single source of truth for both:
//   1. C++ placer (via cg_fp_golden.h wrapper)
//   2. Verilator DPI testbench (via cg_golden_dpi.cpp wrapper)
//
// Arithmetic rules (matching Cyclone V hardware):
//   - Multiplies: 27×27 DSP blocks produce 54-bit products
//   - Accumulators: use full-precision products (~40-bit after shift),
//     accumulated in 64-bit (maps to LUT adders on FPGA)
//   - Division: LUT-based, accepts 64-bit inputs, produces 27-bit output
//   - Storage: all vectors stored as 27-bit fixed-point

#pragma once

#include <cstdint>
#include <vector>

#ifndef CG_GOLDEN_FRAC_BITS
#define CG_GOLDEN_FRAC_BITS 14
#endif

struct CGGolden {
    static constexpr int TOTAL_BITS = 27;
    static constexpr int FRAC_BITS  = CG_GOLDEN_FRAC_BITS;
    static constexpr int FRAC_SCALE = 1 << FRAC_BITS;
    static constexpr int32_t BIT_MASK =
        static_cast<int32_t>((1LL << TOTAL_BITS) - 1);

    // -- Fixed-point primitives -----------------------------------------------

    static int32_t sign_extend(int32_t v) {
        v &= BIT_MASK;
        if (v & (1 << (TOTAL_BITS - 1)))
            v |= ~BIT_MASK;
        return v;
    }

    // 27-bit multiply, truncated to 27-bit output.
    // Use for scalar ops (alpha*d[i], beta*d[i]).
    static int32_t fp_mul(int32_t a, int32_t b) {
        int64_t full = static_cast<int64_t>(a) * static_cast<int64_t>(b);
        return sign_extend(static_cast<int32_t>(full >> FRAC_BITS));
    }

    // Full-precision multiply — returns ~40-bit shifted product.
    // On FPGA: DSP gives 54-bit product; shift right by FRAC_BITS.
    // Used inside dot products and SPMV where the accumulator is wide.
    static int64_t fp_mul_wide(int32_t a, int32_t b) {
        int64_t full = static_cast<int64_t>(a) * static_cast<int64_t>(b);
        return full >> FRAC_BITS;
    }

    // Division with wide (64-bit) inputs, 27-bit output.
    // On FPGA this is LUT-based (not DSP), so width is flexible.
    static int32_t fp_div_wide(int64_t a, int64_t b) {
        if (b == 0) return 0;
        int64_t num = a << FRAC_BITS;
        return sign_extend(static_cast<int32_t>(num / b));
    }

    // -- CSR format (int32_t values, already in fixed-point) ------------------

    struct CSR {
        int n;
        std::vector<int32_t> row_ptr;
        std::vector<int32_t> col_idx;
        std::vector<int32_t> vals;
    };

    // -- Linear algebra -------------------------------------------------------

    // SPMV: wide accumulator with full-precision products, 27-bit output.
    static void spmv(const CSR& A, const std::vector<int32_t>& x,
                     std::vector<int32_t>& y, int n) {
        for (int i = 0; i < n; ++i) {
            int64_t s = 0;
            for (int j = A.row_ptr[i]; j < A.row_ptr[i + 1]; ++j)
                s += fp_mul_wide(A.vals[j], x[A.col_idx[j]]);
            y[i] = sign_extend(static_cast<int32_t>(s));
        }
    }

    // Dot product: wide accumulator with full-precision products, wide output.
    static int64_t vec_dot(const std::vector<int32_t>& a,
                           const std::vector<int32_t>& b, int n) {
        int64_t s = 0;
        for (int i = 0; i < n; ++i)
            s += fp_mul_wide(a[i], b[i]);
        return s;
    }

    // -- CG solver ------------------------------------------------------------

    static void cg_solve(const CSR& Q, const std::vector<int32_t>& cx,
                         std::vector<int32_t>& x, int max_iter,
                         int32_t eps_sq) {
        int n = Q.n;
        std::vector<int32_t> Qx(n), r(n), d(n), q(n);
        spmv(Q, x, Qx, n);
        for (int i = 0; i < n; ++i)
            r[i] = sign_extend(-(cx[i] + Qx[i]));
        d = r;
        int64_t rr = vec_dot(r, r, n);

        for (int k = 0; k < max_iter; ++k) {
            spmv(Q, d, q, n);
            int64_t dq = vec_dot(d, q, n);
            if (dq == 0) break;
            int32_t alpha = fp_div_wide(rr, dq);

            for (int i = 0; i < n; ++i) {
                x[i] = sign_extend(x[i] + fp_mul(alpha, d[i]));
                r[i] = sign_extend(r[i] - fp_mul(alpha, q[i]));
            }

            int64_t rr_new = vec_dot(r, r, n);
            if (rr_new <= eps_sq) break;
            if (k > 0 && rr_new >= rr) break;

            int32_t beta = fp_div_wide(rr_new, rr);
            for (int i = 0; i < n; ++i)
                d[i] = sign_extend(r[i] + fp_mul(beta, d[i]));
            rr = rr_new;
        }
    }
};
