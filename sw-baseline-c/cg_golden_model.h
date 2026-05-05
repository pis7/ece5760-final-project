// Bit-exact fixed-point CG golden model -- int64 storage, __int128
// multiplies, no I/O, no placer or HW driver dependencies. Single
// source of truth for both:
//   1. C++ placer (via cg_golden_driver.h wrapper)
//   2. Verilator DPI testbench (via cg_golden_dpi.cpp wrapper)
//
// TOTAL_BITS defaults to 27 (the locked-down Cyclone V bitstream)
// but may be overridden via CG_GOLDEN_TOTAL_BITS up to 64 to match a
// wider verilated v6 build.
//
// Arithmetic rules (matching Cyclone V hardware at 27 bits):
//   - Multiplies: 27×27 DSP blocks produce 54-bit products
//   - Accumulators: use full-precision products (~40-bit after shift),
//     accumulated in 64-bit (maps to LUT adders on FPGA)
//   - Division:
//       * default (matches v1, v2 hardware AND any width >27): LUT-based
//         shift-subtract, accepts 64-bit inputs, produces TOTAL_BITS output.
//       * with -DCG_GOLDEN_USE_NR (matches v3..v5 hardware): Newton-Raphson
//         reciprocal with 256x17 seed ROM and 2 NR iters; mirrors
//         fpga/hw/v3/FpMath.v::FpDiv bit-exactly. Only valid for
//         TOTAL_BITS<=27; force-disabled for wider widths since the
//         RTL generate switches to shift-subtract too.
//   - Storage: all vectors stored as TOTAL_BITS fixed-point

#pragma once

#include <cstdint>
#include <vector>

#ifndef CG_GOLDEN_FRAC_BITS
#define CG_GOLDEN_FRAC_BITS 14
#endif

#ifndef CG_GOLDEN_TOTAL_BITS
#define CG_GOLDEN_TOTAL_BITS 27
#endif

// NR path is hard-wired to 48-bit / Q1.16 internals; fall back to SS
// whenever the active total width pushes past the NR design point so
// the golden mirrors the RTL's generate-driven divider choice.
#if defined(CG_GOLDEN_USE_NR) && (CG_GOLDEN_TOTAL_BITS > 27)
#undef CG_GOLDEN_USE_NR
#endif

struct CGGolden {
    static constexpr int TOTAL_BITS = CG_GOLDEN_TOTAL_BITS;
    static constexpr int FRAC_BITS  = CG_GOLDEN_FRAC_BITS;
    static constexpr int64_t FRAC_SCALE = static_cast<int64_t>(1) << FRAC_BITS;
    static constexpr int64_t BIT_MASK =
        (TOTAL_BITS >= 64)
            ? -1LL
            : ((static_cast<int64_t>(1) << TOTAL_BITS) - 1);
    static constexpr uint64_t SAT_MAG =
        (static_cast<uint64_t>(1) << (TOTAL_BITS - 1)) - 1;

    // -- Fixed-point primitives -----------------------------------------------

    static int64_t sign_extend(int64_t v) {
        v &= BIT_MASK;
        if (v & (static_cast<int64_t>(1) << (TOTAL_BITS - 1)))
            v |= ~BIT_MASK;
        return v;
    }

    // TOTAL_BITS multiply, truncated to TOTAL_BITS output.
    // Use for scalar ops (alpha*d[i], beta*d[i]).
    static int64_t fp_mul(int64_t a, int64_t b) {
        __int128 full = static_cast<__int128>(a) * static_cast<__int128>(b);
        return sign_extend(static_cast<int64_t>(full >> FRAC_BITS));
    }

    // Full-precision multiply — returns full shifted product.
    // On FPGA: DSP gives 2*TOTAL_BITS product; shift right by FRAC_BITS.
    // Used inside dot products and SPMV where the accumulator is wide.
    // Returns __int128 so 64-bit total widths don't truncate.
    static __int128 fp_mul_wide(int64_t a, int64_t b) {
        __int128 full = static_cast<__int128>(a) * static_cast<__int128>(b);
        return full >> FRAC_BITS;
    }

#ifdef CG_GOLDEN_USE_NR
    // -- NR reciprocal divide (mirrors fpga/hw/v3/FpMath.v::FpDiv) ------------
    //
    // 256x17 reciprocal seed ROM. Entry i = round(2^32 / bn_mid) where
    // bn_mid = 0x10000 + (i << 8) + 0x80, the Q1.16 midpoint of bin i.
    // Identical to RTL recip_rom; if you regenerate one, regenerate both.
    static constexpr uint32_t recip_rom[256] = {
        0x0ff80, 0x0fe82, 0x0fd86, 0x0fc8c, 0x0fb94, 0x0fa9e, 0x0f9a9, 0x0f8b7,
        0x0f7c6, 0x0f6d7, 0x0f5ea, 0x0f4ff, 0x0f415, 0x0f32d, 0x0f247, 0x0f163,
        0x0f080, 0x0ef9f, 0x0eebf, 0x0ede1, 0x0ed05, 0x0ec2a, 0x0eb51, 0x0ea7a,
        0x0e9a4, 0x0e8cf, 0x0e7fc, 0x0e72b, 0x0e65b, 0x0e58c, 0x0e4bf, 0x0e3f4,
        0x0e329, 0x0e260, 0x0e199, 0x0e0d3, 0x0e00e, 0x0df4b, 0x0de88, 0x0ddc8,
        0x0dd08, 0x0dc4a, 0x0db8d, 0x0dad1, 0x0da17, 0x0d95e, 0x0d8a6, 0x0d7ef,
        0x0d73a, 0x0d685, 0x0d5d2, 0x0d520, 0x0d46f, 0x0d3bf, 0x0d311, 0x0d263,
        0x0d1b7, 0x0d10c, 0x0d062, 0x0cfb9, 0x0cf11, 0x0ce6a, 0x0cdc4, 0x0cd1f,
        0x0cc7b, 0x0cbd8, 0x0cb36, 0x0ca96, 0x0c9f6, 0x0c957, 0x0c8b9, 0x0c81c,
        0x0c780, 0x0c6e5, 0x0c64b, 0x0c5b2, 0x0c51a, 0x0c482, 0x0c3ec, 0x0c357,
        0x0c2c2, 0x0c22e, 0x0c19b, 0x0c109, 0x0c078, 0x0bfe8, 0x0bf59, 0x0beca,
        0x0be3c, 0x0bdaf, 0x0bd23, 0x0bc98, 0x0bc0d, 0x0bb83, 0x0bafb, 0x0ba72,
        0x0b9eb, 0x0b964, 0x0b8de, 0x0b859, 0x0b7d5, 0x0b751, 0x0b6ce, 0x0b64c,
        0x0b5cb, 0x0b54a, 0x0b4ca, 0x0b44b, 0x0b3cc, 0x0b34e, 0x0b2d1, 0x0b254,
        0x0b1d8, 0x0b15d, 0x0b0e3, 0x0b069, 0x0aff0, 0x0af77, 0x0aeff, 0x0ae88,
        0x0ae11, 0x0ad9b, 0x0ad26, 0x0acb1, 0x0ac3d, 0x0abc9, 0x0ab56, 0x0aae4,
        0x0aa72, 0x0aa01, 0x0a990, 0x0a920, 0x0a8b1, 0x0a842, 0x0a7d3, 0x0a766,
        0x0a6f8, 0x0a68c, 0x0a620, 0x0a5b4, 0x0a549, 0x0a4df, 0x0a475, 0x0a40c,
        0x0a3a3, 0x0a33a, 0x0a2d3, 0x0a26b, 0x0a204, 0x0a19e, 0x0a138, 0x0a0d3,
        0x0a06e, 0x0a00a, 0x09fa6, 0x09f43, 0x09ee0, 0x09e7e, 0x09e1c, 0x09dba,
        0x09d59, 0x09cf9, 0x09c99, 0x09c39, 0x09bda, 0x09b7c, 0x09b1d, 0x09ac0,
        0x09a62, 0x09a05, 0x099a9, 0x0994d, 0x098f1, 0x09896, 0x0983b, 0x097e1,
        0x09787, 0x0972e, 0x096d5, 0x0967c, 0x09624, 0x095cc, 0x09574, 0x0951d,
        0x094c7, 0x09470, 0x0941b, 0x093c5, 0x09370, 0x0931b, 0x092c7, 0x09273,
        0x0921f, 0x091cc, 0x09179, 0x09127, 0x090d5, 0x09083, 0x09032, 0x08fe1,
        0x08f90, 0x08f40, 0x08ef0, 0x08ea0, 0x08e51, 0x08e02, 0x08db3, 0x08d65,
        0x08d17, 0x08cc9, 0x08c7c, 0x08c2f, 0x08be2, 0x08b96, 0x08b4a, 0x08aff,
        0x08ab3, 0x08a68, 0x08a1e, 0x089d3, 0x08989, 0x08940, 0x088f6, 0x088ad,
        0x08864, 0x0881c, 0x087d3, 0x0878c, 0x08744, 0x086fd, 0x086b6, 0x0866f,
        0x08628, 0x085e2, 0x0859c, 0x08557, 0x08511, 0x084cc, 0x08488, 0x08443,
        0x083ff, 0x083bb, 0x08377, 0x08334, 0x082f1, 0x082ae, 0x0826b, 0x08229,
        0x081e7, 0x081a5, 0x08164, 0x08123, 0x080e2, 0x080a1, 0x08060, 0x08020,
    };

    // 48-bit leading-zero count. Returns 48 when input is zero.
    // Mirrors RTL count_leading_zeros() in FpMath.v.
    static int lzc48(uint64_t x) {
        x &= (1ULL << 48) - 1;
        if (x == 0) return 48;
        for (int k = 47; k >= 0; --k)
            if (x & (1ULL << k)) return 47 - k;
        return 48;  // unreachable
    }

    // Newton-Raphson reciprocal divide. Bit-exact with the v3 FpDiv RTL:
    // same LZC, same ROM, same NR truncations, same denorm shift, same
    // saturation. Inputs treated as 48-bit signed; output is Q(I).(F)
    // sign-extended into int64_t. Only compiled when TOTAL_BITS<=27.
    static int64_t fp_div_wide(__int128 a, __int128 b) {
        bool sign_q = (a < 0) ^ (b < 0);
        uint64_t abs_a = static_cast<uint64_t>(a < 0 ? -a : a);
        uint64_t abs_b = static_cast<uint64_t>(b < 0 ? -b : b);
        abs_a &= (1ULL << 48) - 1;
        abs_b &= (1ULL << 48) - 1;

        // b == 0 → saturate to max-magnitude with sign of a.
        if (abs_b == 0) {
            int64_t mag = static_cast<int64_t>(SAT_MAG);
            return sign_extend(sign_q ? -mag : mag);
        }

        int      lzc      = lzc48(abs_b);
        uint64_t shifted  = abs_b << lzc;
        // b_norm = shifted[47:31], 17-bit Q1.16 with bit 16 always 1.
        uint32_t b_norm   = static_cast<uint32_t>((shifted >> 31) & 0x1FFFFu);
        uint32_t rom_idx  = (b_norm >> 8) & 0xFFu;
        uint32_t r0       = recip_rom[rom_idx];

        // Two NR iterations. Each: r_new = r_in * (2 - b_norm * r_in).
        // Identical truncations to RTL FpDiv. The 2nd iter washes out the
        // truncation noise from the 1st so the result is close to exact
        // within the 14-bit fractional output, matching shift-subtract on
        // most operand pairs.
        auto nr_step = [&](uint32_t r_in) -> uint32_t {
            uint64_t ma = static_cast<uint64_t>(b_norm) * r_in;             // 34-bit
            uint32_t tm = (0x20000u - static_cast<uint32_t>((ma >> 16) & 0x3FFFFu))
                          & 0x3FFFFu;                                        // 18-bit
            uint64_t mb = static_cast<uint64_t>(r_in) * tm;                  // 35-bit
            return static_cast<uint32_t>((mb >> 16) & 0x1FFFFu);             // 17-bit
        };
        uint32_t r1 = nr_step(r0);
        r1          = nr_step(r1);

        // Final multiply: 48 * 17 = 65-bit unsigned. Use __uint128_t.
        __uint128_t m3 = static_cast<__uint128_t>(abs_a) * r1;

        // Denormalize: shift = 49 - lzc (range 2..49 for non-zero abs_b).
        int shift_amt = 49 - lzc;
        __uint128_t shifted_m3 = m3 >> shift_amt;

        // Saturate: any bit at position >= TOTAL_BITS-1 means overflow.
        bool overflow = (shifted_m3 >> (TOTAL_BITS - 1)) != 0;
        uint64_t result_pos = overflow
            ? SAT_MAG
            : (static_cast<uint64_t>(shifted_m3) & SAT_MAG);

        int64_t result = sign_q ? -static_cast<int64_t>(result_pos)
                                :  static_cast<int64_t>(result_pos);
        return sign_extend(result);
    }
#else
    // Default: integer shift-subtract divide. Mirrors fpga/hw/v2/FpMath.v
    // (and the v6 FpDivSS branch picked when p_total_bits>27) bit-for-bit
    // -- abs(a)/abs(b), pre-shifted by FRAC_BITS, then sign-applied.
    //
    // Implemented as a direct MSB-first shift-subtract loop instead of
    // (a << FRAC_BITS) / b so we don't overflow __uint128_t at wide
    // configurations (e.g. Q32.32 / N=50: accumulator hits ~102 bits and
    // the pre-shifted dividend would be ~134 bits). The 128-bit running
    // remainder is enough because abs_b is at most p_wide_bits ~=
    // 2*TOTAL_BITS-FRAC_BITS+log2(N) bits, comfortably <128 for
    // TOTAL_BITS<=64.
    static int64_t fp_div_wide(__int128 a, __int128 b) {
        if (b == 0) return 0;
        bool sign_q = (a < 0) ^ (b < 0);
        __uint128_t abs_a = a < 0 ? static_cast<__uint128_t>(-a)
                                  : static_cast<__uint128_t>(a);
        __uint128_t abs_b = b < 0 ? static_cast<__uint128_t>(-b)
                                  : static_cast<__uint128_t>(b);

        // Process the MSBs of abs_a first (matching the RTL's
        // {abs_a, {FRAC_BITS{0}}} dividend layout), then FRAC_BITS of
        // zeros. We only need the bottom TOTAL_BITS-1 bits of the
        // quotient (matches `quotient[p_total_bits-2:0]` in v2 RTL), so
        // tracking it in a uint64_t is sufficient.
        __uint128_t rem  = 0;
        uint64_t    quot = 0;
        const int   total_iters = 128 + FRAC_BITS;
        for (int k = total_iters - 1; k >= 0; --k) {
            rem <<= 1;
            // Bit (k - FRAC_BITS) of abs_a shifts into rem; lower
            // FRAC_BITS positions are implicit zeros.
            if (k >= FRAC_BITS) {
                int bit_idx = k - FRAC_BITS;
                if ((abs_a >> bit_idx) & __uint128_t(1)) rem |= 1;
            }
            if (rem >= abs_b) {
                rem -= abs_b;
                // Quotient bit position from the top: total_iters-1-iter,
                // where iter goes 0..total_iters-1 as k goes high..low.
                // That collapses to: this iteration sets bit k of the
                // quotient. We only care about the bottom TOTAL_BITS-1
                // bits so high-bit overflows are silently dropped (the
                // RTL truncates with the same `quotient[p_total_bits-2:0]`
                // slice).
                if (k < TOTAL_BITS - 1) quot |= (uint64_t(1) << k);
            }
        }

        int64_t result = sign_q ? -static_cast<int64_t>(quot)
                                :  static_cast<int64_t>(quot);
        return sign_extend(result);
    }
#endif

    // -- CSR format (int64_t values, already in fixed-point) -----------------

    struct CSR {
        int n;
        std::vector<int32_t> row_ptr;   // indices, 32-bit
        std::vector<int32_t> col_idx;   // indices, 32-bit
        std::vector<int64_t> vals;      // fixed-point, sign-extended
    };

    // -- Linear algebra -------------------------------------------------------

    // SPMV: wide accumulator with full-precision products, TOTAL_BITS output.
    // Accumulator is __int128 so sums of n=p_max_n wide products at
    // TOTAL_BITS=64 don't overflow before the final truncation.
    static void spmv(const CSR& A, const std::vector<int64_t>& x,
                     std::vector<int64_t>& y, int n) {
        for (int i = 0; i < n; ++i) {
            __int128 s = 0;
            for (int j = A.row_ptr[i]; j < A.row_ptr[i + 1]; ++j)
                s += fp_mul_wide(A.vals[j], x[A.col_idx[j]]);
            y[i] = sign_extend(static_cast<int64_t>(s));
        }
    }

    // Dot product: wide accumulator with full-precision products. Returns
    // __int128 so the divider can keep full precision when TOTAL_BITS=64.
    static __int128 vec_dot(const std::vector<int64_t>& a,
                            const std::vector<int64_t>& b, int n) {
        __int128 s = 0;
        for (int i = 0; i < n; ++i)
            s += fp_mul_wide(a[i], b[i]);
        return s;
    }

    // -- CG solver ------------------------------------------------------------

    static void cg_solve(const CSR& Q, const std::vector<int64_t>& cx,
                         std::vector<int64_t>& x, int max_iter,
                         int64_t eps_sq) {
        int n = Q.n;
        std::vector<int64_t> Qx(n), r(n), d(n), q(n);
        spmv(Q, x, Qx, n);
        for (int i = 0; i < n; ++i)
            r[i] = sign_extend(-(cx[i] + Qx[i]));
        d = r;
        __int128 rr = vec_dot(r, r, n);

        for (int k = 0; k < max_iter; ++k) {
            spmv(Q, d, q, n);
            __int128 dq = vec_dot(d, q, n);
            if (dq == 0) break;
            int64_t alpha = fp_div_wide(rr, dq);

            for (int i = 0; i < n; ++i) {
                x[i] = sign_extend(x[i] + fp_mul(alpha, d[i]));
                r[i] = sign_extend(r[i] - fp_mul(alpha, q[i]));
            }

            __int128 rr_new = vec_dot(r, r, n);
            if (rr_new <= eps_sq) break;
            if (k > 0 && rr_new >= rr) break;

            int64_t beta = fp_div_wide(rr_new, rr);
            for (int i = 0; i < n; ++i)
                d[i] = sign_extend(r[i] + fp_mul(beta, d[i]));
            rr = rr_new;
        }
    }
};
