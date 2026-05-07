// Verilator driver for v2 / v3 / v4 CGTop (single shared on-chip RAM).
//
// TOTAL_BITS defaults to 27 for the FPGA bitstream; verilated builds
// may override via HW_TOTAL_BITS up to 64. The single Avalon port
// widens to QData (uint64_t) for >32-bit, IData (uint32_t) otherwise;
// the driver uniformly snapshots it as uint64_t.

#pragma once

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "VCGTop.h"
#include "verilated.h"

class CGHwDriver {
public:
#ifndef HW_MAX_N
    static constexpr int MAX_N       = 50;
#else
    static constexpr int MAX_N       = HW_MAX_N;
#endif
#ifndef HW_TOTAL_BITS
    static constexpr int TOTAL_BITS  = 27;
#else
    static constexpr int TOTAL_BITS  = HW_TOTAL_BITS;
#endif
#ifndef HW_FRAC_BITS
    static constexpr int FRAC_BITS   = 14;
#else
    static constexpr int FRAC_BITS   = HW_FRAC_BITS;
#endif
    static constexpr int INT_BITS    = TOTAL_BITS - FRAC_BITS;
    static constexpr int64_t FRAC_SCALE = static_cast<int64_t>(1) << FRAC_BITS;
    static constexpr int64_t BIT_MASK   =
        (TOTAL_BITS >= 64) ? -1LL
                           : ((static_cast<int64_t>(1) << TOTAL_BITS) - 1);

    static constexpr int Q_VAL_BASE  = 0;
    static constexpr int Q_COL_BASE  = MAX_N * MAX_N;
    static constexpr int Q_ROWP_BASE = 2 * MAX_N * MAX_N;
    static constexpr int CX_X_BASE   = 2 * MAX_N * MAX_N + MAX_N + 1;
    static constexpr int CX_Y_BASE   = 2 * MAX_N * MAX_N + 2 * MAX_N + 1;
    static constexpr int X_BASE      = 2 * MAX_N * MAX_N + 3 * MAX_N + 1;
    static constexpr int Y_BASE      = 2 * MAX_N * MAX_N + 4 * MAX_N + 1;
    static constexpr int TOTAL_WORDS = 2 * MAX_N * MAX_N + 5 * MAX_N + 1;

    // Public mirrors -- canonical storage for the placer; setters keep
    // the matching m10k slot in lockstep so we never repack at solve
    // time. refresh_xy_from_m10k() pulls x/y back after each solve.
    std::vector<double> c_x, c_y, x_pos, y_pos;
    CSRMatrix Q;

    CGHwDriver()
        : m10k_mem_(TOTAL_WORDS, 0) {
        ctx_ = new VerilatedContext;
        dut_ = new VCGTop(ctx_);
    }

    ~CGHwDriver() {
        dut_->final();
        delete dut_;
        delete ctx_;
    }

    void resize_n(int n) {
        c_x.assign(n, 0.0);
        c_y.assign(n, 0.0);
        x_pos.assign(n, 0.0);
        y_pos.assign(n, 0.0);
    }

    void set_cx(int i, double v) { c_x[i]   = v; m10k_mem_[CX_X_BASE + i] = double_to_fp(v); }
    void set_cy(int i, double v) { c_y[i]   = v; m10k_mem_[CX_Y_BASE + i] = double_to_fp(v); }
    void set_x (int i, double v) { x_pos[i] = v; m10k_mem_[X_BASE    + i] = double_to_fp(v); }
    void set_y (int i, double v) { y_pos[i] = v; m10k_mem_[Y_BASE    + i] = double_to_fp(v); }

    void load_q_initial(const CSRMatrix& src) {
        Q = src;
        int nnz = Q.nnz();
        assert(nnz <= MAX_N * MAX_N);
        q_diag_pos_.assign(Q.n, -1);
        for (int i = 0; i < Q.n; ++i) {
            for (int j = Q.row_ptr[i]; j < Q.row_ptr[i + 1]; ++j) {
                if (Q.col_idx[j] == i) {
                    q_diag_pos_[i] = j;
                    break;
                }
            }
        }
        for (int j = 0; j < nnz; ++j) {
            m10k_mem_[Q_VAL_BASE + j] = double_to_fp(Q.vals[j]);
            m10k_mem_[Q_COL_BASE + j] = Q.col_idx[j];
        }
        for (int i = 0; i <= Q.n; ++i)
            m10k_mem_[Q_ROWP_BASE + i] = Q.row_ptr[i];
    }

    void set_q_diag(int i, double v) {
        int j = q_diag_pos_[i];
        assert(j >= 0);
        Q.vals[j] = v;
        m10k_mem_[Q_VAL_BASE + j] = double_to_fp(v);
    }

    uint64_t last_solve_cycles() const { return last_solve_cycles_; }

    void solve(int max_iter, double eps) {
        int n = Q.n;
        assert(n <= MAX_N);

        dut_->rst = 1;
        dut_->sw_go = 0;
        dut_->sw_done_ack = 0;
        dut_->max_iter = max_iter;
        dut_->eps_sq = static_cast<uint32_t>(double_to_fp(eps * eps));
        dut_->n = n;
        for (int i = 0; i < 5; ++i) tick();
        dut_->rst = 0;
        tick();

        last_solve_cycles_ = 0;
        dut_->sw_go = 1;
        tick();
        ++last_solve_cycles_;
        dut_->sw_go = 0;

        int timeout = 100000000;
        while (!dut_->sw_done && --timeout > 0) {
            tick();
            ++last_solve_cycles_;
        }
        if (timeout == 0) {
            fprintf(stderr, "CGHwDriver: timeout waiting for sw_done\n");
            return;
        }

        dut_->sw_done_ack = 1;
        tick();
        dut_->sw_done_ack = 0;
        tick();

        refresh_xy_from_m10k(n);
    }

private:
    VerilatedContext* ctx_;
    VCGTop* dut_;
    std::vector<int64_t> m10k_mem_;
    uint64_t last_solve_cycles_ = 0;

    std::vector<int> q_diag_pos_;

    static int64_t double_to_fp(double v) {
        int64_t raw = static_cast<int64_t>(v * FRAC_SCALE);
        return sign_extend(raw);
    }

    static double fp_to_double(int64_t v) {
        v = sign_extend(v);
        return static_cast<double>(v) / FRAC_SCALE;
    }

    static int64_t sign_extend(int64_t v) {
        v &= BIT_MASK;
        if (v & (static_cast<int64_t>(1) << (TOTAL_BITS - 1)))
            v |= ~BIT_MASK;
        return v;
    }

    void tick() {
        // Sample addr/wr/wdata while clk is low so we see the address being
        // driven during the cycle that's about to end -- matches what an SV
        // `always_ff @(posedge clk)` shim sees on its NBA. Sampling AFTER
        // the rising-edge eval would read the NEW state's address, one
        // cycle ahead of real hardware.
        dut_->clk = 0;
        dut_->eval();
        bool     cs       = dut_->on_chip_ram_chipselect && dut_->on_chip_ram_clken;
        uint32_t addr_pre = dut_->on_chip_ram_address;
        bool     wr       = dut_->on_chip_ram_write;
        uint64_t wdata    = dut_->on_chip_ram_writedata;

        dut_->clk = 1;
        dut_->eval();

        if (cs && addr_pre < TOTAL_WORDS) {
            if (wr)
                m10k_mem_[addr_pre] = static_cast<int64_t>(wdata);
            else
                dut_->on_chip_ram_readdata =
                    static_cast<uint64_t>(m10k_mem_[addr_pre]);
        }
        // Re-evaluate so any combinational consumer of readdata sees the
        // freshly-updated value before the next cycle.
        dut_->eval();
    }

    void refresh_xy_from_m10k(int n) {
        for (int i = 0; i < n; ++i) {
            x_pos[i] = fp_to_double(m10k_mem_[X_BASE + i]);
            y_pos[i] = fp_to_double(m10k_mem_[Y_BASE + i]);
        }
    }
};
