// Verilator driver for CGTop — replaces software cg_solve with hardware CG
//
// Instantiates the Verilator model of CGTop, packs CSR + vectors into the
// flat M10K memory layout, drives the sw_go/sw_done handshake, and unpacks
// the solution vectors.

#pragma once

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "VCGTop.h"
#include "verilated.h"

class CGHwDriver {
public:
    static constexpr int MAX_N       = 50;
    static constexpr int TOTAL_BITS  = 27;
#ifndef HW_FRAC_BITS
    static constexpr int FRAC_BITS   = 14;
#else
    static constexpr int FRAC_BITS   = HW_FRAC_BITS;
#endif
    static constexpr int INT_BITS    = TOTAL_BITS - FRAC_BITS;
    static constexpr int FRAC_SCALE  = 1 << FRAC_BITS;
    static constexpr int32_t BIT_MASK = static_cast<int32_t>((1LL << TOTAL_BITS) - 1);

    // Memory layout (must match CGTop parameters)
    static constexpr int Q_VAL_BASE  = 0;
    static constexpr int Q_COL_BASE  = MAX_N * MAX_N;
    static constexpr int Q_ROWP_BASE = 2 * MAX_N * MAX_N;
    static constexpr int CX_X_BASE   = 2 * MAX_N * MAX_N + MAX_N + 1;
    static constexpr int CX_Y_BASE   = 2 * MAX_N * MAX_N + 2 * MAX_N + 1;
    static constexpr int X_BASE      = 2 * MAX_N * MAX_N + 3 * MAX_N + 1;
    static constexpr int Y_BASE      = 2 * MAX_N * MAX_N + 4 * MAX_N + 1;
    static constexpr int TOTAL_WORDS = 2 * MAX_N * MAX_N + 5 * MAX_N + 1;

    CGHwDriver() {
        ctx_ = new VerilatedContext;
        dut_ = new VCGTop(ctx_);
    }

    ~CGHwDriver() {
        dut_->final();
        delete dut_;
        delete ctx_;
    }

    // Clock cycles between sw_go assertion and sw_done for the most recent
    // solve() call. This is the apples-to-apples count we'd see on the FPGA.
    uint64_t last_solve_cycles() const { return last_solve_cycles_; }

    // Solve Qx = -cx and Qy = -cy using the hardware CG.
    // Modifies x and y in place.
    void solve(const CSRMatrix& Q,
               const std::vector<double>& cx,
               const std::vector<double>& cy,
               std::vector<double>& x,
               std::vector<double>& y,
               int max_iter, double eps) {
        int n = Q.n;
        assert(n <= MAX_N);

        pack_memory(Q, cx, cy, x, y, n);

        // Reset
        dut_->rst = 1;
        dut_->sw_go = 0;
        dut_->sw_done_ack = 0;
        dut_->max_iter = max_iter;
        dut_->eps_sq = double_to_fp(eps * eps);
        dut_->n = n;
        for (int i = 0; i < 5; ++i) tick();
        dut_->rst = 0;
        tick();

        // Assert sw_go for one cycle
        last_solve_cycles_ = 0;
        dut_->sw_go = 1;
        tick();
        ++last_solve_cycles_;
        dut_->sw_go = 0;

        // Wait for sw_done
        int timeout = 1000000;
        while (!dut_->sw_done && --timeout > 0) {
            tick();
            ++last_solve_cycles_;
        }
        if (timeout == 0) {
            fprintf(stderr, "CGHwDriver: timeout waiting for sw_done\n");
            return;
        }

        // Acknowledge
        dut_->sw_done_ack = 1;
        tick();
        dut_->sw_done_ack = 0;
        tick();

        unpack_results(x, y, n);
    }

private:
    VerilatedContext* ctx_;
    VCGTop* dut_;
    int32_t m10k_mem_[TOTAL_WORDS];
    uint64_t last_solve_cycles_ = 0;

    static int32_t double_to_fp(double v) {
        int32_t raw = static_cast<int32_t>(v * FRAC_SCALE);
        return sign_extend(raw);
    }

    static double fp_to_double(int32_t v) {
        v = sign_extend(v);
        return static_cast<double>(v) / FRAC_SCALE;
    }

    static int32_t sign_extend(int32_t v) {
        v &= BIT_MASK;
        if (v & (1 << (TOTAL_BITS - 1)))
            v |= ~BIT_MASK;
        return v;
    }

    void tick() {
        // Settle combinational logic with clk=0 so we see the address being
        // driven during the cycle that is about to end -- the same address
        // an SV `always_ff @(posedge clk)` shim samples when its NBA fires.
        // Sampling AFTER the rising-edge eval would read the NEW state's
        // address, which is one cycle ahead of what real hardware sees and
        // breaks any DUT that drives a different address each cycle.
        dut_->clk = 0;
        dut_->eval();
        bool     cs       = dut_->on_chip_ram_chipselect && dut_->on_chip_ram_clken;
        uint32_t addr_pre = dut_->on_chip_ram_address;
        bool     wr       = dut_->on_chip_ram_write;
        uint32_t wdata    = dut_->on_chip_ram_writedata;

        dut_->clk = 1;
        dut_->eval();

        if (cs && addr_pre < TOTAL_WORDS) {
            if (wr)
                m10k_mem_[addr_pre] = static_cast<int32_t>(wdata);
            else
                dut_->on_chip_ram_readdata =
                    static_cast<uint32_t>(m10k_mem_[addr_pre]);
        }
        // Re-evaluate so any combinational consumer of readdata in the DUT
        // sees the freshly-updated value before the next clock cycle.
        dut_->eval();
    }

    void pack_memory(const CSRMatrix& Q,
                     const std::vector<double>& cx,
                     const std::vector<double>& cy,
                     const std::vector<double>& x,
                     const std::vector<double>& y,
                     int n) {
        // Zero all memory
        for (int i = 0; i < TOTAL_WORDS; ++i) m10k_mem_[i] = 0;

        // CSR values and column indices
        int nnz = Q.nnz();
        assert(nnz <= MAX_N * MAX_N);
        for (int j = 0; j < nnz; ++j) {
            m10k_mem_[Q_VAL_BASE + j] = double_to_fp(Q.vals[j]);
            m10k_mem_[Q_COL_BASE + j] = Q.col_idx[j];
        }

        // Row pointers
        for (int i = 0; i <= n; ++i)
            m10k_mem_[Q_ROWP_BASE + i] = Q.row_ptr[i];

        // Vectors
        for (int i = 0; i < n; ++i) {
            m10k_mem_[CX_X_BASE + i] = double_to_fp(cx[i]);
            m10k_mem_[CX_Y_BASE + i] = double_to_fp(cy[i]);
            m10k_mem_[X_BASE + i]    = double_to_fp(x[i]);
            m10k_mem_[Y_BASE + i]    = double_to_fp(y[i]);
        }
    }

    void unpack_results(std::vector<double>& x, std::vector<double>& y, int n) {
        for (int i = 0; i < n; ++i) {
            x[i] = fp_to_double(m10k_mem_[X_BASE + i]);
            y[i] = fp_to_double(m10k_mem_[Y_BASE + i]);
        }
    }
};
