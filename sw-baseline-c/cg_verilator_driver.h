// Verilator driver for CGTop — replaces software cg_solve with hardware CG
//
// Instantiates the Verilator model of CGTop, packs CSR + vectors into the
// flat M10K memory layout, drives the sw_go/sw_done handshake, and unpacks
// the solution vectors.

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

    // Public mirrors of the working state. These ARE the canonical
    // storage for the placer -- it reads them directly (fast, doubles
    // already in CPU memory) and writes them via the setters below,
    // which keep the matching m10k slot in lockstep so we never have
    // to "pack" anything at solve time.
    //
    // After CG, refresh_xy_from_m10k() pulls the new x / y back into
    // these mirrors in one bulk pass (called from solve()).
    std::vector<double> c_x, c_y, x_pos, y_pos;
    // Active Q. load_q_initial() pushes structure + values to m10k
    // once; subsequent diagonal-only updates go through set_q_diag().
    // The mirror is what the placer reads (e.g. for the SW fallback's
    // cg_solve, which expects a CSRMatrix).
    CSRMatrix Q;

    CGHwDriver() {
        ctx_ = new VerilatedContext;
        dut_ = new VCGTop(ctx_);
        std::memset(m10k_mem_, 0, sizeof(m10k_mem_));
    }

    ~CGHwDriver() {
        dut_->final();
        delete dut_;
        delete ctx_;
    }

    // Resize the mirror vectors. Call once after the placer knows n.
    void resize_n(int n) {
        c_x.assign(n, 0.0);
        c_y.assign(n, 0.0);
        x_pos.assign(n, 0.0);
        y_pos.assign(n, 0.0);
    }

    // Per-element setters: update the mirror AND the matching m10k slot.
    void set_cx(int i, double v) { c_x[i]   = v; m10k_mem_[CX_X_BASE + i] = double_to_fp(v); }
    void set_cy(int i, double v) { c_y[i]   = v; m10k_mem_[CX_Y_BASE + i] = double_to_fp(v); }
    void set_x (int i, double v) { x_pos[i] = v; m10k_mem_[X_BASE    + i] = double_to_fp(v); }
    void set_y (int i, double v) { y_pos[i] = v; m10k_mem_[Y_BASE    + i] = double_to_fp(v); }

    // One-time bulk upload of Q (structure + values). Caller must
    // ensure src has a diagonal entry in every row -- the placer feeds
    // csr_add_diagonal(Q_base, 0.0) so missing diagonals get inserted
    // with val=0; subsequent set_q_diag() calls update those slots.
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

    // Update the diagonal entry of row i (mirror + m10k). load_q_initial
    // must have been called first; the diagonal slot must exist.
    void set_q_diag(int i, double v) {
        int j = q_diag_pos_[i];
        assert(j >= 0);
        Q.vals[j] = v;
        m10k_mem_[Q_VAL_BASE + j] = double_to_fp(v);
    }

    // Clock cycles between sw_go assertion and sw_done for the most recent
    // solve() call. This is the apples-to-apples count we'd see on the FPGA.
    uint64_t last_solve_cycles() const { return last_solve_cycles_; }

    // Solve Qx = -cx and Qy = -cy using the hardware CG. Everything
    // (Q + c_x + c_y + x_pos + y_pos) already lives in m10k via the
    // setters; this just runs the FSM. x_pos / y_pos mirrors are
    // refreshed from m10k on completion.
    void solve(int max_iter, double eps) {
        int n = Q.n;
        assert(n <= MAX_N);

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

        refresh_xy_from_m10k(n);
    }

private:
    VerilatedContext* ctx_;
    VCGTop* dut_;
    int32_t m10k_mem_[TOTAL_WORDS];
    uint64_t last_solve_cycles_ = 0;

    // Cached diagonal-entry index per row: Q.col_idx[q_diag_pos_[i]] == i.
    // Populated by load_q_initial; consumed by set_q_diag.
    std::vector<int> q_diag_pos_;

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

    void refresh_xy_from_m10k(int n) {
        for (int i = 0; i < n; ++i) {
            x_pos[i] = fp_to_double(m10k_mem_[X_BASE + i]);
            y_pos[i] = fp_to_double(m10k_mem_[Y_BASE + i]);
        }
    }
};
