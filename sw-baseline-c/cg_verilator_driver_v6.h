// Verilator driver for v6 CGTop (parallel x/y solve datapaths).
//
// v6 duplicates the v5 Q-CSR triplet across two CGEngines, so this
// driver models 10 behavioral memories: q_val_x/q_col_x/q_rowp_x for
// the x engine, q_val_y/q_col_y/q_rowp_y for the y engine, plus
// cx/cy/x/y. Q is loaded into both copies. The external solve()
// protocol matches v5 verbatim -- one go pulse, one done.

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

    // Per-slave depths
    static constexpr int Q_VAL_DEPTH  = MAX_N * MAX_N;
    static constexpr int Q_COL_DEPTH  = MAX_N * MAX_N;
    static constexpr int Q_ROWP_DEPTH = MAX_N + 1;
    static constexpr int VEC_DEPTH    = MAX_N;

    std::vector<double> c_x, c_y, x_pos, y_pos;
    CSRMatrix Q;

    CGHwDriver() {
        ctx_ = new VerilatedContext;
        dut_ = new VCGTop(ctx_);
        std::memset(q_val_x_mem_,  0, sizeof(q_val_x_mem_));
        std::memset(q_col_x_mem_,  0, sizeof(q_col_x_mem_));
        std::memset(q_rowp_x_mem_, 0, sizeof(q_rowp_x_mem_));
        std::memset(q_val_y_mem_,  0, sizeof(q_val_y_mem_));
        std::memset(q_col_y_mem_,  0, sizeof(q_col_y_mem_));
        std::memset(q_rowp_y_mem_, 0, sizeof(q_rowp_y_mem_));
        std::memset(cx_mem_,       0, sizeof(cx_mem_));
        std::memset(cy_mem_,       0, sizeof(cy_mem_));
        std::memset(x_mem_,        0, sizeof(x_mem_));
        std::memset(y_mem_,        0, sizeof(y_mem_));
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

    void set_cx(int i, double v) { c_x[i]   = v; cx_mem_[i] = double_to_fp(v); }
    void set_cy(int i, double v) { c_y[i]   = v; cy_mem_[i] = double_to_fp(v); }
    void set_x (int i, double v) { x_pos[i] = v; x_mem_ [i] = double_to_fp(v); }
    void set_y (int i, double v) { y_pos[i] = v; y_mem_ [i] = double_to_fp(v); }

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
        // v6: write Q into both per-engine M10K trios.
        for (int j = 0; j < nnz; ++j) {
            int32_t v = double_to_fp(Q.vals[j]);
            int32_t c = Q.col_idx[j];
            q_val_x_mem_[j] = v;
            q_col_x_mem_[j] = c;
            q_val_y_mem_[j] = v;
            q_col_y_mem_[j] = c;
        }
        for (int i = 0; i <= Q.n; ++i) {
            q_rowp_x_mem_[i] = Q.row_ptr[i];
            q_rowp_y_mem_[i] = Q.row_ptr[i];
        }
    }

    void set_q_diag(int i, double v) {
        int j = q_diag_pos_[i];
        assert(j >= 0);
        Q.vals[j] = v;
        int32_t fp = double_to_fp(v);
        q_val_x_mem_[j] = fp;
        q_val_y_mem_[j] = fp;
    }

    uint64_t last_solve_cycles() const { return last_solve_cycles_; }

    void solve(int max_iter, double eps) {
        int n = Q.n;
        assert(n <= MAX_N);

        dut_->rst = 1;
        dut_->sw_go = 0;
        dut_->sw_done_ack = 0;
        dut_->max_iter = max_iter;
        dut_->eps_sq = double_to_fp(eps * eps);
        dut_->n = n;
        for (int i = 0; i < 5; ++i) tick();
        dut_->rst = 0;
        tick();

        last_solve_cycles_ = 0;
        dut_->sw_go = 1;
        tick();
        ++last_solve_cycles_;
        dut_->sw_go = 0;

        int timeout = 1000000;
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
    int32_t q_val_x_mem_  [Q_VAL_DEPTH];
    int32_t q_col_x_mem_  [Q_COL_DEPTH];
    int32_t q_rowp_x_mem_ [Q_ROWP_DEPTH];
    int32_t q_val_y_mem_  [Q_VAL_DEPTH];
    int32_t q_col_y_mem_  [Q_COL_DEPTH];
    int32_t q_rowp_y_mem_ [Q_ROWP_DEPTH];
    int32_t cx_mem_       [VEC_DEPTH];
    int32_t cy_mem_       [VEC_DEPTH];
    int32_t x_mem_        [VEC_DEPTH];
    int32_t y_mem_        [VEC_DEPTH];
    uint64_t last_solve_cycles_ = 0;

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

    static void service_port(
        bool cs_clken, uint32_t addr, bool wr, uint32_t wdata,
        int32_t* mem, int depth, uint32_t* readdata_out)
    {
        if (cs_clken && addr < static_cast<uint32_t>(depth)) {
            if (wr) {
                mem[addr] = static_cast<int32_t>(wdata);
            } else {
                *readdata_out = static_cast<uint32_t>(mem[addr]);
            }
        }
    }

    void tick() {
        dut_->clk = 0;
        dut_->eval();

        // Snapshot pre-edge port state for each of the ten slaves.
        bool     q_val_x_cs   = dut_->q_val_x_ram_chipselect  && dut_->q_val_x_ram_clken;
        uint32_t q_val_x_a    = dut_->q_val_x_ram_address;
        bool     q_val_x_wr   = dut_->q_val_x_ram_write;
        uint32_t q_val_x_wd   = dut_->q_val_x_ram_writedata;

        bool     q_col_x_cs   = dut_->q_col_x_ram_chipselect  && dut_->q_col_x_ram_clken;
        uint32_t q_col_x_a    = dut_->q_col_x_ram_address;
        bool     q_col_x_wr   = dut_->q_col_x_ram_write;
        uint32_t q_col_x_wd   = dut_->q_col_x_ram_writedata;

        bool     q_rowp_x_cs  = dut_->q_rowp_x_ram_chipselect && dut_->q_rowp_x_ram_clken;
        uint32_t q_rowp_x_a   = dut_->q_rowp_x_ram_address;
        bool     q_rowp_x_wr  = dut_->q_rowp_x_ram_write;
        uint32_t q_rowp_x_wd  = dut_->q_rowp_x_ram_writedata;

        bool     q_val_y_cs   = dut_->q_val_y_ram_chipselect  && dut_->q_val_y_ram_clken;
        uint32_t q_val_y_a    = dut_->q_val_y_ram_address;
        bool     q_val_y_wr   = dut_->q_val_y_ram_write;
        uint32_t q_val_y_wd   = dut_->q_val_y_ram_writedata;

        bool     q_col_y_cs   = dut_->q_col_y_ram_chipselect  && dut_->q_col_y_ram_clken;
        uint32_t q_col_y_a    = dut_->q_col_y_ram_address;
        bool     q_col_y_wr   = dut_->q_col_y_ram_write;
        uint32_t q_col_y_wd   = dut_->q_col_y_ram_writedata;

        bool     q_rowp_y_cs  = dut_->q_rowp_y_ram_chipselect && dut_->q_rowp_y_ram_clken;
        uint32_t q_rowp_y_a   = dut_->q_rowp_y_ram_address;
        bool     q_rowp_y_wr  = dut_->q_rowp_y_ram_write;
        uint32_t q_rowp_y_wd  = dut_->q_rowp_y_ram_writedata;

        bool     cx_cs   = dut_->cx_ram_chipselect && dut_->cx_ram_clken;
        uint32_t cx_a    = dut_->cx_ram_address;
        bool     cx_wr   = dut_->cx_ram_write;
        uint32_t cx_wd   = dut_->cx_ram_writedata;

        bool     cy_cs   = dut_->cy_ram_chipselect && dut_->cy_ram_clken;
        uint32_t cy_a    = dut_->cy_ram_address;
        bool     cy_wr   = dut_->cy_ram_write;
        uint32_t cy_wd   = dut_->cy_ram_writedata;

        bool     x_cs    = dut_->x_ram_chipselect  && dut_->x_ram_clken;
        uint32_t x_a     = dut_->x_ram_address;
        bool     x_wr    = dut_->x_ram_write;
        uint32_t x_wd    = dut_->x_ram_writedata;

        bool     y_cs    = dut_->y_ram_chipselect  && dut_->y_ram_clken;
        uint32_t y_a     = dut_->y_ram_address;
        bool     y_wr    = dut_->y_ram_write;
        uint32_t y_wd    = dut_->y_ram_writedata;

        dut_->clk = 1;
        dut_->eval();

        uint32_t q_val_x_rd  = dut_->q_val_x_ram_readdata;
        uint32_t q_col_x_rd  = dut_->q_col_x_ram_readdata;
        uint32_t q_rowp_x_rd = dut_->q_rowp_x_ram_readdata;
        uint32_t q_val_y_rd  = dut_->q_val_y_ram_readdata;
        uint32_t q_col_y_rd  = dut_->q_col_y_ram_readdata;
        uint32_t q_rowp_y_rd = dut_->q_rowp_y_ram_readdata;
        uint32_t cx_rd       = dut_->cx_ram_readdata;
        uint32_t cy_rd       = dut_->cy_ram_readdata;
        uint32_t x_rd        = dut_->x_ram_readdata;
        uint32_t y_rd        = dut_->y_ram_readdata;

        service_port(q_val_x_cs,  q_val_x_a,  q_val_x_wr,  q_val_x_wd,  q_val_x_mem_,  Q_VAL_DEPTH,  &q_val_x_rd);
        service_port(q_col_x_cs,  q_col_x_a,  q_col_x_wr,  q_col_x_wd,  q_col_x_mem_,  Q_COL_DEPTH,  &q_col_x_rd);
        service_port(q_rowp_x_cs, q_rowp_x_a, q_rowp_x_wr, q_rowp_x_wd, q_rowp_x_mem_, Q_ROWP_DEPTH, &q_rowp_x_rd);
        service_port(q_val_y_cs,  q_val_y_a,  q_val_y_wr,  q_val_y_wd,  q_val_y_mem_,  Q_VAL_DEPTH,  &q_val_y_rd);
        service_port(q_col_y_cs,  q_col_y_a,  q_col_y_wr,  q_col_y_wd,  q_col_y_mem_,  Q_COL_DEPTH,  &q_col_y_rd);
        service_port(q_rowp_y_cs, q_rowp_y_a, q_rowp_y_wr, q_rowp_y_wd, q_rowp_y_mem_, Q_ROWP_DEPTH, &q_rowp_y_rd);
        service_port(cx_cs,       cx_a,       cx_wr,       cx_wd,       cx_mem_,       VEC_DEPTH,    &cx_rd);
        service_port(cy_cs,       cy_a,       cy_wr,       cy_wd,       cy_mem_,       VEC_DEPTH,    &cy_rd);
        service_port(x_cs,        x_a,        x_wr,        x_wd,        x_mem_,        VEC_DEPTH,    &x_rd);
        service_port(y_cs,        y_a,        y_wr,        y_wd,        y_mem_,        VEC_DEPTH,    &y_rd);

        dut_->q_val_x_ram_readdata  = q_val_x_rd;
        dut_->q_col_x_ram_readdata  = q_col_x_rd;
        dut_->q_rowp_x_ram_readdata = q_rowp_x_rd;
        dut_->q_val_y_ram_readdata  = q_val_y_rd;
        dut_->q_col_y_ram_readdata  = q_col_y_rd;
        dut_->q_rowp_y_ram_readdata = q_rowp_y_rd;
        dut_->cx_ram_readdata       = cx_rd;
        dut_->cy_ram_readdata       = cy_rd;
        dut_->x_ram_readdata        = x_rd;
        dut_->y_ram_readdata        = y_rd;

        dut_->eval();
    }

    void refresh_xy_from_m10k(int n) {
        for (int i = 0; i < n; ++i) {
            x_pos[i] = fp_to_double(x_mem_[i]);
            y_pos[i] = fp_to_double(y_mem_[i]);
        }
    }
};
