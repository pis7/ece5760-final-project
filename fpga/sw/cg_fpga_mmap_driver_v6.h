// FPGA mmap driver for v6 CGTop (parallel x/y solve datapaths).
//
// Differs from cg_fpga_mmap_driver_v5.h in that v6 duplicates the
// Q-CSR triplet across two CGEngines, so the Qsys system exposes TEN
// on-chip RAM slaves: q_val_x/q_col_x/q_rowp_x for the x engine,
// q_val_y/q_col_y/q_rowp_y for the y engine, plus cx/cy/x/y. The
// driver mmaps each region separately and writes Q to both copies in
// load_q_initial(). The solve() protocol is identical to v5 -- one go
// pulse, one done.
//
// HW_MAX_N == 50 (default) bridge layout. Avalon-MM / AXI requires
// each slave's base to be naturally aligned to its size, so the four
// 16 KB Q slaves are packed contiguous from 0xC0000000, then the
// 4-KB-page-sized slaves follow above 0xC0010000:
//
//   q_val_x_ram  -> ARM 0xC0000000  (16 KB)
//   q_col_x_ram  -> ARM 0xC0004000  (16 KB)
//   q_val_y_ram  -> ARM 0xC0008000  (16 KB)
//   q_col_y_ram  -> ARM 0xC000C000  (16 KB)
//   q_rowp_x_ram -> ARM 0xC0010000  ( 4 KB mmap;  256 B Qsys)
//   q_rowp_y_ram -> ARM 0xC0011000  ( 4 KB mmap;  256 B Qsys)
//   cx_ram       -> ARM 0xC0012000
//   cy_ram       -> ARM 0xC0013000
//   x_ram        -> ARM 0xC0014000
//   y_ram        -> ARM 0xC0015000
//
// PIO control/status mapping is unchanged from v4/v5.

#pragma once

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

class CGHwDriver {
public:
#ifndef HW_MAX_N
    static constexpr int MAX_N       = 50;
#else
    static constexpr int MAX_N       = HW_MAX_N;
#endif
    static_assert(MAX_N == 50,
        "v6 mmap driver only ships with a Qsys layout for MAX_N=50; "
        "regenerate the Qsys system and add a new branch in the "
        "bridge-base block below before using a different MAX_N.");
    static constexpr int TOTAL_BITS  = 27;
#ifndef HW_FRAC_BITS
    static constexpr int FRAC_BITS   = 14;
#else
    static constexpr int FRAC_BITS   = HW_FRAC_BITS;
#endif
    static constexpr int INT_BITS    = TOTAL_BITS - FRAC_BITS;
    static constexpr int FRAC_SCALE  = 1 << FRAC_BITS;
    static constexpr int32_t BIT_MASK =
        static_cast<int32_t>((1LL << TOTAL_BITS) - 1);

    // -- Per-slave depths ------------------------------------------------
    static constexpr int Q_VAL_DEPTH  = MAX_N * MAX_N;
    static constexpr int Q_COL_DEPTH  = MAX_N * MAX_N;
    static constexpr int Q_ROWP_DEPTH = MAX_N + 1;
    static constexpr int VEC_DEPTH    = MAX_N;

    // -- Per-slave physical bases (ARM h2f bridge) -- 50-cell layout.
    // Each base is naturally aligned to the slave's size (Avalon-MM /
    // AXI requirement). 16 KB Q slaves packed contiguous from
    // 0xC0000000; 4-KB-page slaves above 0xC0010000.
    static constexpr off_t Q_VAL_X_BRIDGE_BASE  = 0xC0000000;
    static constexpr off_t Q_COL_X_BRIDGE_BASE  = 0xC0004000;
    static constexpr off_t Q_VAL_Y_BRIDGE_BASE  = 0xC0008000;
    static constexpr off_t Q_COL_Y_BRIDGE_BASE  = 0xC000C000;
    static constexpr off_t Q_ROWP_X_BRIDGE_BASE = 0xC0010000;
    static constexpr off_t Q_ROWP_Y_BRIDGE_BASE = 0xC0011000;
    static constexpr off_t CX_BRIDGE_BASE       = 0xC0012000;
    static constexpr off_t CY_BRIDGE_BASE       = 0xC0013000;
    static constexpr off_t X_BRIDGE_BASE        = 0xC0014000;
    static constexpr off_t Y_BRIDGE_BASE        = 0xC0015000;
    static constexpr size_t Q_VAL_SIZE_BYTES    = 0x4000;  // 16 KB
    static constexpr size_t Q_COL_SIZE_BYTES    = 0x4000;  // 16 KB
    static constexpr size_t Q_ROWP_SIZE_BYTES   = 0x1000;  // 4 KB mmap
    static constexpr size_t VEC_SIZE_BYTES      = 0x1000;  // 4 KB mmap

    // -- PIO bridge (same as v4/v5) --------------------------------------
    static constexpr off_t  H2F_LW_BRIDGE_BASE = 0xFF200000;
    static constexpr size_t H2F_LW_SPAN        = 0x00005000;
    static constexpr off_t  CG_CTRL_OFFSET     = 0x00;
    static constexpr off_t  CG_MAX_ITER_OFFSET = 0x10;
    static constexpr off_t  CG_EPS_SQ_OFFSET   = 0x20;
    static constexpr off_t  CG_N_OFFSET        = 0x30;
    static constexpr off_t  CG_STATUS_OFFSET   = 0x40;

    static constexpr uint32_t CTRL_SW_GO       = 1u << 0;
    static constexpr uint32_t CTRL_SW_DONE_ACK = 1u << 1;
    static constexpr uint32_t CTRL_SOFT_RST    = 1u << 2;

    static constexpr uint32_t STATUS_SW_DONE   = 1u << 0;

    std::vector<double> c_x, c_y, x_pos, y_pos;
    CSRMatrix Q;

    CGHwDriver() {
        mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd_ < 0) {
            std::perror("open /dev/mem (run as root?)");
            std::exit(1);
        }

        q_val_x_  = map_region("q_val_x",  Q_VAL_X_BRIDGE_BASE,  Q_VAL_SIZE_BYTES);
        q_col_x_  = map_region("q_col_x",  Q_COL_X_BRIDGE_BASE,  Q_COL_SIZE_BYTES);
        q_rowp_x_ = map_region("q_rowp_x", Q_ROWP_X_BRIDGE_BASE, Q_ROWP_SIZE_BYTES);
        q_val_y_  = map_region("q_val_y",  Q_VAL_Y_BRIDGE_BASE,  Q_VAL_SIZE_BYTES);
        q_col_y_  = map_region("q_col_y",  Q_COL_Y_BRIDGE_BASE,  Q_COL_SIZE_BYTES);
        q_rowp_y_ = map_region("q_rowp_y", Q_ROWP_Y_BRIDGE_BASE, Q_ROWP_SIZE_BYTES);
        cx_       = map_region("cx",       CX_BRIDGE_BASE,       VEC_SIZE_BYTES);
        cy_       = map_region("cy",       CY_BRIDGE_BASE,       VEC_SIZE_BYTES);
        x_        = map_region("x",        X_BRIDGE_BASE,        VEC_SIZE_BYTES);
        y_        = map_region("y",        Y_BRIDGE_BASE,        VEC_SIZE_BYTES);

        lw_map_ = mmap(nullptr, H2F_LW_SPAN, PROT_READ | PROT_WRITE,
                       MAP_SHARED, mem_fd_, H2F_LW_BRIDGE_BASE);
        if (lw_map_ == MAP_FAILED) {
            std::perror("mmap lw bridge");
            std::exit(1);
        }
        auto lw_base = reinterpret_cast<volatile uint8_t*>(lw_map_);
        ctrl_     = reinterpret_cast<volatile uint32_t*>(lw_base + CG_CTRL_OFFSET);
        max_iter_ = reinterpret_cast<volatile uint32_t*>(lw_base + CG_MAX_ITER_OFFSET);
        eps_sq_   = reinterpret_cast<volatile uint32_t*>(lw_base + CG_EPS_SQ_OFFSET);
        n_reg_    = reinterpret_cast<volatile uint32_t*>(lw_base + CG_N_OFFSET);
        status_   = reinterpret_cast<volatile uint32_t*>(lw_base + CG_STATUS_OFFSET);

        *ctrl_ = CTRL_SOFT_RST;
        usleep(1000);
        *ctrl_ = 0;

        for (int i = 0; i < Q_VAL_DEPTH;  ++i) { q_val_x_[i]  = 0; q_val_y_[i]  = 0; }
        for (int i = 0; i < Q_COL_DEPTH;  ++i) { q_col_x_[i]  = 0; q_col_y_[i]  = 0; }
        for (int i = 0; i < Q_ROWP_DEPTH; ++i) { q_rowp_x_[i] = 0; q_rowp_y_[i] = 0; }
        for (int i = 0; i < VEC_DEPTH;    ++i) {
            cx_[i] = 0; cy_[i] = 0; x_[i] = 0; y_[i] = 0;
        }
    }

    ~CGHwDriver() {
        unmap(q_val_x_,  Q_VAL_SIZE_BYTES);
        unmap(q_col_x_,  Q_COL_SIZE_BYTES);
        unmap(q_rowp_x_, Q_ROWP_SIZE_BYTES);
        unmap(q_val_y_,  Q_VAL_SIZE_BYTES);
        unmap(q_col_y_,  Q_COL_SIZE_BYTES);
        unmap(q_rowp_y_, Q_ROWP_SIZE_BYTES);
        unmap(cx_,       VEC_SIZE_BYTES);
        unmap(cy_,       VEC_SIZE_BYTES);
        unmap(x_,        VEC_SIZE_BYTES);
        unmap(y_,        VEC_SIZE_BYTES);
        if (lw_map_ != MAP_FAILED) munmap(lw_map_, H2F_LW_SPAN);
        if (mem_fd_ >= 0) close(mem_fd_);
    }

    void resize_n(int n) {
        c_x.assign(n, 0.0);
        c_y.assign(n, 0.0);
        x_pos.assign(n, 0.0);
        y_pos.assign(n, 0.0);
    }

    void set_cx(int i, double v) { c_x[i]   = v; cx_[i] = static_cast<uint32_t>(double_to_fp(v)); }
    void set_cy(int i, double v) { c_y[i]   = v; cy_[i] = static_cast<uint32_t>(double_to_fp(v)); }
    void set_x (int i, double v) { x_pos[i] = v; x_ [i] = static_cast<uint32_t>(double_to_fp(v)); }
    void set_y (int i, double v) { y_pos[i] = v; y_ [i] = static_cast<uint32_t>(double_to_fp(v)); }

    void load_q_initial(const CSRMatrix& src) {
        Q = src;
        int nnz = Q.nnz();
        assert(nnz <= Q_VAL_DEPTH);
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
            uint32_t v = static_cast<uint32_t>(double_to_fp(Q.vals[j]));
            uint32_t c = static_cast<uint32_t>(Q.col_idx[j]);
            q_val_x_[j] = v;
            q_col_x_[j] = c;
            q_val_y_[j] = v;
            q_col_y_[j] = c;
        }
        for (int i = 0; i <= Q.n; ++i) {
            uint32_t r = static_cast<uint32_t>(Q.row_ptr[i]);
            q_rowp_x_[i] = r;
            q_rowp_y_[i] = r;
        }
    }

    void set_q_diag(int i, double v) {
        int j = q_diag_pos_[i];
        assert(j >= 0);
        Q.vals[j] = v;
        uint32_t fp = static_cast<uint32_t>(double_to_fp(v));
        q_val_x_[j] = fp;
        q_val_y_[j] = fp;
    }

    void solve(int max_iter, double eps) {
        int n = Q.n;
        assert(n <= MAX_N);

        *max_iter_ = static_cast<uint32_t>(max_iter);
        *eps_sq_   = static_cast<uint32_t>(double_to_fp(eps * eps));
        *n_reg_    = static_cast<uint32_t>(n);

        *ctrl_ = CTRL_SW_GO;
        while ((*status_ & STATUS_SW_DONE) == 0) { /* spin */ }

        *ctrl_ = CTRL_SW_DONE_ACK;
        while ((*status_ & STATUS_SW_DONE) != 0) { /* spin */ }
        *ctrl_ = 0;

        refresh_xy_from_sram(n);
    }

    uint64_t last_solve_cycles() const { return 0; }

private:
    int   mem_fd_   = -1;
    void* lw_map_   = MAP_FAILED;
    volatile uint32_t* q_val_x_  = nullptr;
    volatile uint32_t* q_col_x_  = nullptr;
    volatile uint32_t* q_rowp_x_ = nullptr;
    volatile uint32_t* q_val_y_  = nullptr;
    volatile uint32_t* q_col_y_  = nullptr;
    volatile uint32_t* q_rowp_y_ = nullptr;
    volatile uint32_t* cx_       = nullptr;
    volatile uint32_t* cy_       = nullptr;
    volatile uint32_t* x_        = nullptr;
    volatile uint32_t* y_        = nullptr;
    volatile uint32_t* ctrl_     = nullptr;
    volatile uint32_t* max_iter_ = nullptr;
    volatile uint32_t* eps_sq_   = nullptr;
    volatile uint32_t* n_reg_    = nullptr;
    volatile uint32_t* status_   = nullptr;

    std::vector<int> q_diag_pos_;

    volatile uint32_t* map_region(const char* name, off_t bridge_base, size_t bytes) {
        void* m = mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                       MAP_SHARED, mem_fd_, bridge_base);
        if (m == MAP_FAILED) {
            std::fprintf(stderr, "mmap %s @ 0x%lx (%zu B)\n",
                         name, static_cast<long>(bridge_base), bytes);
            std::perror("mmap");
            std::exit(1);
        }
        return reinterpret_cast<volatile uint32_t*>(m);
    }

    static void unmap(volatile uint32_t* p, size_t bytes) {
        if (p) munmap(const_cast<uint32_t*>(p), bytes);
    }

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

    void refresh_xy_from_sram(int n) {
        for (int i = 0; i < n; ++i) {
            x_pos[i] = fp_to_double(static_cast<int32_t>(x_[i]));
            y_pos[i] = fp_to_double(static_cast<int32_t>(y_[i]));
        }
    }
};
