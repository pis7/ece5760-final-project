// FPGA mmap driver for v5 CGTop (multi-block on-chip RAM topology).
//
// Differs from cg_fpga_mmap_driver_v4.h in that v5's Qsys design exposes
// SEVEN independent on-chip RAM slaves (one each for q_val, q_col,
// q_rowp, cx, cy, x, y) instead of one shared SRAM. The driver mmaps
// each slave separately and writes/reads at local offset 0 within each
// region.
//
// Two Qsys layouts are supported, picked at compile time by HW_MAX_N:
//
// HW_MAX_N == 50 (default): 16 KB Q-{val,col} slaves, 4 KB-page slaves
// for the rest. h2f offsets:
//
//   q_val_ram  -> h2f 0x0000  (4096 words = 16 KB)  -> ARM 0xC0000000
//   q_col_ram  -> h2f 0x4000  (4096 words = 16 KB)  -> ARM 0xC0004000
//   q_rowp_ram -> h2f 0x8000  (  64 words =  256 B) -> ARM 0xC0008000
//   cx_ram     -> h2f 0x9000  (  64 words =  256 B) -> ARM 0xC0009000
//   cy_ram     -> h2f 0xA000  (  64 words =  256 B) -> ARM 0xC000A000
//   x_ram      -> h2f 0xB000  (  64 words =  256 B) -> ARM 0xC000B000
//   y_ram      -> h2f 0xC000  (  64 words =  256 B) -> ARM 0xC000C000
//
// HW_MAX_N == 75: 32 KB Q-{val,col} slaves (5625 words = 22500 B used),
// 512 B "real" slaves for the rest -- but each is still given its own
// 4 KB page for mmap so the kernel can map them as separate regions.
//
//   q_val_ram  -> h2f 0x00000 (8192 words = 32 KB)   -> ARM 0xC0000000
//   q_col_ram  -> h2f 0x08000 (8192 words = 32 KB)   -> ARM 0xC0008000
//   q_rowp_ram -> h2f 0x10000 ( 128 words =  512 B)  -> ARM 0xC0010000
//   cx_ram     -> h2f 0x11000 ( 128 words =  512 B)  -> ARM 0xC0011000
//   cy_ram     -> h2f 0x12000 ( 128 words =  512 B)  -> ARM 0xC0012000
//   x_ram      -> h2f 0x13000 ( 128 words =  512 B)  -> ARM 0xC0013000
//   y_ram      -> h2f 0x14000 ( 128 words =  512 B)  -> ARM 0xC0014000
//
// Other HW_MAX_N values are rejected at compile time -- regenerate the
// Qsys system and add a new branch here.
//
// PIO control/status mapping is unchanged from v4 (same h2f_lw bridge).

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
    static_assert(MAX_N == 50 || MAX_N == 75,
        "v5 mmap driver only has Qsys layouts for MAX_N=50 or MAX_N=75; "
        "regenerate the Qsys system and add a new branch in the bridge-base "
        "block below before using a different MAX_N.");
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

    // -- Per-slave physical bases (ARM h2f bridge) ------------------------
    // Tight pack from ARM 0xC0000000 with each slave page-aligned (4 KB+).
    // The 50-cell layout uses 16 KB Q-{val,col} slaves; the 75-cell layout
    // doubles them to 32 KB and bumps the per-vec slave allocation from
    // 256 B to 512 B (still mmap'd as 4 KB pages).
#if HW_MAX_N == 75
    static constexpr off_t Q_VAL_BRIDGE_BASE  = 0xC0000000;
    static constexpr off_t Q_COL_BRIDGE_BASE  = 0xC0008000;
    static constexpr off_t Q_ROWP_BRIDGE_BASE = 0xC0010000;
    static constexpr off_t CX_BRIDGE_BASE     = 0xC0011000;
    static constexpr off_t CY_BRIDGE_BASE     = 0xC0012000;
    static constexpr off_t X_BRIDGE_BASE      = 0xC0013000;
    static constexpr off_t Y_BRIDGE_BASE      = 0xC0014000;
    static constexpr size_t Q_VAL_SIZE_BYTES  = 0x8000;  // 32 KB
    static constexpr size_t Q_COL_SIZE_BYTES  = 0x8000;  // 32 KB
    static constexpr size_t Q_ROWP_SIZE_BYTES = 0x1000;  // 4 KB mmap (512 B Qsys)
    static constexpr size_t VEC_SIZE_BYTES    = 0x1000;  // 4 KB mmap (512 B Qsys)
#else
    // Default: 50-cell layout.
    static constexpr off_t Q_VAL_BRIDGE_BASE  = 0xC0000000;
    static constexpr off_t Q_COL_BRIDGE_BASE  = 0xC0004000;
    static constexpr off_t Q_ROWP_BRIDGE_BASE = 0xC0008000;
    static constexpr off_t CX_BRIDGE_BASE     = 0xC0009000;
    static constexpr off_t CY_BRIDGE_BASE     = 0xC000A000;
    static constexpr off_t X_BRIDGE_BASE      = 0xC000B000;
    static constexpr off_t Y_BRIDGE_BASE      = 0xC000C000;
    static constexpr size_t Q_VAL_SIZE_BYTES  = 0x4000;  // 16 KB
    static constexpr size_t Q_COL_SIZE_BYTES  = 0x4000;  // 16 KB
    static constexpr size_t Q_ROWP_SIZE_BYTES = 0x1000;  // 4 KB mmap (256 B Qsys)
    static constexpr size_t VEC_SIZE_BYTES    = 0x1000;  // 4 KB mmap (256 B Qsys)
#endif

    // -- PIO bridge (same as v4) ------------------------------------------
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

        q_val_  = map_region("q_val",  Q_VAL_BRIDGE_BASE,  Q_VAL_SIZE_BYTES);
        q_col_  = map_region("q_col",  Q_COL_BRIDGE_BASE,  Q_COL_SIZE_BYTES);
        q_rowp_ = map_region("q_rowp", Q_ROWP_BRIDGE_BASE, Q_ROWP_SIZE_BYTES);
        cx_     = map_region("cx",     CX_BRIDGE_BASE,     VEC_SIZE_BYTES);
        cy_     = map_region("cy",     CY_BRIDGE_BASE,     VEC_SIZE_BYTES);
        x_      = map_region("x",      X_BRIDGE_BASE,      VEC_SIZE_BYTES);
        y_      = map_region("y",      Y_BRIDGE_BASE,      VEC_SIZE_BYTES);

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

        for (int i = 0; i < Q_VAL_DEPTH;  ++i) q_val_[i]  = 0;
        for (int i = 0; i < Q_COL_DEPTH;  ++i) q_col_[i]  = 0;
        for (int i = 0; i < Q_ROWP_DEPTH; ++i) q_rowp_[i] = 0;
        for (int i = 0; i < VEC_DEPTH;    ++i) {
            cx_[i] = 0; cy_[i] = 0; x_[i] = 0; y_[i] = 0;
        }
    }

    ~CGHwDriver() {
        unmap(q_val_,  Q_VAL_SIZE_BYTES);
        unmap(q_col_,  Q_COL_SIZE_BYTES);
        unmap(q_rowp_, Q_ROWP_SIZE_BYTES);
        unmap(cx_,     VEC_SIZE_BYTES);
        unmap(cy_,     VEC_SIZE_BYTES);
        unmap(x_,      VEC_SIZE_BYTES);
        unmap(y_,      VEC_SIZE_BYTES);
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
        for (int j = 0; j < nnz; ++j) {
            q_val_[j] = static_cast<uint32_t>(double_to_fp(Q.vals[j]));
            q_col_[j] = static_cast<uint32_t>(Q.col_idx[j]);
        }
        for (int i = 0; i <= Q.n; ++i)
            q_rowp_[i] = static_cast<uint32_t>(Q.row_ptr[i]);
    }

    void set_q_diag(int i, double v) {
        int j = q_diag_pos_[i];
        assert(j >= 0);
        Q.vals[j] = v;
        q_val_[j] = static_cast<uint32_t>(double_to_fp(v));
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
    volatile uint32_t* q_val_  = nullptr;
    volatile uint32_t* q_col_  = nullptr;
    volatile uint32_t* q_rowp_ = nullptr;
    volatile uint32_t* cx_     = nullptr;
    volatile uint32_t* cy_     = nullptr;
    volatile uint32_t* x_      = nullptr;
    volatile uint32_t* y_      = nullptr;
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
