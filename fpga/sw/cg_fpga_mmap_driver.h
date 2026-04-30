// FPGA mmap driver for CGTop. Same interface as the Verilator-based
// CGHwDriver in sw-baseline-c/cg_verilator_driver.h, but talks to the
// real DE1-SoC via /dev/mem instead of a simulated model.
//
// Assumes the Qsys layout:
//   - On-chip SRAM (32 KB) is mapped via the HPS h2f AXI bridge
//     (ARM base 0xC0000000, slave offset 0x00000000).
//   - Control/status PIOs are on the h2f_lw bridge
//     (ARM base 0xFF200000) at the following slave offsets:
//        cg_ctrl     0x00  (ARM->FPGA, {5'b0, rst, sw_done_ack, sw_go})
//        cg_max_iter 0x10  (ARM->FPGA, 32b)
//        cg_eps_sq   0x20  (ARM->FPGA, 32b, fixed-point)
//        cg_n        0x30  (ARM->FPGA, 32b)
//        cg_status   0x40  (FPGA->ARM, {7'b0, sw_done})
//
// Depends on the CSRMatrix struct defined by placer.cpp; include this
// header AFTER CSRMatrix is declared.

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
    // -- Fixed-point config (must match Verilog CGTop parameters) -------------
    static constexpr int MAX_N       = 50;
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

    // -- SRAM word-offset layout (must match CGTop base addresses) -----------
    static constexpr int Q_VAL_BASE  = 0;
    static constexpr int Q_COL_BASE  = MAX_N * MAX_N;
    static constexpr int Q_ROWP_BASE = 2 * MAX_N * MAX_N;
    static constexpr int CX_X_BASE   = 2 * MAX_N * MAX_N + MAX_N + 1;
    static constexpr int CX_Y_BASE   = 2 * MAX_N * MAX_N + 2 * MAX_N + 1;
    static constexpr int X_BASE      = 2 * MAX_N * MAX_N + 3 * MAX_N + 1;
    static constexpr int Y_BASE      = 2 * MAX_N * MAX_N + 4 * MAX_N + 1;
    static constexpr int TOTAL_WORDS = 2 * MAX_N * MAX_N + 5 * MAX_N + 1;

    // -- Physical-address layout ----------------------------------------------
    static constexpr off_t  H2F_BRIDGE_BASE    = 0xC0000000;
    static constexpr off_t  H2F_LW_BRIDGE_BASE = 0xFF200000;
    static constexpr size_t H2F_LW_SPAN        = 0x00005000;
    static constexpr off_t  SRAM_OFFSET        = 0x00000000;
    static constexpr size_t SRAM_SIZE_BYTES    = 0x00008000;  // 32 KB

    // h2f_lw offsets (byte-addressed in ARM space)
    static constexpr off_t CG_CTRL_OFFSET     = 0x00;
    static constexpr off_t CG_MAX_ITER_OFFSET = 0x10;
    static constexpr off_t CG_EPS_SQ_OFFSET   = 0x20;
    static constexpr off_t CG_N_OFFSET        = 0x30;
    static constexpr off_t CG_STATUS_OFFSET   = 0x40;

    // cg_ctrl bit positions
    static constexpr uint32_t CTRL_SW_GO       = 1u << 0;
    static constexpr uint32_t CTRL_SW_DONE_ACK = 1u << 1;
    static constexpr uint32_t CTRL_SOFT_RST    = 1u << 2;

    // cg_status bit positions
    static constexpr uint32_t STATUS_SW_DONE   = 1u << 0;

    CGHwDriver() {
        mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd_ < 0) {
            std::perror("open /dev/mem (run as root?)");
            std::exit(1);
        }

        sram_map_ = mmap(nullptr, SRAM_SIZE_BYTES, PROT_READ | PROT_WRITE,
                         MAP_SHARED, mem_fd_,
                         H2F_BRIDGE_BASE + SRAM_OFFSET);
        if (sram_map_ == MAP_FAILED) {
            std::perror("mmap sram");
            std::exit(1);
        }
        sram_ = reinterpret_cast<volatile uint32_t*>(sram_map_);

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

        // Soft reset the solver so we always start from a clean IDLE.
        *ctrl_ = CTRL_SOFT_RST;
        usleep(1000);
        *ctrl_ = 0;
    }

    ~CGHwDriver() {
        if (sram_map_ != MAP_FAILED) munmap(sram_map_, SRAM_SIZE_BYTES);
        if (lw_map_   != MAP_FAILED) munmap(lw_map_,   H2F_LW_SPAN);
        if (mem_fd_ >= 0) close(mem_fd_);
    }

    // Solve Qx = -cx and Qy = -cy via the FPGA. Modifies x and y in place.
    void solve(const CSRMatrix& Q,
               const std::vector<double>& cx,
               const std::vector<double>& cy,
               std::vector<double>& x,
               std::vector<double>& y,
               int max_iter, double eps) {
        int n = Q.n;
        assert(n <= MAX_N);

        pack_memory(Q, cx, cy, x, y, n);

        *max_iter_ = static_cast<uint32_t>(max_iter);
        *eps_sq_   = static_cast<uint32_t>(double_to_fp(eps * eps));
        *n_reg_    = static_cast<uint32_t>(n);

        // Pulse sw_go, then wait for sw_done.
        *ctrl_ = CTRL_SW_GO;
        while ((*status_ & STATUS_SW_DONE) == 0) { /* spin */ }

        // Ack: drop sw_go, raise sw_done_ack, wait for the FSM to
        // return to IDLE (sw_done falls), then drop the ack.
        *ctrl_ = CTRL_SW_DONE_ACK;
        while ((*status_ & STATUS_SW_DONE) != 0) { /* spin */ }
        *ctrl_ = 0;

        unpack_results(x, y, n);
    }

    // No on-FPGA cycle counter is wired up (no PIO for it in Qsys/FPGATop).
    // Return 0 as a sentinel; placer.cpp treats 0 as "no measurement".
    uint64_t last_solve_cycles() const { return 0; }

private:
    int   mem_fd_   = -1;
    void* sram_map_ = MAP_FAILED;
    void* lw_map_   = MAP_FAILED;
    volatile uint32_t* sram_     = nullptr;
    volatile uint32_t* ctrl_     = nullptr;
    volatile uint32_t* max_iter_ = nullptr;
    volatile uint32_t* eps_sq_   = nullptr;
    volatile uint32_t* n_reg_    = nullptr;
    volatile uint32_t* status_   = nullptr;

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

    void pack_memory(const CSRMatrix& Q,
                     const std::vector<double>& cx,
                     const std::vector<double>& cy,
                     const std::vector<double>& x,
                     const std::vector<double>& y,
                     int n) {
        // Zero the solver's footprint in the SRAM.
        for (int i = 0; i < TOTAL_WORDS; ++i) sram_[i] = 0;

        int nnz = Q.nnz();
        assert(nnz <= MAX_N * MAX_N);
        for (int j = 0; j < nnz; ++j) {
            sram_[Q_VAL_BASE + j] = static_cast<uint32_t>(double_to_fp(Q.vals[j]));
            sram_[Q_COL_BASE + j] = static_cast<uint32_t>(Q.col_idx[j]);
        }
        for (int i = 0; i <= n; ++i) {
            sram_[Q_ROWP_BASE + i] = static_cast<uint32_t>(Q.row_ptr[i]);
        }
        for (int i = 0; i < n; ++i) {
            sram_[CX_X_BASE + i] = static_cast<uint32_t>(double_to_fp(cx[i]));
            sram_[CX_Y_BASE + i] = static_cast<uint32_t>(double_to_fp(cy[i]));
            sram_[X_BASE    + i] = static_cast<uint32_t>(double_to_fp(x[i]));
            sram_[Y_BASE    + i] = static_cast<uint32_t>(double_to_fp(y[i]));
        }
    }

    void unpack_results(std::vector<double>& x, std::vector<double>& y, int n) {
        for (int i = 0; i < n; ++i) {
            x[i] = fp_to_double(static_cast<int32_t>(sram_[X_BASE + i]));
            y[i] = fp_to_double(static_cast<int32_t>(sram_[Y_BASE + i]));
        }
    }
};
