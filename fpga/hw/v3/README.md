# v3 CG Solver - Design Summary

Iterates on [v2](../v2/) with three performance improvements that
don't add routing complexity. Same toplevel, FSM, and SIMD model; the
diffs are localized to FpMath.v and LinAlg.v::SPMV.

## Changes vs v2

| Area | v2 | v3 | Win |
| --- | --- | --- | --- |
| FpDiv | shift-subtract, ~63 cycles | LZC + 256x17 ROM + 2 NR iters, ~8 cycles | ~8x faster divide, twice per CG iter |
| SPMV row prologue | reads `rp_ptr[i]` and `rp_ptr[i+1]` per row (4 cycles) | row 0 reads both; rows 1..n-1 carry `rp_hi -> rp_lo` and read only `rp_ptr[i+1]` | 2 cycles saved per row except row 0 |
| SPMV inner loop | 5 cycles/nnz serial | **2 cycles/nnz pipelined** (val/col reads overlap; one MAC fires every other cycle once warm) | per row: `2*nnz + 2` cycles vs `5*nnz`; ~2.5x on the inner loop |

End-to-end on Verilator HW CG (Q1.14 fixed-point throughout):

| Benchmark | v2 HPWL | v3 HPWL | v2 cycles | v3 cycles | Speedup |
| --- | --- | --- | --- | --- | --- |
| tiny1 | 28737 | **20626** (-28%) | 54672 | 25274 | **2.16x** |
| tiny2 | 42196 | **31277** (-26%) | 60112 | 26790 | **2.24x** |
| tiny3 | 22266 | **5692** (-74%) | 238216 | 114810 | **2.07x** |

Cycle counts are the total of all `sw_go`-to-`sw_done` intervals
across every CG solve in a placer run (printed by the Verilator placer
after the timing summary).

## Synth/sim coupling

The [DPI golden](../../../sw-baseline-c/cg_golden.h) supports both
divide algorithms via `#ifdef CG_GOLDEN_USE_NR`. Default is
shift-subtract (matches v1, v2). The v3 testbench target
`VCGTop_tb_v3` and the HW CG placer with `-DHW_CG_VERSION=v3` (now
the default) build with `-DCG_GOLDEN_USE_NR` so DPI compares stay
bit-exact against the v3 RTL.

## A note on the C++ M10K shim

The Verilator-driver in [cg_hw_driver.h](../../../sw-baseline-c/cg_hw_driver.h)
samples its M10K shim's address `bus inputs` **before** the rising-edge
eval, matching what an SV `always_ff @(posedge clk)` shim would see.
Sampling after the rising edge would read the *next* state's address
decode — invisible when an address holds for two cycles (v1, v2, and
the v3 row prologue) but disastrous when the inner loop drives a
different address every cycle. Two earlier 2-cyc/nnz pipelining
attempts were reverted because of this shim bug; the RTL itself was
fine. The fix is in `tick()`; do not move the address-sampling block
back below the `clk=1; eval()` call.

Synthesizable Verilog implementation of the conjugate-gradient (CG) inner
loop, targeting the DE1-SoC (Cyclone V FPGA). Replaces the combinational
v1 reference with a sequential, DSP-mapped, parameterizable-SIMD design
that runs against Qsys on-chip SRAM via an Avalon slave port.

## Files

| File | Role |
| --- | --- |
| [CGTop.v](CGTop.v) | Toplevel: Avalon slave pass-through, instantiates CGCtrl + CGDpath |
| [CGCtrl.v](CGCtrl.v) | Single flat FSM driving every mux, latch, and val/rdy handshake |
| [CGDpath.v](CGDpath.v) | Pure datapath: register files, scalars, linalg submodule instances, muxes |
| [LinAlg.v](LinAlg.v) | VecDot, AXPY, SPMV (with rp_lo/rp_hi collapse), VecNegSub kernels |
| [FpMath.v](FpMath.v) | FpMul, FpMulWide, FpDiv (Newton-Raphson reciprocal, 2 NR iters, ~8 cycles) |

## Top-level architecture

```
        sw_go / sw_done            Avalon slave to on-chip SRAM
           |    ^                   |
           v    |                   v
        +-----------+         +-----------+
        |  CGCtrl   |<------->|  CGDpath  |---->[VecDot, AXPY, SPMV, FpDiv]
        |  (FSM)    | ctrl/   |  (regs +  |
        +-----------+ status  +-----------+
                              | RFs: d_reg, r_reg, x_vec_reg, cx_reg, q_buf
                              | scalars: rr_reg, rr_new, dq, alpha, beta, iter
```

- ARM blocks on `sw_done` while the FPGA owns the Avalon port. No
  intermediate BRAM between datapath and SRAM.
- CGCtrl is the only FSM at this level; CGDpath has no state machine of
  its own (each linalg submodule has its own internal Ctrl/Dpath split).

## Fixed-point format

- `p_total_bits = 27` (`p_int_bits=13`, `p_frac_bits=14`), targeting the
  Cyclone V 27x27 DSP block via `(* multstyle = "dsp" *)`.
- `p_acc_bits = 48` for VecDot/SPMV accumulators and FpDiv operands -
  products accumulate without early truncation.

## Memory layout (parameterized by `p_max_n`)

Single Avalon-mapped on-chip SRAM, addresses in 32-bit words:

| Region | Base | Size |
| --- | --- | --- |
| Q values (CSR) | 0 | `p_max_n^2` |
| Q col_idx (CSR) | `p_max_n^2` | `p_max_n^2` |
| Q row_ptr (CSR) | `2*p_max_n^2` | `p_max_n + 1` |
| cx (x dim) | + `p_max_n` | `p_max_n` |
| cx (y dim) | + `p_max_n` | `p_max_n` |
| x out | + `p_max_n` | `p_max_n` |
| y out | + `p_max_n` | `p_max_n` |

`sel_y` flips between x-dim and y-dim base addresses so a single FSM
runs both solves back-to-back without doubling state count.

## SIMD model: `p_lanes`

- `p_lanes` parameter governs the width of VecDot and AXPY.
  `num_groups = ceil(n / p_lanes)`.
- Streaming phases (VNS, COPY_D, VDOT feeds, AXPY feeds) use
  `stream_idx` as a *group counter*; each handshake covers `p_lanes`
  elements with `elem = (group << log2(p_lanes)) + k`.
- Per-element phases (LD, WB, SPMV COLLECT) use `stream_idx` as an
  *element counter* and drive lane 0 only (other lanes' `we` gated off).
- Out-of-range lanes in the final partial group: CGCtrl drives
  `rd_a_valid[k] = rd_b_valid[k] = 0`, CGDpath returns zero on those
  reads, and `we[k]` is masked off on writeback. Zero contributes
  nothing to VecDot/AXPY.
- DSP count: VecDot `p_lanes` + AXPY `p_lanes` + SPMV 1.
  At `p_lanes=4` -> 9 DSPs.

## Register files (in CGDpath)

Five flip-flop unpacked arrays of size `p_max_n`, each
`p_total_bits`-wide:

| RF | Role |
| --- | --- |
| `d_reg` | search direction |
| `r_reg` | residual |
| `x_vec_reg` | iterate (writes to SRAM at WB) |
| `cx_reg` | RHS (`-c`) loaded from SRAM at LD |
| `q_buf` | SPMV result `Q*d` (or `Q*x_init` for the initial residual) |

Three combinational read ports: two `p_lanes`-wide addressed (`rd_a`,
`rd_b`), one single-lane indexed by SPMV (`rd_vec`). One `p_lanes`-wide
write port with per-lane enable.

Scalar latches: `rr_reg`, `rr_new_latched`, `dq_latched`, `alpha`,
`beta`, `iter`.

## CGCtrl FSM (high-level phases)

```
IDLE -> PREP -> LD_X -> LD_CX -> SPMV_INIT -> VNS_R -> COPY_D
        -> VDOT_INIT -> RR_REG_COPY
        -> [ SPMV_RUN -> VDOT_DQ -> DIV_A -> AXPY_X -> AXPY_R
             -> VDOT_RR -> DIV_B -> AXPY_D -> RUN_CHECK ]*
        -> WB_WRITE -> (sel_y=1, swap to Y phase) -> ...
        -> CG_DONE
```

Per CG iteration the FSM walks one full sweep through SPMV, two
divides, three AXPYs, and two VecDots. Convergence test in `RUN_CHECK`
matches v1 semantics:
`iter >= max_iter || rr_new <= eps_sq || (iter > 1 && rr_new >= rr_old)`.

After x converges, `sel_y` flips and the same FSM runs the y solve
against the y-dim cx/x base addresses.

## Linalg kernels (val/rdy)

All four follow the same `istream_val/rdy` + `ostream_val/rdy`
convention. Each is `{Kernel}Ctrl` + `{Kernel}Dpath` wrapped in a
`{Kernel}_seq` module.

- **VecDot_seq** - `p_lanes` `FpMulWide`s feed an adder tree summed
  into a 48-bit accumulator. One istream handshake per group; one
  final ostream handshake per dot product. Self-clears the accumulator
  on the output handshake so the next solve starts at zero.
- **AXPY_seq** - `p_lanes` `FpMul`s with a coefficient broadcast.
  `mode` selects add vs sub; `coef` is `alpha` or `beta`. One istream
  handshake takes `p_lanes` (a,b) pairs; one ostream handshake emits
  `p_lanes` z values one cycle later.
- **SPMV_seq** - Single-lane (memory-bandwidth-bound on the single
  Avalon port). FSM walks CSR per row: read row_ptr lo and hi (2-cycle
  ADDR/CAPT pairs; row 0 only reads both, rows 1..n-1 carry the
  previous row's `rp_hi` forward as `rp_lo` and read only the new
  `rp_ptr[row+1]`). Inner loop is a 2-cyc/nnz overlapped pipeline:
  alternating cycles drive `q_val_base + j` and `q_col_base + j` while
  the MAC for nnz `k-1` fires concurrently with the address issue for
  nnz `k+1`. All pipeline state (`val_reg`, `col_reg`, `acc`,
  `j_addr_idx`, `j_mac_idx`, `pipe_phase`, `pipe_warm`) lives in a
  single `always_ff` block so the val_reg-overwrite-during-MAC step
  resolves cleanly via NBA semantics. Per-row cost is `2*nnz + 2`
  cycles. Emits `(row_idx, row_val)` per output handshake.
- **VecNegSub_seq** - `result = -(a + b)`. Used inline by CGCtrl's
  `S_VNS_R` phase via the `WD_VNS` write-data mux source rather than
  as a separate streaming module.

## Floating-point math

- **FpMul** - 27x27 -> 27 truncated-to-frac signed multiply,
  combinational, DSP-mapped.
- **FpMulWide** - same multiply but returns the full 48-bit shifted
  product. Used wherever products need to accumulate without
  intermediate truncation (VecDot, SPMV).
- **FpDiv** - Newton-Raphson reciprocal divide with val/rdy.
  Pipeline: 1) sign extract; 2) LZC-normalize `|b|` to `b_norm` in
  Q1.16; 3) ROM seed lookup (256x17 reciprocal table); 4) two NR iters
  `r = r*(2 - b_norm*r)` to recover the truncation noise; 5) multiply
  `|a| * r1`; 6) denormalize via right shift by `49 - lzc`, saturate,
  apply sign. ~8 internal cycles vs ~63 for shift-subtract, with
  matching DPI golden in [cg_golden.h](../../../sw-baseline-c/cg_golden.h)
  so testbench compares stay bit-exact.

## Avalon slave bus ownership

`ctrl_mem_src_spmv` mux in CGDpath picks between CGCtrl (LD/WB phases)
and SPMV (CSR walk) as the Avalon master. CGCtrl drives `chipselect`,
`clken`, `byteenable=4'b1111` constant; only `address`, `write`,
`writedata` are dynamic.

## Key parameters (CGTop)

| Param | Default | Notes |
| --- | --- | --- |
| `p_lanes` | 4 | SIMD width for VecDot and AXPY |
| `p_max_n` | 50 | RF depth and SRAM region sizing |
| `p_int_bits` | 13 | Fixed-point integer bits |
| `p_frac_bits` | 14 | Fixed-point fractional bits |
| `p_total_bits` | 27 | Targets Cyclone V 27x27 DSP |
| `p_acc_bits` | 48 | Wide accumulator + FpDiv operand width |
| `p_m10k_addr_bits` | 32 | Avalon address width |

## Verification

Bit-exact against the DPI golden CG in `fpga/hw/test/`:

```
cmake ../fpga/hw/test && make
./VCGTop_tb_v3   # must print "ALL 16 TESTS PASSED"
```
