# v3 CG Solver - Design Summary

Iterates on [v2](../v2/) with three synthesizable performance/timing
improvements: a fast Newton-Raphson `FpDiv`, an SPMV row-prologue
collapse, and timing-driven PIO registration plus narrow-width FSM
comparators in `CGCtrl`. A fourth experiment -- a 2-cyc/nnz pipelined
SPMV inner loop -- passed all Verilator tests but failed timing on
Cyclone V silicon (the long `val_reg -> mul -> add -> acc`
combinational chain it relied on was marginal at 50 MHz on real PVT,
and the synthesized version produced garbage placements). That
experiment was abandoned, so v3's SPMV inner loop matches v2's
serial 5-cyc/nnz walk.

## Changes vs v2

| Area | v2 | v3 | Win |
| --- | --- | --- | --- |
| FpDiv | shift-subtract, ~63 cycles | LZC + 256x17 ROM + 2 NR iters, each split into M1/M2 phases (~9 cycles) | faster divide; twice per CG iter |
| SPMV row prologue | reads `rp_ptr[i]` and `rp_ptr[i+1]` per row (4 cycles) | row 0 reads both; rows 1..n-1 carry `rp_hi -> rp_lo` and read only `rp_ptr[i+1]` | 2 cycles saved per row except row 0 |
| SPMV inner loop | 5 cycles/nnz serial | same as v2 (pipelined inner loop attempted, reverted on silicon) | -- |
| PIO inputs | unregistered into CGCtrl/CGDpath | `cg_n`, `cg_max_iter`, `cg_eps_sq`, `sw_go`, `sw_done_ack`, `rst` registered once at CGTop boundary on CLOCK_50 | breaks long fanout from PIOs into FSM next-state mux and dpath comparators |
| FSM comparators | 32-bit (`stream_idx == n - 32'd1` etc) | narrow `n_reg` / `n_minus_1_reg` / `num_groups_reg` / `num_groups_minus_1_reg` (`N_W = $clog2(p_max_n+1)`, 6-bit for `p_max_n=50`), captured at S_IDLE on `sw_go` | shrinks comparator depth ~5x |

End-to-end on Verilator HW CG (27-bit fixed-point throughout, 13 int
+ 14 frac):

| Benchmark | v2 cycles | v3 cycles | Speedup |
| --- | --- | --- | --- |
| tiny1 | 54672 | 38762 | **1.41x** |
| tiny2 | 60112 | 42234 | **1.42x** |
| tiny3 | 238216 | 188640 | **1.26x** |

HPWL trajectories are bit-exact with v2 (and the golden CG via
`-DCG_GOLDEN_USE_NR`). Cycle counts are the total of all
`sw_go`-to-`sw_done` intervals across every CG solve in a placer run.

## Synth/sim coupling

The [DPI golden](../../../sw-baseline-c/cg_golden_model.h) supports both
divide algorithms via `#ifdef CG_GOLDEN_USE_NR`. The v3 testbench target
`VCGTop_tb_v3` and the HW CG placer with `-DHW_CG_VERSION=v3` build with
`-DCG_GOLDEN_USE_NR` so DPI compares stay bit-exact against v3's NR RTL.

## Files

| File | Role |
| --- | --- |
| [CGTop.v](CGTop.v) | Toplevel: registers all PIO inputs, instantiates CGCtrl + CGDpath |
| [CGCtrl.v](CGCtrl.v) | Single flat FSM driving every mux, latch, and val/rdy handshake; narrow `n_reg`/`num_groups_reg` |
| [CGDpath.v](CGDpath.v) | Pure datapath: register files, scalars, linalg submodule instances, muxes |
| [LinAlg.v](LinAlg.v) | VecDot, AXPY, SPMV (v2 serial inner loop with v3 rp_lo/rp_hi prologue collapse), VecNegSub |
| [FpMath.v](FpMath.v) | FpMul, FpMulWide, FpDiv (NR reciprocal: LZC + 256x17 seed ROM + 2 NR iters split into M1/M2 phases) |

## Verification

Bit-exact against the DPI golden CG in `fpga/hw/test/`:

```
cmake ../fpga/hw/test && make
./VCGTop_tb_v3   # must print "ALL 16 TESTS PASSED"
```

End-to-end against the Verilator placer:

```
./run-placer.sh verilated benchmarks/custom/tiny3 v3
```

## Silicon

Synthesized and reflashed on DE1-SoC; produces correct placements
matching the golden CG bit-exactly through the first three CG solves
(small drift thereafter is from outer-placer FP rounding, not CG).
