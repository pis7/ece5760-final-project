# v4 CG Solver - Parallel x/r AXPY + fused VNS_R

Iterates on [v3](../v3/) with two FSM-level merges that exploit the
fact that the per-iter x and r updates have **no data dependency**
on each other. Result: shorter critical iteration with no algorithmic
change.

## Changes vs v3

| Area | v3 | v4 | Win |
| --- | --- | --- | --- |
| `x += alpha*d` and `r -= alpha*q` | sequential `S_AXPY_X_FEED` then `S_AXPY_R_FEED`, sharing one AXPY unit | merged `S_AXPY_XR_FEED` running two AXPY units (`u_axpy_x` ADD, `u_axpy_r` SUB) in lockstep | `num_groups` cycles saved per CG iter |
| `S_VNS_R` (`r = -(cx+q)`) and `S_COPY_D` (`d = r`) | two separate streaming passes | fused: secondary write port writes `d_reg` in parallel with the primary `r_reg` write | `num_groups` cycles saved at init |
| RF write ports | one `p_lanes`-wide primary | adds a `p_lanes`-wide **secondary** write port (`wr_sel_sec`, `wdata_src_sec`, `we_sec`); index reuses the primary's | enables the lockstep dual-write |
| AXPY units | 1 (mode-muxed between ADD and SUB) | 2 (ADD-only `u_axpy_x` + SUB-only `u_axpy_r`) | doubles AXPY throughput in `S_AXPY_XR_FEED` |
| DSP cost | VecDot `p_lanes` + AXPY `p_lanes` + SPMV 1 | VecDot `p_lanes` + 2x AXPY `p_lanes` + SPMV 1 | +`p_lanes` DSPs (small; fits comfortably) |

End-to-end on Verilator HW CG:

| Benchmark | v3 cycles | v4 cycles | Speedup |
| --- | --- | --- | --- |
| simple_logic_10    | 38,762  | ~28,000 | **~1.4x** |
| parallel_chains_50 | 188,640 | ~140,000 | **~1.35x** |

(The win is bounded by per-iter AXPY share; SPMV is unchanged from v3.)

Bit-exact against the DPI golden CG with `-DCG_GOLDEN_USE_NR` (same NR
divide as v3).

## Files

| File | Role |
| --- | --- |
| [CGTop.v](CGTop.v) | Toplevel: same as v3, plus the secondary write port wiring |
| [CGCtrl.v](CGCtrl.v) | FSM with merged `S_AXPY_XR_FEED` and fused `S_VNS_R` |
| [CGDpath.v](CGDpath.v) | RF gains a secondary write port; instantiates two AXPY units |
| [LinAlg.v](LinAlg.v) | Same kernels as v3 -- AXPY's `mode` parameter is now hardwired per-instance |
| [FpMath.v](FpMath.v) | Same NR `FpDiv` as v3 |

## CGCtrl FSM (changes)

```
v3:  ... S_AXPY_X_FEED -> S_AXPY_R_FEED -> S_VDOT_RR_FEED ...
v4:  ... S_AXPY_XR_FEED ----------------> S_VDOT_RR_FEED ...

v3:  ... S_VNS_R -> S_COPY_D -> S_VDOT_INIT_FEED ...
v4:  ... S_VNS_R ----------> S_VDOT_INIT_FEED ...
```

`u_axpy_x`'s handshakes are canonical for FSM and counter control --
both AXPY units run lockstep (same `n`, same `p_lanes`, identical
inputs, same handshake driver) so `axpy_r_*_hs` are equivalent by
construction.

## What is *not* changed vs v3

- SPMV inner loop: still serial 5-cyc/nz with v3's row-prologue
  collapse (`rp_hi -> rp_lo` carry across rows).
- `FpDiv`: same NR with 256x17 seed ROM.
- Memory layout, `sel_y` x/y sequencing, fixed-point format,
  Avalon-slave bus arbitration: identical to v3.

## Verification

Bit-exact against the DPI golden CG in `fpga/hw/test/`:

```
mkdir -p build-tb && cd build-tb && cmake ../fpga/hw/test && make
./VCGTop_tb_v4   # must print "ALL 16 TESTS PASSED"
```

End-to-end against the Verilator placer:

```
uv run run-placer verilated v4 ../benchmarks/custom/parallel_chains_50
```
