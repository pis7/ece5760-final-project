# v2 CG Solver - Design Summary

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
| [LinAlg.v](LinAlg.v) | VecDot, AXPY, SPMV, VecNegSub kernels (each split Ctrl + Dpath + `_seq` wrapper) |
| [FpMath.v](FpMath.v) | FpMul, FpMulWide, FpDiv (shift-subtract) |

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

- 27-bit total (`p_int_bits=13`, `p_frac_bits=14`), targeting the
  Cyclone V 27x27 DSP block via `(* multstyle = "dsp" *)`.
- `p_acc_bits = 48` for VecDot/SPMV accumulators and FpDiv operands -
  products accumulate without early truncation.

## Memory layout (parameterized by `N = p_max_n`)

Single Avalon-mapped on-chip SRAM, addresses in 32-bit words. All
regions are contiguous; total footprint is `2*N^2 + 5*N + 1` words.

| Region | Base | Size |
| --- | --- | --- |
| Q values (CSR) | 0 | `N^2` |
| Q col_idx (CSR) | `N^2` | `N^2` |
| Q row_ptr (CSR) | `2*N^2` | `N + 1` |
| cx (x dim) | `2*N^2 + N + 1` | `N` |
| cx (y dim) | `2*N^2 + 2*N + 1` | `N` |
| x out | `2*N^2 + 3*N + 1` | `N` |
| y out | `2*N^2 + 4*N + 1` | `N` |

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
  `p_lanes=2` (default) -> 5 DSPs; `p_lanes=4` -> 9 DSPs.

## Register files (in CGDpath)

Five flip-flop unpacked arrays of size `p_max_n`, each
27 bits wide:

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
  Avalon port). 12-state inner FSM walks CSR per row: read `rp_lo`
  and `rp_hi` (2-cycle ADDR/CAPT pairs), then for each nnz read val,
  read col, MAC into a 48-bit accumulator. Emits `(row_idx, row_val)`
  per output handshake. A pipelined inner loop (2 cycles/nnz) was
  attempted but caused correctness regressions on tiny3 - reverted
  (see comment in [LinAlg.v](LinAlg.v)).
- **VecNegSub_seq** - `result = -(a + b)`. Used inline by CGCtrl's
  `S_VNS_R` phase via the `WD_VNS` write-data mux source rather than
  as a separate streaming module.

## Floating-point math

- **FpMul** - 27x27 -> 27 truncated-to-frac signed multiply,
  combinational, DSP-mapped.
- **FpMulWide** - same multiply but returns the full 48-bit shifted
  product. Used wherever products need to accumulate without
  intermediate truncation (VecDot, SPMV).
- **FpDiv** - sequential restoring shift-subtract divide with val/rdy.
  Latency = `p_wide_bits + p_frac_bits` iterations + 1 finish cycle
  (~63 cycles for the default widths). Operates on 48-bit signed
  operands, returns a 27-bit quotient.

## Avalon slave bus ownership

`ctrl_mem_src_spmv` mux in CGDpath picks between CGCtrl (LD/WB phases)
and SPMV (CSR walk) as the Avalon master. CGCtrl drives `chipselect`,
`clken`, `byteenable=4'b1111` constant; only `address`, `write`,
`writedata` are dynamic.

## Key parameters (CGTop)

| Param | Default | Notes |
| --- | --- | --- |
| `p_lanes` | 2 | SIMD width for VecDot and AXPY |
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
./VCGTop_tb_v2   # must print "ALL 16 TESTS PASSED"
```
