# v1 CG Solver - Combinational reference

The first cut of Verilog. Translates the [Python FL model](../../fl/)
into RTL one-to-one, with a fully **combinational** datapath. **Not
synthesizable** -- this version exists purely to move the FL model's
semantics into Verilog and to act as a golden reference for the
sequential v2 design.

## Files

| File | Role |
| --- | --- |
| [CGTop.v](CGTop.v) | Toplevel: Avalon slave pass-through, instantiates CGCtrl + CGDpath |
| [CGCtrl.v](CGCtrl.v) | Phase FSM driving the linalg sub-units in lockstep with the FL stages |
| [CGDpath.v](CGDpath.v) | Pure combinational datapath (no `always_ff` register files) |
| [LinAlg.v](LinAlg.v) | VecDot, AXPY, SPMV, VecNegSub combinational kernels |
| [FpMath.v](FpMath.v) | Fixed-point multiply / divide (combinational restoring divide) |
| [M10KLoader.v](M10KLoader.v) | Helper that streams `(addr, value)` pairs from SRAM into the combinational RF |

## Why combinational

The goal of v1 is to validate that the FL model's data flow lowers
correctly into Verilog. By keeping every kernel combinational, the
DUT trivially exposes the same per-stage outputs the FL model would
produce -- making it easy to compare element-by-element with the C++
fixed-point golden via the [DPI testbench](../test/CGTop_tb.v).

The cost is real-silicon impossibility: a single CG iteration is one
giant combinational fan-out from the SRAM read ports to the next
register write, which would never close timing on Cyclone V's
50 MHz Avalon clock. v2 sequentializes this into a flat FSM.

## Memory layout

Single Avalon-mapped on-chip SRAM with the contiguous v4-compatible
layout (kept consistent through v5 for bit-exact DPI golden compares):

| Region | Base | Size |
| --- | --- | --- |
| Q values (CSR) | 0 | `N^2` |
| Q col_idx (CSR) | `N^2` | `N^2` |
| Q row_ptr (CSR) | `2*N^2` | `N + 1` |
| cx (x dim) | `2*N^2 + N + 1` | `N` |
| cx (y dim) | `2*N^2 + 2*N + 1` | `N` |
| x out | `2*N^2 + 3*N + 1` | `N` |
| y out | `2*N^2 + 4*N + 1` | `N` |

`sel_y` flips between x-dim and y-dim base addresses so the FSM runs
both solves back-to-back.

## Verification

Bit-exact against the DPI golden CG in `fpga/hw/test/`:

```
mkdir -p build-tb && cd build-tb && cmake ../fpga/hw/test && make
./VCGTop_tb_v1   # must print "ALL 16 TESTS PASSED"
```

The 16 cases cover small SPD systems, tridiagonals up to `n=50` (the
SRAM-imposed `MAX_N`), the `n=1` boundary, the lane-mask boundary at
`n=49`, dense diagonally-dominant matrices, the arrow pattern,
non-zero initial guesses, and pre-converged starts.

## Where it goes from here

v2 is the synthesizable rewrite: same FL semantics, same memory
layout, same fixed-point format, but with a sequential FSM and
DSP-mapped multipliers. See [`../v2/README.md`](../v2/README.md).
