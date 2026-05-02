# v5 CG Solver - Multi-block on-chip RAM topology

Iterates on [v4](../v4/) by **splitting the single shared Avalon SRAM
into seven dedicated Qsys on-chip RAM slaves**, one per logical region.
Each slave's FPGA-facing port has exactly one consumer, so SPMV can
fetch `q_val[j]` and `q_col[j]` in the same cycle (parallel issue and
capture). Per-nz cost drops from v4's 5 cycles to 3.

Drawn directly from "An Implementation of the Conjugate Gradient
Algorithm on FPGAs" (DuBois, Boorman, Connor, Poole; FCCM 2008) which
striped its CSR arrays across two parallel banks for the same reason.
Their other architectural ideas (two-FPGA partition, ELLPACK,
double-precision DMA) don't transfer to this project; the
multi-bank/multi-port memory partitioning does.

## Changes vs v4

| Area | v4 | v5 | Win |
| --- | --- | --- | --- |
| Avalon slaves | 1 shared SRAM (Q, cx, cy, x, y all multiplexed) | 7 dedicated on-chip RAM slaves: `q_val_ram`, `q_col_ram`, `q_rowp_ram`, `cx_ram`, `cy_ram`, `x_ram`, `y_ram` | no bus mux; one consumer per port |
| SPMV per-nz | 5 cycles (`VAL_ADDR -> VAL_CAPT -> COL_ADDR -> COL_CAPT -> ACC`) | **3 cycles** (`NZ_ADDR -> NZ_CAPT -> ACC`, val + col fetched in parallel) | ~2x SPMV throughput |
| `cx_reg` central RF | 5th flop-based RF, populated by `S_LD_CX_*` | **removed**; cx is read directly from `cx_ram` (M10K) during `S_VNS_R` | saves `p_lanes*p_max_n` flops; fewer FSM phases |
| `S_LD_CX_*` states | exist | **gone** | shorter init |
| `S_VNS_R` | 1 group/cycle, p_lanes-wide | serialized to `S_VNS_R_ADDR -> S_VNS_R_CAPT` (1 element/cycle, single-lane writeback) | needed because `cx_ram` is single-port |
| x/y storage | central RF, addressed via `sel_y` and base-add | still in `x_vec_reg` (banked flop array); `S_LD_X_*` and `S_WB_WRITE` now talk directly to `x_ram` / `y_ram` (sel_y muxed at CGTop) | -- |
| Qsys regenerate | not needed | **required**: 7 on-chip RAM IPs (see table below) | one-time bring-up step |

End-to-end on Verilator HW CG (`parallel_chains_50`):

| Metric | v4 | v5 | Speedup |
| --- | --- | --- | --- |
| Total HW CG cycles (8 solves) | ~186,000 | 131,088 | **~1.42x** |

Bit-exact against the DPI golden CG (16/16 tests pass).

## Files

| File | Role |
| --- | --- |
| [CGTop.v](CGTop.v) | Toplevel: 7 Avalon slave bundles, sel_y mux for cx/cy and x/y |
| [CGCtrl.v](CGCtrl.v) | FSM: removes `S_LD_CX_*`; serializes `S_VNS_R` for single-port cx_ram |
| [CGDpath.v](CGDpath.v) | RF: drops `cx_reg`; `WD_VNS_SCALAR` source replaces `WD_VNS` (reads `vns_cx_rdata` from M10K) |
| [LinAlg.v](LinAlg.v) | SPMV rewritten: 3 cycles/nz inner loop with parallel q_val + q_col read ports |
| [FpMath.v](FpMath.v) | Same NR `FpDiv` as v3/v4 |
| [FPGATop.v](FPGATop.v) | DE1-SoC pin map; 7 on-chip RAM bundles wired to `Computer_System` |

## Memory topology

```
                Avalon h2f bridge (ARM-facing)
                 |       |       |   |   |   |   |
                 v       v       v   v   v   v   v
              q_val   q_col   q_rowp cx  cy  x   y   <- one Qsys on-chip RAM IP per slave
                 |       |       |   |   |   |   |
                 |       |       |   |   |   |   |
                 v       v       v   v   v   v   v
              CGDpath SPMV reads | CGCtrl serial | CGCtrl load + writeback
                 (q_val_addr,    | read for cx   | (sel_y picks x_ram or y_ram)
                  q_col_addr,    | / cy via      |
                  q_rowp_addr -- | sel_y mux     |
                  3 read ports)  |               |
```

Each slave is dual-port: ARM h2f on one side, FPGA-facing on the other.
The dual-port nature of Qsys on-chip RAM keeps the two domains isolated
without explicit arbitration.

## Required Qsys changes (manual, on the DE1-SoC build)

Regenerate `Computer_System.qsys` with seven on-chip RAM IPs whose
FPGA-facing slave port names match [FPGATop.v](FPGATop.v):

| Slave | Words | RTL-facing port prefix |
| --- | --- | --- |
| `q_val_ram` | `MAX_N*MAX_N` (2500) | `q_val_ram_*` |
| `q_col_ram` | `MAX_N*MAX_N` (2500) | `q_col_ram_*` |
| `q_rowp_ram` | `MAX_N+1` (51) | `q_rowp_ram_*` |
| `cx_ram` | `MAX_N` (50) | `cx_ram_*` |
| `cy_ram` | `MAX_N` (50) | `cy_ram_*` |
| `x_ram` | `MAX_N` (50) | `x_ram_*` |
| `y_ram` | `MAX_N` (50) | `y_ram_*` |

Update the bridge bases in
[`../../sw/cg_fpga_mmap_driver_v5.h`](../../sw/cg_fpga_mmap_driver_v5.h)
to match Quartus's actual address assignment. Two layouts ship by
default (`HW_MAX_N=50` and `HW_MAX_N=75`).

## Drivers

Per-version drivers select via CMake variables:
- [`cg_verilator_driver_v5.h`](../../../sw-baseline-c/cg_verilator_driver_v5.h)
  -- 7 behavioral memories, one per slave; mirrors the per-slave shim
  in [`CGTop_tb_v5.v`](../test/CGTop_tb_v5.v)
- [`cg_fpga_mmap_driver_v5.h`](../../sw/cg_fpga_mmap_driver_v5.h)
  -- 7 mmap regions, each at the bridge base for its slave

The v4 driver remains at the bare `cg_fpga_mmap_driver.h` /
`cg_verilator_driver.h` paths. Set `HW_CG_VERSION=v5` (or `v5_deep`,
`v6`) and `HW_FPGA_VERSION=v5` to wire up the right header.

## Verification

Bit-exact against the DPI golden CG in `fpga/hw/test/`:

```
mkdir -p build-tb && cd build-tb && cmake ../fpga/hw/test && make
./VCGTop_tb_v5   # must print "ALL 16 TESTS PASSED"
```

End-to-end against the Verilator placer:

```
uv run run-placer verilated v5 ../benchmarks/custom/parallel_chains_50
```

## Where v5 goes from here

v5's multi-block topology unlocks several follow-up optimizations that
were impossible against the single-port shared SRAM. Implemented:

- **[v5_deep](../v5_deep/)** -- pipelined SPMV inner loop targeting
  1 cycle/nz steady state (4-stage pipeline). Reuses v5's memory
  topology and drivers verbatim; only `LinAlg.v` SPMV changes.
- **[v6](../v6/)** -- parallel x/y solve datapaths. Duplicates Q
  across two M10K trios so two `CGEngine` instances run x and y
  concurrently. ~1.86x cycle speedup over v5 on `parallel_chains_50`.

Deferred (orthogonal to v5_deep / v6):

- **Concurrent kernel firing within one engine.** Today VecDot/AXPY
  and SPMV occupy the FSM one phase at a time. With v5's multi-port
  topology there is no resource conflict between AXPY (RF-only) and
  SPMV (Q ports only), so a scoreboard-driven CGCtrl could overlap
  them. Significant FSM rewrite.
- **Bank x/y across `p_lanes` M10Ks.** Only relevant when problem
  sizes push x/y out of the central RF (above ~thousands of cells
  per dimension). Not triggered at `p_max_n=50`.

## Original design notes

The v5 design was driven by concrete ideas from the
[DuBois 2008 FCCM paper](../../../background-knowledge/papers/An_Implementation_of_the_Conjugate_Gradient_Algorithm_on_FPGAs.pdf).
Their setting (two-FPGA SRC MAPStation, ELLPACK, double precision,
DMA-streamed matrix from a Xeon host) is very different from this
project (one Cyclone V at 50 MHz, CSR, Q13.14 fixed-point, on-chip
Q load), so most of the paper's architecture does not transfer.
What does transfer:

1. **Bank Q's CSR arrays across multiple M10K blocks.** This is what
   v5 ships -- `q_val` and `q_col` are independent slaves, so they
   can be read in the same cycle.
2. **Even/odd vector banking when vectors outgrow the flop budget.**
   Deferred (item 4 above) -- not triggered at `p_max_n=50`.
3. **Concurrent kernel firing instead of FSM-serialized phases.**
   Deferred -- this is the same direction the Rampalli HLS paper
   pushes toward.

What is *not* taken: ELLPACK-ITPACK (placement Q has highly skewed
row densities, ELLPACK would zero-pad enormously), two-FPGA
partitioning (only one FPGA available), DMA-streaming the matrix
each iteration (Q fits on-chip), double precision (Q13.14 has been
DPI-golden-validated bit-exact).
