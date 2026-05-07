# ECE 5760 Final Project -- Analytical Placement via FPGA

This project accelerates analytical placement (a key step in EDA / ASIC
back-end flows) using the DE1-SoC Cyclone V FPGA. Standard cell placement is
formulated as a quadratic optimization problem, `Q x = -c`, and solved via
conjugate gradient descent. The CG inner loop is the hot path and is what the
FPGA accelerates; the HPS ARM handles netlist parsing, partitioning, and
overlap reduction.

## Setup

Requires Python >= 3.14 with [uv](https://docs.astral.sh/uv/), CMake >= 3.10,
a C++17 compiler, and (for RTL simulation) Verilator >= 5.

```bash
uv sync                          # one-time: creates .venv/ and installs deps + entry points
source .venv/bin/activate        # per-terminal: puts entry points (e.g. `visualizer`) on PATH
```

`uv sync` only needs to be re-run when `pyproject.toml` / `uv.lock` change
or after blowing away `.venv/`. If you'd rather not activate the venv, you
can prefix any project command with `uv run` (e.g.
`uv run visualizer DMA-final.json`) -- that works from any subdirectory of
the repo.

---

## Quick start

Every backend is reachable via the [`run-placer`](python-utils/run_placer.py)
entry point from a fresh `build/` directory. The first positional picks
the backend; pass `--sweep` to capture an iter-by-iter slideshow.

```bash
mkdir -p build && cd build

# --- Software placers (no FPGA needed) ---
uv run run-placer python ../benchmarks/iccad04/DMA      # Python baseline (algorithmic spec)
uv run run-placer sw     ../benchmarks/iccad04/DMA      # C++ double-precision CG (any size)
uv run run-placer golden ../benchmarks/custom/parallel_chains_50  # C++ fixed-point golden CG (SW model of the FPGA)

# --- Hardware-in-the-loop simulation (Verilator) ---
uv run run-placer verilated    ../benchmarks/custom/parallel_chains_50   # Verilated RTL CG, default v6
uv run run-placer verilated v3 ../benchmarks/custom/parallel_chains_50   # any of v2|v3|v4|v5|v5_deep|v6

# --- DE1-SoC board (needs .env with BOARD/PASS) ---
uv run run-placer arm  ../benchmarks/iccad04/DMA                  # cross-compile + run SW CG on the board's ARM (any size)
uv run run-placer fpga ../benchmarks/custom/parallel_chains_50    # cross-compile + run FPGA-accelerated CG

# --- Batch: point at a parent dir to run every benchmark inside it ---
uv run run-placer verilated v6 ../benchmarks/custom     # runs every benchmark in custom/
uv run run-placer fpga         ../benchmarks/custom     # runs every custom benchmark on the board

# --- Iter sweep + slideshow (every mode except python) ---
uv run run-placer verilated ../benchmarks/custom/parallel_chains_50 --sweep
```

`golden` and `verilated` accept three knobs to scale past the
synthesized 27-bit / `MAX_N=50` bitstream:

- `--int-bits I` (default 13) and `--frac-bits B` (default 14) set the
  fixed-point Q-format. `int_bits + frac_bits <= 64`. Widths up to 27
  match the locked FPGA bitstream; widths above 27 are accepted only by
  `golden` and `verilated` (verilator re-elaborates the RTL with
  `-Gp_int_bits` / `-Gp_frac_bits`).
- `--max-n N` (default 50) sets the placer's hardware-N cap, the
  Verilog `p_max_n`, and the driver's mirror sizes. Only `golden` and
  `verilated` honor it; `sw` / `arm` / `fpga` reject it because the
  bitstream is fixed at 50.
- `--max-iter K` caps the outer placer loop (default 16). Useful for
  long benchmarks where you want to push past the default cap, or to
  shorten a debugging run. Mutually exclusive with `--sweep`.

`fpga` is the only mode capped at `MAX_N=50`. `sw` / `arm` use doubles
and dynamic vectors so they take any size; `golden` / `verilated` scale
to whatever `--max-n` you pass at the cost of `O(MAX_N^2)` driver
memory (~3 GB at `MAX_N=12000`, heap-allocated, fine on a workstation).

```bash
# ICCAD04 DMA (~12k cells) through the fixed-point golden CG, 30 iters,
# Q44.20 to give alpha*d enough integer headroom (see "Picking the bit
# split" below):
uv run run-placer golden    ../benchmarks/iccad04/DMA \
    --max-n 12000 --int-bits 44 --frac-bits 20 --max-iter 30

# Same shape under the Verilator RTL (slow -- expect ~25s/CG solve at
# this size since Verilator simulates every cycle in software):
uv run run-placer verilated ../benchmarks/iccad04/DMA \
    --max-n 12000 --int-bits 44 --frac-bits 20 --max-iter 30
```

**Picking the bit split.** `int_bits` must be wide enough that no
intermediate (notably `alpha * d[i]` inside CG) overflows
`2^(int_bits-1)` in real units. For ICCAD04-scale designs (positions
~2x10^5 in DBU, alpha doubling per outer iteration), 28 integer bits
is too narrow and CG silently wraps around iter 9; 44 integer bits has
plenty of headroom for 30 outer iters. `frac_bits` only needs to give
sub-DBU precision -- 12-20 is generally fine.

Each run writes `<design>-initial.json`, `<design>-final.json`, and a
`<design>-final.png` rendering of the final placement to the build dir.
`arm`/`fpga` additionally need a gitignored `.env` at the repo
root with `BOARD='root@<ip>'` and `PASS='...'` (see
[Running the placer](#running-the-placer)). `fpga` also requires the CG
bitstream to already be programmed onto the DE1-SoC.

If `run-placer` is invoked from a directory whose name starts with
`build`, that directory is wiped at the start of each invocation so
stale binaries / JSONs from a previous run never bleed into the next one.

### Tests

```bash
# Software / functional model -- 19 cases vs scipy
uv run fpga/fl/test/CGTop-test.py

# Hardware / RTL -- 16 cases per version, bit-exact vs DPI golden
mkdir -p build-tb && cd build-tb && cmake ../fpga/hw/test && make
./VCGTop_tb_v1        # combinational reference (not synthesizable)
./VCGTop_tb_v2        # first synthesizable design
./VCGTop_tb_v3        # NR FpDiv + SPMV row-prologue collapse + PIO timing
./VCGTop_tb_v4        # parallel x/r AXPY + fused VNS_R
./VCGTop_tb_v5        # multi-block on-chip RAM topology (7 slaves)
./VCGTop_tb_v5_deep   # v5 + 4-stage SPMV pipeline (1 cyc/nz steady state)
./VCGTop_tb_v6        # v5 + parallel x/y solve datapaths (10 slaves, 2 engines)
```

The Python suite should print `Result: 19/19 tests passed`; each RTL
target should print `ALL 16 TESTS PASSED`.

---

## Input data: LEF/DEF -> JSON

### Benchmark layout

Every placement input is a pair of industry-standard LEF and DEF files, kept
in `lef/` and `def/` subdirectories of the benchmark folder:

```
benchmarks/
  iccad04/
    DMA/   { lef/dma.lef,  def/dma.def }      # 11k cells
    DSP1/  { lef/dsp1.lef, def/dsp1.def }
    DSP2/  ...
    RISC1/ ...
    RISC2/ ...
  custom/
    simple_logic_10/      { lef/simple_logic_10.lef, def/simple_logic_10.def }
    mixed_macro_10/       ...
    parallel_chains_50/   ...
    mesh_grid_25/         ...
    dense_pack_36/        ...
    size_extremes_mix/    ...
    star_fanout_31/       ...
    two_clusters_bridge/  ...   # see benchmarks/custom/README.md
```

The `iccad04/` set is the ICCAD 2004 Faraday mixed-size suite -- real ASIC
netlists with thousands of standard cells and a handful of large macros.
The `custom/` set is hand-written micron-scale designs used to fit inside
the FPGA's `MAX_N=50` cap (each die is 500 DBU on a side, far below the
`+/-4096` representable range of the Q13.14 fixed-point format).

### What each file contains

- **`<design>.lef`** -- the *library*. Defines the fab's standard cells
  (`MACRO INV`, `MACRO BUF`, ...), giving each one a physical size (`SIZE w
  BY h`) and the names of its pins. Also declares the database resolution
  (`UNITS DATABASE MICRONS 100` -> 1 DBU = 0.01 um). We ignore everything
  routing-related (`LAYER`, `VIA`, `SITE`, `SPACING`) since we only do
  placement, not routing.
- **`<design>.def`** -- the *instance netlist*. Names the design,
  declares the die area (`DIEAREA ( xl yl ) ( xh yh )` in DBU), instantiates
  cells (`COMPONENTS N; - U1 INV + PLACED ( x y ) N ; ...`), declares I/O
  pads (`PINS`), and declares net connectivity (`NETS - net_name ( U1 Y ) (
  U2 A ) ;`). The `PLACED` coordinates are the input placement -- for an
  unplaced design they are dummy values that the placer will overwrite.

### The parser

[`python-utils/lefdef_parser.py`](python-utils/lefdef_parser.py) is a
hand-rolled streaming tokenizer (no external LEF/DEF library) that walks
both files once and emits a single unified JSON. Class-based with one
public method per stage (`find_files`, `parse_lef`, `parse_def`,
`write_json`). It is exposed as the `lefdef-parser` console script
(see [`pyproject.toml`](pyproject.toml)):

```bash
uv run lefdef-parser ../benchmarks/iccad04/DMA
# -> dma_top.json in the current directory
```

The output filename is the design name from the DEF's `DESIGN` keyword (so
`benchmarks/iccad04/DMA` produces `dma_top.json`, not `DMA.json`).

### The JSON schema -- the contract every layer downstream uses

Defined in [`json_utils.py`](json_utils.py) as a set of dataclasses
(`Macro`, `Component`, `IOPin`, `Net`, `Netlist`):

```jsonc
{
  "design_name": "dma_top",
  "dbu_per_micron": 100,
  "die_area": [-204000, -204000, 204400, 204400], // [xl, yl, xh, yh] in DBU
  "macros": {
    "INV":  { "width": 0.1, "height": 0.1, "pins": ["A", "Y"] },     // microns
    "BUF":  { "width": 0.1, "height": 0.1, "pins": ["A", "Y"] },
    ...
  },
  "components": {
    "U1": { "macro_name": "INV", "x": 10, "y": 10 },                  // DBU
    "U2": { "macro_name": "BUF", "x": 20, "y": 10 },
    ...
  },
  "io_pins": [
    { "name": "clk", "net_name": "clk", "x": 0, "y": 12500 },         // DBU
    ...
  ],
  "nets": [
    { "name": "n1", "pins": [["U1", "Y"], ["U2", "A"], ["U5", "A"]] },
    ...
  ]
}
```

Both placers (Python and C++) read this format via `load_netlist()` and
write the post-placement result to `<design>-initial.json` and
`<design>-final.json` with the same schema, just with updated `x` and `y`
on each component. The visualizer, the sweep slideshow, and every backend
in the table below all consume the same JSON -- there is no second
representation of the netlist anywhere in the project.

---

## How the project was built

The project was built bottom-up. Each new layer was validated against the
layer below it before we moved on -- so there is always a known-good
reference to diff against when something breaks. The RTL went through
seven versions; each one's `README.md` documents what changed vs the
previous (`v1` -> `v2` -> `v3` -> `v4` -> `v5` -> `v5_deep` & `v6`).

### 1. Python baseline placer -- [`sw-baseline-python/`](sw-baseline-python/)

[`placer.py`](sw-baseline-python/placer.py) is the algorithmic spec. It
implements the full placer in idiomatic Python on top of `scipy.sparse`:

- **`build_system()`** -- clique decomposition of each net into 2-pin
  weighted edges (weight `2/P` for a `P`-pin net), assembled into a sparse
  CSR matrix `Q` plus RHS vector `c`.
- **`solve_cg()`** -- delegated to `scipy.sparse.linalg.cg`.
- **`partition_and_anchor()`** -- recursive geometric bisection that adds
  exponentially-scaled anchor springs to `Q` to spread overlapping cells.
- **`max_bin_density()`** -- 30x30 density bins; outer loop terminates when
  the worst bin drops below `0.75`.

This is intentionally the slowest implementation; its job is to be
*obviously correct*, so every later layer can compare its output against it.

### 2. C++ port -- [`sw-baseline-c/`](sw-baseline-c/)

[`placer.cpp`](sw-baseline-c/placer.cpp) is a direct port of the Python
baseline: same JSON schema in and out (see [`json_utils.py`](json_utils.py)),
same algorithm, same convergence criterion. The CG itself is hand-rolled
double-precision in [`cg_solve()`](sw-baseline-c/placer.cpp). The C++ port is
~10x faster than the Python and -- as later stages would show -- becomes the
*canonical* placer source: every backend below this point reuses
`placer.cpp` unchanged and only swaps out the CG kernel.

### 3. Python FL model of the CG kernel -- [`fpga/fl/`](fpga/fl/)

Before writing any RTL we built a *functional-level* model of just the inner
CG loop: [`fpga/fl/CGTop.py`](fpga/fl/CGTop.py) and
[`fpga/fl/LinAlg.py`](fpga/fl/LinAlg.py). This model deliberately mirrors
the register-level structure that the eventual RTL will have -- explicit
`SPMV`, `VecDot`, `AXPY`, `VecNegSub` calls operating on register-file-shaped
state (`Qx_reg`, `x_reg`, `d_reg`, `r_reg`, `rr_reg`, ...) -- without yet
committing to fixed-point arithmetic or any specific memory layout.

It is tested by [`fpga/fl/test/CGTop-test.py`](fpga/fl/test/CGTop-test.py),
which runs **19 systems** (identity, diagonal, dense SPD, tridiagonal,
arrow, random, near-singular, ...) against the scipy CG reference and checks
relative error. New tests get auto-collected from any `test_*` function in
the module.

### 4. v1 RTL -- combinational reference -- [`fpga/hw/v1/`](fpga/hw/v1/)

The first cut of Verilog: [`CGTop.v`](fpga/hw/v1/CGTop.v),
[`CGCtrl.v`](fpga/hw/v1/CGCtrl.v), [`CGDpath.v`](fpga/hw/v1/CGDpath.v),
[`FpMath.v`](fpga/hw/v1/FpMath.v), [`LinAlg.v`](fpga/hw/v1/LinAlg.v), plus
an [`M10KLoader.v`](fpga/hw/v1/M10KLoader.v) helper. The datapath is fully
combinational and **not synthesizable** -- this version exists purely to
move the FL model's semantics into Verilog one step at a time.

To validate it, the testbench in [`fpga/hw/test/`](fpga/hw/test/) drives the
DUT and -- via a DPI import (see
[`cg_golden_dpi.cpp`](fpga/hw/test/cg_golden_dpi.cpp)) -- runs the C++
fixed-point golden model in [`cg_golden_model.h`](sw-baseline-c/cg_golden_model.h)
on the same input and checks **bit-exact equality** between every element of
the DUT result and the golden. The testbench has **16 cases** that cover
small SPD systems, tridiagonals up to `n=50` (the SRAM-imposed `MAX_N`), the
`n=1` boundary, the lane-mask boundary at `n=49`, dense diagonally-dominant
matrices, the arrow pattern, non-zero initial guesses, and pre-converged
starts.

### 5. v2 RTL -- synthesizable for DE1-SoC -- [`fpga/hw/v2/`](fpga/hw/v2/)

[`fpga/hw/v2/README.md`](fpga/hw/v2/README.md) has the full design summary;
the headline changes from v1 are:

- **Sequential FSM** in [`CGCtrl.v`](fpga/hw/v2/CGCtrl.v) driving every mux,
  register write-enable, and val/rdy handshake -- one flat FSM, no nested
  control.
- **DSP-mapped multipliers** -- `FpMul`/`FpMulWide` target the Cyclone V's 27x27
  DSPs.
- **Parameterized SIMD** -- `p_lanes` (default 2) governs the width of
  `VecDot` and `AXPY` (`p_lanes` DSPs each); `SPMV` stays single-lane
  because it's memory-bandwidth bound at the single-port SRAM.
- **Fixed-point format** -- 27-bit (13 integer + 14 fractional) with
  48-bit accumulators in dot products and SPMV (no early truncation).
- **Single Avalon slave port** to the Qsys on-chip SRAM (no intermediate
  BRAM), arbitrated between CGCtrl and SPMV.
- **`FpDiv`** -- shift-subtract restoring divide (~63 cycles for a 48-bit
  numerator).

The v2 design fits within 87 DSP blocks on the Cyclone V and runs the same
**16 testbench cases** bit-exactly against the golden.

### 6. v3 RTL -- performance + timing refinements -- [`fpga/hw/v3/`](fpga/hw/v3/)

[`fpga/hw/v3/README.md`](fpga/hw/v3/README.md) explains the changes; the
short version:

1. **Newton-Raphson `FpDiv`** -- LZC + 256x17 reciprocal-seed ROM + 2 NR
   iterations instead of shift-subtract. Roughly ~9 cycles instead of ~63.
2. **SPMV row-prologue collapse** -- v2 reads `rp_ptr[i]` and `rp_ptr[i+1]`
   per row (4 cycles); v3 carries `rp_hi -> rp_lo` and reads only
   `rp_ptr[i+1]` for rows after row 0 (saves 2 cycles per row).
3. **PIO registration + narrow FSM comparators** -- the inputs from the
   lightweight bridge are registered at the `CGTop` boundary and the
   stream-index comparators were narrowed from 32 to 6 bits (enough for
   `p_max_n=50`), which fixed a long combinational fanout from the PIOs.

End-to-end this gives ~1.26x-1.41x cycle-count speedup on the small
benchmarks (e.g., `simple_logic_10`: 54672 -> 38762 cycles). The 16 testbench cases
still pass bit-exactly -- the v3 testbench target is built with
`-DCG_GOLDEN_USE_NR` so the DPI golden uses the same NR divide as the RTL.

A 2-cyc/nnz pipelined SPMV was tried during v3 work; it passed Verilator
but failed timing on real silicon and was abandoned (see the v3 README).
v3's SPMV inner loop is the same serial 5-cyc/nnz walk as v2.

### 7. v4 RTL -- parallel x/r AXPY + fused VNS_R -- [`fpga/hw/v4/`](fpga/hw/v4/)

[`fpga/hw/v4/README.md`](fpga/hw/v4/README.md) has the full breakdown.
The headline change is exploiting the fact that the per-iter `x += alpha*d`
and `r -= alpha*q` updates have **no data dependency** on each other:

1. **Merged `S_AXPY_XR_FEED`** -- two AXPY units (`u_axpy_x` ADD,
   `u_axpy_r` SUB) run in lockstep, sharing the alpha coefficient.
   v3 ran the two updates sequentially with a single AXPY unit.
2. **Fused `S_VNS_R`** -- the RF gains a secondary write port, so the
   `r = -(cx+q)` and `d = r` writes happen in the same cycle, removing
   v3's separate `S_COPY_D` pass.

End-to-end this gives ~1.35x-1.40x cycle-count speedup over v3.
SPMV is unchanged.

### 8. v5 RTL -- multi-block on-chip RAM topology -- [`fpga/hw/v5/`](fpga/hw/v5/)

[`fpga/hw/v5/README.md`](fpga/hw/v5/README.md) explains the changes.
Headline: split the single shared Avalon SRAM into **seven dedicated
Qsys on-chip RAM slaves** (`q_val_ram`, `q_col_ram`, `q_rowp_ram`,
`cx_ram`, `cy_ram`, `x_ram`, `y_ram`). Each slave's FPGA-facing port
has exactly one consumer, so SPMV can fetch `q_val[j]` and `q_col[j]`
in the same cycle. Per-nz cost drops from v4's 5 cycles to 3.

Idea drawn from
[DuBois et al. 2008 (FCCM)](background-knowledge/papers/An_Implementation_of_the_Conjugate_Gradient_Algorithm_on_FPGAs.pdf),
which striped its CSR arrays across two parallel banks for the same
reason. End-to-end ~1.42x cycle-count speedup over v4 on
`parallel_chains_50`.

This requires regenerating `Computer_System.qsys` with seven on-chip
RAM IPs, and ships a per-version ARM mmap driver
([`cg_fpga_mmap_driver_v5.h`](fpga/sw/cg_fpga_mmap_driver_v5.h)) that
mmaps each region independently. v4's drivers stay at the bare
`cg_fpga_mmap_driver.h` / `cg_verilator_driver.h` paths.

### 9. v5_deep -- pipelined SPMV inner loop -- [`fpga/hw/v5_deep/`](fpga/hw/v5_deep/)

[`fpga/hw/v5_deep/README.md`](fpga/hw/v5_deep/README.md). Forks v5; the only
architectural change is a 4-stage SPMV pipeline that issues **1 nz per cycle in
steady state** (vs v5's 3 cycles/nz). On dense rows (`nnz >= 3`, typical for
placement Q) it asymptotes to a 3x SPMV speedup. Reuses v5's seven-slave Qsys
layout and v5 mmap driver verbatim. Note: this version works in simulation but
not on the FPGA - we suspect the issue is related to timing.

### 10. v6 RTL -- parallel x/y solve datapaths -- [`fpga/hw/v6/`](fpga/hw/v6/)

[`fpga/hw/v6/README.md`](fpga/hw/v6/README.md). Forks v5 (not
v5_deep). Runs the x and y analytical-placement solves on **two
independent CGEngines simultaneously**. The Q matrix is duplicated
across two M10K trios (10 Qsys slaves total: 6 Q + cx + cy + x + y)
so the two SPMVs run fully in parallel with zero contention. From the
ARM driver's perspective the protocol is identical to v5 -- one
`sw_go` pulse, one `sw_done` (= AND of both engines' dones).

End-to-end **~1.86x cycle-count speedup over v5** on
`parallel_chains_50` (131,088 -> 70,316 total HW CG cycles).

### 11. One `placer.cpp`, many backends

This is where everything composes. [`placer.cpp`](sw-baseline-c/placer.cpp)
contains a single `#include` block that picks the CG backend at compile
time:

```cpp
#ifdef USE_HW_CG
#if   defined(USE_FP_GOLDEN)            #include "cg_golden_driver.h"
#elif defined(CG_DRIVER_FPGA_MMAP_V6)   #include "cg_fpga_mmap_driver_v6.h"
#elif defined(CG_DRIVER_FPGA_MMAP_V5)   #include "cg_fpga_mmap_driver_v5.h"
#elif defined(CG_DRIVER_FPGA_MMAP)      #include "cg_fpga_mmap_driver.h"
#elif defined(CG_VERILATOR_V6)          #include "cg_verilator_driver_v6.h"
#elif defined(CG_VERILATOR_V5)          #include "cg_verilator_driver_v5.h"
#else                                   #include "cg_verilator_driver.h"
#endif
#endif
```

Each driver header exposes the same `CGHwDriver::solve(Q, cx, cy, x, y, ...)`
interface, so the rest of the placer is backend-agnostic. The resulting
backends are:

| Backend                                                                            | Macros                                          | What it runs                                                            | Used for                            |
| ---------------------------------------------------------------------------------- | ----------------------------------------------- | ----------------------------------------------------------------------- | ----------------------------------- |
| Hand-rolled software CG                                                            | (none)                                          | Double-precision `cg_solve()` in placer.cpp                             | Reference timings, the SW oracle    |
| [`cg_golden_driver.h`](sw-baseline-c/cg_golden_driver.h)                           | `USE_HW_CG`, `USE_FP_GOLDEN`                    | Fixed-point [`CGGolden`](sw-baseline-c/cg_golden_model.h) C++ model     | Bit-exact SW model of the FPGA      |
| [`cg_verilator_driver.h`](sw-baseline-c/cg_verilator_driver.h)                     | `USE_HW_CG` (+ `HW_CG_VERSION=v2/v3/v4`)        | Verilated `CGTop` RTL (single shared SRAM)                              | RTL-in-the-loop, v2-v4              |
| [`cg_verilator_driver_v5.h`](sw-baseline-c/cg_verilator_driver_v5.h)               | `USE_HW_CG`, `CG_VERILATOR_V5` (v5/v5_deep)     | Verilated v5 RTL (7-slave multi-block topology)                         | RTL-in-the-loop, v5/v5_deep         |
| [`cg_verilator_driver_v6.h`](sw-baseline-c/cg_verilator_driver_v6.h)               | `USE_HW_CG`, `CG_VERILATOR_V6` (v6)             | Verilated v6 RTL (10-slave parallel x/y solver)                         | RTL-in-the-loop, v6                 |
| [`cg_fpga_mmap_driver.h`](fpga/sw/cg_fpga_mmap_driver.h)                           | `USE_HW_CG`, `CG_DRIVER_FPGA_MMAP`              | Real DE1-SoC bitstream over `/dev/mem` (single shared SRAM)             | FPGA placer, v4 bitstream           |
| [`cg_fpga_mmap_driver_v5.h`](fpga/sw/cg_fpga_mmap_driver_v5.h)                     | `USE_HW_CG`, `CG_DRIVER_FPGA_MMAP_V5`           | Real DE1-SoC bitstream over `/dev/mem` (7-slave Qsys)                   | FPGA placer, v5 bitstream           |
| [`cg_fpga_mmap_driver_v6.h`](fpga/sw/cg_fpga_mmap_driver_v6.h)                     | `USE_HW_CG`, `CG_DRIVER_FPGA_MMAP_V6`           | Real DE1-SoC bitstream over `/dev/mem` (10-slave Qsys)                  | FPGA placer, v6 bitstream           |

The FPGA build doesn't have its own `placer.cpp` --
[`fpga/sw/placer.cpp`](fpga/sw/placer.cpp) is a symlink to
[`sw-baseline-c/placer.cpp`](sw-baseline-c/placer.cpp). The FPGA-specific
bits live entirely in the `cg_fpga_mmap_driver*.h` family, picked by
the FPGA `CMakeLists.txt` via the `HW_FPGA_VERSION` cache variable.
Same source, different driver.

---

## Running the placer

From a fresh `build/` directory, the [`run-placer`](python-utils/run_placer.py)
console-script handles LEF/DEF parse + cmake + make + run for any backend
(see [`pyproject.toml`](pyproject.toml) for the entry-point definition):

```bash
mkdir -p build && cd build

uv run run-placer python       ../benchmarks/iccad04/DMA               # Python baseline
uv run run-placer sw           ../benchmarks/iccad04/DMA               # SW CG (double precision)
uv run run-placer golden       ../benchmarks/custom/parallel_chains_50 # FP golden CG (SW); --int-bits / --frac-bits / --max-n / --max-iter
uv run run-placer verilated    ../benchmarks/custom/parallel_chains_50 # Verilator CG (default v6); same scaling knobs as golden
uv run run-placer verilated v3 ../benchmarks/custom/parallel_chains_50 # Verilator CG (older version)
uv run run-placer arm          ../benchmarks/iccad04/DMA               # SW CG on the DE1-SoC ARM
uv run run-placer fpga         ../benchmarks/custom/parallel_chains_50 # Real FPGA bitstream (default v6); MAX_N=50, locked Q13.14
```

`--int-bits I` / `--frac-bits B` / `--max-n N` / `--max-iter K` are
documented in the [Quick start](#quick-start) section -- only `golden`
and `verilated` honor the first three; `--max-iter` works on every
mode except `python` and is mutually exclusive with `--sweep`.

The `arm` and `fpga` modes need `arm-linux-gnueabihf-g++` on the host;
SSH/SCP to the board are handled in-process by
[`fabric`](https://www.fabfile.org/) (no `sshpass` required). Board
credentials (`BOARD`, `PASS`) are loaded from a gitignored [`.env`](.env)
at the repo root -- create one with the board's SSH target and root
password before running those modes. `fpga` additionally requires the CG
bitstream to already be programmed onto the DE1-SoC.

```bash
# .env
BOARD='root@10.253.17.19'
PASS='greatpassword123!'
```

## Batch mode -- run every benchmark in a directory

If `<benchmark-path>` is a single benchmark directory (one with `lef/`
and `def/` subdirs that contain at least one `.lef` and one `.def`),
`run-placer` runs that one design and prints a per-run summary. If
the path is a *parent* directory whose immediate children are
benchmark dirs (e.g. `benchmarks/custom`, `benchmarks/iccad04`),
`run-placer` automatically:

1. Builds the placer once (the binary is the same for every benchmark).
2. For remote modes (`arm`, `fpga`), opens **one** SSH session and
   reuses it across every benchmark.
3. Runs each benchmark in the parent dir in alphabetical order.
4. Prints a per-run summary block after each benchmark and a fixed-width
   cross-benchmark comparison table at the end (cells, iters, outcome,
   final HPWL, density, CG avg/total ms, and -- for hardware modes --
   HW CG cycles).

Batch mode is detected from the directory structure -- there is no
extra flag. `python` mode does not support batch (use `sw` instead).

```bash
# Run every custom benchmark on the v6 RTL through Verilator
uv run run-placer verilated v6 ../benchmarks/custom

# Run every custom benchmark on the FPGA in a single SSH session
uv run run-placer fpga ../benchmarks/custom
```

When `run-placer` is invoked from a directory whose name starts with
`build`, the directory is wiped at the start of each run. This keeps
batch invocations from accidentally mixing artifacts from different
modes / RTL versions in the same build dir.

## Iteration sweep + animation

Pass `--sweep` to `run-placer` to run the placer 16 times with
`max_outer_iter` from 1 to 16, capture `<design>-final-iter{NN}.json` for
each step, render a PNG of each via the visualizer, and stitch them into
`<design>-sweep.gif` and `<design>-sweep.mp4`. The sweep stops early as
soon as the placer reports it needed fewer iterations than the cap (so
the slideshow doesn't show duplicate frames). Sweep supports the same
backends as a single run except `python`.

```bash
uv run run-placer sw ../benchmarks/iccad04/DMA --sweep
```

## Visualizing a placement

[`python-utils/visualizer.py`](python-utils/visualizer.py) opens a Tk window
with the die outline, components (blue rectangles), and I/O pins (yellow
dots). Hovering over a cell shows its name and macro type. It is exposed
as a `visualizer` console-script entry point (see
[`pyproject.toml`](pyproject.toml)) so `uv sync` makes it directly
runnable:

```bash
uv run visualizer DMA-final.json
```

Controls: scroll to zoom, drag to pan, F to fit-all, Q to quit. A
`--png <out.png> <in.json>` flag renders the same view headlessly (used
by `run-placer` to produce `<design>-final.png` after every run, and
the per-iter frames in sweep mode).

## Running the tests

```bash
# Python FL model -- 19 cases vs scipy
uv run fpga/fl/test/CGTop-test.py

# RTL testbench -- 16 cases vs DPI golden, one binary per RTL version
mkdir -p build-tb && cd build-tb && cmake ../fpga/hw/test && make
./VCGTop_tb_v1
./VCGTop_tb_v2
./VCGTop_tb_v3
./VCGTop_tb_v4
./VCGTop_tb_v5
./VCGTop_tb_v5_deep
./VCGTop_tb_v6
```

Each RTL target should print `ALL 16 TESTS PASSED`. The Python suite should
print `Result: 19/19 tests passed`.

---

## Directory map

| Path                                       | Stage in the narrative |
| ------------------------------------------ | ---------------------- |
| [sw-baseline-python/](sw-baseline-python/) | (1) Python reference placer |
| [sw-baseline-c/](sw-baseline-c/)           | (2) Canonical C++ placer + (11) all SW-only CG drivers (default, golden, Verilator v2/v3/v4/v5/v5_deep/v6) |
| [fpga/fl/](fpga/fl/)                       | (3) Python FL model of the CG kernel |
| [fpga/fl/test/](fpga/fl/test/)             | (3) FL tests vs scipy |
| [fpga/hw/v1/](fpga/hw/v1/)                 | (4) Combinational reference RTL (not synthesizable) |
| [fpga/hw/v2/](fpga/hw/v2/)                 | (5) First synthesizable RTL |
| [fpga/hw/v3/](fpga/hw/v3/)                 | (6) NR FpDiv + SPMV row-prologue collapse + PIO timing |
| [fpga/hw/v4/](fpga/hw/v4/)                 | (7) Parallel x/r AXPY + fused VNS_R |
| [fpga/hw/v5/](fpga/hw/v5/)                 | (8) Multi-block on-chip RAM topology (7 slaves) |
| [fpga/hw/v5_deep/](fpga/hw/v5_deep/)       | (9) Forks v5 with 1-cyc/nz pipelined SPMV |
| [fpga/hw/v6/](fpga/hw/v6/)                 | (10) Parallel x/y solve datapaths (10 slaves, 2 engines) |
| [fpga/hw/test/](fpga/hw/test/)             | Verilator + DPI golden testbench (one binary per RTL version) |
| [fpga/sw/](fpga/sw/)                       | (11) ARM-side mmap drivers (`cg_fpga_mmap_driver{,_v5,_v6}.h`) and `placer.cpp` symlink to canonical source; `FPGATop.v` lives per-version under `fpga/hw/v*/` |
| [python-utils/](python-utils/)             | LEF/DEF parser, visualizer, run-placer entry points |
| [benchmarks/](benchmarks/)                 | ICCAD04 + custom designs |
| [background-knowledge/](background-knowledge/) | Reference material on placement algorithms |

## Team

Parker Schless (pis7), Colin Muessig (cjm369), Jeremy Ku-Benjet (jk2582)
