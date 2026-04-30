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
uv sync
```

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
    tiny1/ { lef/tiny1.lef, def/tiny1.def }   # ~10 cells, hand-written
    tiny2/ ...
    tiny3/ ...
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

[`python-utils/lefdef-parser.py`](python-utils/lefdef-parser.py) is a
hand-rolled streaming tokenizer (no external LEF/DEF library) that walks
both files once and emits a single unified JSON. Class-based with one
public method per stage (`find_files`, `parse_lef`, `parse_def`,
`write_json`).

```bash
uv run ../python-utils/lefdef-parser.py ../benchmarks/iccad04/DMA
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
on each component. The visualizer, the slideshow tool, and every backend in
the table below all consume the same JSON -- there is no second
representation of the netlist anywhere in the project.

---

## How the project was built

The project was built bottom-up in six stages. Each new layer was validated
against the layer below it before we moved on -- so there is always a known-
good reference to diff against when something breaks.

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
- **DSP-mapped multipliers** -- `FpMul`/`FpMulWide` carry
  `(* multstyle = "dsp" *)` so Quartus targets the Cyclone V's 27x27 DSPs.
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
benchmarks (e.g., `tiny1`: 54672 -> 38762 cycles). The 16 testbench cases
still pass bit-exactly -- the v3 testbench target is built with
`-DCG_GOLDEN_USE_NR` so the DPI golden uses the same NR divide as the RTL.

A 2-cyc/nnz pipelined SPMV was tried during v3 work; it passed Verilator
but failed timing on real silicon and was abandoned (see the v3 README).
v3's SPMV inner loop is the same serial 5-cyc/nnz walk as v2.

### 7. One `placer.cpp`, four backends

This is where everything composes. [`placer.cpp`](sw-baseline-c/placer.cpp)
contains a single `#include` block that picks the CG backend at compile
time:

```cpp
#ifdef USE_HW_CG
#if   defined(USE_FP_GOLDEN)        #include "cg_golden_driver.h"
#elif defined(CG_DRIVER_FPGA_MMAP)  #include "cg_fpga_mmap_driver.h"
#else                               #include "cg_verilator_driver.h"
#endif
#endif
```

Each driver header exposes the same `CGHwDriver::solve(Q, cx, cy, x, y, ...)`
interface, so the rest of the placer is backend-agnostic. The four resulting
backends are:

| Backend                                                        | Macros                              | What it runs                                            | Used for                              |
| -------------------------------------------------------------- | ----------------------------------- | ------------------------------------------------------- | ------------------------------------- |
| Hand-rolled software CG                                        | (none)                              | Double-precision `cg_solve()` in placer.cpp             | Reference timings, the SW oracle      |
| [`cg_golden_driver.h`](sw-baseline-c/cg_golden_driver.h)       | `USE_HW_CG`, `USE_FP_GOLDEN`        | Fixed-point [`CGGolden`](sw-baseline-c/cg_golden_model.h) C++ model | Bit-exact SW model of the FPGA        |
| [`cg_verilator_driver.h`](sw-baseline-c/cg_verilator_driver.h) | `USE_HW_CG` (+ `HW_CG_VERSION=v2/v3`) | Verilated `CGTop` RTL                                   | RTL-in-the-loop placement (slow)      |
| [`cg_fpga_mmap_driver.h`](fpga/sw/cg_fpga_mmap_driver.h)       | `USE_HW_CG`, `CG_DRIVER_FPGA_MMAP`  | Real DE1-SoC bitstream over `/dev/mem`                  | The actual FPGA-accelerated placer    |

The FPGA build doesn't have its own `placer.cpp` --
[`fpga/sw/placer.cpp`](fpga/sw/placer.cpp) is a symlink to
[`sw-baseline-c/placer.cpp`](sw-baseline-c/placer.cpp). The FPGA-specific
bits live entirely in `fpga/sw/cg_fpga_mmap_driver.h`, which the FPGA
`CMakeLists.txt` selects via `-DCG_DRIVER_FPGA_MMAP`. Same source,
different driver.

---

## Running the placer

From a fresh `build/` directory, [`run-placer.sh`](run-placer.sh) handles
LEF/DEF parse + cmake + make + run for any backend:

```bash
mkdir -p build && cd build

../run-placer.sh python    ../benchmarks/iccad04/DMA   # Python baseline
../run-placer.sh sw        ../benchmarks/iccad04/DMA   # SW CG (double precision)
../run-placer.sh golden    ../benchmarks/iccad04/DMA   # FP golden CG (SW)
../run-placer.sh verilated ../benchmarks/iccad04/DMA   # Verilator CG (default v3)
../run-placer.sh verilated ../benchmarks/custom/tiny3 v2
../run-placer.sh arm       ../benchmarks/iccad04/DMA   # SW CG on the DE1-SoC ARM
../run-placer.sh fpga      ../benchmarks/iccad04/DMA   # Real FPGA bitstream
../run-placer.sh vis       DMA-final.json              # Tk visualizer
```

The `arm` and `fpga` modes need `arm-linux-gnueabihf-g++` and `sshpass` on
the host and assume the board is reachable at the IP set near the top of
[`run-placer.sh`](run-placer.sh). `fpga` additionally requires the CG
bitstream to already be programmed onto the DE1-SoC.

## Iteration sweep + animation

[`placer-sweep.sh`](placer-sweep.sh) accepts the same backend modes as
`run-placer.sh` but runs the placer 16 times with `max_outer_iter` from 1 to
16, captures `<design>-final-iter{NN}.json` for each step, renders a PNG of
each via the visualizer, and stitches them into `<design>-sweep.gif` and
`<design>-sweep.mp4`. It stops early as soon as the placer reports it
needed fewer iterations than the cap (so the slideshow doesn't show
duplicate frames).

```bash
../placer-sweep.sh sw ../benchmarks/iccad04/DMA
```

## Visualizing a placement

[`python-utils/visualizer.py`](python-utils/visualizer.py) opens a Tk window
with the die outline, components (blue rectangles), and I/O pins (yellow
dots). Hovering over a cell shows its name and macro type.

```bash
uv run ../python-utils/visualizer.py DMA-final.json
```

Controls: scroll to zoom, drag to pan, F to fit-all, Q to quit. A
`--png <out.png> <in.json>` flag renders the same view headlessly (used by
the sweep above).

## Running the tests

```bash
# Python FL model -- 19 cases vs scipy
uv run fpga/fl/test/CGTop-test.py

# RTL testbench -- 16 cases vs DPI golden, for each of v1/v2/v3
mkdir -p build-tb && cd build-tb && cmake ../fpga/hw/test && make
./VCGTop_tb_v1
./VCGTop_tb_v2
./VCGTop_tb_v3
```

Each RTL target should print `ALL 16 TESTS PASSED`. The Python suite should
print `Result: 19/19 tests passed`.

---

## Directory map

| Path                                       | Stage in the narrative |
| ------------------------------------------ | ---------------------- |
| [sw-baseline-python/](sw-baseline-python/) | (1) Python reference placer |
| [sw-baseline-c/](sw-baseline-c/)           | (2) Canonical C++ placer + (7) all SW-only CG drivers (default, golden, Verilator) |
| [fpga/fl/](fpga/fl/)                       | (3) Python FL model of the CG kernel |
| [fpga/fl/test/](fpga/fl/test/)             | (3) FL tests vs scipy |
| [fpga/hw/v1/](fpga/hw/v1/)                 | (4) Combinational reference RTL |
| [fpga/hw/v2/](fpga/hw/v2/)                 | (5) Synthesizable RTL |
| [fpga/hw/v3/](fpga/hw/v3/)                 | (6) Performance/timing refinements |
| [fpga/hw/test/](fpga/hw/test/)             | (4-6) Verilator + DPI golden testbench (one for each of v1/v2/v3) |
| [fpga/hw/FPGATop.v](fpga/hw/FPGATop.v)     | DE1-SoC top: wires CGTop into Qsys SRAM + control PIOs |
| [fpga/sw/](fpga/sw/)                       | (7) ARM-side driver and the `cg_fpga_mmap_driver.h` |
| [python-utils/](python-utils/)             | LEF/DEF parser, visualizer, slideshow |
| [benchmarks/](benchmarks/)                 | ICCAD04 + custom designs |
| [background-knowledge/](background-knowledge/) | Reference material on placement algorithms |

## Team

Parker Schless (pis7), Colin Muessig (cjm369), Jeremy Ku-Benjet (jk2582)
