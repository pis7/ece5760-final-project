# ECE 5760 Final Project - Analytical Placement via FPGA

This project accelerates analytical placement (a key EDA/ASIC algorithm) using
the DE1-SoC Cyclone V FPGA. Standard cell placement is formulated as a
quadratic optimization problem and solved via conjugate gradient descent.

## Setup

Requires Python >= 3.14, [uv](https://docs.astral.sh/uv/), CMake >= 3.10, and
a C++17 compiler.

```bash
uv sync
```

## Benchmarks

ICCAD 2004 Faraday mixed-size benchmarks are in `benchmarks/iccad04/`. Each
design directory contains `lef/` and `def/` subdirectories:

```
benchmarks/iccad04/
  DMA/lef/dma.lef, DMA/def/dma.def
  DSP1/lef/dsp1.lef, DSP1/def/dsp1.def
  DSP2/lef/dsp2.lef, DSP2/def/dsp2.def
  RISC1/lef/risc1.lef, RISC1/def/risc1.def
  RISC2/lef/risc2.lef, RISC2/def/risc2.def
```

## Usage

All scripts should be run from a `build/` directory:

```bash
mkdir -p build
cd build
```

### Parse LEF/DEF to JSON

```bash
uv run ../design-file-tools/lefdef-parser.py ../benchmarks/iccad04/DMA
```

### Placer (Python)

```bash
uv run ../sw-baseline-python/placer.py DMA.json
```

### Placer (C++)

```bash
cmake ../sw-baseline-c
make
./placer DMA.json
```

Both placers read the same netlist JSON and write `<design>-initial.json` and
`<design>-final.json` with updated component positions.

### End-to-end driver script

`run-placer.sh` wraps the LEF/DEF parse + build + run flow. It also handles
cross-compiling and remote execution on the DE1-SoC:

```bash
./run-placer.sh python    benchmarks/iccad04/DMA      # Python baseline
./run-placer.sh sw        benchmarks/iccad04/DMA      # C++ (double precision)
./run-placer.sh golden    benchmarks/iccad04/DMA      # C++ with fixed-point golden CG
./run-placer.sh verilated benchmarks/iccad04/DMA      # C++ with Verilator RTL CG
./run-placer.sh arm       benchmarks/iccad04/DMA      # Cross-compile SW placer, run on DE1-SoC
./run-placer.sh fpga      benchmarks/iccad04/DMA      # FPGA-accelerated, run on DE1-SoC
./run-placer.sh vis       DMA-final.json              # Tk visualizer
```

The `arm` and `fpga` modes require `arm-linux-gnueabihf-g++` and `sshpass` on
the host, and assume the board is reachable at the IP configured at the top
of `run-placer.sh`. `fpga` additionally requires the CG bitstream to be
programmed on the DE1-SoC.

### Visualizer

Opens a Tk GUI showing cell placements on the die.

```bash
uv run ../design-file-tools/visualizer.py DMA-final.json
```

Controls: scroll to zoom, drag to pan, R to reset view.

## Directory Structure

- `sw-baseline-python/` -- Python reference placer implementation
- `sw-baseline-c/` -- C++ placer implementation (CMake build); the canonical
  `placer.cpp` lives here and is shared by the FPGA build via a symlink.
- `fpga/` -- FPGA-side code:
  - `fpga/hw/v1/` -- original combinational Verilog (not synthesizable; for
    reference and Verilator cosim).
  - `fpga/hw/v2/` -- synthesizable CG solver for DE1-SoC (sequential
    datapath, parameterized parallelism, fits within 87 DSP blocks).
  - `fpga/hw/test/` -- Verilator testbench with DPI golden-model CG.
  - `fpga/hw/FPGATop.v` -- toplevel wrapping Qsys + CGTop, wired to the
    on-chip SRAM and PIOs.
  - `fpga/sw/` -- ARM-side placer driver. `placer.cpp` is a symlink to
    `sw-baseline-c/placer.cpp`; `cg_hw_driver.h` is an mmap-based driver
    that talks to the real FPGA via `/dev/mem`.
  - `fpga/fl/` -- Python functional-level model of the CG solver.
- `design-file-tools/` -- LEF/DEF parser and visualizer
- `benchmarks/` -- ICCAD04 and custom benchmark designs
- `background-knowledge/` -- Reference material on placement algorithms
- `local/` -- Proposal and documentation (not checked in)

## Team

Parker Schless (pis7), Colin Muessig (cjm369), Jeremy Ku-Benjet (jk2582)
