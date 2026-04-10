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

### Visualizer

Opens a Tk GUI showing cell placements on the die.

```bash
uv run ../design-file-tools/visualizer.py DMA-final.json
```

Controls: scroll to zoom, drag to pan, R to reset view.

## Directory Structure

- `sw-baseline-python/` -- Python reference placer implementation
- `sw-baseline-c/` -- C++ placer implementation (CMake build)
- `design-file-tools/` -- LEF/DEF parser and visualizer
- `benchmarks/` -- ICCAD04 and custom benchmark designs
- `background-knowledge/` -- Reference material on placement algorithms
- `docs-local/` -- Proposal and documentation (not checked in)

## Team

Parker Schless (pis7), Colin Muessig (cjm369), Jeremy Ku-Benjet (jk2582)
