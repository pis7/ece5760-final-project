# ECE 5760 Final Project - Analytical Placement via FPGA

This project accelerates analytical placement (a key EDA/ASIC algorithm) using
the DE1-SoC Cyclone V FPGA. Standard cell placement is formulated as a
quadratic optimization problem and solved via conjugate gradient descent.

## Setup

Requires Python >= 3.14 and [uv](https://docs.astral.sh/uv/).

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

### Placer

Runs the placement algorithm on a benchmark directory and writes the result
to `./<design_name>/` in the same LEF/DEF directory format.

```bash
uv run ../sw-baseline-python/placer.py ../benchmarks/iccad04/DMA
```

Output:
```
build/DMA/
  lef/dma.lef   (copy of original)
  def/dma.def   (updated component positions)
```

### Visualizer

Opens a Tk GUI showing macro sizes and placements on the die. Works on both
input benchmarks and placer output directories.

```bash
# View original benchmark
uv run ../design-file-tools/visualizer.py ../benchmarks/iccad04/DMA

# View placer output
uv run ../design-file-tools/visualizer.py DMA
```

Controls:
- Scroll wheel: zoom in/out
- Click + drag: pan
- R: reset view to fit all

## Directory Structure

- `sw-baseline-python/` -- Python reference placer implementation
- `design-file-tools/` -- LEF/DEF parser and visualizer
- `sw-baseline-c/` -- C implementation for HPS ARM processor (planned)
- `benchmarks/` -- ICCAD04 benchmark designs
- `background-knowledge/` -- Reference material on LEF/DEF format and
  analytical placement algorithms
- `docs-local/` -- Proposal and documentation (not checked in)

## Team

Parker Schless (pis7), Colin Muessig (cjm369), Jeremy Ku-Benjet (jk2582)
