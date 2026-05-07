Introduction
--------------------------------------------------------------------------------
(Jeremy)

- Placement is the process by which we place logic blocks (which are called
  standard cells/macros) in an ASIC/SoC to make it easy to route connections
- Analytical placement applies analytical methods to solve an optimization
  problem which in our case is to minimize wirelength. In doing this, the cells
  tend to cluster together, so we then need to pull them apart using a step
  called partitioning. The solve and partitioning steps are then repeated until
  we reach a desired density target.
- The cost function is formulated as the quadratic programming problem: $\phi(x)
  = (1/2) x^T Q x + c^T x + constant$, which we solve by setting its gradient
  ($\nabla\phi(x) = Qx + c$) to 0: $Qx = -c$, where Q is a connectivity matrix
  encoding the strength of connections between macros, x is the vector of
  coordinates to solve for, and c is a vector representing connections from the
  cells to fixed points such as I/O pins. This solve needs to be performed for
  both the X and Y coordinates separately.
- Conjugate gradient portion of analytical global placement algorithm with
  quadratic wirelength implemented on FPGA.

(Parker)
- Solve $Qx = -c$, starting from x0. Returns solution in x.
  Matches the CG algorithm from the project proposal (Listing 1):
```bash
    x  = x0
    r  = -c - Q * x                       # SPMV -> speed optimized
    d  = r
    rr = dot(r, r)                        # Unrolled
    for k = 1, 2, ... do                  
        q      = Q * d                    # SPMV -> speed optimized
        alpha  = rr / dot(d, q)           # Unrolled
        x      = x + alpha * d            # Unrolled + parallelized
        r      = r - alpha * q            # Unrolled + parallelized
        rr_new = dot(r, r)                # Unrolled
        if rr_new < eps^2 then return x
        beta   = rr_new / rr
        d      = r + beta * d             # Unrolled
        rr     = rr_new
    end for 
```

Benchmarks
--------------------------------------------------------------------------------
(Colin, Parker runs commands, either Parker/Jeremy explains script outputs)

The `run-placer` script starts with a set of LEF/DEF files and parses these
into an intermediate JSON format (the LEF file is a library of macros, and the DEF 
file gives an initial placement which we optimize). It then cross-compiles `placer.cpp`,
SCPs it and the JSON to the HPS, and runs the program.

The script has a couple different operating modes. When running in arm mode, we use a
fully software-based CG solver. 

```bash
run-placer arm ../benchmarks/custom/parallel_chains_50 --sweep
code parallel_chains_50-sweep.gif 
# PARKER / JEREMY: explain the visualization tool & output
```

When in FPGA mode, we instead offload CG computation onto the FPGA. To create that
interface, we have some PIOs plus memory mapping done using `mmap`. The Q, x, and c
vectors all live in M10K memory and we really only copy values when we need to. This 
actually uses the same `placer.cpp` script as the ARM mode, but the file will know 
to run differently in eithe case becasue of a a define flag we set here.

```bash
run-placer fpga ../benchmarks/custom/parallel_chains_50 --sweep
code parallel_chains_50-sweep.gif
# PARKER / JEREMY: explain command outputs

vis parallel_chains_50.json
# PARKER / JEREMY: explain command outputs, compare cg-total, cg-average and placer
# total; highlight 10-15x speed-up for CG average, but only 10% improvement in total
# placer time since CG solve does not dominate for small designs
```

While we are limited on FPGA resources, we did verilate a hardware version which 
can place a full ICCAD benchmark with comparable placement quality to the software 
version. This proves that given more FPGA resources, our hardware would be able to 
place larger designs.

TinyFlow Demo
--------------------------------------------------------------------------------
(Jeremy, Parker runs commands)

These all started from lef/def files:

```bash
code ../benchmarks/custom/parallel_chains_50/lef/parallel_chains_50.lef
code ../benchmarks/custom/parallel_chains_50/def/parallel_chains_50.def
```

This is cool, but it's hard to get a feel for the actual design. Let's start
from verilog.

```bash
code ../../rtl/Demo1.v
```

This uses infrastructure from a project Parker and Jeremy worked on for the
ASICs class called TinyFlow, where students write their own algorithms to
perform Verilog to netlist synthesis. We use this to parse and lower this
netlist to lef/def for input to our placer.

```bash
pyhflow ../designs/demo1-lef-def-only.yml
./run-flow
code 01-tinyflow-synth/post-synth.v
code 03-summarize-results/lef/Demo1.lef
code 03-summarize-results/def/Demo1.def

run-placer fpga ../../project1-group11/asic/build-demo1/03-summarize-results/ --sweep
code Demo1-sweep.gif 
vis Demo1-final.json
```

Now you write some verilog code!!!
Let's do a global placement of it.

(Prof writes code)

```bash
code ../../rtl/Demo2.v
pyhflow ../designs/demo2-lef-def-only.yml
./run-flow

run-placer fpga ../../project1-group11/asic/build-demo2/03-summarize-results/ --sweep
code Demo2-sweep.gif 
vis Demo2-final.json
```
