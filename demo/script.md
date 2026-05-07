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
(Colin, Parker runs commands)

Central `run-placer` script parses lef/def files into intermediate json format,
cross-compiles `placer.cpp` for HPS. SCP's `placer.cpp` and json to HPS and runs
the program.

When running in arm mode, we use the software-based CG solver. When in FPGA
mode, we use a hardware connector shim for the CG solve which implements memory
mapping and PIO communications. The same `placer.cpp` script is the same between
the both, a define flag determines which version we use.

Note that in the FPGA version, the Q, x, and c vectors all live in M10K memory
to begin using `mmap` so we only copy values when we need to - this is a prime
example of hardware-software codesign that we take advantage of.

```bash
run-placer arm ../benchmarks/custom/parallel_chains_50 --sweep
code parallel_chains_50-sweep.gif 

run-placer fpga ../benchmarks/custom/parallel_chains_50 --sweep
code parallel_chains_50-sweep.gif

vis parallel_chains_50.json
```

Compare cg-total, cg-average, and placer total for all of these We can see what
the placer is doing at each step using the above command. 10-15x speedup for cg
average, only about 10% improvement in total placer time since cg solve doesnt
dominate for small designs.

We did verilate a hardware version which can place the full iccad DMA benchmark
with comparable placement quality to the software version. This proves that
given more FPGA resources, our hardware would be able to place larger designs.

TinyFlow Demo
--------------------------------------------------------------------------------
(Jeremy, Parker runs commands)

These all started from lef/def files. The lef file is a library of macros and
the def files is an inital placement which we optimize.

```bash
code ../benchmarks/custom/parallel_chains_50/def/parallel_chains_50.lef
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

run-placer fpga ../../project1-group11/asic/build-demo1/03-summarize-results/03-summarize-results --sweep
code Demo1-sweep.gif 
vis Demo1-final.json
```

Now you write some verilog code!!!
Let's do a global placement of it.

(Prof writes code)

```bash
pyhflow ../designs/demo2-lef-def-only.yml
./run-flow

run-placer fpga ../../project1-group11/asic/build-demo2/03-summarize-results/ --sweep
code Demo2-sweep.gif 
vis Demo2-final.json
```
