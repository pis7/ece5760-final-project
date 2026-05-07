# v6 CG Solver - Parallel x/y solve datapaths

Forks [v5](../v5/) (not [v5_deep](../v5_deep/)) and runs the x and y
analytical-placement solves on **two independent CGEngines
simultaneously**. The Q matrix is duplicated across two M10K trios so
the two SPMVs (the per-iter dominant kernel) run fully in parallel
with zero contention. cx/cy/x/y were already dimension-private in v5
and pass through unchanged.

## Changes vs v5

| Area | v5 | v6 | Win |
| --- | --- | --- | --- |
| Solve scheduling | x then y, one engine, time-multiplexed via `sel_y_reg` | x and y in parallel, two `CGEngine` instances | ~2x SPMV (the dominant kernel) |
| Q SRAM count | 3 slaves (`q_val`, `q_col`, `q_rowp`) | 6 slaves (3 per engine; Q duplicated) | engines never contend on Q reads |
| `CGCtrl` FSM | flips `sel_y_reg` after first writeback, loops back to `S_LD_X_ADDR` for y pass | single-dimension solver; `S_WB_WRITE -> S_CG_DONE` unconditional | simpler control |
| `CGTop` | wires one CGCtrl + CGDpath, muxes `cx_ram`/`cy_ram` and `x_ram`/`y_ram` by `sel_y` | thin wrapper that instantiates `CGEngine` twice; no muxing | each slave port has exactly one consumer |
| Per-engine `p_lanes` | 8 (one engine) | 8 per engine (two engines) | same throughput |
| Avalon slave count | 7 (`q_val,q_col,q_rowp,cx,cy,x,y`) | 10 (6 Q + cx,cy,x,y) | -- |
| ARM driver protocol | one `sw_go`/`sw_done` cycle | **identical** -- one go, one done | no software flow change |
| Q load on ARM side | one memcpy into `q_val_ram` etc. | two memcpys (one per engine's Q copy) | trivial cost (Q load is one-shot) |

End-to-end on Verilator HW CG (`parallel_chains_50`):

| Metric | v5 | v6 | Speedup |
| --- | --- | --- | --- |
| Total HW CG cycles | 131,088 | 70,316 | **1.86x** |
| Average cycles/solve | 16,386 | 8,789 | **1.86x** |

HPWL trajectory matches v5 within fixed-point noise (the same DPI
golden runs x then y serially; v6 must produce the same final x and y
vectors -- and does).

## Files

| File | Role |
| --- | --- |
| [CGTop.v](CGTop.v) | Toplevel: instantiates two `CGEngine`s, fans out `sw_go`, ANDs both `sw_done`s |
| [CGEngine.v](CGEngine.v) | **NEW** wrapper: one `CGCtrl` + one `CGDpath` exposing one dimension's slaves |
| [CGCtrl.v](CGCtrl.v) | Single-dimension FSM (no `sel_y_reg`, no x->y loopback) |
| [CGDpath.v](CGDpath.v) | Pure datapath, identical to v5 (only comment-level changes) |
| [LinAlg.v](LinAlg.v) | Unchanged from v5: VecDot, AXPY, SPMV (3 cyc/nz), FpDiv |
| [FpMath.v](FpMath.v) | Unchanged from v5: FpMul, FpMulWide, FpDiv (NR) |
| [FPGATop.v](FPGATop.v) | DE1-SoC pin map; 10 on-chip RAM bundles + 5 PIOs; `\`WIRE_RAM_PORTS` macro |

## Top-level architecture

```
                       sw_go / sw_done                 Avalon h2f bridge
                          |     ^                       |
                          v     |                       v
                       +------- CGTop ----------+
                       |                        |
            engine_x   |                        |   engine_y
       +-- CGEngine --+|                        |+-- CGEngine --+
       | CGCtrl       ||                        || CGCtrl       |
       | CGDpath      ||                        || CGDpath      |
       | LinAlg ports ||                        || LinAlg ports |
       +--------------+|                        |+--------------+
              |        |                        |        |
              | Q_x trio + cx + x               | Q_y trio + cy + y
              v        |        AND of dones    |        v
       q_val_x  q_col_x  q_rowp_x      q_val_y  q_col_y  q_rowp_y
       cx                x                      cy                y
```

`sw_done = engine_x.sw_done & engine_y.sw_done`. `sw_done_ack` fans
out to both engines. `max_iter`, `eps_sq`, `n` are broadcast.

## Tradeoff: duplicate Q vs. arbitrate (the v6 design choice)

v6 takes the duplication path. The alternatives considered:

- **Arbitrate one set of Q slaves between two engines.** Both SPMVs
  want a Q-read every cycle, so cycle-alternated arbitration halves
  each engine's SPMV throughput. That defeats the whole point --
  SPMV is the dominant per-iter kernel and the only reason to add a
  second engine.
- **True dual-port M10K with two FPGA-facing read ports.** The stock
  Altera On-Chip Memory IP only exposes two slave ports total, and
  v5 already uses both (one h2f, one FPGA-facing). Routing around
  this needs a custom altsyncram + Avalon-MM shim -- complexity for
  ~10 M10K of savings we don't need.
- **Duplicate Q.** What v6 does. At `p_max_n=50` Q duplication costs
  ~10 M10K (`q_val=2500*27b ~7`, `q_col=2500*~6b ~2`, `q_rowp ~1`,
  doubled). Cyclone V 5CSEMA5 has 397 M10K -- <3% of the budget.

ARM writes Q twice during one-time design load; the per-iter loop is
thousands of solves so the extra memcpy is negligible.

## Required Qsys changes (manual, on the DE1-SoC build)

Regenerate `Computer_System.qsys` with **ten** on-chip RAM IPs whose
FPGA-facing slave port names match [FPGATop.v](FPGATop.v):

| Slave | Words | RTL-facing port prefix |
| --- | --- | --- |
| `q_val_x_ram` | `MAX_N*MAX_N` (2500) | `q_val_x_ram_*` |
| `q_col_x_ram` | `MAX_N*MAX_N` (2500) | `q_col_x_ram_*` |
| `q_rowp_x_ram` | `MAX_N+1` (51) | `q_rowp_x_ram_*` |
| `q_val_y_ram` | `MAX_N*MAX_N` (2500) | `q_val_y_ram_*` |
| `q_col_y_ram` | `MAX_N*MAX_N` (2500) | `q_col_y_ram_*` |
| `q_rowp_y_ram` | `MAX_N+1` (51) | `q_rowp_y_ram_*` |
| `cx_ram` | `MAX_N` (50) | `cx_ram_*` |
| `cy_ram` | `MAX_N` (50) | `cy_ram_*` |
| `x_ram` | `MAX_N` (50) | `x_ram_*` |
| `y_ram` | `MAX_N` (50) | `y_ram_*` |

Avalon-MM / AXI requires natural alignment, so the four 16 KB Q
slaves must sit at 16 KB-aligned bases. Suggested h2f layout (refresh
from Quartus's address report after Qsys regenerate):

```
q_val_x_ram  -> 0xC0000000  (16 KB)
q_col_x_ram  -> 0xC0004000  (16 KB)
q_val_y_ram  -> 0xC0008000  (16 KB)
q_col_y_ram  -> 0xC000C000  (16 KB)
q_rowp_x_ram -> 0xC0010000  ( 4 KB mmap;  256 B Qsys)
q_rowp_y_ram -> 0xC0011000  ( 4 KB mmap;  256 B Qsys)
cx_ram       -> 0xC0012000
cy_ram       -> 0xC0013000
x_ram        -> 0xC0014000
y_ram        -> 0xC0015000
```

Update the bridge bases in
[`../../sw/cg_fpga_mmap_driver_v6.h`](../../sw/cg_fpga_mmap_driver_v6.h)
to match.

## ARM driver protocol

External interface is identical to v5: pulse `sw_go`, wait for
`sw_done`, pulse `sw_done_ack`, wait for `sw_done` to clear. The only
software-side change is that `load_Q()` writes Q into both physical
M10K trios -- see
[`cg_verilator_driver_v6.h`](../../../sw-baseline-c/cg_verilator_driver_v6.h)
and
[`cg_fpga_mmap_driver_v6.h`](../../sw/cg_fpga_mmap_driver_v6.h).

## Scaling beyond the bitstream defaults (verilated only)

The FPGA bitstream is locked to `p_int_bits=13`, `p_frac_bits=14`,
`p_max_n=50`. The verilator path is fully parameterized over those and
is the only way to explore wider Q-format / larger N for v6. Pass the
knobs straight to `run-placer` (see the [top-level
README](../../../README.md#quick-start) for the full story):

```bash
uv run run-placer verilated ../benchmarks/iccad04/DMA \
    --max-n 12000 --int-bits 44 --frac-bits 20 --max-iter 30
```

What this exercised in v6 and the changes that landed for it:

- **RTL** -- already parameterized via `p_int_bits`, `p_frac_bits`,
  `p_max_n` end-to-end (CGTop -> CGEngine -> CGCtrl/CGDpath ->
  LinAlg/FpMath). Verilator re-elaborates with `-Gp_int_bits=...`
  `-Gp_frac_bits=...` `-Gp_max_n=...`, no RTL edits needed.
- **FpDiv width split** -- [FpMath.v](FpMath.v) already had a
  `generate` that picks `FpDivNR` when `p_total_bits <= 27` and
  `FpDivSS` (parameterized shift-subtract) otherwise. NR is hard-wired
  to 48-bit / Q1.16 internals so it can't be stretched past 27;
  `FpDivSS` covers the wider regime correctly at the cost of
  `(p_wide_bits + p_frac_bits)` iteration latency per divide.
- **Verilator driver mirror** --
  [`cg_verilator_driver_v6.h`](../../../sw-baseline-c/cg_verilator_driver_v6.h)
  used to declare its ten behavioral memories as fixed-size C arrays
  inline in `CGHwDriver`, which made the whole class
  `~MAX_N*MAX_N` words on the stack. Fine at `MAX_N=50` (~30 KB) but
  ~3 GB at `MAX_N=12000`, which segfaults `Placer placer(argv[1])` in
  `main()` before the first printf. The mirrors are now
  `std::vector`s, sized in the constructor's member-init list (heap).
- **Solve timeout** -- the driver's `while (!sw_done)` polling cap
  was 1,000,000 sim cycles, sized for `parallel_chains_50`. At
  `MAX_N=12000` a single solve can need many millions of cycles
  (each SPMV is `ceil(n/p_lanes)` handshakes per CG iter, and
  `CG_MAX_ITER` is 1000); the cap is now 100,000,000.

The FPGA mmap driver
([`cg_fpga_mmap_driver_v6.h`](../../sw/cg_fpga_mmap_driver_v6.h)) is
unchanged -- it writes through the h2f bridge instead of mirroring,
so it has no `MAX_N`-quadratic storage to grow.

**Bit-budget caveat.** `FpMul` (used in AXPY for `alpha*d[i]` and
`beta*d[i]`) wraps on overflow, it does not saturate. `int_bits` must
be wide enough that the largest intermediate fits in
`2^(int_bits-1)` real units; for ICCAD04-scale positions plus the
exponentially-growing partition `alpha`, 28 integer bits silently
wraps and the placement piles cells at the die corners. 44 bits is
the sweet spot for 30-iter ICCAD04 runs.

## Verification

Bit-exact against the DPI golden CG in `fpga/hw/test/`:

```
mkdir -p build-tb && cd build-tb && cmake ../fpga/hw/test && make
./VCGTop_tb_v6   # must print "ALL 16 TESTS PASSED"
```

End-to-end against the Verilator placer:

```
uv run run-placer verilated v6 ../benchmarks/custom/parallel_chains_50
```

The DPI golden solves x then y serially; v6 runs them in parallel but
must produce the same final x and y vectors -- and does (16/16 cases
pass bit-exact).

## Future work

- **Deeper SPMV pipeline (toward 1 cycle/nz)** -- orthogonal to v6
  and already prototyped in [v5_deep](../v5_deep/). Could land on top
  of v6 by forking the v5_deep SPMV into the v6 CGEngine.
- **Concurrent kernel firing within a single engine** -- overlap
  VecDot/AXPY with SPMV when dependencies allow. Orthogonal to v6.
- **Bank x/y across `p_lanes` M10Ks** -- only relevant when problem
  sizes push x/y out of the central RF. Not triggered at `p_max_n=50`.
