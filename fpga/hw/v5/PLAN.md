# v5 — Candidate optimizations from DuBois et al. 2008 (LANL/SRC MAPStation CG)

Source paper: `background-knowledge/papers/An_Implementation_of_the_Conjugate_Gradient_Algorithm_on_FPGAs.pdf`

## Context

Looking at "An Implementation of the Conjugate Gradient Algorithm on FPGAs" (DuBois, Boorman, Connor, Poole; FCCM 2008) for ideas to fold into the existing v3/v4 placer CG core on the DE1-SoC. Goal: extract concrete, applicable optimizations — not port the paper.

The paper's setting is very different from this project:
- **Hardware**: SRC MAPStation, *two* FPGAs sharing On-Board Memory (OBM), 100 MHz, DMA-streaming from a Xeon host. We have one Cyclone V at ~50 MHz with a single-port Avalon-MM Qsys SRAM.
- **Datatype**: IEEE double precision. We use Q13.14 (27-bit) fixed-point.
- **Matrix format**: ELLPACK-ITPACK. We use CSR.
- **Workload**: PDE discretizations (FE/FD), where row densities are roughly uniform. Our Q comes from netlist clique expansion — row densities are highly variable.

So the paper's headline architecture (two FPGAs + ELLPACK + double-precision streaming + DMA) does not transfer wholesale. But three lower-level ideas do.

## What is salvageable

### 1. Bank Q's CSR arrays across multiple M10K blocks (highest leverage)

The paper's primary FPGA pseudo-code begins with `DMA stripe A => AL0, AL1` and `DMA ja => jaL` (Figure 2 of paper) — i.e. matrix values are *striped* across two parallel banks, and column indices live in a separate bank. Each SMVM cycle then reads one value and one index in parallel.

Our SPMV today serializes: it spends two SRAM cycles per nonzero in CSR (one to fetch `vals[j]`, another to fetch `col_idx[j]`), via the FSM in [../v3/LinAlg.v](../v3/LinAlg.v) (`SPMVCtrl`/`SPMVDpath`, ~lines 495-780). The FSM walks states `S_VAL_ADDR -> S_VAL_CAPT -> S_COL_ADDR -> S_COL_CAPT` per nonzero.

If `vals[]` and `col_idx[]` lived in **separate** M10K-backed banks (not in the shared single-port Avalon SRAM), SPMV could fetch both in the same cycle — roughly halving the per-nonzero cycle count. Implementation sketch:

- During load (`S_LD_*`), DMA Q's CSR into two on-chip M10K-backed buffers: `q_vals_ram` and `q_col_ram` (and a third, `q_rowptr_ram`, but row-pointer reads are at-most twice per row so this matters less).
- Replace the 4-state `vals`/`cols` fetch with a 1-state issue + 1-state capture (or a 2-cycle pipeline) reading both banks.
- Costs: M10K blocks for Q (sized by max-nnz across designs); a separate load path. Frees the Qsys SRAM for vector traffic only.

This is the same lesson the v3 pipelined-SPMV attempt was reaching toward — see the comment block around [../v3/LinAlg.v](../v3/LinAlg.v) lines 549-553, which notes a prior attempt regressed on `tiny3`. Splitting into separate read ports rather than time-multiplexing one port avoids the hazard that likely caused the regression.

**Estimated payoff**: ~2x SPMV throughput, which since SPMV is the per-iteration time-dominant operation translates to a ~1.5-1.8x overall iteration speedup.

### 2. Even/odd vector banking when vectors outgrow the flop budget

The paper's `STREAM DMA r => {reven, deven0, deven1, ...}` (Figure 2 of paper) splits each vector into separately-addressable banks so multiple elements can be read per cycle. We're already getting this effectively for free for `p_lanes`-wide SIMD because today's `x/r/p/d` register files are flop-based and trivially multi-port — see e.g. [../v4/CGDpath.v](../v4/CGDpath.v) (~lines 75-110) where two AXPY units read four ports concurrently in `S_AXPY_XR_FEED`.

This becomes relevant when problem sizes exceed what fits in flops. Cyclone V has 390 M10K blocks, but flop-based register files for 4 vectors of 10k 27-bit words = 1.08M flops, which exceeds the 41,910 ALMs on a 5CSEMA5. At ICCAD04 design scale (thousands of cells), vectors must move to M10K, and at that point M10K dual-port (one read + one write per cycle each) limits throughput to 2 elements/cycle without banking.

**Concrete**: when migrating vector storage to M10K, allocate `p_lanes` separate M10K blocks per vector, addressed by `addr / p_lanes` and selected by `addr % p_lanes`. Preserves today's SIMD throughput.

This is a *prep-now, deploy-when-needed* item. No change to v4 unless/until vectors are pushed to M10K.

### 3. Concurrent kernel firing instead of FSM-serialized phases

The paper splits CG across two FPGAs precisely to run distinct ops concurrently: primary handles `q=A*d`, `alpha_accum = d^T q`, `alpha`; secondary handles `x += alpha*d`, `r -= alpha*q`, `delta_new = r^T r`, `beta`, `d = r + beta*d`. The handshake (`send_perms`/`recv_perms`) gates shared OBM access.

We don't need two FPGAs to capture this. Today `S_VDOT_*`, `S_AXPY_*`, and `S_SPMV_*` are separate FSM phases that own the datapath one-at-a-time. The kernels themselves (VecDot, AXPY) only touch register files; SPMV only touches the SRAM bus. They have **no resource conflict**.

A v5-class change worth considering: rework `CGCtrl` so VecDot/AXPY units can fire concurrently with SPMV when dependencies are satisfied. Specifically:
- After SPMV completes `q = A*d`, kick off `alpha_accum = d^T q`. While that runs, SPMV is idle but the SRAM bus is free — the CGCtrl could already begin streaming Q for the *next* iteration's SPMV (prefetch), or do nothing.
- After `alpha` is computed, fire `AXPY_x` and `AXPY_r` concurrently (already done in v4) and *start* `delta_new = r^T r` as soon as the first lane of `r` is updated (true dataflow).

This is a non-trivial rewrite (state machine + scoreboard for in-flight scalars), and it is not the paper's contribution per se — the paper just illustrates the pattern. But the paper is useful evidence that this dataflow structure is what lets fixed silicon outperform a CPU clocked 30x higher.

## What is *not* worth taking

- **ELLPACK-ITPACK format**: their motivation is uniform row length on PDE meshes. Placement Q has highly skewed row densities (high-fanout pins create dense rows, most rows are sparse). ELLPACK would zero-pad every row to the maximum, wasting most of the storage and most of the SPMV cycles. Stay on CSR.
- **Two-FPGA partitioning** with explicit permission handshakes: only relevant on multi-FPGA platforms (SRC MAPStation has primary + secondary user logic devices over a SNAP fabric).
- **DMA-streaming the matrix from host RAM each iteration**: their A doesn't fit on-chip; ours does. We load Q once from HPS, then iterate.
- **Double precision**: our Q13.14 has been validated bit-exact against a DPI golden model on the existing test suite. No reason to pay double's area/latency.
- **Pre-loop parallel init of `r = b - Ax`, `delta_new`, `q = Ad`**: one-shot work, dwarfed by the iteration loop. Optimizing it has no measurable impact.

## Recommendation, ranked

1. **Do (1): split Q's CSR into separate on-chip banks.** Concrete, ~2x SPMV win, isolated change in `CGCtrl` + `LinAlg`. This is the single most useful idea from the paper.
2. **Defer (2)** until problem sizes force vectors into M10K. Document the banking pattern so it's a drop-in when the time comes.
3. **Consider (3) carefully** as a v5 architectural direction. Concurrent dataflow firing is real headroom but a significant rewrite, and it's the same direction multiple CG-on-FPGA papers (this one, the Rampalli HLS paper) are reaching toward.

## Critical files for any of these

- [../v3/LinAlg.v](../v3/LinAlg.v) — `SPMVCtrl`/`SPMVDpath` (where the banking change lands)
- [../v4/CGCtrl.v](../v4/CGCtrl.v) — top-level FSM (where load-time banking and concurrent firing land)
- [../FPGATop.v](../FPGATop.v) — Qsys SRAM wiring (which the banking partly bypasses)
- [../test/CGTop_tb.v](../test/CGTop_tb.v) — DPI golden testbench (regression coverage for any change)

## Verification path for option (1)

- Add new M10K-backed memories for `q_vals_ram` and `q_col_ram` in `CGDpath`.
- Repurpose the existing `S_LD_*` states to fan-out Q load writes into both new banks.
- Rewrite SPMV's per-nonzero fetch states from 4 cycles (`VAL_ADDR -> VAL_CAPT -> COL_ADDR -> COL_CAPT`) to 2 (`ADDR -> CAPT`, both banks in parallel).
- Run the v3/v4 testbenches (`./VCGTop_tb_v3`, `./VCGTop_tb_v4`) — they must remain bit-exact against DPI golden.
- Run `uv run run-placer verilated v4 ../benchmarks/custom/tiny3` and the larger ICCAD04 cases; compare wall-clock per CG iteration before/after.
