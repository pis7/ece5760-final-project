# v5_deep -- Deeper SPMV pipeline (1 cycle/nz steady state)

Forks off v5. Same on-chip RAM topology, same drivers, same testbench.
The only architectural change is the SPMV inner loop: a 4-stage pipeline
that issues one nz per cycle in steady state, replacing v5's serial
3-cycle-per-nz `S_NZ_ADDR -> S_NZ_CAPT -> S_ACC` walk.

## Pipeline structure

Each cycle in steady state:

```
Cycle T:   q_val_addr = q_col_addr = j_idx; j_idx <= j_idx + 1
Cycle T+1: M10K rdata for j_T settles on the bus (1-cycle M10K latency)
Cycle T+2: dpath latches val_p1 <= q_val_rdata, col_p1 <= q_col_rdata
           vec_rd_idx = col_p1 (combinational RF crossbar read)
Cycle T+3: dpath latches prod_p <= val_p1 * vec[col_p1]
           (FpMulWide, combinational, 27x27 -> 48-bit signed)
Cycle T+4: acc <= acc + prod_p  (gated by issue_d3 valid bit)
```

A 3-deep "issue valid" shift register `issue_d1 -> issue_d2 -> issue_d3`
tracks which pipe stages hold valid in-flight nzs. `acc` only updates
when `issue_d3 == 1`, so the pipe valids cleanly drain at row boundaries.

## Cycle accounting

Per row, with `nnz = N`:

| Phase | v5 cycles | v5_deep cycles |
|---|---|---|
| `S_ROW_INIT` | 1 | 1 |
| Inner loop (N nzs) | `3*N` | `N` |
| Drain to acc | 0 | 3 |
| `S_EMIT` | 1 | 1 |
| **Total** | `3*N + 2` | `N + 5` |

Crossover: v5_deep wins when `N + 5 < 3*N + 2`, i.e. `N > 1.5`, so
`N >= 2` rows speed up. `N = 1` rows regress by 1 cycle. `N = 0` rows
are identical (both go directly to `S_EMIT`, no drain needed).

For an `nnz`-distribution dominated by `N >= 3` (typical in placement Q
where most rows pick up several net-fanout neighbors plus a diagonal),
v5_deep should approach a **3x SPMV speedup** vs v5 in the inner-loop
limit, modulo per-row prologue and EMIT.

Row-pointer phase (`S_RP_*` states) is unchanged from v5: 4 cycles for
row 0, 2 cycles for rows 1+.

## Hazards and how this design handles them

### Last-nz tail
The 3-cycle drain in `S_DRAIN` is set by a 2-bit counter loaded with
`DRAIN_INIT = 2` on the same edge that we leave `S_ISSUE`. The counter
decrements once per `S_DRAIN` cycle, transitioning to `S_EMIT` when it
reaches 0. This guarantees `acc` has absorbed the in-flight MAC before
`ostream_msg_row_val` is sampled.

### `vec[col]` dependency
The combinational chain `col_p1 -> RF crossbar -> vec_rd_data ->
multiplier -> prod_p` is a single pipe stage (Cycle T+2 to T+3 in the
table above). With `p_max_n = 50` and `p_lanes = 16`, the RF crossbar
is `BANK_DEPTH = 4` deep -- a 4:1 mux feeding into a 16:1 mux. Combined
with the 27x27 signed multiply, this should close at the same FMAX as
v5's `S_ACC` cycle (which has the identical combinational depth: v5
does `col_reg -> RF -> vec_rd_data -> mul -> add -> acc` in one cycle).
v5_deep splits the add off into its own stage (`acc <= acc + prod_p`
in Cycle T+4), so the critical path is actually *shorter*.

### `val_reg` / `col_reg` register reuse
v5's single `val_reg`/`col_reg` pair gets overwritten every nz. In a
naive 1-cycle pipeline that re-uses them, the multiply for nz `j`
would race against the next nz's capture. v5_deep avoids this by:
- treating `val_p1` and `col_p1` as the *first pipe stage*, not as
  scratch state -- they free-run, latching whatever is on the bus
  every cycle
- using a registered `prod_p` so the multiply output is decoupled
  from the next cycle's `val_p1`/`col_p1` overwrite
- gating `acc` updates by `issue_d3`, so cycles where the pipe
  contents are not valid (drain entry, row boundaries) do not
  contaminate the accumulator.

## What is *not* changed vs v5

- All RAM topology, drivers, testbench, CGTop/CGCtrl/CGDpath, FpMath,
  FPGATop are byte-identical copies of v5.
- SPMV's port interface (`istream_*`, `ostream_*`, three M10K read
  ports, vec RF read port, `n`) is identical -- just the internal
  control + dpath rewrites.
- Bit-exact semantics: same Q13.14 fixed-point, same FpMulWide, same
  accumulation order (sum is associative-equivalent for fixed-point).

## Verification path

1. `mkdir -p build-tb && cd build-tb && cmake ../fpga/hw/test && make` --
   v5_deep builds via `add_verilator_tb(v5_deep ...)`.
2. `./VCGTop_tb_v5_deep` -- all 16 DPI golden tests must pass bit-exact
   against v5/v4.
3. `uv run run-placer verilated v5_deep ../benchmarks/custom/parallel_chains_50` --
   placer must converge with HPWL matching v5 within fixed-point noise.
4. Compare reported HW CG cycles against v5: expect a substantial
   reduction on dense designs (`parallel_chains_50`'s row densities are small, so
   the win is bounded by N=1 rows; ICCAD04 designs with denser rows
   should show closer to the 3x asymptote).

## Open items

- This change does not touch the FPGA mmap driver or Qsys system. The
  same v5 mmap driver (`fpga/sw/cg_fpga_mmap_driver_v5.h`) and Qsys
  layout work for v5_deep.
- Future work items A-D from the v5 PLAN (overlap x/y solves,
  concurrent kernel firing, banked x/y M10Ks) remain orthogonal to
  v5_deep and can land on top of it.
