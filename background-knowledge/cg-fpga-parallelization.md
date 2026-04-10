# CG Solver FPGA Parallelization Opportunities

Analysis of parallelization opportunities for the conjugate gradient (CG)
solver targeting the DE1-SoC Cyclone V FPGA (112 DSP blocks, 390 M10K SRAM
blocks / ~3.4 Mb total).

## CG Iteration Structure

Each CG iteration has the following dependency chain:

```
spmv(Q, d) -> dot(d, q) -> alpha -> { x += alpha*d, r -= alpha*q } -> dot(r, r) -> beta -> d = r + beta*d
```

## Per-Operation Parallelism

### 1. SpMV (Sparse Matrix-Vector Multiply) -- Biggest Win

- Each output element `q[i] = sum(Q[i,j] * d[j])` is independent across rows.
- Multiple rows can be processed in parallel with separate MAC units.
- With CSR format, each row's non-zeros stream through a MAC pipeline.
- **Bottleneck**: M10K bandwidth (storing the sparse matrix) and DSP block count.

### 2. Dot Products -- Moderate Win

- Partition the vector into P chunks, compute partial sums in parallel, then
  reduce with an adder tree.
- Classic tree reduction: log2(P) cycles for the final sum.
- Two dot products per iteration (`dq` and `rr_new`).

### 3. Vector Updates (AXPY: `x += alpha*d`, `r -= alpha*q`, `d = r + beta*d`) -- Easy Win

- All element-wise and fully parallel up to however many ALUs are instantiated.
- The updates to `x` and `r` use different source vectors and can be fused into
  one pass (executed simultaneously).

## Cross-Operation Parallelism

### 4. Fusing Independent Updates

- `x[i] += alpha * d[i]` and `r[i] -= alpha * q[i]` are independent of each
  other -- execute them simultaneously.
- The `d` update depends on `beta`, which depends on `rr_new`, creating a hard
  serial dependency.

### 5. Pipelining SpMV

- The SpMV of iteration k+1 uses the updated `d`, which depends on `beta`, so
  it cannot start early.
- However, the SpMV itself can be pipelined: start emitting row results while
  later rows are still computing.

## Practical FPGA Mapping

| Operation   | Strategy                                  | Resource Bottleneck           |
|-------------|-------------------------------------------|-------------------------------|
| SpMV        | P parallel row MACs, streaming from M10K  | DSP blocks, memory ports      |
| Dot product | P parallel multiplies + adder tree        | DSP blocks                    |
| AXPY        | P parallel multiply-adds                  | Fabric LUTs (fixed-pt) / DSPs |

## Key Constraints

- **DSP blocks (112)**: With fixed-point MAC, ~16-32 parallel MAC units can be
  instantiated (each SpMV MAC needs a multiplier + accumulator).
- **M10K blocks (390)**: Dual-porting limits read bandwidth. Banked storage can
  help feed multiple parallel row processors.
- **Memory capacity (~3.4 Mb)**: Sufficient for sparse matrices of moderately
  sized netlists, but large benchmarks may require HPS-side partitioning.

## Recommendation

The **SpMV dominates runtime** (O(nnz) work, called twice per iteration). Focus
parallelism there first -- multiple row processors reading from banked M10K
storage. The dot products and AXPYs are O(n) and can share the same parallel
datapath.
