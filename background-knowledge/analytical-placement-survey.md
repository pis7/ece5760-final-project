# Analytical Placement Survey Summary

**Source**: Chang, Jiang, and Chen. "Essential Issues in Analytical Placement
Algorithms." IPSJ Transactions on System LSI Design Methodology, 2007.

This paper provides a comprehensive survey of analytical placement — the
dominant modern approach to placing standard cells on an ASIC die. It is written
for an EDA audience but the core ideas are accessible to anyone with VLSI
background.

---

## 1. What is Placement?

Placement assigns every standard cell (gate) in a synthesized netlist to a
physical (x, y) location on the chip die, subject to two goals:

1. **Minimize wirelength** — typically measured as total half-perimeter
   wirelength (HPWL), the sum of bounding-box half-perimeters across all nets.
2. **No overlaps** — cells cannot physically occupy the same space.

Placement is NP-hard, so practical algorithms use heuristics. Three families
exist:

| Approach              | Strengths                          | Weaknesses                         |
|-----------------------|------------------------------------|------------------------------------|
| Simulated Annealing   | Flexible objectives, good on small | Slow, doesn't scale                |
| Min-Cut               | Fast, scales well, good for macros | Limited objectives, poor whitespace |
| **Analytical**        | Best quality at scale, handles whitespace, multiple objectives | Hard to legalize large macros |

Analytical placement dominates modern academic placers and is the focus of both
this survey and our project.

## 2. Three Stages of Analytical Placement

### Stage 1: Global Placement (most important)

Computes approximate cell positions by solving a mathematical optimization
problem. Allows some overlap — the goal is to get cells "roughly right."

### Stage 2: Legalization

Snaps cells to legal row sites and removes all remaining overlaps, keeping cells
as close to their global placement positions as possible. Common approach:
Tetris-like greedy algorithm (sort cells by x-coordinate, place each at the
nearest legal site).

### Stage 3: Detailed Placement

Local post-processing to improve wirelength: cell swapping (try all orderings
of a small window of adjacent cells), cell matching (bipartite assignment of
cells to slots), and global moving/swapping (relocate individual cells to better
whitespace).

**Our project focuses on global placement only** — legalization and detailed
placement are out of scope.

---

## 3. Global Placement: Four Key Ingredients

### 3.1 Wirelength Models

HPWL is the gold-standard metric but is **not differentiable**, so a smooth
approximation is needed for gradient-based optimization. Five models are
surveyed:

#### Quadratic Model (simplest — used in our project)
Approximates wirelength as the sum of squared distances between connected pin
pairs:

```
W = (1/2) * sum over all nets of sum over pin pairs (i,j):
        w_ij * (x_i - x_j)^2
```

Because this only handles 2-pin connections, multi-pin nets must be decomposed:

- **Clique model**: Create a weighted edge between every pair of pins in a net.
  For a net with P pins, this creates P(P-1)/2 edges. Weights are scaled by
  1/P to approximate HPWL.
- **Star model**: Introduce a virtual "star pin" per net; connect every real pin
  to it. Equivalent to the clique model when weights are scaled by 1/P.

The quadratic model produces a convex quadratic objective that can be written in
matrix form:

```
min (1/2) x^T Q x + c^T x
```

where Q is the **connectivity matrix** (sparse, symmetric, positive-definite)
and c captures connections to fixed cells (I/O pads). The unique minimum is
found by solving the linear system **Qx = -c**. This is the system our FPGA
accelerates via conjugate gradient.

Gordian-L improves the quadratic model by adjusting weights iteratively:
`w_ij = (4/P^2) * 1/|x_i - x_j|`, which linearizes the quadratic distance to
better approximate HPWL.

#### Bound2Bound Model (better HPWL accuracy)
Removes all connections between "inner" pins (non-boundary), keeping only
connections to boundary (min/max coordinate) pins. With proper weights, the
quadratic cost **exactly equals HPWL**. Used by Kraftwerk2. Still results in a
Qx = -c system.

#### Log-Sum-Exponential (LSE) Model
Smoothly approximates max/min functions using:
```
LSE_e = gamma * [log(sum exp(x_k/gamma)) + log(sum exp(-x_k/gamma)) + ...]
```
As gamma -> 0, LSE -> HPWL. Differentiable but **nonlinear** — requires
nonlinear optimization (not a simple linear system solve). Used by APlace,
mPL6, NTUplace3.

#### Lp-norm and CHKS Models
Other smooth HPWL approximations. Lp-norm uses `(sum x_k^p)^(1/p)` with large
p. CHKS recursively smooths two-variable max functions. Both are less common
than LSE.

### 3.2 Overlap Reduction Techniques

Without overlap reduction, minimizing wirelength alone causes all cells to
collapse to a single point. Six techniques exist:

#### Partitioning (used in our project)
The simplest approach. Given a placement:
1. Draw a geometric bisection line through the placement region
2. Assign cells to left/right (or top/bottom) sub-regions based on current
   positions
3. Optionally refine the partition (e.g., Fiduccia-Mattheyses)
4. Recurse: subdivide each sub-region further

This progressively constrains cell movement to smaller regions, spreading them
out. A more sophisticated variant uses a **transportation problem** to assign
cells to sub-regions while respecting capacity constraints and minimizing
displacement.

#### Cell Shifting
Divide the chip into bins. For each row of bins, compute utilization per bin,
then adjust bin boundaries so over-utilized bins shrink and under-utilized bins
grow. Linearly remap cell positions from old to new bin boundaries. Simple and
fast.

#### Min-Cost Flow Assignment
Cluster nearby cells, partition the chip into uniform sub-regions (one per
cluster), then solve a min-cost flow / bipartite matching to assign clusters to
sub-regions minimizing HPWL degradation.

#### Diffusion
Model cell density as material concentration and simulate physical diffusion
(heat equation). Cells move from high-density regions to low-density regions
following the density gradient. Discretized on a bin grid using FTCS (forward
time, centered space).

#### Density Control (most popular)
Divide chip into bins and compute a smooth density function D_b(x,y) measuring
how much cell area occupies each bin. Add a constraint D_b <= M_b (max allowed
density per bin). The density function must be smoothed to be differentiable:

- **Bell-shaped smoothing** (APlace, NTUplace3): Smooth overlap function with a
  piecewise quadratic bell curve
- **Helmholtz smoothing** (mPL6): Solve the Helmholtz equation to smooth density
- **Poisson smoothing** (Kraftwerk2, FDP, mFAR): Treat density as electrostatic
  potential, solve Poisson equation

#### Frequency Control
Transform the density matrix to the frequency domain via DCT. Define a
distribution cost that penalizes uneven frequencies. This cost is a convex
quadratic function of cell positions.

### 3.3 Integrating Wirelength and Overlap Reduction

The wirelength objective (pull cells together) directly contradicts overlap
reduction (push cells apart). Three integration methods exist:

#### Fixed Point Method (used in our project)
After overlap reduction produces target positions ("anchor points") for each
cell, add pseudo-connections (springs) from each cell to its anchor. The
modified objective becomes:

```
min (1/2) x^T Q x + c^T x + sum_i alpha_i * (x_i - x_hat_i)^2
```

This simply modifies Q (add alpha_i to diagonal) and c, so the problem remains
a **single sparse linear system solve** — the same FPGA hardware is reused each
iteration with updated data. The process repeats: solve -> spread -> add
anchors -> solve again, until overlap is acceptable.

#### Penalty Method
Add a density penalty term directly to the objective:

```
min W(x,y) + lambda * sum_b (D_b(x,y) - M_b)^2
```

Lambda is increased over iterations to progressively enforce spreading. Requires
nonlinear optimization (conjugate gradient on the nonlinear objective). Used by
APlace, NTUplace3.

#### Region Constraint Method
For partitioning-based overlap reduction: add constraints (or use net splitting)
to keep cells within their assigned sub-regions during re-optimization.

### 3.4 Optimization Techniques

Two categories based on the mathematical formulation:

#### Quadratic Programming
When using the quadratic wirelength model + fixed point or partitioning, the
problem is always quadratic: solve **Qx = -c** (sparse, symmetric,
positive-definite). Solved via conjugate gradient. This is what we implement.

Placers: BonnPlace, DPlace, FastPlace, FDP, Gordian, Kraftwerk2, mFAR, RQL,
UPlace.

#### Nonlinear Programming
When using LSE wirelength + density penalty, the problem is nonlinear. Solved
via conjugate gradient with line search on the nonlinear objective. Often uses a
multilevel (coarsening/uncoarsening) framework for scalability.

Placers: APlace, mPL6, NTUplace3, Vaastu.

---

## 4. Placer Comparison Table

| Placer      | Wirelength   | Overlap Reduction      | Integration      | Optimization |
|-------------|-------------|------------------------|------------------|-------------|
| APlace      | LSE         | Density (Bell-Shaped)  | Penalty          | Nonlinear   |
| BonnPlace   | Quadratic   | Partitioning           | Region Constraint| Quadratic   |
| DPlace      | Quadratic   | Diffusion              | Fixed Point      | Quadratic   |
| FastPlace   | Quadratic   | Cell Shifting          | Fixed Point      | Quadratic   |
| FDP         | Quadratic   | Density (Poisson)      | Fixed Point      | Quadratic   |
| Gordian     | Quadratic   | Partitioning           | Region Constraint| Quadratic   |
| Kraftwerk2  | Bound2Bound | Density (Poisson)      | Fixed Point      | Quadratic   |
| mFAR        | Quadratic   | Density (Poisson)      | Fixed Point      | Quadratic   |
| mPL6        | LSE         | Density (Helmholtz)    | Penalty          | Nonlinear   |
| NTUplace3   | LSE         | Density (Bell-Shaped)  | Penalty          | Nonlinear   |
| RQL         | Quadratic   | Cell Shifting          | Fixed Point      | Quadratic   |
| UPlace      | Quadratic   | Frequency              | Penalty          | Quadratic   |

## 5. Example Placers in Detail

### NTUplace3 (Nonlinear)
1. Multilevel coarsening: cluster cells with first-choice clustering until
   <6000 blocks
2. Initial placement: minimize quadratic wirelength via CG
3. At each level, solve: `min W_LSE(x,y) + lambda * sum (D_b - M_b)^2`
   - lambda initialized from ratio of wirelength/density gradients
   - lambda doubled each iteration until cells are spread enough
4. Decluster and repeat at finer levels
5. Legalize with extended Tetris method (prioritize large blocks)
6. Detailed placement: cell swapping + cell matching

### Kraftwerk2 (Quadratic)
1. Initial placement: minimize Bound2Bound wirelength (a few CG iterations)
2. Global placement loop (until overlap < 20%):
   - Compute demand/supply system and Poisson-smoothed density potential
   - Apply Bound2Bound weights
   - For each direction: build system `(Q + Q_dot) * delta_x = -Q_dot * D_hat_x`
     - Q = connectivity, Q_dot = spreading force weights, D_hat = density gradient
   - Solve and update positions
   - Quality control: adjust spreading force weights
3. Tetris legalization
4. Cell flipping/swapping

---

## 6. Relevance to Our Project

Our implementation maps most closely to the **quadratic programming** family
(like Gordian, FastPlace, Kraftwerk2):

- **Wirelength model**: Quadratic with clique net decomposition
- **Overlap reduction**: Partitioning (recursive bisection)
- **Integration**: Fixed point method (anchor springs)
- **Optimization**: Conjugate gradient solve of Qx = -c

The CG solve is the computational bottleneck that we offload to the FPGA. The
key operations (sparse matrix-vector multiply, dot products, vector add/scale)
are highly parallel and pipeline-friendly.

Potential improvements suggested by this survey:
- Switch from clique to **Bound2Bound** net model for exact HPWL matching
  (still results in same Qx = -c system, just different weights)
- Use **Gordian-L weights** (w = 4/P^2 * 1/|x_i - x_j|) to iteratively
  linearize the quadratic model toward HPWL
- Use **density-based** overlap reduction (Poisson smoothing) instead of
  partitioning for smoother convergence
