# Quadratic Wirelength Placer Overview

1. Each macro/cell/block has its unique parameters defined in the LEF file (e.g.
   shape, pin names, etc.)
2. A given design instantiating many such macros is defined in DEF, this
   provides a "placement" for the design by instantiating macros at given
   locations.
3. The LEF and DEF are parsed into a unified netlist JSON file by
   `lefdef-parser.py` using the `LefDefParser` class. The placer then loads
   this JSON into a `Netlist` object via `load_netlist()` (from
   `json_utils.py`) and initializes cell positions via
   `init_cell_positions()`. The `Netlist` class provides cached derived data
   including `cell_names`, `cell_index`, `io_pin_map`, `cell_widths`,
   `cell_heights`, `cell_areas`, and `total_cell_area`.
4. Overall goal is twofold:
   - Minimize wirelength
   - Prevent overlap by spreading cells apart
5. True wirelength is represented as half-perimeter wirelength (HPWL) `W = sum
   over nets e: (max x_i - min x_i) + (max y_i - min y_i)` - but this is not
   differentiable, thus we cannot optimize for it
   - We must apply a "good-enough" approximation that is differentiable - use
     quadratic wirelength objective: `phi(x) = sum over all edges (i,j):  w_ij *
       (x_i - x_j)^2`
   - In the simple case where each net is connected to just 2 pins, 1 edge = 1
     net. But many times each net has multiple pins connected to it. Thus we use
     the clique model where we consider a net connected to P pins to have
     `P*(P-1)/2` edges (essentially consider every pin on the net to be
     connected to every other pin on the net)
   - `w_ij` is a weight set on each edge to adjust the quadratic objective to
     approximate the true HPWL. This is a heuristic, thus there are many ways to
     calculate such a weight. We simply set each edge's weight to `2/P` where P
     is the number of pins for this edge's net (prevents larger nets from
     dominating objective).
6. To solve the quadratic objective above, we convert `phi(x)` into matrix form
   (analogous for both X and Y which are independent problems). Each edge
   contributes `w * (x_i - x_j)^2 = w*x_i^2 - 2w*x_i*x_j + w*x_j^2`.
   Summing over all edges, the full objective can be written as:
   `phi(x) = (1/2) x^T Q x + c^T x + constant`
   - To optimize the objective, we want to find where the gradient of the
     objective function is 0: `d(phi)/dx = Qx + c = 0` -> `Qx = -c`
   - Finally, we can now say that our original problem of wanting to minimize
     the wirelength amounts to solving for the vector `x` of positions (for both
     X and Y) given some matrix `Q` and some vector `c` which are derived from
     the specific design
7. Now we need to build our matrix `Q` and vector `c` (this explanation is the
   concrete version of matrix conversion of the problem in the above general
   explanation). There are two edge cases: movable (cell) to movable (cell) and
   movable (cell) to fixed (block pin)
   - Movable to movable case:
     + To do this, we first we expand the original `phi(x)` objective: `w * (x_i -
       x_j)^2 = w*x_i^2 - 2w*x_i*x_j + w*x_j^2`
     + Next we take the gradient with respect to both `x_i` and `x_j`: `d/dx_i =
       2w*x_i - 2w*x_j`, `d/dx_j = -2w*x_i + 2w*x_j`
     + Thus, this edge contributes `+2w` for `Q[i][i]` and `Q[j][j]`, and `-2w`
       for `Q[i][j]` and `Q[j][i]`
     + However, the matrix form has a `1/2` factor out front (`(1/2) x^T Q x`),
       so the `2` cancels and the net contribution to `Q` is `+w` on diagonal
       and `-w` off-diagonal per edge
   - Movable to fixed case:
      + In this case, the objective for this edge becomes `w * (x_i - f)^2 =
        w*x_i^2 - 2w*f*x_i + w*f^2` where `f` is the fixed pin position
      + Next we take the gradient with respect to `x_i` which is the only
        variable here: `d/dx_i = 2w*x_i - 2w*f`
      + Thus we only have a single contribution of `+2w` to `Q[i][i]` (which
        becomes `+w` after the `1/2` factor), and the `-2w*f` term contributes
        to the `c` vector as `c[i] += -w*f` (again the 2 is absorbed by the
        `1/2` factor)
   - `Q[i][i]` represents the sum of all edge weights incident to cell `i` and
     thus the total "pull" on cell `i` from all its connections, while
     `Q[i][j]` represents the size of the "pull" between cells `i` and `j`.
   - Note: `Q` is shared for both the X and Y problem since it only encodes
     connectivity information based on number of pins per net, not per x and y.
     If we were to use a different formulation for our weight variables (such as
     Gordian-L), we would need to maintain a `Q` matrix for both x and y.
     However, we have a different `c` vector for X and Y since these encode
     fixed coordinates instead of pure weights as in the `Q` matrix.
   - Implemented with `build_system()` - create connectivity matrix `Q` and
     vectors `c_x`, `c_y`
     + Initialize all X and Y cell positions to their middle points
     + For each net:
       + Form a list for fixed I/O pins on this net and a list for movable cells
         connected to this net
       + Set the weight for this net as `2/P` where `P` is the total number of
         pins (both movable cells and fixed I/O pins) on the net
       + The connectivity matrix encodes weights for each connection between `i`
         and `j`, thus it is a sparse matrix - it would be wasteful to simply
         instantiate an `n x n` matrix as many entries will be 0
         + Instead we use a COO format (coordinate triplet) where we encode the
           matrix `Q` as three parallel lists: `rows`, `cols`, `vals` - so this maps to
           `Q[rows[k]][cols[k]] += vals[k]` for each `k` from 0 to `nnz`
           (number of nonzero entries). This is great for building `Q` because
           we simply append a new entry for every calculation we do
         + The COO matrix will contain multiple entries per `(row, col)`
           combination - we need to sum these entries. Thus we use the CSR
           (compressed sparse row) format to compress these entries. CSR is
           essentially scattering the entries into row buckets and then sorting
           by column with any duplicate `(row, col)` entries summed. CSR stores
           three parallel arrays:
           - `indptr[i]` to `indptr[i+1]` gives the range of entries for row `i`
           - `indices[k]` gives the column idx of entry `k`
           - `data[k]` is the value of entry `k`
8. The problem `Qx = -c` is now solved for both x and y positions separately
   using conjugate gradient descent.
   - We use conjugate gradient descent since it is guaranteed to converge in at
     most `n` steps for a symmetric positive-definite matrix, which is what `Q` is.
   - Additionally, it is much faster than standard gradient descent since it
     picks a new search direction that is not the same as any of the other
     directions to prevent undoing previous work, as opposed to standard
     gradient descent which just chooses the direction of steepest descent on
     each iteration.
   - In the end, we are just solving a system of linear equations (`Qx = -c`),
     and so we can use a more direct, optimal method such as CG instead of a
     more generalized method like gradient descent.
   - Implemented with `solve_cg()` - given the shared matrix `Q`, the `c`
     vectors for x and y, as well as the current x and y positions of cells,
     solve for the new x and y positions of cells using conjugate gradient
     descent by solving the equation `Qx = -c` for both x and y. Clamp any
     positions outside of the die boundary to inside the die using
     `_clamp_to_die()`.
      + This is a generic optimization problem and has nothing to do with pins,
        macros, etc - just pure numbers. The scipy implementation is used here
        for brevity, but the exact algorithm is implemented verbatim in:
        https://gregorygundersen.com/blog/2022/03/20/conjugate-gradient-descent/
        which includes the derivation.
9. The cell positions have now been optimized for wirelength, but they are
   likely all pulled in very close to each other near the center of the die
   since cell overlap has not been considered. Thus, we need to perform a
   spreading step to reduce cell overlap.
   - Partitioning step: we start with a single region which is the entire chip
     containing all cells. For each partitioning level that we perform on this
     iteration, we bisect all regions and create a new list of regions with
     twice as many regions (since we cut each previous region in half through
     bisecting it).
     + The bisection alternates between X and Y cuts (even levels cut X, odd
       levels cut Y). For a given cut direction, the cells in each region are
       sorted by their current position along that axis.
     + The split point is chosen using an **area-based median**: we walk through
       the sorted cells, accumulating their areas (from `Netlist.cell_areas`),
       and split once the cumulative area reaches 50% of the region's total
       cell area. This ensures each sub-region receives roughly equal cell area
       rather than equal cell count, which is critical for mixed-size designs
       where a few large macros would otherwise cause severe imbalance.
   - We now have a number of regions with their associated cells corresponding
     to the algorithm iteration we are currently on (`partition_level`). We now
     need to perform overlap reduction given these regions using the fixed-point
     method which directly updates the `Q` matrix and `c` vectors for the next
     iteration of the algorithm.
     + Each cell gets assigned an "anchor" which is a fixed point representing
       the location we would like the cell to be (where it should be "pulled"
       to) - which we set as the center of the region it is in based on the
       previous partitioning step.
     + We compute a scalar `alpha` to represent how strongly the cells are
       pulled to their anchors.
       + It is computed as a function directly proportional to the average value
         of the diagonal of `Q` (so that the "force" is proportional to the
         existing wirelength forces), and exponentially proportional to the
         current level of partitioning that we are performing such that it
         doubles for each level: `alpha = avg_diag * 0.1 * (2.0 ** (partition_level - 1))`
       + In this way, early iterations have small `alpha` and thus cells can
         move freely to optimize wirelength, while later iterations have a
         strong `alpha` to be pulled more closely to their final regions.
     + The term `alpha * (x_i - anchor_i)^2` is now added to the wirelength
       objective for each cell `i`. This essentially sets the "spring force"
       which pulls cell `i` to its anchor point proportional to `alpha` (the
       "spring stiffness"), and is larger for cells further from their anchor.
       + This is manifested by updating `Q` and the `c` vectors, which we do
         by taking the derivative of this additional term and setting to 0:
         `alpha * (x_i - anchor_i)^2 = alpha*x_i^2 - 2*alpha*anchor_i*x_i +
         alpha*anchor_i^2` -> derivative is `2*alpha*x_i - 2*alpha*anchor_i`.
         After the `1/2` factor, the diagonal entries of `Q` get `+alpha`
         added to them and the `c` vectors get `alpha*anchor_i` subtracted
         from them.
       + The above update corresponds to `(Q_base + alpha*I) * x = -(c_base -
         alpha*anchors)`, which can be interpreted as the system attempting to
         balance the forces from the wirelength optimization with those from the
         anchor points.
10. After building the system and doing an initial conjugate gradient solve for
    the wirelength, the `partition_and_anchor()` function followed by the
    `solve_cg()` function are called until either a maximum number of iterations
    has been reached or the total overlap score of all the cells (as computed
    trivially by `check_overlap()`) is less than a user-defined threshold. The
    partitioning level is incremented by one on each such iteration.
    + `check_overlap()` works by dividing the entire die into a fixed grid of
      bins and computing the absolute density of each bin (total cell area in
      the bin divided by bin area).
    + The algorithm finds which bins each cell overlaps and adds the exact
      overlap area to a density map for each bin. The final density map is
      normalized such that for each bin, a value of `1.0` means the bin is
      "perfectly full," whereas a value `> 1.0` means the bin is "overfull."
      The peak (maximum) density across all bins is compared against the
      user-provided threshold.
11. The final placement is written out as a netlist JSON file (via
    `dump_netlist()` from `json_utils.py`) with the updated cell positions.
    This JSON can be loaded by the visualizer (`visualizer.py`) for inspection.

## Differences from the Survey Paper

The survey (Chang, Jiang, Chen 2007) describes several analytical placers. Our
implementation most closely resembles Gordian/BonnPlace (quadratic wirelength +
partitioning + fixed point). The following differences exist between our
implementation and what the paper describes:

1. **Partition cell assignment**: The paper defines physical partitioning as
   drawing a geometric bisection line at the midpoint of a region and assigning
   cells based on which side of the line they currently sit on (from the CG
   solve). Our implementation instead sorts cells by position and splits at the
   area-based median (where cumulative cell area reaches 50%). This guarantees
   balanced area per sub-region but ignores the geometric cutline.

2. **No transportation problem**: The paper mentions a more sophisticated
   variant that uses a transportation problem to assign cells to sub-regions
   while respecting capacity constraints and minimizing displacement. We do not
   implement this. Without it, a sub-region can be assigned more cell area than
   it can physically hold, which is the primary reason the DSP_CORE benchmark
   (with two macros consuming 22% of die area) fails to converge.

3. **Static Q matrix**: Placers like Kraftwerk2 and Gordian-L recompute edge
   weights each iteration based on current cell positions (e.g. Gordian-L uses
   `w_ij = (4/P^2) * 1/|x_i - x_j|` to iteratively linearize the quadratic
   distance toward HPWL). Our `Q_base` is built once and never updated -- only
   the anchor diagonal changes. This means our wirelength approximation does not
   improve as cells spread apart.

4. **Partition tree rebuilt from scratch**: The paper's placers (Gordian,
   BonnPlace) build the partition tree incrementally -- each iteration adds one
   new level of bisection on top of the existing tree. Our implementation
   rebuilds the entire tree from the full die each iteration (re-bisecting
   through all levels). This means earlier cuts can change between iterations as
   cell positions shift, which can cause cells to flip between sub-regions and
   fight the anchor springs.

5. **No Fiduccia-Mattheyses refinement**: The paper mentions optional FM
   refinement after partitioning to improve cut quality by swapping cells across
   the cut to minimize the number of nets crossing it. We do not implement this.

6. **Convergence metric**: Kraftwerk2 terminates when total overlap falls below
   20%. We use peak bin density (maximum density across a 30x30 grid of bins)
   with a threshold of 1.5x, which is a different and more lenient metric.

7. **Alpha scaling heuristic**: The paper does not prescribe a specific formula
   for how the anchor spring stiffness scales across iterations. Our formula
   `alpha = avg_diag * 0.1 * (2.0 ** (partition_level - 1))` is a heuristic
   with tuning parameters (0.1 base multiplier, doubling rate) that may not be
   optimal for all benchmarks.
