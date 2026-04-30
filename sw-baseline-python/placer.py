"""Analytical global placer using quadratic wirelength and recursive bisection.

Algorithm (see docs-local/ece5760-final-project-diagram.png):
  HPS outer loop:
    1. Build connectivity matrix Q and support vectors c
    2. Solve Qx = -c via conjugate gradient
    3. Partition design, update Q and anchor points
    4. Check overlap -- if not acceptable, go to 2
  CG inner loop (step 2):
    1. Sparse matrix-vector multiply (Q * d)
    2. Compute step size alpha (dot products)
    3. Update cell positions x and residual r
    4. Update search direction d
    5. Check convergence
"""

import os
import sys

import numpy as np
from scipy.sparse import coo_matrix, csr_array, diags
from scipy.sparse.linalg import cg

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../python-utils"))
from json_utils import Netlist, dump_netlist, load_netlist


# -- Placer constants ----------------------------------------------------------

# Skip nets with more pins than this (large clock/power nets)
MAX_NET_DEGREE: int = 100

# Number of bins per dimension for overlap density check
DENSITY_BINS: int = 30

# Target max bin density for convergence
TARGET_DENSITY: float = 0.75

# Maximum outer iterations (partition + re-solve cycles)
MAX_OUTER_ITER: int = 15


class Placer:
    """Analytical global placement executor.

    All intermediate data is stored as instance variables so each step
    can reference results from previous steps.
    """

    def __init__(self, netlist_json: str) -> None:
        self.netlist: Netlist = load_netlist(netlist_json)

        # Cell center positions (database units)
        self.x_pos: np.ndarray = np.array([])
        self.y_pos: np.ndarray = np.array([])

        # Base linear system (from clique decomposition)
        self.Q_base: csr_array | None = None
        self.c_base_x: np.ndarray = np.array([])
        self.c_base_y: np.ndarray = np.array([])

        # Active linear system (base + anchor springs)
        self.Q: csr_array | None = None
        self.c_x: np.ndarray = np.array([])
        self.c_y: np.ndarray = np.array([])

        # Partition state
        self.partition_level: int = 0

    def init_cell_positions(self, method: str) -> None:
        """Initialize cell positions within the die area."""
        die_x1, die_y1, die_x2, die_y2 = self.netlist.die_area
        die_cx = (die_x1 + die_x2) / 2.0
        die_cy = (die_y1 + die_y2) / 2.0
        rng = np.random.default_rng(seed=42)
        nl = self.netlist
        for i, name in enumerate(nl.cell_names):
            comp = nl.components[name]
            if method == "random":
                comp.x = rng.uniform(die_x1, die_x2)
                comp.y = rng.uniform(die_y1, die_y2)
            elif method == "origin":
                comp.x = 0
                comp.y = 0
            elif method == "center":
                comp.x = die_cx - nl.cell_widths[i]  / 2.0
                comp.y = die_cy - nl.cell_heights[i] / 2.0

    # -- Build connectivity matrix and support vectors -------------------------

    def build_system(self) -> None:
        """Build sparse connectivity matrix Q and RHS vectors c.

        Uses clique net decomposition: each multi-pin net is decomposed into
        weighted two-pin connections between all pin pairs. The weight for a
        net with P pins is 2/P per edge.

        The resulting system Qx = -c_x (and Qy = -c_y) gives the
        wirelength-optimal cell positions when solved.
        """
        nl = self.netlist
        n = nl.num_cells

        # Initial positions (cell centers from component corners + half-size)
        self.x_pos = np.array([
            nl.components[name].x + nl.cell_widths[i] / 2
            for i, name in enumerate(nl.cell_names)
        ])
        self.y_pos = np.array([
            nl.components[name].y + nl.cell_heights[i] / 2
            for i, name in enumerate(nl.cell_names)
        ])

        # Build Q and c via clique decomposition (COO triplets)
        q_rows: list[int] = []
        q_cols: list[int] = []
        q_vals: list[float] = []
        self.c_base_x = np.zeros(n)
        self.c_base_y = np.zeros(n)

        nets_used = 0
        for net in nl.nets:
            movable: list[int] = []
            fixed_x: list[float] = []
            fixed_y: list[float] = []
            seen: set[str] = set()

            # Find all movable macros and fixed pins
            for comp_name, pin_name in net.pins:
                if comp_name == "PIN":
                    if pin_name not in seen:
                        seen.add(pin_name)
                        io_pin = nl.io_pin_map.get(pin_name)
                        if io_pin is not None:
                            fixed_x.append(io_pin.x)
                            fixed_y.append(io_pin.y)
                else:
                    if comp_name not in seen:
                        seen.add(comp_name)
                        idx = nl.cell_index.get(comp_name)
                        if idx is not None:
                            movable.append(idx)

            # Skip nets of < 1 pin or are too large
            p = len(movable) + len(fixed_x)
            if p < 2 or p > MAX_NET_DEGREE:
                continue
            nets_used += 1

            # Simple weight model: each net contributes a total weight of 2.0,
            # split evenly across all edges in the clique (P*(P-1)/2 edges for P
            # pins)
            w = 2.0 / p

            # Movable-movable clique edges
            for a in range(len(movable)):
                for b in range(a + 1, len(movable)):
                    i, j = movable[a], movable[b]
                    q_rows.extend([i, j, i, j])
                    q_cols.extend([i, j, j, i])
                    q_vals.extend([w, w, -w, -w])

            # Movable-fixed edges
            for idx in movable:
                for k in range(len(fixed_x)):
                    q_rows.append(idx)
                    q_cols.append(idx)
                    q_vals.append(w)
                    self.c_base_x[idx] -= w * fixed_x[k]
                    self.c_base_y[idx] -= w * fixed_y[k]

        # Calculate sparse Q matrix in CSR format
        self.Q_base = coo_matrix(
            (q_vals, (q_rows, q_cols)), shape=(n, n)
        ).tocsr()

        # Active system starts as base (no anchors)
        self.Q = self.Q_base.copy()
        self.c_x = self.c_base_x.copy()
        self.c_y = self.c_base_y.copy()

        self.partition_level = 0

        print(f"  {n} cells, {len(nl.io_pins)} I/O pins, "
              f"{nets_used}/{len(nl.nets)} nets")
        print(f"  Q: {self.Q_base.nnz} nonzeros")

    # -- Conjugate gradient solver ---------------------------------------------

    def solve_cg(self, max_iter: int = 1000, rtol: float = 1e-5) -> None:
        """Solve Qx = -c for both x and y coordinates via CG."""
        assert self.Q is not None
        self.x_pos, _ = cg(self.Q, -self.c_x, x0=self.x_pos,
                            rtol=rtol, maxiter=max_iter)
        self.y_pos, _ = cg(self.Q, -self.c_y, x0=self.y_pos,
                            rtol=rtol, maxiter=max_iter)
        self._clamp_to_die()

    def _clamp_to_die(self) -> None:
        """Clamp cell centers to within the die area."""
        die_x1, die_y1, die_x2, die_y2 = self.netlist.die_area
        half_w = self.netlist.cell_widths / 2
        half_h = self.netlist.cell_heights / 2
        self.x_pos = np.maximum(self.x_pos, die_x1 + half_w)
        self.x_pos = np.minimum(self.x_pos, die_x2 - half_w)
        self.y_pos = np.maximum(self.y_pos, die_y1 + half_h)
        self.y_pos = np.minimum(self.y_pos, die_y2 - half_h)

    # -- Partition and update anchors ------------------------------------------

    def partition_and_anchor(self) -> None:
        """Recursive geometric bisection to generate anchor points.

        Each call adds one more level of bisection. Anchor springs are
        added to Q and c to pull cells toward the centers of their
        assigned sub-regions (fixed point integration method).
        """
        assert self.Q_base is not None
        self.partition_level += 1
        n = self.netlist.num_cells

        die_x1, die_y1, die_x2, die_y2 = self.netlist.die_area

        # Build partition tree: partition_level rounds of bisection
        regions: list[tuple[float, float, float, float, list[int]]] = [
            (die_x1, die_y1, die_x2, die_y2, list(range(n)))
        ]

        # Perform bisection on each region at the current level, alternating x/y
        # cuts
        for level in range(self.partition_level):
            new_regions: list[tuple[float, float, float, float, list[int]]] = []

            # Alternate cut direction: even levels cut vertically (x), odd
            # levels cut horizontally (y)
            cut_x = (level % 2 == 0)

            for rx1, ry1, rx2, ry2, indices in regions:
                if len(indices) <= 1:
                    new_regions.append((rx1, ry1, rx2, ry2, indices))
                    continue

                # Sort cells by position and split at median area
                pos = self.x_pos if cut_x else self.y_pos
                indices.sort(key=lambda i: pos[i])
                areas = self.netlist.cell_areas
                half_area = sum(areas[i] for i in indices) / 2
                cumulative = 0.0
                mid = len(indices) // 2  # fallback
                for k, i in enumerate(indices):
                    cumulative += areas[i]
                    if cumulative >= half_area:
                        mid = max(1, k + 1)
                        break
                left = indices[:mid]
                right = indices[mid:]

                if cut_x:
                    mx = (rx1 + rx2) / 2
                    new_regions.append((rx1, ry1, mx, ry2, left))
                    new_regions.append((mx, ry1, rx2, ry2, right))
                else:
                    my = (ry1 + ry2) / 2
                    new_regions.append((rx1, ry1, rx2, my, left))
                    new_regions.append((rx1, my, rx2, ry2, right))

            regions = new_regions

        # Compute anchor points (region centers)
        anchors_x = np.zeros(n)
        anchors_y = np.zeros(n)
        for rx1, ry1, rx2, ry2, indices in regions:
            cx = (rx1 + rx2) / 2
            cy = (ry1 + ry2) / 2
            for i in indices:
                anchors_x[i] = cx
                anchors_y[i] = cy

        # Anchor weight scales exponentially with partition level
        diag = self.Q_base.diagonal()
        pos_diag = diag[diag > 0]
        avg_diag = float(pos_diag.mean()) if len(pos_diag) > 0 else 1.0
        alpha = avg_diag * 0.1 * (2.0 ** (self.partition_level - 1))

        # Update active system: Q = Q_base + alpha*I, c = c_base - alpha*anchor
        anchor_diag = diags(
            [np.full(n, alpha)], offsets=[0], shape=(n, n),
            format="csr",
        )
        self.Q = self.Q_base + anchor_diag
        self.c_x = self.c_base_x - alpha * anchors_x
        self.c_y = self.c_base_y - alpha * anchors_y

        print(f"  Partition level {self.partition_level}: "
              f"{len(regions)} regions, alpha={alpha:.2f}")

    # -- Check overlap acceptance ----------------------------------------------

    def max_bin_density(self) -> float:
        """Compute the peak bin density of the current placement."""
        die_x1, die_y1, die_x2, die_y2 = self.netlist.die_area
        die_w = die_x2 - die_x1
        die_h = die_y2 - die_y1

        bin_w = die_w / DENSITY_BINS
        bin_h = die_h / DENSITY_BINS
        bin_area = bin_w * bin_h

        density_map = np.zeros((DENSITY_BINS, DENSITY_BINS))

        for i in range(self.netlist.num_cells):
            cx, cy = self.x_pos[i], self.y_pos[i]
            w, h = self.netlist.cell_widths[i], self.netlist.cell_heights[i]

            x1 = max(cx - w / 2, die_x1)
            y1 = max(cy - h / 2, die_y1)
            x2 = min(cx + w / 2, die_x2)
            y2 = min(cy + h / 2, die_y2)

            bx1 = max(0, int((x1 - die_x1) / bin_w))
            by1 = max(0, int((y1 - die_y1) / bin_h))
            bx2 = min(DENSITY_BINS - 1, int((x2 - die_x1) / bin_w))
            by2 = min(DENSITY_BINS - 1, int((y2 - die_y1) / bin_h))

            for bx in range(bx1, bx2 + 1):
                for by in range(by1, by2 + 1):
                    ox1 = max(x1, die_x1 + bx * bin_w)
                    oy1 = max(y1, die_y1 + by * bin_h)
                    ox2 = min(x2, die_x1 + (bx + 1) * bin_w)
                    oy2 = min(y2, die_y1 + (by + 1) * bin_h)
                    area = max(0.0, ox2 - ox1) * max(0.0, oy2 - oy1)
                    density_map[bx, by] += area

        density_map /= bin_area
        return float(density_map.max())

    def check_overlap(self, target_density: float = 1.5) -> bool:
        """Check if placement density is acceptable using bin-based metric."""
        md = self.max_bin_density()
        print(f"  Max bin density: {md:.2f}")
        return md <= target_density

    # -- Metrics ---------------------------------------------------------------

    def compute_hpwl(self) -> float:
        """Compute total half-perimeter wirelength across all nets."""
        total = 0.0
        for net in self.netlist.nets:
            min_x = float("inf")
            max_x = float("-inf")
            min_y = float("inf")
            max_y = float("-inf")
            count = 0

            for comp_name, pin_name in net.pins:
                if comp_name == "PIN":
                    io_pin = self.netlist.io_pin_map.get(pin_name)
                    if io_pin is not None:
                        min_x = min(min_x, io_pin.x)
                        max_x = max(max_x, io_pin.x)
                        min_y = min(min_y, io_pin.y)
                        max_y = max(max_y, io_pin.y)
                        count += 1
                else:
                    idx = self.netlist.cell_index.get(comp_name)
                    if idx is not None:
                        min_x = min(min_x, self.x_pos[idx])
                        max_x = max(max_x, self.x_pos[idx])
                        min_y = min(min_y, self.y_pos[idx])
                        max_y = max(max_y, self.y_pos[idx])
                        count += 1

            if count >= 2:
                total += (max_x - min_x) + (max_y - min_y)

        return total

    # -- Output ----------------------------------------------------------------

    def _update_components(self) -> None:
        """Write solved positions back to netlist components."""
        nl = self.netlist
        for i, name in enumerate(nl.cell_names):
            comp = nl.components[name]
            comp.x = self.x_pos[i] - nl.cell_widths[i] / 2
            comp.y = self.y_pos[i] - nl.cell_heights[i] / 2

    def write_output(self, tag: str = "") -> str:
        """Write placement results as a JSON file.

        Writes <design_name>-<tag>.json (or <design_name>.json if no tag)
        with the same schema as the input netlist JSON but with updated
        component positions. Returns the output file path.
        """
        name = self.netlist.design_name
        out_name = f"{name}-{tag}.json" if tag else f"{name}.json"
        dump_netlist(self.netlist, out_name)
        return out_name


# -- Entry point ---------------------------------------------------------------


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <netlist.json>")
        print(f"  e.g. {sys.argv[0]} DMA.json")
        sys.exit(1)

    placer = Placer(sys.argv[1])

    # Initialize cell positions and write initial JSON
    print("Initializing cell positions...")
    placer.init_cell_positions("center")
    out_path = placer.write_output("initial")
    print(f"  Initial placement written to {out_path}")

    # Step 1: Build connectivity matrix
    print("Step 1: Building connectivity matrix...")
    placer.build_system()

    # Step 2: Initial CG solve (wirelength-optimal, no spreading)
    print("Step 2: Initial CG solve...")
    placer.solve_cg()
    print(f"  HPWL: {placer.compute_hpwl():.0f}")

    # Steps 3-4: Iterative spreading via partition + re-solve
    converged = False
    prev_density = placer.max_bin_density()
    for iteration in range(1, MAX_OUTER_ITER + 1):
        print(f"Iteration {iteration}:")
        placer.partition_and_anchor()

        x_saved = placer.x_pos.copy()
        y_saved = placer.y_pos.copy()

        placer.solve_cg()
        print(f"  HPWL: {placer.compute_hpwl():.0f}")

        new_density = placer.max_bin_density()
        # Only revert once spreading has started working (density meaningfully
        # close to target). Early iterations often increase density
        # temporarily. The 1.5x factor matters for inits that start cells at
        # the die center -- their initial density can already be near 2x
        # target before any spreading happens, which would trip a looser
        # threshold and end the placer after one iteration.
        if prev_density < 1.5 * TARGET_DENSITY and new_density > prev_density:
            print(f"  Max bin density: {new_density:.2f} (worse than {prev_density:.2f}, reverting)")
            placer.x_pos = x_saved
            placer.y_pos = y_saved
            break
        print(f"  Max bin density: {new_density:.2f}")
        prev_density = new_density
        if new_density <= TARGET_DENSITY:
            print("Overlap acceptable -- placement complete.")
            converged = True
            break
    if not converged:
        print(f"Max iterations ({MAX_OUTER_ITER}) reached.")

    # Write final output
    placer._update_components()
    out_path = placer.write_output("final")
    print(f"Final placement written to {out_path}")


if __name__ == "__main__":
    main()
