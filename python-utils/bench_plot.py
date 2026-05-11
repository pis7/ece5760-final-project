"""Plot per-benchmark grouped bars (ARM vs FPGA) from bench-avg.csv,
one PNG per metric column.

Reads the CSV produced by bench-avg (one row per (benchmark, mode)
with averaged metrics) and writes one PNG per metric in METRICS into
the output directory. Each PNG is a grouped bar chart -- ARM (Cornell
Navy) on the left, FPGA (Cornell Carnelian) on the right, with each
bar's numeric value annotated on top. Missing values (e.g. arm rows
have no hw_cg_* cycle counts) are skipped: no bar, no label.

Usage (typically from the same build/ directory bench-avg ran in,
since bench-avg.csv defaults to cwd). PNGs land in ./plots/ by default
so they sit in their own subdir alongside the per-mode build subdirs:
  uv run bench-plot [--csv bench-avg.csv] [--out-dir plots]
"""

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# --- Cornell palette --------------------------------------------------------


# brand.cornell.edu/design-center/colors -- carnelian is the primary brand
# color; the rest are secondary/accent.
_CORNELL = {
    "carnelian":      "#B31B1B",
    "dark_gray":      "#222222",
    "light_gray":     "#F7F7F7",
    "dark_warm_gray": "#A2998B",
    "sea_gray":       "#9FAD9F",
    "link_blue":      "#006699",
    "navy":           "#073949",
    "green":          "#6EB43F",
    "orange":         "#F8981D",
}

# ARM (baseline) gets Navy; FPGA (the project's contribution) gets
# Carnelian -- Cornell's primary brand color -- so the accelerator
# results read first.
_MODE_COLORS = {"arm": _CORNELL["navy"], "fpga": _CORNELL["carnelian"]}
_MODE_ORDER = ("arm", "fpga")


# --- Metric registry --------------------------------------------------------


@dataclass(frozen=True)
class Metric:
    """One plottable metric -- a CSV column + chart presentation."""
    column: str        # column name in bench-avg.csv
    title: str         # plot title
    ylabel: str        # y-axis label
    value_fmt: str     # bar-label format, e.g. "{:.3f}"
    out_name: str      # output PNG filename


# One PNG per entry. Add/remove freely; columns absent from the CSV
# are skipped at plot time.
METRICS: list[Metric] = [
    Metric(
        column="iters_used_avg",
        title="Outer iterations used per benchmark",
        ylabel="Iterations (avg)",
        value_fmt="{:.1f}",
        out_name="iters-used.png",
    ),
    Metric(
        column="converged_frac",
        title="Convergence rate per benchmark",
        ylabel="Converged (fraction of runs)",
        value_fmt="{:.2f}",
        out_name="converged-frac.png",
    ),
    Metric(
        column="reverted_frac",
        title="Revert rate per benchmark",
        ylabel="Reverted (fraction of runs)",
        value_fmt="{:.2f}",
        out_name="reverted-frac.png",
    ),
    Metric(
        column="hpwl_final_avg",
        title="Final HPWL per benchmark",
        ylabel="Final HPWL",
        value_fmt="{:.0f}",
        out_name="hpwl-final.png",
    ),
    Metric(
        column="max_bin_density_final_avg",
        title="Final max bin density per benchmark",
        ylabel="Max bin density",
        value_fmt="{:.2f}",
        out_name="max-bin-density.png",
    ),
    Metric(
        column="cg_avg_ms_avg",
        title="CG inner-loop average time per benchmark",
        ylabel="CG average (ms)",
        value_fmt="{:.3f}",
        out_name="cg-avg-ms.png",
    ),
    Metric(
        column="cg_total_ms_avg",
        title="CG total time per benchmark",
        ylabel="CG total (ms)",
        value_fmt="{:.3f}",
        out_name="cg-total-ms.png",
    ),
    Metric(
        column="placer_total_ms_avg",
        title="Placer total runtime per benchmark",
        ylabel="Placer total (ms)",
        value_fmt="{:.3f}",
        out_name="placer-total-ms.png",
    ),
    Metric(
        column="hw_cg_avg_cycles_avg",
        title="HW CG average cycles per benchmark",
        ylabel="HW CG average (cycles)",
        value_fmt="{:.0f}",
        out_name="hw-cg-avg-cycles.png",
    ),
    Metric(
        column="hw_cg_total_cycles_avg",
        title="HW CG total cycles per benchmark",
        ylabel="HW CG total (cycles)",
        value_fmt="{:.0f}",
        out_name="hw-cg-total-cycles.png",
    ),
]


# --- Plotter ----------------------------------------------------------------


class BenchPlotter:
    """Loads bench-avg.csv once, emits one grouped-bar PNG per metric."""

    def __init__(self, csv_path: Path, out_dir: Path) -> None:
        self.csv_path: Path = csv_path
        self.out_dir: Path = out_dir
        self.rows: list[dict[str, str]] = []

    # -- Stages --------------------------------------------------------------

    def load_csv(self) -> None:
        if not self.csv_path.is_file():
            raise SystemExit(f"CSV not found: {self.csv_path}")
        with self.csv_path.open() as f:
            self.rows = list(csv.DictReader(f))
        if not self.rows:
            raise SystemExit(f"No rows in CSV: {self.csv_path}")

    def plot_all(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        for metric in METRICS:
            self._plot_metric(metric)

    # -- Helpers -------------------------------------------------------------

    def _plot_metric(self, metric: Metric) -> None:
        # Lazy imports keep --help / load_csv fast.
        import numpy as np
        import matplotlib.pyplot as plt

        if metric.column not in self.rows[0]:
            print(f"  Skipping {metric.out_name}: "
                  f"column '{metric.column}' absent from CSV")
            return

        benchmarks: list[str] = sorted({r["benchmark"] for r in self.rows})
        # mode -> {benchmark -> float | None}.  None encodes empty-string
        # CSV cells (e.g. arm rows have no hw_cg_* values).
        by_mode: dict[str, dict[str, Optional[float]]] = {}
        for r in self.rows:
            raw = r.get(metric.column, "")
            value: Optional[float] = (
                None if raw.strip() == "" else float(raw)
            )
            by_mode.setdefault(r["mode"], {})[r["benchmark"]] = value

        modes: list[str] = [
            mode for mode in _MODE_ORDER
            if mode in by_mode
            and any(by_mode[mode].get(b) is not None for b in benchmarks)
        ]
        if not modes:
            print(f"  Skipping {metric.out_name}: "
                  f"no data for column '{metric.column}'")
            return

        x = np.arange(len(benchmarks))
        bar_w = 0.8 / len(modes)
        fig_w = max(8.0, 1.2 * len(benchmarks))
        fig, ax = plt.subplots(figsize=(fig_w, 5.5))

        for i, mode in enumerate(modes):
            offset = (i - (len(modes) - 1) / 2.0) * bar_w
            raw_vals: list[Optional[float]] = [
                by_mode[mode].get(b) for b in benchmarks
            ]
            # NaN -> no bar drawn; "" label -> no annotation.
            heights = [np.nan if v is None else v for v in raw_vals]
            labels = [
                "" if v is None else metric.value_fmt.format(v)
                for v in raw_vals
            ]
            bars = ax.bar(
                x + offset, heights,
                width=bar_w,
                label=mode.upper(),
                color=_MODE_COLORS.get(mode),
                edgecolor="black",
                linewidth=0.5,
            )
            ax.bar_label(bars, labels=labels, padding=2, fontsize=8)

        ax.set_xticks(x)
        ax.set_xticklabels(benchmarks, rotation=25, ha="right")
        ax.set_ylabel(metric.ylabel)
        ax.set_title(metric.title)
        ax.legend(title="Mode")
        ax.margins(y=0.15)  # headroom so bar_label text isn't clipped
        ax.grid(axis="y", linestyle="--", alpha=0.5)
        fig.tight_layout()

        out_path = self.out_dir / metric.out_name
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"Wrote {out_path}")


# --- CLI --------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="bench-plot",
        description=(
            "Plot one grouped (ARM vs FPGA) bar PNG per metric column "
            "in bench-avg.csv, each bar annotated with its numeric value."
        ),
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("bench-avg.csv"),
        help="Input CSV (default ./bench-avg.csv).",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("plots"),
        help="Directory to write PNGs into (default ./plots/).",
    )
    args = parser.parse_args()

    plotter = BenchPlotter(
        csv_path=args.csv.resolve(),
        out_dir=args.out_dir.resolve(),
    )
    plotter.load_csv()
    plotter.plot_all()


if __name__ == "__main__":
    main()
