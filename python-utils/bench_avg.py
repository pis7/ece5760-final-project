"""Run the arm + fpga placer N times each over a benchmark folder and
average the metrics into a CSV.

Drives the orchestration directly via run_placer's public helpers so
cmake + make run **once per mode** (not once per sample) and the SSH
session + per-benchmark scp of the placer binary + JSON happen once
per mode as well. Each sample after that is just an ssh-invoke of
the existing on-board binary.

Artifacts for each mode are kept in their own subdirectory under cwd:
  <build>/arm/   ARM placer binary + cmake state + per-bench JSONs
  <build>/fpga/  FPGA placer binary + cmake state + per-bench JSONs
Nothing outside those subdirs is wiped, so re-running bench-avg only
rebuilds within `arm/` and `fpga/`.

For each benchmark in the provided folder, this script runs the
DE1-SoC ARM placer N times and the FPGA-accelerated placer N times
(default 5 each), parses each invocation's RunSummary, averages the
numeric metrics across the N repetitions, and writes one CSV row per
(benchmark, mode) pair.

Usage (typically from a build*/ directory):
  uv run bench-avg <benchmark-folder> [--runs N] [--out FILE]

The folder is forwarded to run_placer.discover_benchmarks; a single
benchmark dir (with lef/ and def/) or a parent of such dirs both
work. Requires BOARD/PASS in the repo-root .env.
"""

import argparse
import csv
import shutil
from contextlib import chdir
from dataclasses import dataclass, fields
from pathlib import Path
from statistics import mean
from typing import Optional

from run_placer import (
    BoardSession,
    RunSummary,
    build_spec_for,
    cmake_build,
    discover_benchmarks,
    find_repo_root,
    parse_lefdef,
    parse_run_summary,
    require_board_creds,
    step,
)


# --- Constants ---------------------------------------------------------------

DEFAULT_RUNS = 5
MODES = ("arm", "fpga")


# --- CSV row schema ----------------------------------------------------------


@dataclass
class AveragedRow:
    """One CSV row: averages over N runs of (benchmark, mode)."""
    benchmark: str
    mode: str
    runs: int
    n_cells: int
    iters_used_avg: float
    converged_frac: float
    reverted_frac: float
    hpwl_final_avg: float
    max_bin_density_final_avg: float
    cg_avg_ms_avg: float
    cg_total_ms_avg: float
    placer_total_ms_avg: float
    hw_cg_avg_cycles_avg: Optional[float]
    hw_cg_total_cycles_avg: Optional[float]


# --- The runner --------------------------------------------------------------


class BenchAverager:
    """One cmake + one SSH session per mode; N placer invocations per
    benchmark amortize that setup."""

    def __init__(self, bench_folder: Path, runs: int, out_path: Path) -> None:
        self.bench_folder: Path = bench_folder.resolve()
        self.runs: int = runs
        self.out_path: Path = out_path.resolve()
        self.build_dir: Path = Path.cwd().resolve()
        self.repo_root: Path = find_repo_root()
        # (design_name, mode) -> [RunSummary x N]
        self.collected: dict[tuple[str, str], list[RunSummary]] = {}
        self.rows: list[AveragedRow] = []

    # -- Stages --------------------------------------------------------------

    def collect(self) -> None:
        """For each mode: parse JSONs, build once, then N placer reps."""
        benches: list[Path] = discover_benchmarks(self.bench_folder)
        step(
            f"bench-avg: {len(benches)} benchmark(s), "
            f"{self.runs} reps per mode"
        )
        for b in benches:
            print(f"  - {b.name}")
        for mode in MODES:
            self._collect_mode(mode, benches)

    def average(self) -> None:
        """Reduce self.collected into one AveragedRow per (benchmark, mode)."""
        rows: list[AveragedRow] = []
        for (design, mode), runs in sorted(self.collected.items()):
            n = len(runs)
            have_cycles = all(
                s.hw_cg_total_cycles is not None
                and s.hw_cg_avg_cycles is not None
                for s in runs
            )
            rows.append(AveragedRow(
                benchmark=design,
                mode=mode,
                runs=n,
                n_cells=runs[0].n_cells,
                iters_used_avg=mean(s.iters_used for s in runs),
                converged_frac=sum(1 for s in runs if s.converged) / n,
                reverted_frac=sum(1 for s in runs if s.reverted) / n,
                hpwl_final_avg=mean(s.hpwl_final for s in runs),
                max_bin_density_final_avg=mean(
                    s.max_bin_density_final for s in runs
                ),
                cg_avg_ms_avg=mean(s.cg_avg_ms for s in runs),
                cg_total_ms_avg=mean(s.cg_total_ms for s in runs),
                placer_total_ms_avg=mean(s.placer_total_ms for s in runs),
                hw_cg_avg_cycles_avg=(
                    mean(s.hw_cg_avg_cycles for s in runs)  # type: ignore[arg-type]
                    if have_cycles else None
                ),
                hw_cg_total_cycles_avg=(
                    mean(s.hw_cg_total_cycles for s in runs)  # type: ignore[arg-type]
                    if have_cycles else None
                ),
            ))
        self.rows = rows

    def write_csv(self) -> None:
        if not self.rows:
            raise SystemExit("No averaged results to write.")
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [f.name for f in fields(AveragedRow)]
        with self.out_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows:
                writer.writerow({
                    k: ("" if v is None else v)
                    for k, v in row.__dict__.items()
                })
        print(f"\nWrote averaged metrics to {self.out_path}")

    # -- Helpers -------------------------------------------------------------

    def _collect_mode(self, mode: str, benches: list[Path]) -> None:
        """Wipe the mode's subdir, parse all JSONs, build once, run N reps."""
        step(f"############ Mode: {mode} ############")
        # Each mode gets its own subdir so arm and fpga artifacts (cmake
        # state, placer binary, per-bench JSONs) never collide. Wipe the
        # subdir at the start of each mode so each bench-avg run starts
        # from a clean cmake state -- nothing outside the subdir is touched.
        mode_dir = self.build_dir / mode
        self._reset_dir(mode_dir)
        # parse_lefdef writes <design>.json via parser.write_json (which is
        # cwd-relative), so chdir into the mode subdir to land the JSONs
        # alongside the placer binary cmake_build is about to produce.
        design_by_bench: dict[Path, tuple[str, Path]] = {}
        with chdir(mode_dir):
            for bench in benches:
                step(f"Generating JSON: {bench.name}")
                design, json_file = parse_lefdef(bench)
                design_by_bench[bench] = (design, json_file)
            spec = build_spec_for(mode, "v6", 13, 14, 50)
            step(f"Building placer for {mode} into {mode_dir.name}/ "
                 "(once for all samples)")
            cmake_build(spec, self.repo_root, mode_dir)
        self._run_reps(mode, benches, design_by_bench, mode_dir)

    @staticmethod
    def _reset_dir(d: Path) -> None:
        """Create d if missing and wipe its contents. Mirrors the spirit of
        run_placer.clean_build_dir but scoped to a single subdirectory."""
        d.mkdir(parents=True, exist_ok=True)
        for entry in d.iterdir():
            if entry.is_symlink() or not entry.is_dir():
                entry.unlink()
            else:
                shutil.rmtree(entry)

    def _run_reps(
        self,
        mode: str,
        benches: list[Path],
        design_by_bench: dict[Path, tuple[str, Path]],
        mode_dir: Path,
    ) -> None:
        """Open one SSH session, scp binary + JSON per benchmark once, then
        ssh-invoke the placer N times per benchmark over the same session.
        After the last rep, scp the placer's <design>-initial.json and
        <design>-final.json back into mode_dir, and render the final PNG."""
        creds = require_board_creds(self.repo_root)
        placer_path = mode_dir / "placer"
        with BoardSession(creds=creds, board_dir="/home/root") as board:
            # One-time per-benchmark setup on the board.
            for bench in benches:
                design, json_file = design_by_bench[bench]
                board_dir = self._board_dir(design, mode)
                board.set_board_dir(board_dir)
                step(f"Copying to board ({board_dir})")
                board.put(placer_path)
                board.put(json_file)
            # N reps, all benchmarks per rep, one SSH session, no rebuilds.
            for rep in range(1, self.runs + 1):
                step(f"Mode {mode}: rep {rep}/{self.runs}")
                is_last_rep = rep == self.runs
                for bench in benches:
                    design, json_file = design_by_bench[bench]
                    board.set_board_dir(self._board_dir(design, mode))
                    output = board.run(
                        f"./placer {json_file.name}", capture=True
                    )
                    summary = parse_run_summary(output, design)
                    print(
                        f"  {design:<25} iters={summary.iters_used:>2} "
                        f"({summary.tag:<9}) "
                        f"hpwl={summary.hpwl_final:>8.0f} "
                        f"cg_avg={summary.cg_avg_ms:>7.3f} ms"
                    )
                    self.collected.setdefault(
                        (design, mode), []
                    ).append(summary)
                    # Each rep overwrites the on-board JSONs, so pulling
                    # them back on the last rep gives one canonical copy.
                    if is_last_rep:
                        self._save_artifacts(board, design, mode_dir)

    @staticmethod
    def _save_artifacts(
        board: BoardSession, design: str, mode_dir: Path
    ) -> None:
        """scp <design>-{initial,final}.json from the board into mode_dir,
        then render <design>-final.png locally via visualizer."""
        for kind in ("initial", "final"):
            name = f"{design}-{kind}.json"
            board.get(name, mode_dir / name)
        # Lazy imports keep cold paths (parse-only) free of the visualizer
        # dep (Pillow).
        from json_utils import load_netlist
        from visualizer import render_to_png
        final_json = mode_dir / f"{design}-final.json"
        final_png = mode_dir / f"{design}-final.png"
        netlist = load_netlist(str(final_json))
        render_to_png(netlist, str(final_png))

    @staticmethod
    def _board_dir(design: str, mode: str) -> str:
        """Match run_placer.RunPlacer._board_dir: fpga gets a `-fpga`
        suffix so arm and fpga binaries don't collide on the board."""
        suffix = "-fpga" if mode == "fpga" else ""
        return f"/home/root/build-{design}{suffix}"


# --- CLI ---------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="bench-avg",
        description=(
            "Run the placer arm + fpga modes N times each over a "
            "benchmark folder, then write per-(benchmark, mode) averaged "
            "metrics to a CSV. One cmake build and one SSH session per "
            "mode -- no rebuilds between samples."
        ),
    )
    parser.add_argument(
        "benchmark_folder",
        type=Path,
        help=(
            "Single-benchmark or parent-of-benchmarks directory "
            "(forwarded to run_placer.discover_benchmarks)."
        ),
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=DEFAULT_RUNS,
        help=f"Repetitions per mode (default {DEFAULT_RUNS}).",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("bench-avg.csv"),
        help="CSV output path (default bench-avg.csv in cwd).",
    )
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be >= 1")

    runner = BenchAverager(
        bench_folder=args.benchmark_folder,
        runs=args.runs,
        out_path=args.out,
    )
    runner.collect()
    runner.average()
    runner.write_csv()


if __name__ == "__main__":
    main()
