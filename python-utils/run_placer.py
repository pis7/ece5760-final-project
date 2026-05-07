"""Run the analytical placer end-to-end, with an optional iteration sweep mode.

Console-script entry point (see pyproject.toml). Parses LEF/DEF, builds
the requested backend, runs it, and -- for arm/fpga modes -- shuttles
binaries and result JSONs to the DE1-SoC over SSH.

Usage (from a build/ directory):
  uv run run-placer <mode> [hw_version] <benchmark-path> [--sweep]

Modes:
  python                 Baseline Python placer                                (run-only)
  sw                     Full software C++ placer (double precision)           (run + sweep)
  golden                 C++ placer with fixed-point golden CG                 (run + sweep)
                         --int-bits I  Q-format int bits (default 13)
                         --frac-bits B Q-format frac bits (default 14;
                                       I + B <= 64; >27 only supported in
                                       golden / verilated modes)
                         --max-n   N   Max cell count (default 50)
  verilated [v2|v3|v4|v5|v5_deep|v6] C++ placer with Verilator RTL CG (default v6) (run + sweep)
                         --int-bits I  Q-format int bits (default 13)
                         --frac-bits B Q-format frac bits (default 14)
                         --max-n   N   Max cell count / Verilog p_max_n (default 50)
  arm                    Cross-compile SW placer, run on DE1-SoC ARM           (run + sweep)
  fpga      [v4|v5|v6]   Cross-compile FPGA-accelerated placer, run on DE1-SoC (run + sweep)
                         (default v6; selects the mmap driver to link in)

Default MAX_N is 50 (the bitstream's M10K depth). golden / verilated
modes accept --max-n to override; arm/fpga are locked. ICCAD benchmarks
are not supported on any backend; use the custom/ benchmarks instead.

Sweep mode runs the placer with max_outer_iter from 1..MAX_SWEEP_ITER,
captures the per-iter final placement, renders one PNG frame per
iteration, and stitches the frames into a looping GIF + MP4. It stops
early once the placer reports it needed fewer iterations than the cap
(with a 2-frame minimum so the slideshow always has something to
animate).

When the benchmark path is a directory of benchmarks (e.g.
`benchmarks/custom`, `benchmarks/iccad04`), every contained benchmark
is run in sequence and a cross-benchmark summary table is printed at
the end. Auto-detected from directory structure -- no extra flag.

When invoked from a directory whose name starts with `build`, the
contents are wiped before the run so each invocation starts from a
clean slate (no stale binaries or JSONs from a different mode).
"""

import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


# --- Constants ---------------------------------------------------------------

MAX_SWEEP_ITER = 16
MIN_FRAMES = 2

SLIDESHOW_FRAME_MS = 400
SLIDESHOW_HOLD_LAST_MS = 1500
SLIDESHOW_MP4_FPS = 2.5

_LOCAL_MODES = {"sw", "golden", "verilated"}
_REMOTE_MODES = {"arm", "fpga"}


# --- Project layout helpers --------------------------------------------------


def find_repo_root() -> Path:
    """Locate the project root (the dir holding pyproject.toml).

    The script is expected to be invoked from a build/ subdir of the repo,
    mirroring the workflow `mkdir -p build && cd build`.
    """
    cwd = Path.cwd().resolve()
    for candidate in [cwd, *cwd.parents]:
        if (candidate / "pyproject.toml").is_file():
            return candidate
    raise SystemExit(
        "Error: not inside the project (no pyproject.toml in cwd ancestors)"
    )


def step(msg: str) -> None:
    """Print a `=== msg ===` stage banner, preceded by a blank line for
    visual separation from the previous stage's output."""
    print(f"\n=== {msg} ===", flush=True)


def clean_build_dir() -> None:
    """If cwd looks like a build dir (basename starts with 'build') and is
    not the repo root itself, wipe its contents so each run starts clean.

    Safety: the repo-root check prevents disasters if a `build` dir is
    accidentally created at (or symlinked from) the project root.
    """
    cwd = Path.cwd().resolve()
    if not cwd.name.startswith("build"):
        return
    if cwd == find_repo_root():
        return
    entries = list(cwd.iterdir())
    if not entries:
        return
    step(f"Cleaning build dir ({cwd})")
    for entry in entries:
        if entry.is_symlink() or not entry.is_dir():
            entry.unlink()
        else:
            shutil.rmtree(entry)


# --- .env / board credentials ------------------------------------------------


@dataclass
class BoardCreds:
    host: str       # e.g. "root@10.253.17.19"
    password: str


def load_board_creds(repo_root: Path) -> Optional[BoardCreds]:
    """Read BOARD/PASS from .env (preferred) or the OS env. Return None if
    either is missing."""
    env: dict[str, Optional[str]] = {}
    env_path = repo_root / ".env"
    if env_path.is_file():
        from dotenv import dotenv_values
        env = {k: v for k, v in dotenv_values(env_path).items()}
    host = env.get("BOARD") or os.environ.get("BOARD")
    password = env.get("PASS") or os.environ.get("PASS")
    if not host or not password:
        return None
    return BoardCreds(host=host, password=password)


def require_board_creds(repo_root: Path) -> BoardCreds:
    """Same as load_board_creds, but exit with a friendly error if missing."""
    creds = load_board_creds(repo_root)
    if creds is None:
        env_path = repo_root / ".env"
        sys.exit(f"Error: BOARD and PASS must be set (e.g. via {env_path})")
    return creds


# --- LEF/DEF parsing ---------------------------------------------------------


def parse_lefdef(bench_path: Path) -> tuple[str, Path]:
    """Parse the LEF/DEF pair under bench_path, write JSON in cwd, and
    return (design_name, json_path)."""
    from lefdef_parser import LefDefParser
    parser = LefDefParser(str(bench_path))
    parser.find_files()
    parser.parse_lef()
    parser.parse_def()
    out_path = Path(parser.write_json()).resolve()
    print(
        f"Design: {parser.design_name}\n"
        f"  {len(parser.macros)} macros, {len(parser.components)} components, "
        f"{len(parser.io_pins)} I/O pins, {len(parser.nets)} nets\n"
        f"  Written to {out_path}"
    )
    return parser.design_name, out_path


def looks_like_benchmark(path: Path) -> bool:
    """True iff path has lef/ and def/ subdirs each containing at least one
    .lef / .def file. Mirrors LefDefParser.find_files semantics."""
    if not path.is_dir():
        return False
    lef_dir = path / "lef"
    def_dir = path / "def"
    if not (lef_dir.is_dir() and def_dir.is_dir()):
        return False
    return any(lef_dir.glob("*.lef")) and any(def_dir.glob("*.def"))


def discover_benchmarks(path: Path) -> list[Path]:
    """Resolve a benchmark path into a list of one or more benchmark dirs.

    If the path itself is a single benchmark, returns [path].
    Otherwise scans immediate subdirs and returns the ones that look like
    benchmarks (sorted by name). Empty result -> SystemExit.
    """
    if looks_like_benchmark(path):
        return [path]
    if not path.is_dir():
        raise SystemExit(f"No benchmarks found at {path}: not a directory")
    children = [c for c in sorted(path.iterdir()) if looks_like_benchmark(c)]
    if not children:
        raise SystemExit(
            f"No benchmarks found at {path}: neither lef/+def/ nor "
            "subdirs containing them"
        )
    return children


# --- Build orchestration -----------------------------------------------------


@dataclass
class BuildSpec:
    """Describes how a given mode is built locally."""
    source_subdir: str        # path under repo root, e.g. "sw-baseline-c"
    defines: list[str]        # extra -D flags for cmake


_SW_C = "sw-baseline-c"
_FPGA_SW = "fpga/sw"
_ARM_DEFINES = [
    "-DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++",
    "-DSTATIC_BUILD=ON",
]


def build_spec_for(
    mode: str, hw_version: str, int_bits: int, frac_bits: int, max_n: int
) -> BuildSpec:
    """Map a mode (and hw_version / int_bits / frac_bits / max_n for
    verilated|golden) to its BuildSpec.

    Defaults are int=13, frac=14 (Q13.14, total 27 -- the FPGA bitstream's
    format) and max_n=50 (the bitstream's M10K depth). Wider widths /
    larger N are only honored by `golden` and `verilated`; other modes
    either don't use fixed-point at all (sw) or are locked to the
    synthesized bitstream (fpga). The CLI parser rejects these flags
    outside golden/verilated, so by the time we get here forwarding the
    knob to non-fixed-point modes is harmless.
    """
    if mode == "sw":
        return BuildSpec(_SW_C, [])
    if mode == "golden":
        return BuildSpec(
            _SW_C,
            [
                "-DUSE_FP_GOLDEN=ON",
                f"-DINT_BITS={int_bits}",
                f"-DFRAC_BITS={frac_bits}",
                f"-DMAX_N={max_n}",
            ],
        )
    if mode == "verilated":
        return BuildSpec(
            _SW_C,
            [
                "-DUSE_HW_CG=ON",
                f"-DHW_CG_VERSION={hw_version}",
                f"-DINT_BITS={int_bits}",
                f"-DFRAC_BITS={frac_bits}",
                f"-DMAX_N={max_n}",
            ],
        )
    if mode == "arm":
        return BuildSpec(_SW_C, list(_ARM_DEFINES))
    if mode == "fpga":
        return BuildSpec(
            _FPGA_SW,
            [*_ARM_DEFINES, f"-DHW_FPGA_VERSION={hw_version}"],
        )
    raise ValueError(f"build_spec_for: unsupported mode {mode!r}")


def cmake_build(spec: BuildSpec, repo_root: Path, build_dir: Path) -> None:
    """Run `cmake <source>` then `make -j` in build_dir, streaming output."""
    source_dir = repo_root / spec.source_subdir
    subprocess.run(
        ["cmake", str(source_dir), *spec.defines],
        cwd=build_dir,
        check=True,
    )
    subprocess.run(["make", "-j"], cwd=build_dir, check=True)


# --- Local placer runs -------------------------------------------------------


def run_local_placer_tee(
    json_file: Path, build_dir: Path, max_iter: Optional[int] = None
) -> str:
    """Run ./placer JSON [max_iter], streaming output to the terminal AND
    capturing it.

    Returns the combined stdout/stderr text. Raises SystemExit on failure.
    """
    cmd: list[str] = ["./placer", str(json_file)]
    if max_iter is not None:
        cmd.append(str(max_iter))
    proc = subprocess.Popen(
        cmd,
        cwd=build_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    buf: list[str] = []
    for line in proc.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        buf.append(line)
    rc = proc.wait()
    if rc != 0:
        raise SystemExit(f"Placer failed (rc={rc})")
    return "".join(buf)


def run_local_placer_capture(
    json_file: Path, max_iter: int, build_dir: Path
) -> str:
    """Run ./placer JSON N, capturing combined stdout/stderr. Raises on failure."""
    result = subprocess.run(
        ["./placer", str(json_file), str(max_iter)],
        cwd=build_dir,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        sys.stderr.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise SystemExit(
            f"Placer failed at N={max_iter} (rc={result.returncode})"
        )
    return result.stdout + result.stderr


def run_python_placer(json_file: Path, repo_root: Path, build_dir: Path) -> None:
    """Invoke the Python baseline placer via `uv run`. Streams output."""
    placer_py = repo_root / "sw-baseline-python" / "placer.py"
    subprocess.run(
        ["uv", "run", str(placer_py), str(json_file)],
        cwd=build_dir,
        check=True,
    )


# --- Output parsing + summary ------------------------------------------------


@dataclass
class RunSummary:
    """Parsed metrics from a single placer run."""
    design_name: str = ""
    n_cells: int = 0
    iters_used: int = 0
    converged: bool = False
    reverted: bool = False
    hpwl_final: float = 0.0
    max_bin_density_final: float = 0.0
    cg_total_ms: float = 0.0
    cg_avg_ms: float = 0.0
    placer_total_ms: float = 0.0
    hw_cg_total_cycles: Optional[int] = None
    hw_cg_avg_cycles: Optional[float] = None

    @property
    def tag(self) -> str:
        if self.converged:
            return "converged"
        if self.reverted:
            return "reverted"
        return "capped"


def parse_run_summary(output: str, design_name: str) -> RunSummary:
    """Parse marker lines printed by placer.cpp main() into a RunSummary."""
    m_iters = re.search(r"^Outer iterations used:\s+(\d+)", output, re.M)
    m_conv = re.search(r"^Converged:\s+(\S+)", output, re.M)
    m_rev = re.search(r"^Reverted:\s+(\S+)", output, re.M)
    if not (m_iters and m_conv and m_rev):
        sys.stderr.write(output)
        raise SystemExit(
            "Could not find marker lines in placer output. "
            "Did placer.cpp print 'Outer iterations used: K'?"
        )
    s = RunSummary(design_name=design_name)
    s.iters_used = int(m_iters.group(1))
    s.converged = m_conv.group(1) == "true"
    s.reverted = m_rev.group(1) == "true"

    m_size = re.search(r"^\s+(\d+) cells,", output, re.M)
    if m_size:
        s.n_cells = int(m_size.group(1))

    # The placer prints HPWL for every iteration including ones it later
    # reverts; slice to keep only [initial..iter[iters_used]] so the spike
    # value isn't reported as the final.
    hpwl_floats = [float(x) for x in re.findall(
        r"^\s+HPWL:\s+([0-9.]+)", output, re.M
    )]
    if hpwl_floats:
        s.hpwl_final = hpwl_floats[: s.iters_used + 1][-1]

    m_total = re.search(r"^\s+CG total:\s+([0-9.]+) ms", output, re.M)
    m_avg = re.search(r"^\s+CG average:\s+([0-9.]+) ms", output, re.M)
    m_pl = re.search(r"^\s+Placer total:\s+([0-9.]+) ms", output, re.M)
    if m_total: s.cg_total_ms = float(m_total.group(1))
    if m_avg: s.cg_avg_ms = float(m_avg.group(1))
    if m_pl: s.placer_total_ms = float(m_pl.group(1))

    densities = re.findall(r"^\s+Max bin density:\s+([0-9.]+)", output, re.M)
    if densities:
        s.max_bin_density_final = float(densities[-1])

    m_hw_total = re.search(r"^\s+HW CG total:\s+(\d+) cycles", output, re.M)
    m_hw_avg = re.search(r"^\s+HW CG average:\s+([0-9.]+) cycles", output, re.M)
    if m_hw_total:
        s.hw_cg_total_cycles = int(m_hw_total.group(1))
    if m_hw_avg:
        s.hw_cg_avg_cycles = float(m_hw_avg.group(1))

    return s


def print_summary(s: RunSummary, *, header: Optional[str] = None) -> None:
    """Pretty-print one RunSummary as a tidy block."""
    title = header or f"Run summary: {s.design_name}"
    step(title)
    print(f"  Cells:          {s.n_cells}")
    print(f"  Iters:          {s.iters_used} ({s.tag})")
    print(f"  Final HPWL:     {s.hpwl_final:.0f}")
    print(f"  Final density:  {s.max_bin_density_final:.2f}")
    cg_avg_extra = (
        f", {s.hw_cg_avg_cycles:.0f} cycles"
        if s.hw_cg_avg_cycles is not None else ""
    )
    cg_total_extra = (
        f", {s.hw_cg_total_cycles} cycles"
        if s.hw_cg_total_cycles is not None else ""
    )
    print(f"  CG average:     {s.cg_avg_ms:.3f} ms{cg_avg_extra}")
    print(f"  CG total:       {s.cg_total_ms:.3f} ms{cg_total_extra}")
    print(f"  Placer total:   {s.placer_total_ms:.3f} ms")


def print_cross_summary(summaries: list[RunSummary]) -> None:
    """Print a fixed-width comparison table across multiple runs."""
    if not summaries:
        return
    step(f"Cross-benchmark summary ({len(summaries)} runs)")
    have_cycles = all(
        s.hw_cg_total_cycles is not None and s.hw_cg_avg_cycles is not None
        for s in summaries
    )
    name_w = max(len("design"), *(len(s.design_name) for s in summaries))
    cols: list[str] = [
        f"{'design':<{name_w}}",
        f"{'cells':>5}",
        f"{'iters':>5}",
        f"{'outcome':<10}",
        f"{'final_HPWL':>10}",
        f"{'density':>7}",
        f"{'cg_avg_ms':>9}",
        f"{'cg_total_ms':>11}",
        f"{'placer_ms':>10}",
    ]
    if have_cycles:
        cols += [f"{'cg_avg_cyc':>10}", f"{'cg_total_cyc':>12}"]
    header = "  ".join(cols)
    print(header)
    print("-" * len(header))
    for s in summaries:
        row: list[str] = [
            f"{s.design_name:<{name_w}}",
            f"{s.n_cells:>5}",
            f"{s.iters_used:>5}",
            f"{s.tag:<10}",
            f"{s.hpwl_final:>10.0f}",
            f"{s.max_bin_density_final:>7.2f}",
            f"{s.cg_avg_ms:>9.3f}",
            f"{s.cg_total_ms:>11.3f}",
            f"{s.placer_total_ms:>10.3f}",
        ]
        if have_cycles:
            row += [
                f"{s.hw_cg_avg_cycles:>10.0f}",
                f"{s.hw_cg_total_cycles:>12d}",
            ]
        print("  ".join(row))


# --- Board session (fabric wrapper) ------------------------------------------


@dataclass
class BoardSession:
    """A fabric Connection + remote working directory, with put/get/run.

    Used as a context manager so the SSH connection is cleaned up on exit.
    """
    creds: BoardCreds
    board_dir: str
    _conn: Any = None  # fabric.Connection, lazily imported

    def __enter__(self) -> "BoardSession":
        from fabric import Connection
        self._conn = Connection(
            host=self.creds.host,
            connect_kwargs={
                "password": self.creds.password,
                "look_for_keys": False,
                "allow_agent": False,
            },
        )
        self._conn.run(
            f"mkdir -p {shlex.quote(self.board_dir)}", hide=True
        )
        return self

    def __exit__(self, *exc: Any) -> None:
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    def put(self, local: Path, remote_name: Optional[str] = None) -> None:
        """Copy local -> board_dir/remote_name (defaulting to local.name)."""
        target = f"{self.board_dir}/{remote_name or local.name}"
        self._conn.put(str(local), target)

    def get(self, remote_name: str, local: Path) -> None:
        """Copy board_dir/remote_name -> local."""
        self._conn.get(f"{self.board_dir}/{remote_name}", str(local))

    def run(self, command: str, *, capture: bool = False) -> str:
        """Run `cd board_dir && command`. Streams unless capture=True.

        Always returns the captured stdout (Fabric/Invoke populate
        result.stdout regardless of `hide`). Raises SystemExit on non-zero
        return code.
        """
        full = f"cd {shlex.quote(self.board_dir)} && {command}"
        result = self._conn.run(full, hide=capture, warn=True)
        if not result.ok:
            if capture:
                sys.stderr.write(result.stdout)
                sys.stderr.write(result.stderr)
            raise SystemExit(
                f"Remote command failed (rc={result.return_code}): {command}"
            )
        return result.stdout

    def set_board_dir(self, new_dir: str) -> None:
        """Switch the session's working dir, creating it if necessary.

        Used by multi-benchmark mode to reuse one SSH connection across
        per-benchmark remote dirs.
        """
        self.board_dir = new_dir
        self._conn.run(f"mkdir -p {shlex.quote(new_dir)}", hide=True)


# --- Slideshow (PNG frames -> GIF + MP4) -------------------------------------


class SlideshowMaker:
    """Compose a list of PNGs into a looping GIF and an MP4 video.

    Frames are held for FRAME_MS each, with the final frame held HOLD_LAST_MS.
    """

    def __init__(self, png_paths: list[str]) -> None:
        if not png_paths:
            raise ValueError("Slideshow needs at least one PNG frame.")
        for p in png_paths:
            if not os.path.isfile(p):
                raise FileNotFoundError(f"PNG frame not found: {p}")
        self.png_paths: list[str] = png_paths

    def _load_frames(self) -> list[Any]:
        """Load all PNGs as PIL Images, resizing to the first frame's size."""
        from PIL import Image
        frames = [Image.open(p).convert("RGB") for p in self.png_paths]
        target_size = frames[0].size
        for i, f in enumerate(frames[1:], start=1):
            if f.size != target_size:
                print(f"  Warning: {self.png_paths[i]} is {f.size}, "
                      f"resizing to {target_size}")
                frames[i] = f.resize(target_size, Image.LANCZOS)
        return frames

    def write_gif(self, out_path: str,
                  frame_ms: int = SLIDESHOW_FRAME_MS,
                  hold_last_ms: int = SLIDESHOW_HOLD_LAST_MS) -> None:
        frames = self._load_frames()
        n = len(frames)
        if n == 1:
            durations = [hold_last_ms]
        else:
            durations = [frame_ms] * (n - 1) + [hold_last_ms]
        frames[0].save(
            out_path,
            save_all=True,
            append_images=frames[1:],
            duration=durations,
            loop=0,
            optimize=False,
            disposal=2,
        )
        print(f"  Wrote {out_path} ({n} frames, loop forever)")

    def write_mp4(self, out_path: str, fps: float = SLIDESHOW_MP4_FPS) -> None:
        import numpy as np
        import imageio.v2 as imageio
        frames = self._load_frames()
        n = len(frames)
        # Hold last frame for HOLD_LAST_MS by repeating it.
        extra = max(1, int(round((SLIDESHOW_HOLD_LAST_MS / 1000.0) * fps)))
        with imageio.get_writer(out_path, fps=fps, codec="libx264",
                                quality=8, macro_block_size=1,
                                output_params=["-loglevel", "error"]) as writer:
            for f in frames:
                writer.append_data(np.asarray(f))
            for _ in range(extra - 1):
                writer.append_data(np.asarray(frames[-1]))
        print(f"  Wrote {out_path} ({n} frames + {extra - 1} hold)")


# --- The runner --------------------------------------------------------------


@dataclass
class _Args:
    mode: str
    benchmark_path: Path
    hw_version: str         # only used when mode == "verilated" / "fpga"
    int_bits: int           # only used when mode == "verilated" / "golden"
    frac_bits: int          # only used when mode == "verilated" / "golden"
    max_n: int              # only used when mode == "verilated" / "golden"
    sweep: bool
    max_iter: Optional[int]   # None = use placer's built-in default


class RunPlacer:
    """End-to-end placer driver. Runs once by default; sweeps when --sweep
    is passed."""

    def __init__(
        self,
        args: _Args,
        *,
        benchmark_override: Optional[Path] = None,
        shared_board: Optional[BoardSession] = None,
    ) -> None:
        self.args: _Args = args
        self.repo_root: Path = find_repo_root()
        self.build_dir: Path = Path.cwd().resolve()
        self.is_remote: bool = args.mode in _REMOTE_MODES
        self.design_name: str = ""
        self.json_file: Path = Path()
        self.frames: list[Path] = []  # populated by sweep mode
        self.benchmark_path: Path = benchmark_override or args.benchmark_path
        self.shared_board: Optional[BoardSession] = shared_board
        self.summary: Optional[RunSummary] = None

    # -- Stages --------------------------------------------------------------

    def parse_lefdef(self) -> None:
        step("Generating JSON")
        self.design_name, self.json_file = parse_lefdef(self.benchmark_path)

    def build(self) -> None:
        if self.args.mode == "python":
            return  # No build needed for the Python baseline.
        spec: BuildSpec = build_spec_for(
            self.args.mode,
            self.args.hw_version,
            self.args.int_bits,
            self.args.frac_bits,
            self.args.max_n,
        )
        step(self._build_banner())
        cmake_build(spec, self.repo_root, self.build_dir)

    def run(self) -> None:
        if self.args.sweep:
            self._run_sweep()
        else:
            self._run_single()

    def report(self) -> None:
        step("Done")
        if self.args.sweep:
            print(
                f"  Frames: {self.design_name}-final-iter*.png "
                f"({len(self.frames)} files)"
            )
            print(
                f"  Slideshow: {self.design_name}-sweep.gif, "
                f"{self.design_name}-sweep.mp4"
            )
        else:
            print(
                f"  Results: {self.design_name}-initial.json, "
                f"{self.design_name}-final.json, "
                f"{self.design_name}-final.png"
            )
        if self.summary is not None:
            print_summary(self.summary)

    # -- Single-run path -----------------------------------------------------

    def _run_single(self) -> None:
        if self.args.mode == "python":
            step("Running Python placer")
            run_python_placer(self.json_file, self.repo_root, self.build_dir)
            self._render_final_png()
            return
        if not self.is_remote:
            banner = "Running placer"
            if self.args.mode == "verilated":
                banner = f"Running placer ({self.args.hw_version})"
            step(banner)
            output = run_local_placer_tee(
                self.json_file, self.build_dir, self.args.max_iter
            )
        elif self.shared_board is not None:
            output = self._run_remote_with_board(self.shared_board)
        else:
            creds = require_board_creds(self.repo_root)
            board_dir = self._board_dir(sweep=False)
            step(f"Copying to board ({board_dir})")
            with BoardSession(creds=creds, board_dir=board_dir) as board:
                output = self._run_remote_with_board(board)
        self.summary = parse_run_summary(output, self.design_name)
        self._render_final_png()

    def _render_final_png(self) -> None:
        """Render <design>-final.json into <design>-final.png. Single-run
        analogue of the per-iter PNGs sweep mode emits."""
        final_json = self.build_dir / f"{self.design_name}-final.json"
        if not final_json.is_file():
            return
        final_png = self.build_dir / f"{self.design_name}-final.png"
        self._render_png(final_json, final_png)

    def _run_remote_with_board(self, board: BoardSession) -> str:
        """Run the placer on the board through an open BoardSession."""
        if self.shared_board is not None:
            board.set_board_dir(self._board_dir(sweep=False))
            step(f"Copying to board ({board.board_dir})")
        board.put(self.build_dir / "placer")
        board.put(self.json_file)
        note = (
            "(FPGA bitstream must already be loaded)"
            if self.args.mode == "fpga"
            else ""
        )
        step(f"Running placer on board {note}".rstrip())
        cmd = f"./placer {self.json_file.name}"
        if self.args.max_iter is not None:
            cmd += f" {self.args.max_iter}"
        output = board.run(cmd)
        step("Copying results back")
        board.get(
            f"{self.design_name}-initial.json",
            self.build_dir / f"{self.design_name}-initial.json",
        )
        board.get(
            f"{self.design_name}-final.json",
            self.build_dir / f"{self.design_name}-final.json",
        )
        return output

    # -- Sweep path ----------------------------------------------------------

    def _run_sweep(self) -> None:
        if not self.is_remote:
            self._sweep_loop(None)
        elif self.shared_board is not None:
            self._sweep_with_board(self.shared_board)
        else:
            creds = require_board_creds(self.repo_root)
            board_dir = self._board_dir(sweep=True)
            step(f"Copying to board ({board_dir})")
            with BoardSession(creds=creds, board_dir=board_dir) as board:
                self._sweep_with_board(board)
        self._render_slideshow()

    def _sweep_with_board(self, board: BoardSession) -> None:
        if self.shared_board is not None:
            board.set_board_dir(self._board_dir(sweep=True))
            step(f"Copying to board ({board.board_dir})")
        board.put(self.build_dir / "placer")
        board.put(self.json_file)
        self._sweep_loop(board)

    def _sweep_loop(self, board: Optional[BoardSession]) -> None:
        step(f"Running sweep (1..{MAX_SWEEP_ITER})")
        last_n = 0
        for n in range(1, MAX_SWEEP_ITER + 1):
            summary = self._run_one(n, board)
            self.summary = summary  # last write wins -- the "natural" final
            if n == 1:
                self._capture_initial_frame(board)
            self._capture_frame(n, board)
            last_n = n
            print(
                f"  iter {n:2d}/{MAX_SWEEP_ITER}: "
                f"kept={summary.iters_used} ({summary.tag})"
            )
            if summary.iters_used < n and n >= MIN_FRAMES:
                print(
                    f"  Placer used only {summary.iters_used} of {n} "
                    f"iterations -- stopping sweep."
                )
                break
        if last_n > 0:
            # Mirror non-sweep mode: leave a canonical <design>-final.json
            # in the build dir. _capture_frame renamed each iter's final
            # away, so copy the highest-N per-iter JSON back.
            last_json = (
                self.build_dir / f"{self.design_name}-final-iter{last_n:02d}.json"
            )
            canonical = self.build_dir / f"{self.design_name}-final.json"
            shutil.copyfile(last_json, canonical)

    def _capture_initial_frame(self, board: Optional[BoardSession]) -> None:
        """Snapshot the placer's all-cells-at-die-center placement as the
        iter-0 frame. Runs once after the first placer invocation, since
        placer.cpp only writes <design>-initial.json when it actually runs."""
        iter0_json = (
            self.build_dir / f"{self.design_name}-final-iter00.json"
        )
        if board is not None:
            board.get(f"{self.design_name}-initial.json", iter0_json)
        else:
            initial = self.build_dir / f"{self.design_name}-initial.json"
            if not initial.is_file():
                raise SystemExit(
                    f"Error: expected {initial.name} after placer run"
                )
            shutil.copyfile(initial, iter0_json)
        iter0_png = (
            self.build_dir / f"{self.design_name}-final-iter00.png"
        )
        self._render_png(iter0_json, iter0_png)
        self.frames.append(iter0_png)

    def _run_one(self, n: int, board: Optional[BoardSession]) -> RunSummary:
        if board is not None:
            output = board.run(
                f"./placer {self.json_file.name} {n}", capture=True
            )
        else:
            output = run_local_placer_capture(
                self.json_file, n, self.build_dir
            )
        return parse_run_summary(output, self.design_name)

    def _capture_frame(self, n: int, board: Optional[BoardSession]) -> None:
        per_iter_json = (
            self.build_dir / f"{self.design_name}-final-iter{n:02d}.json"
        )
        if board is not None:
            board.get(f"{self.design_name}-final.json", per_iter_json)
        else:
            final = self.build_dir / f"{self.design_name}-final.json"
            if not final.is_file():
                raise SystemExit(
                    f"Error: expected {final.name} after placer run"
                )
            final.rename(per_iter_json)

        per_iter_png = (
            self.build_dir / f"{self.design_name}-final-iter{n:02d}.png"
        )
        self._render_png(per_iter_json, per_iter_png)
        self.frames.append(per_iter_png)

    def _render_slideshow(self) -> None:
        step(f"Building slideshow ({len(self.frames)} frames)")
        out_prefix = f"{self.design_name}-sweep"
        print(
            f"Slideshow: {len(self.frames)} frames -> "
            f"{out_prefix}.gif, {out_prefix}.mp4"
        )
        maker = SlideshowMaker([str(p) for p in self.frames])
        maker.write_gif(str(self.build_dir / f"{out_prefix}.gif"))
        maker.write_mp4(str(self.build_dir / f"{out_prefix}.mp4"))

    @staticmethod
    def _render_png(json_path: Path, png_path: Path) -> None:
        from json_utils import load_netlist
        from visualizer import render_to_png
        netlist = load_netlist(str(json_path))
        render_to_png(netlist, str(png_path))

    # -- Helpers -------------------------------------------------------------

    def _board_dir(self, sweep: bool) -> str:
        suffix = ""
        if self.args.mode == "fpga":
            suffix = "-fpga"
        if sweep:
            suffix += "-sweep"
        return f"/home/root/build-{self.design_name}{suffix}"

    def _build_banner(self) -> str:
        m = self.args.mode
        if m == "sw":
            return "Building C++ placer (software)"
        if m == "golden":
            return (
                f"Building C++ placer (FP golden CG, "
                f"Q{self.args.int_bits}.{self.args.frac_bits}, "
                f"max_n={self.args.max_n})"
            )
        if m == "verilated":
            return (
                f"Building C++ placer (Verilator CG, {self.args.hw_version}, "
                f"Q{self.args.int_bits}.{self.args.frac_bits}, "
                f"max_n={self.args.max_n})"
            )
        if m == "arm":
            return "Cross-compiling placer for ARM"
        return (
            f"Cross-compiling FPGA placer for ARM "
            f"({self.args.hw_version} mmap driver)"
        )


# --- Multi-benchmark orchestration -------------------------------------------


def run_multi(args: _Args, benches: list[Path]) -> None:
    """Run a sequence of benchmarks with one shared build (and one shared
    SSH session for remote modes), then print a cross-benchmark table."""
    n = len(benches)
    step(f"Multi-benchmark mode: {n} benchmarks")
    for i, b in enumerate(benches, start=1):
        print(f"  [{i}/{n}] {b.name}")

    # Build once -- the binary is the same for every benchmark.
    builder = RunPlacer(args, benchmark_override=benches[0])
    builder.build()

    summaries: list[RunSummary] = []
    is_remote = args.mode in _REMOTE_MODES

    def _run_each(board: Optional[BoardSession]) -> None:
        for i, bench in enumerate(benches, start=1):
            step(f"[{i}/{n}] Benchmark: {bench.name}")
            runner = RunPlacer(
                args, benchmark_override=bench, shared_board=board
            )
            runner.parse_lefdef()
            runner.run()
            if runner.summary is not None:
                print_summary(
                    runner.summary,
                    header=f"Run summary: {runner.design_name} [{i}/{n}]",
                )
                summaries.append(runner.summary)

    if is_remote:
        creds = require_board_creds(builder.repo_root)
        # Open one session; per-benchmark dirs are set inside the loop.
        with BoardSession(creds=creds, board_dir="/home/root") as board:
            _run_each(board)
    else:
        _run_each(None)

    print_cross_summary(summaries)


# --- CLI ---------------------------------------------------------------------


def _parse_args(argv: Optional[list[str]] = None) -> _Args:
    parser = argparse.ArgumentParser(
        prog="run-placer",
        description=(
            "Run the analytical placer end-to-end. With --sweep, runs the "
            "placer with max_outer_iter from 1..16 and stitches the per-iter "
            "placements into a looping slideshow (GIF + MP4)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="mode", required=True, metavar="mode")

    def _add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "--sweep",
            action="store_true",
            help=(
                "Sweep max_outer_iter from 1..16 and emit a slideshow of "
                "per-iter placements. Not supported with mode=python."
            ),
        )
        p.add_argument(
            "--max-iter",
            type=int,
            default=None,
            help=(
                "Cap the placer's outer iteration count "
                "(passes argv[2] to ./placer). Default: placer's built-in "
                "MAX_OUTER_ITER (16). Not combinable with --sweep."
            ),
        )

    def _add_q_format(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "--int-bits",
            type=int,
            default=13,
            help=(
                "Q-format integer bits (default 13). int_bits + frac_bits "
                "<= 64; widths above 27 only work with golden / verilated."
            ),
        )
        p.add_argument(
            "--frac-bits",
            type=int,
            default=14,
            help=(
                "Q-format fractional bits (default 14). Forwarded to "
                "verilator as -Gp_frac_bits."
            ),
        )
        p.add_argument(
            "--max-n",
            type=int,
            default=50,
            help=(
                "Max cell count (default 50). Sets Verilog p_max_n and "
                "the C++ driver's MAX_N. Honored by golden / verilated; "
                "rejected for sw / arm / fpga (the bitstream is locked)."
            ),
        )

    plain_modes = [
        ("python", "Baseline Python placer (no --sweep support)"),
        ("sw", "Full software C++ placer (double precision)"),
        ("arm", "Cross-compile SW placer, run on DE1-SoC ARM"),
    ]
    for mode_name, help_text in plain_modes:
        sp = sub.add_parser(mode_name, help=help_text)
        sp.add_argument(
            "benchmark_path",
            type=Path,
            help="Path to a benchmark directory (containing lef/ and def/).",
        )
        sp.set_defaults(hw_version="v6")  # unused; keeps the attr defined
        _add_common(sp)

    sp_golden = sub.add_parser(
        "golden",
        help="C++ placer with fixed-point golden CG.",
    )
    sp_golden.add_argument(
        "benchmark_path",
        type=Path,
        help="Path to a benchmark directory (containing lef/ and def/).",
    )
    sp_golden.set_defaults(hw_version="v6")  # unused; keeps the attr defined
    _add_q_format(sp_golden)
    _add_common(sp_golden)

    sp_ver = sub.add_parser(
        "verilated",
        help="C++ placer with Verilator RTL CG. Optional [v2|v3|v4|v5|v5_deep|v6] before path.",
    )
    sp_ver.add_argument(
        "hw_version",
        nargs="?",
        choices=["v2", "v3", "v4", "v5", "v5_deep", "v6"],
        default="v6",
        help="Verilator RTL version (default v6).",
    )
    sp_ver.add_argument(
        "benchmark_path",
        type=Path,
        help="Path to a benchmark directory (containing lef/ and def/).",
    )
    _add_q_format(sp_ver)
    _add_common(sp_ver)

    sp_fpga = sub.add_parser(
        "fpga",
        help="Cross-compile FPGA-accelerated placer, run on DE1-SoC. Optional [v4|v5|v6] before path.",
    )
    sp_fpga.add_argument(
        "hw_version",
        nargs="?",
        choices=["v4", "v5", "v6"],
        default="v6",
        help="FPGA mmap driver version (default v6).",
    )
    sp_fpga.add_argument(
        "benchmark_path",
        type=Path,
        help="Path to a benchmark directory (containing lef/ and def/).",
    )
    _add_common(sp_fpga)

    args = parser.parse_args(argv)

    if args.sweep and args.mode == "python":
        parser.error("--sweep is not supported with mode=python")

    is_multi = (
        args.benchmark_path.is_dir()
        and not looks_like_benchmark(args.benchmark_path)
        and any(looks_like_benchmark(c) for c in args.benchmark_path.iterdir())
    )
    if is_multi and args.mode == "python":
        parser.error("multi-benchmark mode does not support mode=python")

    int_bits: int  = getattr(args, "int_bits",  13)
    frac_bits: int = getattr(args, "frac_bits", 14)
    max_n: int     = getattr(args, "max_n",     50)
    if int_bits < 1 or frac_bits < 1:
        parser.error("--int-bits and --frac-bits must each be >= 1")
    if int_bits + frac_bits > 64:
        parser.error(
            f"--int-bits ({int_bits}) + --frac-bits ({frac_bits}) "
            "exceeds 64 -- the verilator driver caps fixed-point storage "
            "at int64_t."
        )
    if int_bits + frac_bits != 27 and args.mode not in ("golden", "verilated"):
        parser.error(
            f"--int-bits + --frac-bits = {int_bits + frac_bits} but only "
            "golden / verilated honor widths different from 27 (sw uses "
            "doubles; arm/fpga are locked to the 27-bit bitstream)."
        )
    if max_n < 1:
        parser.error("--max-n must be >= 1")
    if max_n != 50 and args.mode not in ("golden", "verilated"):
        parser.error(
            f"--max-n = {max_n} but only golden / verilated honor a "
            "non-default max-n (sw uses dynamic vectors; arm/fpga are "
            "locked to the bitstream's M10K depth of 50)."
        )

    max_iter: Optional[int] = getattr(args, "max_iter", None)
    if max_iter is not None:
        if max_iter < 1:
            parser.error("--max-iter must be >= 1")
        if args.mode == "python":
            parser.error("--max-iter is not supported with mode=python")
        if args.sweep:
            parser.error("--max-iter and --sweep are mutually exclusive")

    return _Args(
        mode=args.mode,
        benchmark_path=args.benchmark_path,
        hw_version=args.hw_version,
        int_bits=int_bits,
        frac_bits=frac_bits,
        max_n=max_n,
        sweep=args.sweep,
        max_iter=max_iter,
    )


def main() -> None:
    args = _parse_args()
    clean_build_dir()
    benches = discover_benchmarks(args.benchmark_path)
    if len(benches) == 1:
        runner = RunPlacer(args, benchmark_override=benches[0])
        runner.parse_lefdef()
        runner.build()
        runner.run()
        runner.report()
    else:
        run_multi(args, benches)


if __name__ == "__main__":
    main()
