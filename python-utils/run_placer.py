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
  verilated [v2|v3|v4|v5|v5_deep] C++ placer with Verilator RTL CG (default v5) (run + sweep)
  arm                    Cross-compile SW placer, run on DE1-SoC ARM           (run + sweep)
  fpga      [v4|v5]      Cross-compile FPGA-accelerated placer, run on DE1-SoC (run + sweep)
                         (default v5; selects the mmap driver to link in)
                         --p-max-n N   bitstream's p_max_n (default 50; rebuilds driver)

Sweep mode runs the placer with max_outer_iter from 1..MAX_SWEEP_ITER,
captures the per-iter final placement, renders one PNG frame per
iteration, and stitches the frames into a looping GIF + MP4. It stops
early once the placer reports it needed fewer iterations than the cap
(with a 2-frame minimum so the slideshow always has something to
animate).
"""

import argparse
import os
import re
import shlex
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


def build_spec_for(mode: str, hw_version: str, p_max_n: int) -> BuildSpec:
    """Map a mode (and hw_version / p_max_n for verilated|fpga) to its BuildSpec."""
    if mode == "sw":
        return BuildSpec(_SW_C, [])
    if mode == "golden":
        return BuildSpec(_SW_C, ["-DUSE_FP_GOLDEN=ON"])
    if mode == "verilated":
        return BuildSpec(_SW_C, ["-DUSE_HW_CG=ON", f"-DHW_CG_VERSION={hw_version}"])
    if mode == "arm":
        return BuildSpec(_SW_C, list(_ARM_DEFINES))
    if mode == "fpga":
        return BuildSpec(
            _FPGA_SW,
            [*_ARM_DEFINES, f"-DHW_FPGA_VERSION={hw_version}", f"-DMAX_N={p_max_n}"],
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


def run_local_placer_streaming(json_file: Path, build_dir: Path) -> None:
    """Run ./placer JSON, streaming output to the terminal. Raises on failure."""
    subprocess.run(["./placer", str(json_file)], cwd=build_dir, check=True)


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


# --- Output marker parsing (sweep) -------------------------------------------


@dataclass
class PlacerMarkers:
    iters_used: int
    converged: bool
    reverted: bool

    @property
    def tag(self) -> str:
        if self.converged:
            return "converged"
        if self.reverted:
            return "reverted"
        return "capped"


def parse_placer_markers(output: str) -> PlacerMarkers:
    """Parse the marker lines printed at the end of placer.cpp main()."""
    m_iters = re.search(r"^Outer iterations used:\s+(\d+)", output, re.M)
    m_conv = re.search(r"^Converged:\s+(\S+)", output, re.M)
    m_rev = re.search(r"^Reverted:\s+(\S+)", output, re.M)
    if not (m_iters and m_conv and m_rev):
        sys.stderr.write(output)
        raise SystemExit(
            "Could not find marker lines in placer output. "
            "Did placer.cpp print 'Outer iterations used: K'?"
        )
    return PlacerMarkers(
        iters_used=int(m_iters.group(1)),
        converged=m_conv.group(1) == "true",
        reverted=m_rev.group(1) == "true",
    )


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

        Raises SystemExit on non-zero return code.
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
        return result.stdout if capture else ""


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
    hw_version: str   # only used when mode == "verilated" / "fpga"
    p_max_n: int      # only used when mode == "fpga" (must match bitstream p_max_n)
    sweep: bool


class RunPlacer:
    """End-to-end placer driver. Runs once by default; sweeps when --sweep
    is passed."""

    def __init__(self, args: _Args) -> None:
        self.args: _Args = args
        self.repo_root: Path = find_repo_root()
        self.build_dir: Path = Path.cwd().resolve()
        self.is_remote: bool = args.mode in _REMOTE_MODES
        self.design_name: str = ""
        self.json_file: Path = Path()
        self.frames: list[Path] = []  # populated by sweep mode

    # -- Stages --------------------------------------------------------------

    def parse_lefdef(self) -> None:
        step("Generating JSON")
        self.design_name, self.json_file = parse_lefdef(self.args.benchmark_path)

    def build(self) -> None:
        if self.args.mode == "python":
            return  # No build needed for the Python baseline.
        spec: BuildSpec = build_spec_for(
            self.args.mode, self.args.hw_version, self.args.p_max_n
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
                f"{self.design_name}-final.json"
            )

    # -- Single-run path -----------------------------------------------------

    def _run_single(self) -> None:
        if self.args.mode == "python":
            step("Running Python placer")
            run_python_placer(self.json_file, self.repo_root, self.build_dir)
            return
        if not self.is_remote:
            banner = "Running placer"
            if self.args.mode == "verilated":
                banner = f"Running placer ({self.args.hw_version})"
            step(banner)
            run_local_placer_streaming(self.json_file, self.build_dir)
            return
        # arm / fpga single run
        creds = require_board_creds(self.repo_root)
        board_dir = self._board_dir(sweep=False)
        step(f"Copying to board ({board_dir})")
        with BoardSession(creds=creds, board_dir=board_dir) as board:
            board.put(self.build_dir / "placer")
            board.put(self.json_file)
            note = (
                "(FPGA bitstream must already be loaded)"
                if self.args.mode == "fpga"
                else ""
            )
            step(f"Running placer on board {note}".rstrip())
            board.run(f"./placer {self.json_file.name}")
            step("Copying results back")
            board.get(
                f"{self.design_name}-initial.json",
                self.build_dir / f"{self.design_name}-initial.json",
            )
            board.get(
                f"{self.design_name}-final.json",
                self.build_dir / f"{self.design_name}-final.json",
            )

    # -- Sweep path ----------------------------------------------------------

    def _run_sweep(self) -> None:
        if self.is_remote:
            creds = require_board_creds(self.repo_root)
            board_dir = self._board_dir(sweep=True)
            step(f"Copying to board ({board_dir})")
            with BoardSession(creds=creds, board_dir=board_dir) as board:
                board.put(self.build_dir / "placer")
                board.put(self.json_file)
                self._sweep_loop(board)
        else:
            self._sweep_loop(None)
        self._render_slideshow()

    def _sweep_loop(self, board: Optional[BoardSession]) -> None:
        step(f"Running sweep (1..{MAX_SWEEP_ITER})")
        for n in range(1, MAX_SWEEP_ITER + 1):
            markers = self._run_one(n, board)
            self._capture_frame(n, board)
            print(
                f"  iter {n:2d}/{MAX_SWEEP_ITER}: "
                f"kept={markers.iters_used} ({markers.tag})"
            )
            if markers.iters_used < n and n >= MIN_FRAMES:
                print(
                    f"  Placer used only {markers.iters_used} of {n} "
                    f"iterations -- stopping sweep."
                )
                break

    def _run_one(self, n: int, board: Optional[BoardSession]) -> PlacerMarkers:
        if board is not None:
            output = board.run(
                f"./placer {self.json_file.name} {n}", capture=True
            )
        else:
            output = run_local_placer_capture(
                self.json_file, n, self.build_dir
            )
        return parse_placer_markers(output)

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
            return "Building C++ placer (FP golden CG)"
        if m == "verilated":
            return f"Building C++ placer (Verilator CG, {self.args.hw_version})"
        if m == "arm":
            return "Cross-compiling placer for ARM"
        return (
            f"Cross-compiling FPGA placer for ARM "
            f"({self.args.hw_version} mmap driver, p_max_n={self.args.p_max_n})"
        )


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

    plain_modes = [
        ("python", "Baseline Python placer (no --sweep support)"),
        ("sw", "Full software C++ placer (double precision)"),
        ("golden", "C++ placer with fixed-point golden CG"),
        ("arm", "Cross-compile SW placer, run on DE1-SoC ARM"),
    ]
    for mode_name, help_text in plain_modes:
        sp = sub.add_parser(mode_name, help=help_text)
        sp.add_argument(
            "benchmark_path",
            type=Path,
            help="Path to a benchmark directory (containing lef/ and def/).",
        )
        sp.set_defaults(hw_version="v4")  # unused; keeps the attr defined
        _add_common(sp)

    sp_ver = sub.add_parser(
        "verilated",
        help="C++ placer with Verilator RTL CG. Optional [v2|v3|v4|v5|v5_deep] before path.",
    )
    sp_ver.add_argument(
        "hw_version",
        nargs="?",
        choices=["v2", "v3", "v4", "v5", "v5_deep"],
        default="v5",
        help="Verilator RTL version (default v5).",
    )
    sp_ver.add_argument(
        "benchmark_path",
        type=Path,
        help="Path to a benchmark directory (containing lef/ and def/).",
    )
    _add_common(sp_ver)

    sp_fpga = sub.add_parser(
        "fpga",
        help="Cross-compile FPGA-accelerated placer, run on DE1-SoC. Optional [v4|v5] before path.",
    )
    sp_fpga.add_argument(
        "hw_version",
        nargs="?",
        choices=["v4", "v5"],
        default="v5",
        help="FPGA mmap driver version (default v5).",
    )
    sp_fpga.add_argument(
        "benchmark_path",
        type=Path,
        help="Path to a benchmark directory (containing lef/ and def/).",
    )
    sp_fpga.add_argument(
        "--p-max-n",
        type=int,
        default=50,
        help=(
            "Max cell count the bitstream supports (default 50). Must match "
            "the synthesized Verilog p_max_n; the mmap driver is rebuilt "
            "with HW_MAX_N set to this value."
        ),
    )
    _add_common(sp_fpga)

    args = parser.parse_args(argv)

    if args.sweep and args.mode == "python":
        parser.error("--sweep is not supported with mode=python")

    p_max_n = getattr(args, "p_max_n", 50)
    if p_max_n <= 0:
        parser.error("--p-max-n must be a positive integer")

    return _Args(
        mode=args.mode,
        benchmark_path=args.benchmark_path,
        hw_version=args.hw_version,
        p_max_n=p_max_n,
        sweep=args.sweep,
    )


def main() -> None:
    args = _parse_args()
    runner = RunPlacer(args)
    runner.parse_lefdef()
    runner.build()
    runner.run()
    runner.report()


if __name__ == "__main__":
    main()
