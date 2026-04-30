"""Build an animated slideshow (GIF + MP4) from a sequence of PNG frames.

Usage:
  uv run python-utils/slideshow.py <out-prefix> <png1> [png2 ...]

Writes <out-prefix>.gif (loops forever) and <out-prefix>.mp4. Frames are held
for FRAME_MS milliseconds each, with the final frame held HOLD_LAST_MS.
"""

import os
import sys


FRAME_MS = 400
HOLD_LAST_MS = 1500
MP4_FPS = 2.5  # roughly matches FRAME_MS but ignores the final-frame hold


class SlideshowMaker:
    """Compose a list of PNGs into a looping GIF and an MP4 video."""

    def __init__(self, png_paths: list[str]) -> None:
        if not png_paths:
            raise ValueError("Slideshow needs at least one PNG frame.")
        for p in png_paths:
            if not os.path.isfile(p):
                raise FileNotFoundError(f"PNG frame not found: {p}")
        self.png_paths: list[str] = png_paths

    def _load_frames(self):  # type: ignore[no-untyped-def]
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
                  frame_ms: int = FRAME_MS,
                  hold_last_ms: int = HOLD_LAST_MS) -> None:
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

    def write_mp4(self, out_path: str, fps: float = MP4_FPS) -> None:
        import numpy as np
        import imageio.v2 as imageio
        frames = self._load_frames()
        n = len(frames)
        # Hold last frame for HOLD_LAST_MS by repeating it.
        extra = max(1, int(round((HOLD_LAST_MS / 1000.0) * fps)))
        with imageio.get_writer(out_path, fps=fps, codec="libx264",
                                quality=8, macro_block_size=1,
                                output_params=["-loglevel", "error"]) as writer:
            for f in frames:
                writer.append_data(np.asarray(f))
            for _ in range(extra - 1):
                writer.append_data(np.asarray(frames[-1]))
        print(f"  Wrote {out_path} ({n} frames + {extra - 1} hold)")


def main() -> None:
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <out-prefix> <png1> [png2 ...]")
        print(f"  e.g. {sys.argv[0]} DMA-sweep DMA-final-iter01.png DMA-final-iter02.png")
        sys.exit(1)
    out_prefix = sys.argv[1]
    png_paths = sys.argv[2:]

    print(f"Slideshow: {len(png_paths)} frames -> {out_prefix}.gif, {out_prefix}.mp4")
    maker = SlideshowMaker(png_paths)
    maker.write_gif(f"{out_prefix}.gif")
    maker.write_mp4(f"{out_prefix}.mp4")


if __name__ == "__main__":
    main()
