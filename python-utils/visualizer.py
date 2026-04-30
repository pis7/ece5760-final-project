"""Visualize placement from a netlist JSON file.

Usage:
  uv run python-utils/visualizer.py <netlist.json>
      Launch the interactive Tk viewer.
  uv run python-utils/visualizer.py --png <out.png> <netlist.json>
      Render a single PNG headlessly (no Tk window, no $DISPLAY needed).

Tk controls:
  - Scroll wheel: zoom in/out
  - Click + drag: pan
  - F: fit all
  - Q: quit
"""

import os
import sys
from abc import ABC, abstractmethod

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from json_utils import Component, Netlist, load_netlist


CANVAS_W = 1200
CANVAS_H = 900
PAD = 40  # pixel padding around die area
LARGE_MACRO_FRAC = 0.02  # macros above this fraction of die area are "large"


# -- Renderer abstraction -----------------------------------------------------


class Renderer(ABC):
    """Backend-agnostic 2D drawing primitives used by PlacementScene."""

    @abstractmethod
    def width(self) -> int: ...

    @abstractmethod
    def height(self) -> int: ...

    @abstractmethod
    def rect(self, x0: float, y0: float, x1: float, y1: float,
             fill: str | None = None, outline: str | None = None,
             width: int = 1) -> int | None:
        """Draw a rectangle. Returns a backend-specific item id if hit-testing
        is supported (Tk canvas item id), otherwise None."""
        ...

    @abstractmethod
    def oval(self, cx: float, cy: float, r: float,
             fill: str | None = None, outline: str | None = None) -> None: ...

    @abstractmethod
    def text(self, x: float, y: float, s: str,
             fill: str = "white", font_size: int = 9) -> None: ...


class PillowRenderer(Renderer):
    """Renderer that draws into a PIL Image."""

    def __init__(self, w: int, h: int, bg: str = "black") -> None:
        from PIL import Image, ImageDraw, ImageFont
        self._w = w
        self._h = h
        self.image = Image.new("RGB", (w, h), bg)
        self.draw = ImageDraw.Draw(self.image)
        # Try to load a TrueType font for legend text; fall back to bitmap.
        self._font_cache: dict[int, ImageFont.ImageFont] = {}
        self._font_paths = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/TTF/DejaVuSans.ttf",
            "/Library/Fonts/Arial.ttf",
        ]

    def _font(self, size: int):  # type: ignore[no-untyped-def]
        from PIL import ImageFont
        if size in self._font_cache:
            return self._font_cache[size]
        font: ImageFont.ImageFont
        for p in self._font_paths:
            try:
                font = ImageFont.truetype(p, size=size)
                break
            except OSError:
                continue
        else:
            font = ImageFont.load_default()
        self._font_cache[size] = font
        return font

    def width(self) -> int:
        return self._w

    def height(self) -> int:
        return self._h

    def rect(self, x0: float, y0: float, x1: float, y1: float,
             fill: str | None = None, outline: str | None = None,
             width: int = 1) -> int | None:
        if x1 < x0:
            x0, x1 = x1, x0
        if y1 < y0:
            y0, y1 = y1, y0
        self.draw.rectangle((x0, y0, x1, y1), fill=fill, outline=outline, width=width)
        return None

    def oval(self, cx: float, cy: float, r: float,
             fill: str | None = None, outline: str | None = None) -> None:
        self.draw.ellipse((cx - r, cy - r, cx + r, cy + r),
                          fill=fill, outline=outline)

    def text(self, x: float, y: float, s: str,
             fill: str = "white", font_size: int = 9) -> None:
        # Anchor "lm" = left-middle (matches Tk anchor=tk.W with center y).
        self.draw.text((x, y), s, fill=fill, font=self._font(font_size + 3),
                       anchor="lm")


class TkRenderer(Renderer):
    """Renderer that draws into a tk.Canvas."""

    def __init__(self, canvas) -> None:  # type: ignore[no-untyped-def]
        self.canvas = canvas

    def width(self) -> int:
        return self.canvas.winfo_width() or CANVAS_W

    def height(self) -> int:
        return self.canvas.winfo_height() or CANVAS_H

    def rect(self, x0: float, y0: float, x1: float, y1: float,
             fill: str | None = None, outline: str | None = None,
             width: int = 1) -> int | None:
        kw: dict = {"width": width}
        if fill is not None:
            kw["fill"] = fill
        if outline is not None:
            kw["outline"] = outline
        else:
            kw["outline"] = ""
        return self.canvas.create_rectangle(x0, y0, x1, y1, **kw)

    def oval(self, cx: float, cy: float, r: float,
             fill: str | None = None, outline: str | None = None) -> None:
        kw: dict = {}
        if fill is not None:
            kw["fill"] = fill
        kw["outline"] = outline if outline is not None else ""
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, **kw)

    def text(self, x: float, y: float, s: str,
             fill: str = "white", font_size: int = 9) -> None:
        import tkinter as tk
        self.canvas.create_text(x, y, text=s, anchor=tk.W, fill=fill,
                                font=("TkDefaultFont", font_size))


# -- Scene (geometry + drawing logic, backend-agnostic) -----------------------


class PlacementScene:
    """Computes view transform and dispatches drawing primitives to a renderer."""

    def __init__(self, netlist: Netlist) -> None:
        self.netlist: Netlist = netlist
        self.dbu_per_micron: int = netlist.dbu_per_micron

        self.die_xl: float = netlist.die_area[0]
        self.die_yl: float = netlist.die_area[1]
        self.die_xh: float = netlist.die_area[2]
        self.die_yh: float = netlist.die_area[3]

        die_w = self.die_xh - self.die_xl
        die_h = self.die_yh - self.die_yl
        self.die_area_dbu2: float = die_w * die_h

        self.scale: float = 1.0
        self.offset_x: float = 0.0
        self.offset_y: float = 0.0
        self.hide_large: bool = False

        # Maps backend item ids (Tk canvas item ids) -> Component. Built during
        # _draw_components; used by PlacementViewer for hover tooltips. Empty
        # for backends without hit-testing (e.g. PillowRenderer).
        self.comp_items: dict[int, Component] = {}

    def fit(self, canvas_w: int, canvas_h: int) -> None:
        """Set scale and offset to fit the die area in the canvas."""
        die_w = self.die_xh - self.die_xl
        die_h = self.die_yh - self.die_yl
        if die_w <= 0 or die_h <= 0:
            return
        sx = (canvas_w - 2 * PAD) / die_w
        sy = (canvas_h - 2 * PAD) / die_h
        self.scale = min(sx, sy)
        self.offset_x = (canvas_w - die_w * self.scale) / 2 - self.die_xl * self.scale
        self.offset_y = (canvas_h - die_h * self.scale) / 2 + self.die_yh * self.scale

    def to_canvas(self, dbu_x: float, dbu_y: float) -> tuple[float, float]:
        cx = dbu_x * self.scale + self.offset_x
        cy = -dbu_y * self.scale + self.offset_y
        return cx, cy

    def draw(self, r: Renderer) -> None:
        self._draw_die(r)
        self._draw_components(r)
        self._draw_io_pins(r)
        self._draw_legend(r)

    def _draw_die(self, r: Renderer) -> None:
        x0, y0 = self.to_canvas(self.die_xl, self.die_yl)
        x1, y1 = self.to_canvas(self.die_xh, self.die_yh)
        r.rect(x0, y0, x1, y1, outline="#555555", width=2)

    def _draw_components(self, r: Renderer) -> None:
        nl = self.netlist
        area_threshold = self.die_area_dbu2 * LARGE_MACRO_FRAC
        dbu = self.dbu_per_micron
        self.comp_items = {}
        for comp in nl.components.values():
            macro = nl.macros.get(comp.macro_name)
            if macro is None:
                continue
            w = macro.width * dbu
            h = macro.height * dbu
            if self.hide_large and w * h > area_threshold:
                continue
            x0, y0 = self.to_canvas(comp.x, comp.y)
            x1, y1 = self.to_canvas(comp.x + w, comp.y + h)
            item = r.rect(x0, y0, x1, y1, fill="#4080c0", outline="white", width=1)
            if item is not None:
                self.comp_items[item] = comp

    def _draw_io_pins(self, r: Renderer) -> None:
        die_size = min(self.die_xh - self.die_xl, self.die_yh - self.die_yl)
        radius = max(3, die_size * 0.008 * self.scale)
        for pin in self.netlist.io_pins:
            cx, cy = self.to_canvas(pin.x, pin.y)
            r.oval(cx, cy, radius, fill="#e0e040")

    def _draw_legend(self, r: Renderer) -> None:
        items = [
            ("#4080c0", "Macro"),
            ("#e0e040", "I/O pin"),
        ]
        x = r.width() - 10
        y = 10
        for color, label in items:
            r.rect(x - 185, y, x - 170, y + 12, fill=color)
            r.text(x - 165, y + 6, label, fill="white", font_size=9)
            y += 18


# -- Module-level helper for headless PNG render ------------------------------


def render_to_png(netlist: Netlist, png_path: str,
                  w: int = CANVAS_W, h: int = CANVAS_H) -> None:
    """Render a netlist to a PNG file with the same look as the Tk fit-all view."""
    scene = PlacementScene(netlist)
    scene.fit(w, h)
    renderer = PillowRenderer(w, h, bg="black")
    scene.draw(renderer)
    renderer.image.save(png_path)


# -- Tk viewer ----------------------------------------------------------------


class PlacementViewer:
    """Tk canvas viewer for netlist placement data with pan and zoom."""

    def __init__(self, netlist: Netlist) -> None:
        import tkinter as tk

        self.scene: PlacementScene = PlacementScene(netlist)

        self.drag_x: int = 0
        self.drag_y: int = 0

        self.root: tk.Tk = tk.Tk()
        self.root.title(f"Placement: {netlist.design_name}")

        toolbar: tk.Frame = tk.Frame(self.root, bg="#333")
        toolbar.pack(fill=tk.X)
        self.hide_large_var: tk.BooleanVar = tk.BooleanVar(value=False)
        tk.Checkbutton(
            toolbar, text="Hide large macros (>2% die area)",
            variable=self.hide_large_var, command=self._on_toggle_hide,
            bg="#333", fg="white", selectcolor="#555",
            activebackground="#444", activeforeground="white",
        ).pack(side=tk.LEFT, padx=5, pady=2)

        self.canvas: tk.Canvas = tk.Canvas(
            self.root, width=CANVAS_W, height=CANVAS_H, bg="black"
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.status: tk.Label = tk.Label(
            self.root, text="", anchor=tk.W, bg="#222", fg="white"
        )
        self.status.pack(fill=tk.X)

        self.tooltip: tk.Label = tk.Label(
            self.canvas, text="", bg="#222", fg="white",
            padx=4, pady=2, borderwidth=1, relief=tk.SOLID,
            font=("TkDefaultFont", 9),
        )

        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", self._on_leave)
        self.canvas.bind("<MouseWheel>", self._on_scroll)
        self.canvas.bind("<Button-4>", self._on_scroll_up)
        self.canvas.bind("<Button-5>", self._on_scroll_down)
        self.root.bind("<f>", self._on_fit_all)
        self.root.bind("<q>", lambda e: self.root.destroy())
        self.root.bind("<Configure>", self._on_resize)

        self._update_status()
        self.canvas.bind("<Map>", self._on_first_map)

    def _renderer(self) -> TkRenderer:
        return TkRenderer(self.canvas)

    def _draw(self) -> None:
        self.canvas.delete("all")
        self.scene.draw(self._renderer())

    def _on_toggle_hide(self) -> None:
        self.scene.hide_large = self.hide_large_var.get()
        self._draw()

    def _update_status(self) -> None:
        nl = self.scene.netlist
        self.status.config(
            text=(
                f"  {nl.design_name}  |  "
                f"{len(nl.macros)} macros, {len(nl.components)} components, "
                f"{len(nl.io_pins)} I/O pins, {len(nl.nets)} nets  |  "
                f"Scroll=zoom, Drag=pan, F=fit all, Q=quit"
            )
        )

    # -- Event handlers --------------------------------------------------------

    def _on_drag_start(self, event) -> None:  # type: ignore[no-untyped-def]
        self.drag_x = event.x
        self.drag_y = event.y

    def _on_drag(self, event) -> None:  # type: ignore[no-untyped-def]
        dx = event.x - self.drag_x
        dy = event.y - self.drag_y
        self.scene.offset_x += dx
        self.scene.offset_y += dy
        self.drag_x = event.x
        self.drag_y = event.y
        self.tooltip.place_forget()
        self._draw()

    def _on_motion(self, event) -> None:  # type: ignore[no-untyped-def]
        # Find the topmost component item under the cursor and show name + type.
        items = self.canvas.find_overlapping(event.x, event.y, event.x, event.y)
        comp_items = self.scene.comp_items
        for item in reversed(items):
            comp = comp_items.get(item)
            if comp is not None:
                self.tooltip.config(
                    text=f"{comp.name}\n{comp.macro_name}",
                    justify="left",
                )
                self.tooltip.update_idletasks()
                tw = self.tooltip.winfo_reqwidth()
                th = self.tooltip.winfo_reqheight()
                cw = self.canvas.winfo_width()
                ch = self.canvas.winfo_height()
                x = event.x + 12
                y = event.y + 12
                if x + tw > cw:
                    x = event.x - 12 - tw
                if y + th > ch:
                    y = event.y - 12 - th
                self.tooltip.place(x=x, y=y)
                return
        self.tooltip.place_forget()

    def _on_leave(self, event) -> None:  # type: ignore[no-untyped-def]
        self.tooltip.place_forget()

    def _on_scroll(self, event) -> None:  # type: ignore[no-untyped-def]
        if event.delta > 0:
            self._zoom(1.2, event.x, event.y)
        else:
            self._zoom(1 / 1.2, event.x, event.y)

    def _on_scroll_up(self, event) -> None:  # type: ignore[no-untyped-def]
        self._zoom(1.2, event.x, event.y)

    def _on_scroll_down(self, event) -> None:  # type: ignore[no-untyped-def]
        self._zoom(1 / 1.2, event.x, event.y)

    def _zoom(self, factor: float, cx: int, cy: int) -> None:
        self.scene.offset_x = cx - factor * (cx - self.scene.offset_x)
        self.scene.offset_y = cy - factor * (cy - self.scene.offset_y)
        self.scene.scale *= factor
        self._draw()

    def _on_first_map(self, event) -> None:  # type: ignore[no-untyped-def]
        self.canvas.unbind("<Map>")
        self.scene.fit(self.canvas.winfo_width(), self.canvas.winfo_height())
        self._draw()

    def _on_fit_all(self, event) -> None:  # type: ignore[no-untyped-def]
        self.scene.fit(self.canvas.winfo_width(), self.canvas.winfo_height())
        self._draw()

    def _on_resize(self, event) -> None:  # type: ignore[no-untyped-def]
        pass

    def show(self) -> None:
        self.root.mainloop()


# -- CLI ----------------------------------------------------------------------


def _usage_and_exit() -> None:
    print(f"Usage: {sys.argv[0]} <netlist.json>")
    print(f"       {sys.argv[0]} --png <out.png> <netlist.json>")
    print(f"  e.g. {sys.argv[0]} DMA.json")
    print(f"  e.g. {sys.argv[0]} --png DMA-final.png DMA-final.json")
    sys.exit(1)


def main() -> None:
    args = sys.argv[1:]
    if not args:
        _usage_and_exit()

    if args[0] == "--png":
        if len(args) != 3:
            _usage_and_exit()
        png_path = args[1]
        json_path = args[2]
        netlist = load_netlist(json_path)
        print(f"Design: {netlist.design_name}")
        print(f"  {len(netlist.macros)} macros, {len(netlist.components)} components, "
              f"{len(netlist.io_pins)} I/O pins, {len(netlist.nets)} nets")
        render_to_png(netlist, png_path)
        print(f"  Wrote {png_path}")
        return

    if len(args) != 1:
        _usage_and_exit()
    netlist = load_netlist(args[0])
    print(f"Design: {netlist.design_name}")
    print(f"  {len(netlist.macros)} macros, {len(netlist.components)} components, "
          f"{len(netlist.io_pins)} I/O pins, {len(netlist.nets)} nets")
    viewer = PlacementViewer(netlist)
    viewer.show()


if __name__ == "__main__":
    main()
