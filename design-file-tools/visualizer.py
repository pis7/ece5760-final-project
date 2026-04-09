"""Visualize placement from a netlist JSON file using Tk.

Usage: uv run design-file-tools/visualizer.py <netlist.json>
  e.g. cd build && uv run ../design-file-tools/visualizer.py DMA.json

Controls:
  - Scroll wheel: zoom in/out
  - Click + drag: pan
  - F: fit all
"""

import os
import sys
import tkinter as tk

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from json_utils import Netlist, load_netlist


class PlacementViewer:
    """Tk canvas viewer for netlist placement data with pan and zoom."""

    CANVAS_W = 1200
    CANVAS_H = 900
    PAD = 40  # pixel padding around die area

    def __init__(self, netlist: Netlist) -> None:
        self.netlist: Netlist = netlist
        self.dbu_per_micron: int = netlist.dbu_per_micron

        # Die bounds in DBU
        self.die_xl: float = netlist.die_area[0]
        self.die_yl: float = netlist.die_area[1]
        self.die_xh: float = netlist.die_area[2]
        self.die_yh: float = netlist.die_area[3]

        # Precompute die area for large-macro filtering
        die_w = self.die_xh - self.die_xl
        die_h = self.die_yh - self.die_yl
        self.die_area_dbu2: float = die_w * die_h

        # View transform state
        self.scale: float = 1.0
        self.offset_x: float = 0.0
        self.offset_y: float = 0.0
        self.drag_x: int = 0
        self.drag_y: int = 0

        # Build GUI
        self.root: tk.Tk = tk.Tk()
        self.root.title(f"Placement: {netlist.design_name}")

        # Toolbar
        toolbar: tk.Frame = tk.Frame(self.root, bg="#333")
        toolbar.pack(fill=tk.X)
        self.hide_large_var: tk.BooleanVar = tk.BooleanVar(value=False)
        tk.Checkbutton(
            toolbar, text="Hide large macros (>2% die area)",
            variable=self.hide_large_var, command=self._draw,
            bg="#333", fg="white", selectcolor="#555",
            activebackground="#444", activeforeground="white",
        ).pack(side=tk.LEFT, padx=5, pady=2)

        self.canvas: tk.Canvas = tk.Canvas(
            self.root, width=self.CANVAS_W, height=self.CANVAS_H, bg="black"
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Status bar
        self.status: tk.Label = tk.Label(
            self.root, text="", anchor=tk.W, bg="#222", fg="white"
        )
        self.status.pack(fill=tk.X)

        # Bind events
        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<MouseWheel>", self._on_scroll)
        self.canvas.bind("<Button-4>", self._on_scroll_up)
        self.canvas.bind("<Button-5>", self._on_scroll_down)
        self.root.bind("<f>", self._on_fit_all)
        self.root.bind("<q>", lambda e: self.root.destroy())
        self.root.bind("<Configure>", self._on_resize)

        self._update_status()
        self.canvas.bind("<Map>", self._on_first_map)

    def _fit_view(self) -> None:
        """Set scale and offset to fit the die area in the canvas."""
        die_w = self.die_xh - self.die_xl
        die_h = self.die_yh - self.die_yl
        if die_w <= 0 or die_h <= 0:
            return
        cw = self.canvas.winfo_width() or self.CANVAS_W
        ch = self.canvas.winfo_height() or self.CANVAS_H
        sx = (cw - 2 * self.PAD) / die_w
        sy = (ch - 2 * self.PAD) / die_h
        self.scale = min(sx, sy)
        self.offset_x = (cw - die_w * self.scale) / 2 - self.die_xl * self.scale
        self.offset_y = (ch - die_h * self.scale) / 2 + self.die_yh * self.scale

    def _to_canvas(self, dbu_x: float, dbu_y: float) -> tuple[float, float]:
        """Convert DEF database units to canvas pixel coordinates."""
        cx = dbu_x * self.scale + self.offset_x
        cy = -dbu_y * self.scale + self.offset_y
        return cx, cy

    def _draw(self) -> None:
        """Redraw everything on the canvas."""
        self.canvas.delete("all")
        self._draw_die()
        self._draw_components()
        self._draw_io_pins()
        self._draw_legend()

    def _draw_die(self) -> None:
        """Draw the die area outline."""
        x0, y0 = self._to_canvas(self.die_xl, self.die_yl)
        x1, y1 = self._to_canvas(self.die_xh, self.die_yh)
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="#555", width=2)

    def _draw_components(self) -> None:
        """Draw all placed components as rectangles sized by their macro."""
        nl = self.netlist
        hide_large = self.hide_large_var.get()
        area_threshold = self.die_area_dbu2 * 0.02
        dbu = self.dbu_per_micron
        for comp in nl.components.values():
            macro = nl.macros.get(comp.macro_name)
            if macro is None:
                continue
            w = macro.width * dbu
            h = macro.height * dbu
            if hide_large and w * h > area_threshold:
                continue
            x0, y0 = self._to_canvas(comp.x, comp.y)
            x1, y1 = self._to_canvas(comp.x + w, comp.y + h)
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill="#4080c0", outline=""
            )

    def _draw_io_pins(self) -> None:
        """Draw I/O pins as small circles."""
        die_size = min(self.die_xh - self.die_xl, self.die_yh - self.die_yl)
        r = max(3, die_size * 0.008 * self.scale)
        for pin in self.netlist.io_pins:
            cx, cy = self._to_canvas(pin.x, pin.y)
            self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r, fill="#e0e040", outline=""
            )

    def _draw_legend(self) -> None:
        """Draw a color legend in the top-right corner."""
        items = [
            ("#4080c0", "Macro"),
            ("#e0e040", "I/O pin"),
        ]
        x = self.canvas.winfo_width() - 10
        y = 10
        for color, label in items:
            self.canvas.create_rectangle(
                x - 185, y, x - 170, y + 12, fill=color, outline=""
            )
            self.canvas.create_text(
                x - 165, y + 6, text=label, anchor=tk.W,
                fill="white", font=("TkDefaultFont", 9)
            )
            y += 18

    def _update_status(self) -> None:
        """Update the status bar text."""
        nl = self.netlist
        self.status.config(
            text=(
                f"  {nl.design_name}  |  "
                f"{len(nl.macros)} macros, {len(nl.components)} components, "
                f"{len(nl.io_pins)} I/O pins, {len(nl.nets)} nets  |  "
                f"Scroll=zoom, Drag=pan, F=fit all, Q=quit"
            )
        )

    # -- Event handlers --------------------------------------------------------

    def _on_drag_start(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        self.drag_x = event.x
        self.drag_y = event.y

    def _on_drag(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        dx = event.x - self.drag_x
        dy = event.y - self.drag_y
        self.offset_x += dx
        self.offset_y += dy
        self.drag_x = event.x
        self.drag_y = event.y
        self._draw()

    def _on_scroll(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        if event.delta > 0:
            self._zoom(1.2, event.x, event.y)
        else:
            self._zoom(1 / 1.2, event.x, event.y)

    def _on_scroll_up(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        self._zoom(1.2, event.x, event.y)

    def _on_scroll_down(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        self._zoom(1 / 1.2, event.x, event.y)

    def _zoom(self, factor: float, cx: int, cy: int) -> None:
        self.offset_x = cx - factor * (cx - self.offset_x)
        self.offset_y = cy - factor * (cy - self.offset_y)
        self.scale *= factor
        self._draw()

    def _on_first_map(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        self.canvas.unbind("<Map>")
        self._fit_view()
        self._draw()

    def _on_fit_all(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        self._fit_view()
        self._draw()

    def _on_resize(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        pass

    def show(self) -> None:
        """Enter the Tk main loop."""
        self.root.mainloop()


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <netlist.json>")
        print(f"  e.g. {sys.argv[0]} DMA.json")
        sys.exit(1)

    netlist = load_netlist(sys.argv[1])

    print(f"Design: {netlist.design_name}")
    print(f"  {len(netlist.macros)} macros, {len(netlist.components)} components, "
          f"{len(netlist.io_pins)} I/O pins, {len(netlist.nets)} nets")

    viewer = PlacementViewer(netlist)
    viewer.show()


if __name__ == "__main__":
    main()
