"""Shared netlist dataclasses and JSON I/O for the project.

All tools (lefdef-parser, placer, visualizer) use this module for the
common Netlist data model and JSON serialization.
"""

import functools
import json
from dataclasses import dataclass, field
from typing import Any

import numpy as np


# -- Dataclasses ---------------------------------------------------------------


@dataclass
class Macro:
    name: str
    width: float  # microns
    height: float  # microns
    pins: list[str] = field(default_factory=list)


@dataclass
class Component:
    name: str
    macro_name: str
    x: float  # database units
    y: float  # database units


@dataclass
class IOPin:
    name: str
    net_name: str
    x: float  # database units
    y: float  # database units


@dataclass
class Net:
    name: str
    pins: list[tuple[str, str]]  # [(component_name, pin_name), ...]


@dataclass
class Netlist:
    design_name: str
    dbu_per_micron: int
    die_area: tuple[float, float, float, float]
    macros: dict[str, Macro]
    components: dict[str, Component]
    io_pins: list[IOPin]
    nets: list[Net]

    @property
    def num_cells(self) -> int:
        return len(self.components)

    @functools.cached_property
    def cell_names(self) -> list[str]:
        return list(self.components.keys())

    @functools.cached_property
    def cell_index(self) -> dict[str, int]:
        return {name: i for i, name in enumerate(self.cell_names)}

    @functools.cached_property
    def io_pin_map(self) -> dict[str, IOPin]:
        return {p.name: p for p in self.io_pins}

    @functools.cached_property
    def cell_widths(self) -> np.ndarray:
        dbu = self.dbu_per_micron
        w = np.zeros(self.num_cells)
        for i, name in enumerate(self.cell_names):
            macro = self.macros.get(self.components[name].macro_name)
            if macro is not None:
                w[i] = macro.width * dbu
        return w

    @functools.cached_property
    def cell_heights(self) -> np.ndarray:
        dbu = self.dbu_per_micron
        h = np.zeros(self.num_cells)
        for i, name in enumerate(self.cell_names):
            macro = self.macros.get(self.components[name].macro_name)
            if macro is not None:
                h[i] = macro.height * dbu
        return h

    @functools.cached_property
    def cell_areas(self) -> np.ndarray:
        return self.cell_widths * self.cell_heights

    @functools.cached_property
    def total_cell_area(self) -> float:
        return float(self.cell_areas.sum())


# -- JSON I/O ------------------------------------------------------------------


def load_netlist(path: str) -> Netlist:
    """Load a netlist JSON file into a Netlist object."""
    with open(path) as f:
        d = json.load(f)
    return Netlist(
        design_name=d["design_name"],
        dbu_per_micron=d["dbu_per_micron"],
        die_area=tuple(d["die_area"]),
        macros={
            name: Macro(name, m["width"], m["height"], m["pins"])
            for name, m in d["macros"].items()
        },
        components={
            name: Component(name, c["macro_name"], c["x"], c["y"])
            for name, c in d["components"].items()
        },
        io_pins=[
            IOPin(p["name"], p["net_name"], p["x"], p["y"])
            for p in d["io_pins"]
        ],
        nets=[
            Net(n["name"], [(pin[0], pin[1]) for pin in n["pins"]])
            for n in d["nets"]
        ],
    )


def dump_netlist(netlist: Netlist, path: str) -> None:
    """Serialize a Netlist to a JSON file."""
    data: dict[str, Any] = {
        "design_name": netlist.design_name,
        "dbu_per_micron": netlist.dbu_per_micron,
        "die_area": list(netlist.die_area),
        "macros": {
            name: {"width": m.width, "height": m.height, "pins": m.pins}
            for name, m in netlist.macros.items()
        },
        "components": {
            name: {"macro_name": c.macro_name, "x": c.x, "y": c.y}
            for name, c in netlist.components.items()
        },
        "io_pins": [
            {"name": p.name, "net_name": p.net_name, "x": p.x, "y": p.y}
            for p in netlist.io_pins
        ],
        "nets": [
            {"name": n.name, "pins": [list(pin) for pin in n.pins]}
            for n in netlist.nets
        ],
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
