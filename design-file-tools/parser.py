"""Minimal LEF/DEF parsers for analytical global placement.

Only extracts data needed for quadratic placement with a cell-center model:
  - LEF: macro names, sizes, and pin names
  - DEF: die area, component instances + positions, I/O pin positions, net connectivity
"""

from dataclasses import dataclass, field
from io import TextIOWrapper


# -- LEF data classes ----------------------------------------------------------


@dataclass
class Macro:
    name: str
    width: float  # database units
    height: float  # database units
    pins: list[str] = field(default_factory=list)


# -- DEF data classes ----------------------------------------------------------


@dataclass
class Component:
    name: str  # instance name, e.g. "glog/U13"
    macro_name: str  # LEF macro, e.g. "MAS2"
    x: float  # placement x (database units)
    y: float  # placement y (database units)


@dataclass
class IOPin:
    name: str  # e.g. "CMAinx[13]"
    net_name: str  # net it belongs to
    x: float  # placed x (database units)
    y: float  # placed y (database units)


@dataclass
class Net:
    name: str
    pins: list[tuple[str, str]]  # [(component_name, pin_name), ...]
    # component_name is "PIN" for top-level I/O pins


# -- Helpers -------------------------------------------------------------------


def _read_tokens(fin: TextIOWrapper) -> list[str]:
    """Read the next non-empty line and return whitespace-split tokens."""
    while True:
        line = fin.readline()
        if not line:
            return []
        toks = line.split()
        if toks:
            return toks


def _read_statement(fin: TextIOWrapper) -> list[str]:
    """Read tokens until a semicolon-terminated statement is complete."""
    toks = _read_tokens(fin)
    while toks and toks[-1] != ";":
        toks += _read_tokens(fin)
    return toks


def _skip_block(fin: TextIOWrapper, end_keyword: str) -> None:
    """Skip lines until we see 'END <end_keyword>'."""
    while True:
        toks = _read_tokens(fin)
        if not toks:
            return
        if toks[0] == "END" and len(toks) > 1 and toks[1] == end_keyword:
            return


# -- LEF Parser ----------------------------------------------------------------


class LefParser:
    """Parse a LEF file, extracting only macro definitions (name, size, pin names)."""

    def __init__(self, path: str) -> None:
        self.dbu_per_micron: int = 100
        self.macros: dict[str, Macro] = {}
        self._parse(path)

    def _parse(self, path: str) -> None:
        with open(path) as fin:
            while True:
                toks = _read_tokens(fin)
                if not toks:
                    return
                keyword = toks[0]
                if keyword == "END" and len(toks) > 1 and toks[1] == "LIBRARY":
                    return
                elif keyword == "UNITS":
                    self._parse_units(fin)
                elif keyword == "MACRO":
                    macro = self._parse_macro(fin, toks[1])
                    self.macros[macro.name] = macro
                elif keyword in (
                    "LAYER", "VIA", "VIARULE", "SITE", "SPACING",
                ):
                    _skip_block(fin, toks[1] if len(toks) > 1 else keyword)
                # Skip single-line statements (VERSION, NAMESCASESENSITIVE, etc.)

    def _parse_units(self, fin: TextIOWrapper) -> None:
        while True:
            toks = _read_tokens(fin)
            if not toks:
                return
            if toks[0] == "END":
                return
            if toks[0] == "DATABASE" and toks[1] == "MICRONS":
                self.dbu_per_micron = int(toks[2])

    def _parse_macro(self, fin: TextIOWrapper, name: str) -> Macro:
        width = height = 0.0
        pins: list[str] = []
        while True:
            toks = _read_tokens(fin)
            if not toks:
                break
            if toks[0] == "END" and len(toks) > 1 and toks[1] == name:
                break
            elif toks[0] == "SIZE":
                width = float(toks[1])
                height = float(toks[3])  # SIZE w BY h ;
            elif toks[0] == "PIN":
                pin_name = toks[1]
                pins.append(pin_name)
                _skip_block(fin, pin_name)
            # Skip CLASS, FOREIGN, ORIGIN, SYMMETRY, SITE, OBS, etc.
        return Macro(name, width, height, pins)


# -- DEF Parser ----------------------------------------------------------------


class DefParser:
    """Parse a DEF file, extracting components, I/O pins, and nets."""

    def __init__(self, path: str) -> None:
        self.design_name: str = ""
        self.dbu_per_micron: int = 100
        self.die_area: tuple[float, float, float, float] = (0, 0, 0, 0)
        self.components: dict[str, Component] = {}
        self.io_pins: list[IOPin] = []
        self.nets: list[Net] = []
        self._parse(path)

    def _parse(self, path: str) -> None:
        with open(path) as fin:
            while True:
                toks = _read_tokens(fin)
                if not toks:
                    return
                keyword = toks[0]
                if keyword == "END" and len(toks) > 1 and toks[1] == "DESIGN":
                    return
                elif keyword == "DESIGN":
                    self.design_name = toks[1]
                elif keyword == "UNITS":
                    self.dbu_per_micron = int(toks[3])  # UNITS DISTANCE MICRONS n ;
                elif keyword == "DIEAREA":
                    # DIEAREA ( x1 y1 ) ( x2 y2 ) ;
                    self.die_area = (
                        float(toks[2]),
                        float(toks[3]),
                        float(toks[6]),
                        float(toks[7]),
                    )
                elif keyword == "COMPONENTS":
                    self._parse_components(fin, int(toks[1]))
                elif keyword == "PINS":
                    self._parse_pins(fin, int(toks[1]))
                elif keyword == "NETS":
                    self._parse_nets(fin, int(toks[1]))
                elif keyword == "PROPERTYDEFINITIONS":
                    _skip_block(fin, "PROPERTYDEFINITIONS")
                # Skip ROW, TRACKS, GCELLGRID, VIAS, VERSION, etc.

    def _parse_components(self, fin: TextIOWrapper, count: int) -> None:
        for _ in range(count):
            toks = _read_statement(fin)
            # - inst_name macro_name + PLACED/FIXED ( x y ) orient ;
            name = toks[1]
            macro_name = toks[2]
            # Find the coordinates after '('
            paren_idx = toks.index("(")
            x = float(toks[paren_idx + 1])
            y = float(toks[paren_idx + 2])
            self.components[name] = Component(name, macro_name, x, y)
        _read_tokens(fin)  # END COMPONENTS

    def _parse_pins(self, fin: TextIOWrapper, count: int) -> None:
        for _ in range(count):
            toks = _read_statement(fin)
            # - pin_name + NET net_name + ... + PLACED ( x y ) orient ;
            pin_name = toks[1]
            # Extract NET name
            net_name = pin_name  # default: net name matches pin name
            if "NET" in toks:
                net_name = toks[toks.index("NET") + 1]
            # Extract placement coordinates if present
            if "PLACED" in toks:
                placed_idx = toks.index("PLACED")
                # PLACED ( x y ) orient
                x = float(toks[placed_idx + 2])
                y = float(toks[placed_idx + 3])
                self.io_pins.append(IOPin(pin_name, net_name, x, y))
            # Skip pins without placement (e.g. VSS/VDD)
        _read_tokens(fin)  # END PINS

    def _parse_nets(self, fin: TextIOWrapper, count: int) -> None:
        for _ in range(count):
            toks = _read_statement(fin)
            # - net_name ( comp pin ) ( comp pin ) ... ;
            net_name = toks[1]
            pins: list[tuple[str, str]] = []
            idx = 2
            while idx < len(toks) - 1:  # stop before ";"
                if toks[idx] == "(":
                    comp = toks[idx + 1]
                    pin = toks[idx + 2]
                    pins.append((comp, pin))
                    idx += 4  # skip ( comp pin )
                else:
                    idx += 1
            self.nets.append(Net(net_name, pins))
        _read_tokens(fin)  # END NETS
