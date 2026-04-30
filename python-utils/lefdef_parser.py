"""Parse LEF/DEF files into a unified netlist JSON file.

Usage (from build directory):
  uv run lefdef-parser ../benchmarks/iccad04/DMA
  # Outputs <design_name>.json in the current directory.
"""

import glob
import sys
from io import TextIOWrapper

from json_utils import (
    Component,
    IOPin,
    Macro,
    Net,
    Netlist,
    dump_netlist,
)


class LefDefParser:
    """Parse LEF and DEF files and serialize to a netlist JSON file."""

    def __init__(self, benchmark_dir: str) -> None:
        self.benchmark_dir: str = benchmark_dir
        self.lef_path: str = ""
        self.def_path: str = ""

        # LEF data
        self.dbu_per_micron: int = 100
        self.macros: dict[str, Macro] = {}

        # DEF data
        self.design_name: str = ""
        self.die_area: tuple[float, float, float, float] = (0, 0, 0, 0)
        self.components: dict[str, Component] = {}
        self.io_pins: list[IOPin] = []
        self.nets: list[Net] = []

    # -- File discovery ----------------------------------------------------

    def find_files(self) -> None:
        """Find LEF and DEF files in the benchmark directory."""
        lef_files = glob.glob(f"{self.benchmark_dir}/lef/*.lef")
        def_files = glob.glob(f"{self.benchmark_dir}/def/*.def")
        if not lef_files:
            print(f"Error: no .lef file found in {self.benchmark_dir}/lef/")
            sys.exit(1)
        if not def_files:
            print(f"Error: no .def file found in {self.benchmark_dir}/def/")
            sys.exit(1)
        self.lef_path = lef_files[0]
        self.def_path = def_files[0]
        print(f"LEF: {self.lef_path}")
        print(f"DEF: {self.def_path}")

    # -- Token helpers -----------------------------------------------------

    @staticmethod
    def _read_tokens(fin: TextIOWrapper) -> list[str]:
        """Read the next non-empty line and return whitespace-split tokens."""
        while True:
            line = fin.readline()
            if not line:
                return []
            toks = line.split()
            if toks:
                return toks

    @staticmethod
    def _read_statement(fin: TextIOWrapper) -> list[str]:
        """Read tokens until a semicolon-terminated statement is complete."""
        toks = LefDefParser._read_tokens(fin)
        while toks and toks[-1] != ";":
            toks += LefDefParser._read_tokens(fin)
        return toks

    @staticmethod
    def _skip_block(fin: TextIOWrapper, end_keyword: str) -> None:
        """Skip lines until we see 'END <end_keyword>'."""
        while True:
            toks = LefDefParser._read_tokens(fin)
            if not toks:
                return
            if toks[0] == "END" and len(toks) > 1 and toks[1] == end_keyword:
                return

    # -- LEF parsing -------------------------------------------------------

    def _parse_lef_units(self, fin: TextIOWrapper) -> None:
        while True:
            toks = self._read_tokens(fin)
            if not toks or toks[0] == "END":
                return
            if toks[0] == "DATABASE" and toks[1] == "MICRONS":
                self.dbu_per_micron = int(toks[2])

    def _parse_lef_macro(self, fin: TextIOWrapper, name: str) -> None:
        width = height = 0.0
        pins: list[str] = []
        while True:
            toks = self._read_tokens(fin)
            if not toks:
                break
            if toks[0] == "END" and len(toks) > 1 and toks[1] == name:
                break
            elif toks[0] == "SIZE":
                width = float(toks[1])
                height = float(toks[3])
            elif toks[0] == "PIN":
                pin_name = toks[1]
                pins.append(pin_name)
                self._skip_block(fin, pin_name)
        self.macros[name] = Macro(name, width, height, pins)

    def parse_lef(self) -> None:
        """Parse the LEF file, extracting macro definitions."""
        with open(self.lef_path) as fin:
            while True:
                toks = self._read_tokens(fin)
                if not toks:
                    break
                keyword = toks[0]
                if keyword == "END" and len(toks) > 1 and toks[1] == "LIBRARY":
                    break
                elif keyword == "UNITS":
                    self._parse_lef_units(fin)
                elif keyword == "MACRO":
                    self._parse_lef_macro(fin, toks[1])
                elif keyword in (
                    "LAYER", "VIA", "VIARULE", "SITE", "SPACING",
                ):
                    self._skip_block(
                        fin, toks[1] if len(toks) > 1 else keyword
                    )

    # -- DEF parsing -------------------------------------------------------

    def _parse_def_components(self, fin: TextIOWrapper, count: int) -> None:
        for _ in range(count):
            toks = self._read_statement(fin)
            name = toks[1]
            macro_name = toks[2]
            paren_idx = toks.index("(")
            x = float(toks[paren_idx + 1])
            y = float(toks[paren_idx + 2])
            self.components[name] = Component(name, macro_name, x, y)
        self._read_tokens(fin)  # END COMPONENTS

    def _parse_def_pins(self, fin: TextIOWrapper, count: int) -> None:
        for _ in range(count):
            toks = self._read_statement(fin)
            pin_name = toks[1]
            net_name = pin_name
            if "NET" in toks:
                net_name = toks[toks.index("NET") + 1]
            if "PLACED" in toks:
                placed_idx = toks.index("PLACED")
                x = float(toks[placed_idx + 2])
                y = float(toks[placed_idx + 3])
                self.io_pins.append(IOPin(pin_name, net_name, x, y))
        self._read_tokens(fin)  # END PINS

    def _parse_def_nets(self, fin: TextIOWrapper, count: int) -> None:
        for _ in range(count):
            toks = self._read_statement(fin)
            net_name = toks[1]
            pins: list[tuple[str, str]] = []
            idx = 2
            while idx < len(toks) - 1:
                if toks[idx] == "(":
                    pins.append((toks[idx + 1], toks[idx + 2]))
                    idx += 4
                else:
                    idx += 1
            self.nets.append(Net(net_name, pins))
        self._read_tokens(fin)  # END NETS

    def parse_def(self) -> None:
        """Parse the DEF file, extracting components, I/O pins, and nets."""
        with open(self.def_path) as fin:
            while True:
                toks = self._read_tokens(fin)
                if not toks:
                    break
                keyword = toks[0]
                if keyword == "END" and len(toks) > 1 and toks[1] == "DESIGN":
                    break
                elif keyword == "DESIGN":
                    self.design_name = toks[1]
                elif keyword == "UNITS":
                    self.dbu_per_micron = int(toks[3])
                elif keyword == "DIEAREA":
                    self.die_area = (
                        float(toks[2]), float(toks[3]),
                        float(toks[6]), float(toks[7]),
                    )
                elif keyword == "COMPONENTS":
                    self._parse_def_components(fin, int(toks[1]))
                elif keyword == "PINS":
                    self._parse_def_pins(fin, int(toks[1]))
                elif keyword == "NETS":
                    self._parse_def_nets(fin, int(toks[1]))
                elif keyword == "PROPERTYDEFINITIONS":
                    self._skip_block(fin, "PROPERTYDEFINITIONS")

    # -- JSON output -------------------------------------------------------

    def write_json(self) -> str:
        """Build a Netlist and write it to JSON. Returns the output path."""
        netlist = Netlist(
            design_name=self.design_name,
            dbu_per_micron=self.dbu_per_micron,
            die_area=self.die_area,
            macros=self.macros,
            components=self.components,
            io_pins=self.io_pins,
            nets=self.nets,
        )
        out_path = f"{self.design_name}.json"
        dump_netlist(netlist, out_path)
        return out_path


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <benchmark_dir>")
        print(f"  e.g. {sys.argv[0]} ../benchmarks/iccad04/DMA")
        sys.exit(1)

    parser = LefDefParser(sys.argv[1])
    parser.find_files()
    parser.parse_lef()
    parser.parse_def()
    out_path = parser.write_json()

    print(f"Design: {parser.design_name}")
    print(f"  {len(parser.macros)} macros, {len(parser.components)} components, "
          f"{len(parser.io_pins)} I/O pins, {len(parser.nets)} nets")
    print(f"  Written to {out_path}")


if __name__ == "__main__":
    main()
