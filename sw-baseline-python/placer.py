"""Analytical global placer using quadratic wirelength and recursive bisection.

Algorithm (see docs-local/ece5760-final-project-diagram.png):
  HPS outer loop:
    1. Parse netlist (LEF + DEF)
    2. Build connectivity matrix Q and support vectors c
    3. Solve Qx = -c via conjugate gradient
    4. Partition design, update Q and anchor points
    5. Check overlap -- if not acceptable, go to 3
  CG inner loop (step 3):
    1. Sparse matrix-vector multiply (Q * d)
    2. Compute step size alpha (dot products)
    3. Update cell positions x and residual r
    4. Update search direction d
    5. Check convergence
"""

import glob
import os
import shutil
import sys
sys.path.insert(0, sys.path[0] + "/../design-file-tools")

from parser import DefParser, LefParser


def find_lef_def(directory: str) -> tuple[str, str]:
    """Find the LEF and DEF files in an ICCAD04-style benchmark directory."""
    lef_files = glob.glob(f"{directory}/lef/*.lef")
    def_files = glob.glob(f"{directory}/def/*.def")
    if not lef_files:
        print(f"Error: no .lef file found in {directory}/lef/")
        sys.exit(1)
    if not def_files:
        print(f"Error: no .def file found in {directory}/def/")
        sys.exit(1)
    return lef_files[0], def_files[0]


class Placer:
    """Analytical global placement executor.

    All intermediate data is stored as instance variables so each step
    can reference results from previous steps.
    """

    def __init__(self, input_dir: str) -> None:
        self.input_dir: str = input_dir
        self.design_name: str = os.path.basename(os.path.normpath(input_dir))
        self.lef_path: str
        self.def_path: str
        self.lef_path, self.def_path = find_lef_def(input_dir)

        # Step 1 outputs
        self.lef: LefParser | None = None
        self.defn: DefParser | None = None

    # -- Step 1: Parse netlist -------------------------------------------------

    def parse_netlist(self) -> None:
        """Parse LEF and DEF files into self.lef and self.defn."""
        self.lef = LefParser(self.lef_path)
        self.defn = DefParser(self.def_path)

    # -- Step 2: Build connectivity matrix and support vectors -----------------

    def build_system(self) -> None:
        raise NotImplementedError

    # -- Step 3: Conjugate gradient solver -------------------------------------

    def solve_cg(self) -> None:
        raise NotImplementedError

    # -- Step 4: Partition and update anchors ----------------------------------

    def partition_and_anchor(self) -> None:
        raise NotImplementedError

    # -- Step 5: Check overlap acceptance --------------------------------------

    def check_overlap(self) -> None:
        raise NotImplementedError

    # -- Output ----------------------------------------------------------------

    def write_output(self) -> str:
        """Write placement results to ./<design_name>/ directory.

        Copies the original LEF file unchanged and writes a new DEF file
        with updated component positions. Returns the output directory path.
        """
        assert self.defn is not None
        assert self.lef is not None

        out_dir = self.design_name
        os.makedirs(f"{out_dir}/lef", exist_ok=True)
        os.makedirs(f"{out_dir}/def", exist_ok=True)

        # Copy LEF unchanged
        lef_basename = os.path.basename(self.lef_path)
        shutil.copy2(self.lef_path, f"{out_dir}/lef/{lef_basename}")

        # Write DEF with updated positions
        def_basename = os.path.basename(self.def_path)
        self._write_def(f"{out_dir}/def/{def_basename}")

        return out_dir

    def _write_def(self, path: str) -> None:
        """Write a DEF file with current component positions.

        Reads the original DEF and replaces component coordinates with
        the current values from self.defn.components.
        """
        assert self.defn is not None

        with open(self.def_path) as fin, open(path, "w") as fout:
            in_components = False

            for line in fin:
                toks = line.split()

                if not in_components:
                    if toks and toks[0] == "COMPONENTS":
                        in_components = True
                    fout.write(line)
                else:
                    # Inside COMPONENTS section
                    if toks and toks[0] == "END" and len(toks) > 1 and toks[1] == "COMPONENTS":
                        fout.write(line)
                        in_components = False
                    elif toks and toks[0] == "-":
                        # Component line: - name macro + PLACED ( x y ) orient ;
                        comp_name = toks[1]
                        comp = self.defn.components.get(comp_name)
                        if comp is not None:
                            fout.write(
                                f"- {comp.name} {comp.macro_name}"
                                f" + PLACED ( {int(comp.x)} {int(comp.y)} ) N ;\n"
                            )
                        else:
                            fout.write(line)
                    else:
                        fout.write(line)



# -- Entry point ---------------------------------------------------------------


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <benchmark_dir>")
        print(f"  e.g. {sys.argv[0]} benchmarks/iccad04/DMA")
        sys.exit(1)

    placer = Placer(sys.argv[1])

    # Step 1: Parse netlist
    placer.parse_netlist()

    # TODO(PASSTHROUGH): Steps 2-5 are skipped -- positions pass through
    # unchanged. Replace this section with:
    #   placer.build_system()
    #   placer.solve_cg()
    #   for each outer iteration:
    #       placer.partition_and_anchor()
    #       placer.solve_cg()
    #       if placer.check_overlap(): break

    # Write output
    out_dir = placer.write_output()
    print(f"Output written to {out_dir}/")


if __name__ == "__main__":
    main()
