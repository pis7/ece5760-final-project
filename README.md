# ECE 5760 Final Project - Analytical Placement via FPGA

This project seeks to accelerate analytical placement algorithms via FPGA.
Baseline code is first written in Python in `sw-baseline-python` and then
translated to C in `sw-baseline-c` to run on the DE0 Cyclone V FPGA board's ARM
processor. Finally, a Verilog implementation executes the kernel of the
algorithm in hardware while using the ARM processor for high-level
orchestration.
