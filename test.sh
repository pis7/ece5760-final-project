#!/bin/bash

verilator -Wall --lint-only -Ihw -Itest SPMV_tb.v
verilator -Wall --lint-only -Ihw -Itest MemController_tb.sv

verilator --binary -j 0 -Ihw -Itest SPMV_tb.v && ./obj_dir/VSPMV_tb
verilator --binary -j 0 -Ihw -Itest MemController_tb.sv && ./obj_dir/VMemController_tb
