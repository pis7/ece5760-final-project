#!/bin/bash

verilator -Wall --lint-only -Ihw -Itest SPMV_tb.v
verilator --binary -j 0 -Ihw -Itest SPMV_tb.v && ./obj_dir/VSPMV_tb

