cmake_minimum_required(VERSION 3.14)
project(placer_fpga LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Always use the FPGA CG driver (mmap over /dev/mem).
option(STATIC_BUILD "Build with static linking" OFF)
set(INT_BITS  "13" CACHE STRING "Fixed-point integer bits (must match Verilog p_int_bits). Default 13.")
set(FRAC_BITS "14" CACHE STRING "Fixed-point fractional bits (must match Verilog p_frac_bits). Default 14.")
set(MAX_N "50" CACHE STRING "Maximum cell count the mmap driver supports (must match Verilog p_max_n)")

# The FPGA bitstream is locked to 27-bit Q-format -- reject wider knobs
# at configure time so users get a clear error instead of a silent
# mismatch with the synthesized design.
math(EXPR FPGA_TOTAL_BITS "${INT_BITS} + ${FRAC_BITS}")
if(NOT FPGA_TOTAL_BITS EQUAL 27)
    message(FATAL_ERROR
        "INT_BITS + FRAC_BITS = ${FPGA_TOTAL_BITS} but the FPGA bitstream is "
        "fixed at 27-bit. Wider widths are only supported in the verilated / "
        "golden modes (sw-baseline-c).")
endif()
set(HW_FPGA_VERSION "v6" CACHE STRING "Which mmap driver to wire up: v4 (single SRAM), v5 (multi-block on-chip RAM), or v6 (parallel x/y solve datapaths)")
set_property(CACHE HW_FPGA_VERSION PROPERTY STRINGS v4 v5 v6)

# v4 driver lives at the bare CG_DRIVER_FPGA_MMAP macro; v5 / v6 each
# pick their own header via dedicated macros.
if(HW_FPGA_VERSION STREQUAL "v6")
    set(FPGA_MMAP_DEFINE "CG_DRIVER_FPGA_MMAP_V6")
elseif(HW_FPGA_VERSION STREQUAL "v5")
    set(FPGA_MMAP_DEFINE "CG_DRIVER_FPGA_MMAP_V5")
else()
    set(FPGA_MMAP_DEFINE "CG_DRIVER_FPGA_MMAP")
endif()

add_executable(placer placer.cpp)
target_compile_definitions(placer PRIVATE USE_HW_CG ${FPGA_MMAP_DEFINE} HW_FRAC_BITS=${FRAC_BITS} HW_MAX_N=${MAX_N})

# Share json.hpp with the software baseline to avoid duplicating ~1 MB
# of vendored header.
target_include_directories(placer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../sw-baseline-c
)

target_compile_options(placer PRIVATE -Wall -Wextra -Wno-psabi -Wno-sign-compare)

if(STATIC_BUILD)
    target_link_options(placer PRIVATE -static)
endif()
