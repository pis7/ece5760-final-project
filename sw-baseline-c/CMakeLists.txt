cmake_minimum_required(VERSION 3.14)
project(placer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Static linking for cross-compilation to DE1-SoC
option(STATIC_BUILD "Build with static linking" OFF)

# Use Verilator CGTop for CG solve instead of software CG
option(USE_HW_CG "Use Verilator CGTop for CG solve" OFF)
# Use C++ fixed-point golden model instead of Verilator (faster, no Verilog needed)
option(USE_FP_GOLDEN "Use C++ fixed-point golden model instead of Verilator" OFF)
set(INT_BITS  "13" CACHE STRING "Fixed-point integer bits (must match Verilog p_int_bits). Default 13.")
set(FRAC_BITS "14" CACHE STRING "Fixed-point fractional bits (must match Verilog p_frac_bits). Default 14. INT_BITS+FRAC_BITS<=64; >27 only allowed for golden / verilator targets.")
set(MAX_N     "50" CACHE STRING "Maximum cell count (Verilog p_max_n / driver storage). Default 50. Honored by golden / verilator only.")
set(HW_CG_VERSION "v6" CACHE STRING "Which CG solver to wire up: v2 (shift-subtract divide), v3 (NR divide + rp_lo/rp_hi collapse), v4 (v3 + parallel x/r AXPY), v5 (v4 + multi-block on-chip RAM), v5_deep (v5 + 1 cycle/nz pipelined SPMV), or v6 (v5 + parallel x/y solve datapaths)")
set_property(CACHE HW_CG_VERSION PROPERTY STRINGS v2 v3 v4 v5 v5_deep v6)

math(EXPR TOTAL_BITS "${INT_BITS} + ${FRAC_BITS}")
if(TOTAL_BITS GREATER 64)
    message(FATAL_ERROR "INT_BITS + FRAC_BITS (=${TOTAL_BITS}) exceeds 64.")
endif()
if(INT_BITS LESS 1 OR FRAC_BITS LESS 1)
    message(FATAL_ERROR "INT_BITS and FRAC_BITS must each be >= 1 (got ${INT_BITS}, ${FRAC_BITS}).")
endif()
# Wider widths only meaningful when we're emitting fixed-point math
# (golden or Verilator). Pure-double sw mode ignores the knob entirely
# and the FPGA bitstream is locked to 27-bit -- reject early either way.
if(TOTAL_BITS GREATER 27)
    if(NOT USE_FP_GOLDEN AND NOT USE_HW_CG)
        message(FATAL_ERROR
            "INT_BITS + FRAC_BITS = ${TOTAL_BITS} > 27 is only supported for "
            "USE_FP_GOLDEN=ON or USE_HW_CG=ON (verilated mode). The pure C++ "
            "sw placer uses doubles and the FPGA bitstream is locked to 27-bit.")
    endif()
endif()

# NR-divide variants need the NR DPI golden to stay bit-exact. v2 uses
# shift-subtract so it falls through to the empty define.
if(HW_CG_VERSION STREQUAL "v3" OR HW_CG_VERSION STREQUAL "v4" OR HW_CG_VERSION STREQUAL "v5" OR HW_CG_VERSION STREQUAL "v5_deep" OR HW_CG_VERSION STREQUAL "v6")
    set(CG_GOLDEN_DEFINE "CG_GOLDEN_USE_NR")
else()
    set(CG_GOLDEN_DEFINE "")
endif()

# Pick the right Verilator driver header. v5 / v5_deep share the
# 7-slave multi-block topology; v6 has 10 slaves (Q duplicated for
# parallel x/y solves).
if(HW_CG_VERSION STREQUAL "v6")
    set(CG_VERILATOR_DEFINE "CG_VERILATOR_V6")
elseif(HW_CG_VERSION STREQUAL "v5" OR HW_CG_VERSION STREQUAL "v5_deep")
    set(CG_VERILATOR_DEFINE "CG_VERILATOR_V5")
else()
    set(CG_VERILATOR_DEFINE "")
endif()

if(USE_FP_GOLDEN)
    # Pure C++ fixed-point CG — no Verilator dependency
    add_executable(placer placer.cpp)
    target_compile_definitions(placer PRIVATE USE_HW_CG USE_FP_GOLDEN CG_GOLDEN_FRAC_BITS=${FRAC_BITS} CG_GOLDEN_TOTAL_BITS=${TOTAL_BITS} HW_MAX_N=${MAX_N} ${CG_GOLDEN_DEFINE})
elseif(USE_HW_CG)
    # HW_CG_VERSION selects which RTL gets verilated. v3 is the current
    # target (rp collapse + NR divide + 16 hardened tests). v2 stays for
    # reference / regression-testing the original shift-subtract design.
    # v1 was combinational-only reference and is not wired up here.
    set(VERILOG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../fpga/hw/${HW_CG_VERSION}")
    set(VDIR "${CMAKE_CURRENT_BINARY_DIR}/vobj")
    set(VERILATOR_ROOT "/usr/local/share/verilator")

    # v6 wraps CGCtrl + CGDpath in a CGEngine module (instantiated twice
    # by CGTop). Older versions don't have this file.
    set(EXTRA_RTL "")
    if(EXISTS "${VERILOG_DIR}/CGEngine.v")
      list(APPEND EXTRA_RTL "${VERILOG_DIR}/CGEngine.v")
    endif()

    # Build the Verilator model as a static library
    add_custom_command(
      OUTPUT ${VDIR}/VCGTop__ALL.a
      COMMAND verilator --cc --build
        -Wall -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL
        -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-DECLFILENAME
        --top-module CGTop
        --Mdir ${VDIR}
        --prefix VCGTop
        -Gp_int_bits=${INT_BITS}
        -Gp_frac_bits=${FRAC_BITS}
        -Gp_max_n=${MAX_N}
        ${VERILOG_DIR}/CGTop.v
        ${VERILOG_DIR}/CGCtrl.v
        ${VERILOG_DIR}/CGDpath.v
        ${VERILOG_DIR}/FpMath.v
        ${VERILOG_DIR}/LinAlg.v
        ${EXTRA_RTL}
        -CFLAGS "-std=c++17"
      DEPENDS
        ${VERILOG_DIR}/CGTop.v
        ${VERILOG_DIR}/CGCtrl.v
        ${VERILOG_DIR}/CGDpath.v
        ${VERILOG_DIR}/FpMath.v
        ${VERILOG_DIR}/LinAlg.v
        ${EXTRA_RTL}
      COMMENT "Verilating CGTop (${HW_CG_VERSION}, p_int_bits=${INT_BITS}, p_frac_bits=${FRAC_BITS}, p_max_n=${MAX_N})"
    )
    add_custom_target(verilator_model DEPENDS ${VDIR}/VCGTop__ALL.a)

    add_executable(placer
      placer.cpp
      ${VERILATOR_ROOT}/include/verilated.cpp
      ${VERILATOR_ROOT}/include/verilated_threads.cpp
    )
    add_dependencies(placer verilator_model)
    target_compile_definitions(placer PRIVATE USE_HW_CG HW_FRAC_BITS=${FRAC_BITS} HW_TOTAL_BITS=${TOTAL_BITS} HW_MAX_N=${MAX_N} CG_GOLDEN_FRAC_BITS=${FRAC_BITS} CG_GOLDEN_TOTAL_BITS=${TOTAL_BITS} ${CG_GOLDEN_DEFINE} ${CG_VERILATOR_DEFINE})
    target_include_directories(placer SYSTEM PRIVATE ${VDIR} ${VERILATOR_ROOT}/include)
    target_link_libraries(placer PRIVATE ${VDIR}/VCGTop__ALL.a pthread)
    set_source_files_properties(
      ${VERILATOR_ROOT}/include/verilated.cpp
      ${VERILATOR_ROOT}/include/verilated_threads.cpp
      PROPERTIES COMPILE_OPTIONS "-w"
    )
else()
    add_executable(placer placer.cpp)
endif()

target_compile_options(placer PRIVATE -Wall -Wextra -Wno-psabi -Wno-sign-compare)

if(STATIC_BUILD)
    target_link_options(placer PRIVATE -static)
endif()
