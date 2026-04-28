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
set(FRAC_BITS "14" CACHE STRING "Fixed-point fractional bits (must match Verilog p_frac_bits)")
set(HW_CG_VERSION "v3" CACHE STRING "Which CG solver to wire up: v2 (shift-subtract divide) or v3 (NR divide + rp_lo/rp_hi collapse)")
set_property(CACHE HW_CG_VERSION PROPERTY STRINGS v2 v3)

if(HW_CG_VERSION STREQUAL "v3")
    set(CG_GOLDEN_DEFINE "CG_GOLDEN_USE_NR")
else()
    set(CG_GOLDEN_DEFINE "")
endif()

if(USE_FP_GOLDEN)
    # Pure C++ fixed-point CG — no Verilator dependency
    add_executable(placer placer.cpp)
    target_compile_definitions(placer PRIVATE USE_HW_CG USE_FP_GOLDEN CG_GOLDEN_FRAC_BITS=${FRAC_BITS} ${CG_GOLDEN_DEFINE})
elseif(USE_HW_CG)
    # HW_CG_VERSION selects which RTL gets verilated. v3 is the current
    # target (rp collapse + NR divide + 16 hardened tests). v2 stays for
    # reference / regression-testing the original shift-subtract design.
    # v1 was combinational-only reference and is not wired up here.
    set(VERILOG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../fpga/hw/${HW_CG_VERSION}")
    set(VDIR "${CMAKE_CURRENT_BINARY_DIR}/vobj")
    set(VERILATOR_ROOT "/usr/local/share/verilator")

    # Build the Verilator model as a static library
    add_custom_command(
      OUTPUT ${VDIR}/VCGTop__ALL.a
      COMMAND verilator --cc --build
        -Wall -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL
        -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-DECLFILENAME
        --top-module CGTop
        --Mdir ${VDIR}
        --prefix VCGTop
        -Gp_frac_bits=${FRAC_BITS}
        ${VERILOG_DIR}/CGTop.v
        ${VERILOG_DIR}/CGCtrl.v
        ${VERILOG_DIR}/CGDpath.v
        ${VERILOG_DIR}/FpMath.v
        ${VERILOG_DIR}/LinAlg.v
        -CFLAGS "-std=c++17"
      DEPENDS
        ${VERILOG_DIR}/CGTop.v
        ${VERILOG_DIR}/CGCtrl.v
        ${VERILOG_DIR}/CGDpath.v
        ${VERILOG_DIR}/FpMath.v
        ${VERILOG_DIR}/LinAlg.v
      COMMENT "Verilating CGTop (${HW_CG_VERSION})"
    )
    add_custom_target(verilator_model DEPENDS ${VDIR}/VCGTop__ALL.a)

    add_executable(placer
      placer.cpp
      ${VERILATOR_ROOT}/include/verilated.cpp
      ${VERILATOR_ROOT}/include/verilated_threads.cpp
    )
    add_dependencies(placer verilator_model)
    target_compile_definitions(placer PRIVATE USE_HW_CG HW_FRAC_BITS=${FRAC_BITS} ${CG_GOLDEN_DEFINE})
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
