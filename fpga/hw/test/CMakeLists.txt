cmake_minimum_required(VERSION 3.14)
project(CGTop_tb LANGUAGES CXX)

set(DPI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/cg_golden_dpi.cpp")

set(INT_BITS  "13" CACHE STRING "Fixed-point integer bits (sets CGTop_tb p_int_bits and CG_GOLDEN_TOTAL_BITS).")
set(FRAC_BITS "14" CACHE STRING "Fixed-point fractional bits (sets CGTop_tb p_frac_bits and CG_GOLDEN_FRAC_BITS).")
set(MAX_N     "50" CACHE STRING "Max problem size (sets CGTop_tb p_max_n and CGTop p_max_n).")
math(EXPR TB_TOTAL_BITS "${INT_BITS} + ${FRAC_BITS}")
if(TB_TOTAL_BITS GREATER 64)
    message(FATAL_ERROR "INT_BITS + FRAC_BITS (=${TB_TOTAL_BITS}) exceeds 64.")
endif()

# Build one Verilator testbench target for an RTL version directory.
# Usage:  add_verilator_tb(<version> <rtl_dir> <tb_name> [extra_cflags])
function(add_verilator_tb VERSION RTL_DIR TB)
  set(EXTRA_CFLAGS "${ARGN}")
  set(VDIR "${CMAKE_CURRENT_BINARY_DIR}/vobj_${VERSION}")
  set(TARGET "VCGTop_tb_${VERSION}")

  set(RTL_SRCS
    ${RTL_DIR}/CGTop.v
    ${RTL_DIR}/CGCtrl.v
    ${RTL_DIR}/CGDpath.v
    ${RTL_DIR}/FpMath.v
    ${RTL_DIR}/LinAlg.v
  )
  if(EXISTS "${RTL_DIR}/M10KLoader.v")
    list(APPEND RTL_SRCS ${RTL_DIR}/M10KLoader.v)
  endif()
  if(EXISTS "${RTL_DIR}/CGEngine.v")
    list(APPEND RTL_SRCS ${RTL_DIR}/CGEngine.v)
  endif()

  add_custom_target(${TARGET} ALL
    COMMAND verilator --cc --exe --build --timing --main --trace
      -Wall -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL
      -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-DECLFILENAME
      --top-module CGTop_tb
      --Mdir ${VDIR}
      -Gp_int_bits=${INT_BITS} -Gp_frac_bits=${FRAC_BITS} -Gp_max_n=${MAX_N}
      ${CMAKE_CURRENT_SOURCE_DIR}/${TB}.v
      ${RTL_SRCS}
      ${DPI_SRC}
      -CFLAGS "-std=c++17 ${EXTRA_CFLAGS} -DCG_GOLDEN_FRAC_BITS=${FRAC_BITS} -DCG_GOLDEN_TOTAL_BITS=${TB_TOTAL_BITS} -I${CMAKE_CURRENT_SOURCE_DIR}/../../../sw-baseline-c"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building ${TB} (${VERSION}) with Verilator (Q${INT_BITS}.${FRAC_BITS})"
  )

  add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${VDIR}/VCGTop_tb ${CMAKE_CURRENT_BINARY_DIR}/VCGTop_tb_${VERSION}
  )
endfunction()

# v1/v2 use shift-subtract divide; v3..v5_deep use NR FpDiv (DPI golden
# needs CG_GOLDEN_USE_NR). v5/v5_deep have multi-block memory; v6 has
# parallel x/y engines with 10 slaves.
add_verilator_tb(v1      ${CMAKE_CURRENT_SOURCE_DIR}/../v1      CGTop_tb)
add_verilator_tb(v2      ${CMAKE_CURRENT_SOURCE_DIR}/../v2      CGTop_tb)
add_verilator_tb(v3      ${CMAKE_CURRENT_SOURCE_DIR}/../v3      CGTop_tb    -DCG_GOLDEN_USE_NR)
add_verilator_tb(v4      ${CMAKE_CURRENT_SOURCE_DIR}/../v4      CGTop_tb    -DCG_GOLDEN_USE_NR)
add_verilator_tb(v5      ${CMAKE_CURRENT_SOURCE_DIR}/../v5      CGTop_tb_v5 -DCG_GOLDEN_USE_NR)
add_verilator_tb(v5_deep ${CMAKE_CURRENT_SOURCE_DIR}/../v5_deep CGTop_tb_v5 -DCG_GOLDEN_USE_NR)
add_verilator_tb(v6      ${CMAKE_CURRENT_SOURCE_DIR}/../v6      CGTop_tb_v6 -DCG_GOLDEN_USE_NR)
