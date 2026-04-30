cmake_minimum_required(VERSION 3.14)
project(CGTop_tb LANGUAGES CXX)

set(DPI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/cg_golden_dpi.cpp")
set(TB CGTop_tb)

# Build one Verilator testbench target for an RTL version directory.
#
# Usage:  add_verilator_tb(<version> <rtl_dir> [extra_cflags])
#   <version>    -- e.g. v1, v2, v3. Used as the target suffix.
#   <rtl_dir>    -- absolute path to the directory holding CGTop.v et al.
#   [extra_cflags] -- optional extra -CFLAGS string (e.g. for v3 NR golden).
#
# Builds VCGTop_tb_<version> as an ALL target and stages the binary as
# V<TB>_<version> in the top of the build dir for easy invocation.
function(add_verilator_tb VERSION RTL_DIR)
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
  # v1 also has a separate M10K loader module.
  if(EXISTS "${RTL_DIR}/M10KLoader.v")
    list(APPEND RTL_SRCS ${RTL_DIR}/M10KLoader.v)
  endif()

  add_custom_target(${TARGET} ALL
    COMMAND verilator --cc --exe --build --timing --main
      -Wall -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL
      -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-DECLFILENAME
      --top-module ${TB}
      --Mdir ${VDIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/${TB}.v
      ${RTL_SRCS}
      ${DPI_SRC}
      -CFLAGS "-std=c++17 ${EXTRA_CFLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/../../../sw-baseline-c"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building ${TB} (${VERSION}) with Verilator"
  )

  add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${VDIR}/V${TB} ${CMAKE_CURRENT_BINARY_DIR}/V${TB}_${VERSION}
  )
endfunction()

# v3 needs the NR-divide DPI golden so compares stay bit-exact against the
# v3 RTL's NR FpDiv. v1 and v2 use the default shift-subtract divide.
add_verilator_tb(v1 ${CMAKE_CURRENT_SOURCE_DIR}/../v1)
add_verilator_tb(v2 ${CMAKE_CURRENT_SOURCE_DIR}/../v2)
add_verilator_tb(v3 ${CMAKE_CURRENT_SOURCE_DIR}/../v3 -DCG_GOLDEN_USE_NR)
