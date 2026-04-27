cmake_minimum_required(VERSION 3.14)
project(CGTop_tb LANGUAGES CXX)

set(DPI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/cg_golden_dpi.cpp")
set(TB CGTop_tb)

# ----------------------------------------------------------------------
# v1 target
# ----------------------------------------------------------------------
set(V1_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../v1")
set(V1_VDIR "${CMAKE_CURRENT_BINARY_DIR}/vobj_v1")

set(RTL_SRCS_V1
  ${V1_DIR}/CGTop.v
  ${V1_DIR}/CGCtrl.v
  ${V1_DIR}/CGDpath.v
  ${V1_DIR}/FpMath.v
  ${V1_DIR}/LinAlg.v
  ${V1_DIR}/M10KLoader.v
)

add_custom_target(VCGTop_tb_v1 ALL
  COMMAND verilator --cc --exe --build --timing --main
    -Wall -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL
    -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-DECLFILENAME
    --top-module ${TB}
    --Mdir ${V1_VDIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/${TB}.v
    ${RTL_SRCS_V1}
    ${DPI_SRC}
    -CFLAGS "-std=c++17 -I${CMAKE_CURRENT_SOURCE_DIR}/../../../sw-baseline-c"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building ${TB} (v1) with Verilator"
)

add_custom_command(TARGET VCGTop_tb_v1 POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${V1_VDIR}/V${TB} ${CMAKE_CURRENT_BINARY_DIR}/V${TB}_v1
)

# ----------------------------------------------------------------------
# v2 target
# ----------------------------------------------------------------------
set(V2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../v2")
set(V2_VDIR "${CMAKE_CURRENT_BINARY_DIR}/vobj_v2")

set(RTL_SRCS_V2
  ${V2_DIR}/CGTop.v
  ${V2_DIR}/CGCtrl.v
  ${V2_DIR}/CGDpath.v
  ${V2_DIR}/FpMath.v
  ${V2_DIR}/LinAlg.v
)

add_custom_target(VCGTop_tb_v2 ALL
  COMMAND verilator --cc --exe --build --timing --main
    -Wall -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL
    -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-DECLFILENAME
    --top-module ${TB}
    --Mdir ${V2_VDIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/${TB}.v
    ${RTL_SRCS_V2}
    ${DPI_SRC}
    -CFLAGS "-std=c++17 -I${CMAKE_CURRENT_SOURCE_DIR}/../../../sw-baseline-c"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building ${TB} (v2) with Verilator"
)

add_custom_command(TARGET VCGTop_tb_v2 POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${V2_VDIR}/V${TB} ${CMAKE_CURRENT_BINARY_DIR}/V${TB}_v2
)
