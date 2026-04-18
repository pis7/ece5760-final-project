cmake_minimum_required(VERSION 3.14)
project(CGTop_tb LANGUAGES CXX)

set(VERILOG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../v1")
set(TB CGTop_tb)
set(DPI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/cg_golden_dpi.cpp")
set(VDIR "${CMAKE_CURRENT_BINARY_DIR}/vobj")

set(RTL_SRCS
  ${VERILOG_DIR}/CGTop.v
  ${VERILOG_DIR}/CGCtrl.v
  ${VERILOG_DIR}/CGDpath.v
  ${VERILOG_DIR}/FpMath.v
  ${VERILOG_DIR}/LinAlg.v
  ${VERILOG_DIR}/M10KLoader.v
)

# Use verilator --exe --build directly
add_custom_target(VCGTop_tb ALL
  COMMAND verilator --cc --exe --build --timing --main
    -Wall -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL
    -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-DECLFILENAME
    --top-module ${TB}
    --Mdir ${VDIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/${TB}.v
    ${RTL_SRCS}
    ${DPI_SRC}
    -CFLAGS "-std=c++17 -I${CMAKE_CURRENT_SOURCE_DIR}/../../../sw-baseline-c"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building ${TB} with Verilator"
)

# Copy the binary to the build directory for convenience
add_custom_command(TARGET VCGTop_tb POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${VDIR}/V${TB} ${CMAKE_CURRENT_BINARY_DIR}/V${TB}
)
