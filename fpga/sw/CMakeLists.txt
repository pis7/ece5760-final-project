cmake_minimum_required(VERSION 3.14)
project(placer_fpga LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Always use the FPGA CG driver (mmap over /dev/mem).
option(STATIC_BUILD "Build with static linking" OFF)
set(FRAC_BITS "14" CACHE STRING "Fixed-point fractional bits (must match Verilog p_frac_bits)")

add_executable(placer placer.cpp)
target_compile_definitions(placer PRIVATE USE_HW_CG HW_FRAC_BITS=${FRAC_BITS})

# Share json.hpp with the software baseline to avoid duplicating ~1 MB
# of vendored header.
target_include_directories(placer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../sw-baseline-c
)

target_compile_options(placer PRIVATE -Wall -Wextra -Wno-psabi -Wno-sign-compare)

if(STATIC_BUILD)
    target_link_options(placer PRIVATE -static)
endif()
