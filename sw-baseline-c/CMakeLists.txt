cmake_minimum_required(VERSION 3.10)
project(placer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(placer placer.cpp)
target_compile_options(placer PRIVATE -Wall -Wextra -Wno-psabi)

# Static linking for cross-compilation to DE1-SoC
option(STATIC_BUILD "Build with static linking" OFF)
if(STATIC_BUILD)
    target_link_options(placer PRIVATE -static)
endif()
