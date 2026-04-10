cmake_minimum_required(VERSION 3.10)
project(placer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(placer placer.cpp)
target_compile_options(placer PRIVATE -Wall -Wextra)
