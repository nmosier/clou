cmake_minimum_required(VERSION 3.20)

project(lcm-llvm)

set(CMAKE_CXX_STANDARD 17 REQUIRED)
find_package(LLVM REQUIRED CONFIG)
add_definitions(${LLVM_DEFINITIONS})
include_directories(${LLVM_INCLUDE_DIRS} /usr/local/include)
link_directories(${LLVM_LIBRARY_DIRS} /usr/local/lib)

add_subdirectory(src)
