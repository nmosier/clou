cmake_minimum_required(VERSION 3.16)

project(lcm-llvm)

set(CMAKE_CXX_STANDARD 17)
find_package(LLVM REQUIRED CONFIG)
list(APPEND CMAKE_MODULE_PATH "${LLVM_CMAKE_DIR}")
find_package(Z3 REQUIRED)
add_definitions(${LLVM_DEFINITIONS})
include_directories(${LLVM_INCLUDE_DIRS} ${Z3_INCLUDE_DIR})
link_directories(${LLVM_LIBRARY_DIRS})
include(FindPkgConfig)
pkg_search_module(LIBPROFILER REQUIRED libprofiler)
include_directories(${LIBPROFILER_INCLUDE_DIRS})
link_directories(${LIBPROFILER_LIBRARY_DIRS})

add_subdirectory(src)
