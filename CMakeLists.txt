cmake_minimum_required(VERSION 3.16)

project(clou)

set(CMAKE_CXX_STANDARD 20)
find_package(LLVM REQUIRED CONFIG)
list(APPEND CMAKE_MODULE_PATH "${LLVM_CMAKE_DIR}")
find_package(Z3 REQUIRED)
find_package(Protobuf REQUIRED)
add_definitions(${LLVM_DEFINITIONS})
include_directories(${LLVM_INCLUDE_DIRS} ${Z3_INCLUDE_DIR})
link_directories(${LLVM_LIBRARY_DIRS})
include(FindPkgConfig)
pkg_search_module(LIBPROFILER REQUIRED libprofiler)
include_directories(${LIBPROFILER_INCLUDE_DIRS})
link_directories(${LIBPROFILER_LIBRARY_DIRS})

# set(SANITIZE address)
if(DEFINED SANITIZE)
  add_compile_options(-fsanitize=${SANITIZE})
  add_link_options(-fsanitize=${SANITIZE})
endif()

add_link_options(-w)
add_compile_options(-Wno-c++20-designator -Wno-format-security -Wno-c99-designator -Wno-gnu-zero-variadic-macro-arguments)

enable_testing()

add_subdirectory(src)
add_subdirectory(docs)
add_subdirectory(test)
