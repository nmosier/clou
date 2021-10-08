add_compile_options(-Wall -Wextra -Wno-unused-parameter -pedantic)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(cfg)
add_subdirectory(util)

add_library(lcm MODULE
  lcm.cc           lcm.h
  addr.cc          addr.h
  config.cc        config.h
  aeg.cc aeg.h	   
  inst.cc inst.h   
  uhb.cc           uhb.h
  aeg-util.cc      
  aeg-construct.cc 
  fol.h
  leakage.cc
  taint.h
  taint_bv.cc      taint_bv.h
  shm.cc           shm.h
  spec-prim.cc     spec-prim.h
  binrel.h
  dot.h
  graph.h
  hash.h
  noderef.cc       noderef.h
  progress.h
  tarjan.h
  timer.h
  util.h
  xmacro.h
  $<TARGET_OBJECTS:cfg_objs>
  $<TARGET_OBJECTS:util_objs>
  )

target_compile_options(lcm PRIVATE -Wno-format-security -Wno-gnu-zero-variadic-macro-arguments -g)
target_link_libraries(lcm PRIVATE ${Z3_LIBRARIES} ${LIBPROFILER_LIBRARIES})
if(APPLE)
  target_link_options(lcm PRIVATE -undefined dynamic_lookup)
endif()


if(0)
add_library(alias MODULE
  alias-main.cc
  )
target_compile_options(alias PRIVATE -fno-rtti)
if(APPLE)
  target_link_options(alias PRIVATE -undefined dynamic_lookup)
endif()
endif()


if(0)
add_library(loops MODULE
  loops-main.cc
  )
target_compile_options(loops PRIVATE -fno-rtti)
if(APPLE)
  target_link_options(loops PRIVATE -undefined dynamic_lookup)
endif()
endif()

if(0)
add_executable(tarjan-test
  tarjan-test.cc
  )
endif()
