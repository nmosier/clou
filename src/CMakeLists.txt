add_compile_options(-Wall -Wextra -Wno-unused-parameter -pedantic)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(cfg)
add_subdirectory(aeg)
add_subdirectory(leakage)
add_subdirectory(util)
add_subdirectory(mon)

add_library(clou MODULE
  lcm.cc           lcm.h
  config.cc        config.h
  fol.h
  binrel.h
  graph.h
  noderef.cc       noderef.h
  db.cc db.h
  $<TARGET_OBJECTS:cfg_objs>
  $<TARGET_OBJECTS:util_objs>
  $<TARGET_OBJECTS:aeg_objs>
  $<TARGET_OBJECTS:leakage_objs>
  $<TARGET_OBJECTS:proto>
  )
target_compile_options(clou PRIVATE -Wno-format-security -Wno-gnu-zero-variadic-macro-arguments -g -Wthread-safety-analysis)
target_link_libraries(clou PRIVATE proto ${Z3_LIBRARIES} ${LIBPROFILER_LIBRARIES})
if(APPLE)
  target_link_options(clou PRIVATE -undefined dynamic_lookup)
endif()

add_subdirectory(passes)
