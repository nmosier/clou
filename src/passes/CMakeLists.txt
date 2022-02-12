function(add_pass name)
  add_library(${name} MODULE
    ${name}.cc
    )
  if(APPLE)
    target_link_options(${name} PRIVATE -undefined dynamic_lookup)
  endif()
endfunction()

add_pass(count_functions)
add_pass(print_linkage)
add_pass(analyze_paths)
add_pass(print_debug)
# add_pass(attacker_taint)
add_pass(print_alias)

add_library(annotations SHARED
  annotations.cc
  annotations.h
  )
if(APPLE)
  target_link_options(annotations PRIVATE -undefined dynamic_lookup)
endif()
target_include_directories(annotations INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})



add_library(attacker_taint SHARED
  attacker-taint.cc
  attacker-taint.h
  $<TARGET_OBJECTS:dataflow_objs>
  $<TARGET_OBJECTS:annotations>
  ../util/llvm.cc ../util/llvm.h
  )
if(APPLE)
  target_link_options(attacker_taint PRIVATE -undefined dynamic_lookup)
endif()
target_include_directories(attacker_taint INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(attacker_taint PRIVATE ${LIBPROFILER_LIBRARIES})

add_library(ssb_control SHARED
  ssb-control.cc
  ssb-control.h
  $<TARGET_OBJECTS:dataflow_objs>
  )
if(APPLE)
  target_link_options(ssb_control PRIVATE -undefined dynamic_lookup)
endif()
target_include_directories(ssb_control INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(ssb_control PRIVATE ${LIBPROFILER_INCLUDE_DIRS})
target_link_libraries(ssb_control PRIVATE attacker_taint ${LIBPROFILER_LIBRARIES})

add_pass(print_loops)


add_library(print_transaa MODULE
  print_transaa.cc
  $<TARGET_OBJECTS:dataflow_objs>
  ../util/llvm.cc ../util/llvm.h
  )
if(APPLE)
  target_link_options(print_transaa PRIVATE -undefined dynamic_lookup)
endif()

add_pass(loc)
