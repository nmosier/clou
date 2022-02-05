add_library(dataflow_objs OBJECT
  dataflow.h dataflow.cc
  )

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

add_library(attacker_taint SHARED
  attacker-taint.cc
  attacker-taint.h
  $<TARGET_OBJECTS:dataflow_objs>
  ../util/llvm.cc ../util/llvm.h
  )
if(APPLE)
  target_link_options(attacker_taint PRIVATE -undefined dynamic_lookup)
endif()
target_include_directories(attacker_taint INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ssb_control SHARED
  ssb-control.cc
  ssb-control.h
  $<TARGET_OBJECTS:dataflow_objs>
  )
if(APPLE)
  target_link_options(ssb_control PRIVATE -undefined dynamic_lookup)
endif()
target_include_directories(ssb_control INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ssb_control PRIVATE attacker_taint)


add_pass(print_loops)
