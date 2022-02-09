add_library(dataflow_objs OBJECT
  dataflow.h dataflow.cc
  transient-aa.h transient-aa.cc
  )
target_include_directories(dataflow_objs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
