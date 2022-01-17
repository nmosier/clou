add_library(cfg_objs OBJECT
  cfg.cc cfg.h
  node.h node.cc
  calls.cc calls.h
  expanded.cc expanded.h
  unrolled.cc unrolled.h
  block.cc block.h
  )
target_include_directories(cfg_objs PUBLIC ${Boost_INCLUDE_DIRS})
