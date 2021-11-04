add_library(leakage_objs OBJECT
  leakage.cc leakage.h
  )
target_link_libraries(leakage_objs PRIVATE proto)
