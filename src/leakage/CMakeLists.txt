add_library(leakage_objs OBJECT
  leakage.cc leakage.h
  spectre-v1.cc spectre-v1.h
  spectre-v4.cc spectre-v4.h
  )
target_link_libraries(leakage_objs PRIVATE proto)
