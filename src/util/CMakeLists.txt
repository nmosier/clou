add_subdirectory(fwd)

add_library(util_objs OBJECT
  llvm.h llvm.cc
  tuple.h
  z3.h z3.cc
  output.h
  functional.h
  algorithm.h
  iterator.h
  container.h
  scope.h
  exception.h
  protobuf.h
  sem.h
  $<TARGET_OBJECTS:util_fwd_objs>
  )
set_target_properties(util_objs PROPERTIES LINKER_LANGUAGE CXX)
