add_library(util_objs OBJECT
  llvm.h llvm.cc
  tuple.h
  z3.h
  output.h
  functional.h
  algorithm.h
  iterator.h
  container.h
  )

set_target_properties(util_objs PROPERTIES LINKER_LANGUAGE CXX)

