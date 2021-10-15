add_library(util_objs OBJECT
  llvm.h
  tuple.h
  z3.h
  output.h
  functional.h
  algorithm.h
  iterator.h
  )

set_target_properties(util_objs PROPERTIES LINKER_LANGUAGE CXX)

