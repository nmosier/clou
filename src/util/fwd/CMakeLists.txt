add_library(util_fwd_objs OBJECT
  z3.h
  scope.h
  )
set_target_properties(util_fwd_objs PROPERTIES LINKER_LANGUAGE CXX)
