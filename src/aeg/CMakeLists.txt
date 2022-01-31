add_library(aeg_objs OBJECT
  aeg.h
  aeg.cc
  construct.cc
  util.cc
  exists.cc
  alias.cc
  output.cc
  fwd.h
  context.h context.cc
  constraints.h constraints.cc
  address.h address.cc
  node.h node.cc
  edge.h edge.cc
  inst.h inst.cc
  taint.h taint.cc
  $<TARGET_OBJECTS:proto>
  )
target_link_libraries(aeg_objs PRIVATE proto)
