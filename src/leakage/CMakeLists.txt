protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS leakage.proto)

foreach(PROTO_HDR ${PROTO_HDRS})
  set(PROTO_INCLUDE_STATEMENTS "${PROTO_INCLUDE_STATEMENTS}#include \"${PROTO_HDR}\"\n")
endforeach()

configure_file(proto.h.in proto.h)

add_library(leakage_objs OBJECT
  leakage.cc leakage.h
  spectre-v1.cc spectre-v1.h
  spectre-v4.cc spectre-v4.h
  fol.cc fol.h
  ${PROTO_HDRS} ${PROTO_SRCS}
  leakage.proto
  )
target_link_libraries(leakage_objs PRIVATE proto)
