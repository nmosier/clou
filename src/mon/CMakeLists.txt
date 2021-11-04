find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS mon.proto)

foreach(PROTO_HDR ${PROTO_HDRS})
  set(PROTO_INCLUDE_STATEMENTS "${PROTO_INCLUDE_STATEMENTS}#include \"${PROTO_HDR}\"\n")
endforeach()

configure_file(proto.h.in proto.h)

add_library(proto OBJECT
  ${CMAKE_CURRENT_BINARY_DIR}/proto.h
  ${PROTO_HDRS}
  ${PROTO_SRCS}
  mon.proto
  client.h
  client.cc
  )
target_include_directories(proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/..)
target_link_libraries(proto PUBLIC ${Protobuf_LIBRARIES})

find_package(Curses REQUIRED)

add_executable(mon
  main.cc
  $<TARGET_OBJECTS:proto>
  )
target_link_libraries(mon PRIVATE proto ${CURSES_LIBRARIES})
target_compile_options(mon PRIVATE ${CURSES_CFLAGS})
target_include_directories(mon PRIVATE ${CURSES_INCLUDE_DIRS})
