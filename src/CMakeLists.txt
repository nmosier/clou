add_library(lcm MODULE
  lcm.cc
  util.cc
  addr.cc
  mcfg.cc
  aeg.cc
  )

target_compile_options(lcm PRIVATE -fno-rtti)
target_link_libraries(lcm PRIVATE z3)

if(APPLE)
  target_link_options(lcm PRIVATE -undefined dynamic_lookup)
endif()
