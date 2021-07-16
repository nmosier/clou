add_library(lcm MODULE
  lcm.cc
  util.cc
  addr.cc
  )

target_compile_options(lcm PRIVATE -fno-rtti)

if(APPLE)
  target_link_options(lcm PRIVATE -undefined dynamic_lookup)
endif()
