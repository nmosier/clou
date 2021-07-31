add_library(lcm MODULE
  lcm.cc
  util.cc
  addr.cc
  mcfg.cc
  aeg-po.cc
  config.cc
  cfg.cc
  )

target_compile_options(lcm PRIVATE -fno-rtti -Wno-format-security -g)
target_link_libraries(lcm PRIVATE z3 ${LIBPROFILER_LIBRARIES})

if(APPLE)
  target_link_options(lcm PRIVATE -undefined dynamic_lookup)
endif()


add_executable(cow_test
  cow_test.cc
  )
