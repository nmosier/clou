add_compile_options(-Wall -Wextra -Wno-unused-parameter -pedantic)

add_library(lcm MODULE
  lcm.cc
  util.cc
  addr.cc
  aeg-po2.cc
  config.cc
  aeg.cc
  inst.cc
  aeg-expanded.cc
  aeg-unrolled.cc
  )
target_compile_options(lcm PRIVATE -fno-rtti -Wno-format-security -g)
target_link_libraries(lcm PRIVATE z3 ${LIBPROFILER_LIBRARIES})
if(APPLE)
  target_link_options(lcm PRIVATE -undefined dynamic_lookup)
endif()


add_library(alias MODULE
  alias-main.cc
  )
target_compile_options(alias PRIVATE -fno-rtti)
if(APPLE)
  target_link_options(alias PRIVATE -undefined dynamic_lookup)
endif()


add_library(loops MODULE
  loops-main.cc
  )
target_compile_options(loops PRIVATE -fno-rtti)
if(APPLE)
  target_link_options(loops PRIVATE -undefined dynamic_lookup)
endif()
