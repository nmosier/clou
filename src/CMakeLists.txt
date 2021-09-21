add_compile_options(-Wall -Wextra -Wno-unused-parameter -pedantic)

add_library(lcm MODULE
  lcm.cc
  addr.cc
  aeg-po2.cc
  config.cc
  aeg.cc
  inst.cc
  aeg-expanded.cc
  aeg-unrolled.cc
  uhb.cc
  aeg-util.cc
  aeg-construct.cc
  fol.cc
  leakage.cc
  taint.cc
  shm.cc
  )

target_compile_options(lcm PRIVATE -fno-rtti -Wno-format-security -g)
target_link_libraries(lcm PRIVATE ${Z3_LIBRARIES} ${LIBPROFILER_LIBRARIES})
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

add_executable(tarjan-test
  tarjan-test.cc
  )

add_executable(shm-test
  shm-test.cc
  shm.cc
  )
target_compile_options(shm-test PRIVATE -fno-rtti)
if(APPLE)
  target_link_options(shm-test PRIVATE -undefined dynamic_lookup)
endif()
