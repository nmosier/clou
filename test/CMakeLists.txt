file(GLOB PHT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/pht*.c)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test.sh ${CMAKE_CURRENT_BINARY_DIR}/test.sh)

foreach(PHT_SRC ${PHT_SRCS})
  get_filename_component(PHT_NAME ${PHT_SRC} NAME_WLE)
  add_test(NAME ${PHT_NAME}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/test.sh
    -O ${CMAKE_CURRENT_BINARY_DIR}/${PHT_NAME}
    -T ${PHT_SRC}
    -R ${CMAKE_CURRENT_SOURCE_DIR}/${PHT_NAME}.ref
    -L $<TARGET_FILE:lcm>
    -A "-Epo,tfo,addr,rf,co,rfx,cox -d20 --speculation-primitives=branch -fvictim_function_.*"
    )
endforeach()
