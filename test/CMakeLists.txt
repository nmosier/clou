configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test.sh ${CMAKE_CURRENT_BINARY_DIR}/test.sh)

file(GLOB PHT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/pht*.c)

foreach(PHT_SRC ${PHT_SRCS})
  get_filename_component(PHT_NAME ${PHT_SRC} NAME_WLE)
  set(PHT_ARGFILE ${CMAKE_CURRENT_SOURCE_DIR}/${PHT_NAME}.args)
  unset(PHT_ARGS)
  if(EXISTS ${PHT_ARGFILE})
    file(STRINGS ${PHT_ARGFILE} PHT_ARGLIST)
    list(JOIN PHT_ARGLIST " " PHT_ARGS)
  endif()
  add_test(NAME ${PHT_NAME}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/test.sh
    -O ${CMAKE_CURRENT_BINARY_DIR}/${PHT_NAME}
    -T ${PHT_SRC}
    -R ${CMAKE_CURRENT_SOURCE_DIR}/${PHT_NAME}.ref
    -L $<TARGET_FILE:lcm>
    -A "-Epo,tfo,addr,rf,co,rfx,cox -d20 --speculation-primitives=branch -fvictim_function_.* ${PHT_ARGS}"
    )
endforeach()


file(GLOB STL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/stl*.c)

foreach(STL_SRC ${STL_SRCS})
  get_filename_component(STL_NAME ${STL_SRC} NAME_WLE)
  set(STL_ARGFILE ${CMAKE_CURRENT_SOURCE_DIR}/${STL_NAME}.args)
  unset(STL_ARGS)
  if(EXISTS ${STL_ARGFILE})
    file(STRINGS ${STL_ARGFILE} STL_ARGLIST)
    list(JOIN STL_ARGLIST " " STL_ARGS)
  endif()
  add_test(NAME ${STL_NAME}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/test.sh
    -O ${CMAKE_CURRENT_BINARY_DIR}/${STL_NAME}
    -T ${STL_SRC}
    -R ${CMAKE_CURRENT_SOURCE_DIR}/${STL_NAME}.ref
    -L $<TARGET_FILE:lcm>
    -A "-Epo -d20 --speculation-primitive=addr --leakage-source= --leakage-class=spectre-v4 --max-transient=20 --aa=transient -fcase.* ${STL_ARGS}"
    )
endforeach()
