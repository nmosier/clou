set(DEFAULT_ARGS "-Epo,tfo,addr,addr_gep,rf,rfx -d250 --window 500 --lsq=50 --traceback=2")

set(TEST_SH ${CMAKE_CURRENT_SOURCE_DIR}/test2.sh)
set(TEST_FENCE_SH ${CMAKE_CURRENT_SOURCE_DIR}/test-fence.sh)

file(GLOB PHT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/pht*.c)

find_program(LLVM_OPT NAMES opt opt-12 PATHS ${LLVM_BINARY_DIR} ${LLVM_BINARY_DIR}/bin REQUIRED)

file(GLOB PHT_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/pht*.c ${CMAKE_CURRENT_SOURCE_DIR}/pht*r.ll)
foreach(PHT_SRC ${PHT_SRCS})

  get_filename_component(PHT_NAME ${PHT_SRC} NAME_WLE)
  set(PHT_ARGFILE ${CMAKE_CURRENT_SOURCE_DIR}/${PHT_NAME}.args)
  unset(PHT_ARGS)
  if(EXISTS ${PHT_ARGFILE})
    file(STRINGS ${PHT_ARGFILE} PHT_ARGLIST)
    list(JOIN PHT_ARGLIST " " PHT_ARGS)
  endif()

  set(PHT_ARGS "${DEFAULT_ARGS} -fvictim_function_.* --spectre-v1=mode=classic --aa=transient --partial ${PHT_ARGS}")
  
  add_test(NAME ${PHT_NAME}
    COMMAND ${TEST_SH}
    -O ${CMAKE_CURRENT_BINARY_DIR}/${PHT_NAME}
    -T ${PHT_SRC}
    -R ${CMAKE_CURRENT_SOURCE_DIR}/${PHT_NAME}.ref
    -L $<TARGET_FILE:clou>
    -A "${PHT_ARGS}"
    )

  add_test(NAME ${PHT_NAME}f
    COMMAND ${TEST_FENCE_SH}
    -O ${CMAKE_CURRENT_BINARY_DIR}/${PHT_NAME}f
    -T ${PHT_SRC}
    -L $<TARGET_FILE:clou>
    -A "${PHT_ARGS} --fence"
    )
  set_tests_properties(${PHT_NAME}f PROPERTIES ENVIRONMENT "OPT=${LLVM_OPT}")

endforeach()


file(GLOB STL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/stl*.c ${CMAKE_CURRENT_SOURCE_DIR}/stl*r.ll)
# set(STLS 1 2 3 4 5 6 7 8 9 9_bis 10 11 12 13)
# foreach(STL ${STLS})
foreach(STL_SRC ${STL_SRCS})
  # set(STL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/stl${STL}.c)
  get_filename_component(STL_NAME ${STL_SRC} NAME_WLE)
  string(REGEX REPLACE "^stl" "" STL_TMP ${STL_NAME})
  string(REGEX REPLACE "r$" "" STL ${STL_TMP})
  set(STL_ARGFILE ${CMAKE_CURRENT_SOURCE_DIR}/${STL_NAME}.args)
  unset(STL_ARGS)
  if(EXISTS ${STL_ARGFILE})
    file(STRINGS ${STL_ARGFILE} STL_ARGLIST)
    list(JOIN STL_ARGLIST " " STL_ARGS)
  endif()

  set(STL_ARGS "${DEFAULT_ARGS} --spectre-v4 --aa=transient -fcase_${STL} --partial ${STL_ARGS}")
  
  add_test(NAME ${STL_NAME}
    COMMAND ${TEST_SH}
    -O ${CMAKE_CURRENT_BINARY_DIR}/${STL_NAME}
    -T ${STL_SRC}
    -R ${CMAKE_CURRENT_SOURCE_DIR}/${STL_NAME}.ref
    -L $<TARGET_FILE:clou>
    -A "${STL_ARGS}"
    )

  add_test(NAME ${STL_NAME}f
    COMMAND ${TEST_FENCE_SH}
    -O ${CMAKE_CURRENT_BINARY_DIR}/${STL_NAME}f
    -T ${STL_SRC}
    -L $<TARGET_FILE:clou>
    -A "${STL_ARGS} --fence"
    )
  set_tests_properties(${STL_NAME}f PROPERTIES ENVIRONMENT "OPT=${LLVM_OPT}")
  
endforeach()

file(GLOB FWD_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/fwd*.c)
foreach(FWD_SRC ${FWD_SRCS})

  get_filename_component(FWD_NAME ${FWD_SRC} NAME_WLE)
  set(FWD_ARGS "${DEFAULT_ARGS} --spectre-v1=mode=classic --aa=transient --partial")

  add_test(NAME ${FWD_NAME}
    COMMAND ${TEST_SH}
    -O ${CMAKE_CURRENT_BINARY_DIR}/${FWD_NAME}
    -T ${FWD_SRC}
    -R ${CMAKE_CURRENT_SOURCE_DIR}/${FWD_NAME}.ref
    -L $<TARGET_FILE:clou>
    -A "${FWD_ARGS}"
    )

  add_test(NAME ${FWD_NAME}f
    COMMAND ${TEST_FENCE_SH}
    -O ${CMAKE_CURRENT_BINARY_DIR}/${FWD_NAME}f
    -T ${FWD_SRC}
    -L $<TARGET_FILE:clou>
    -A "${FWD_ARGS} --fence"
    )
  set_tests_properties(${FWD_NAME}f PROPERTIES ENVIRONMENT "OPT=${LLVM_OPT}")
  
endforeach()
