# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test.sh ${CMAKE_CURRENT_BINARY_DIR}/test.sh)
set(TEST_SH ${CMAKE_CURRENT_SOURCE_DIR}/test.sh)

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
    COMMAND ${TEST_SH}
    -O ${CMAKE_CURRENT_BINARY_DIR}/${PHT_NAME}
    -T ${PHT_SRC}
    -R ${CMAKE_CURRENT_SOURCE_DIR}/${PHT_NAME}.ref
    -L $<TARGET_FILE:lcm>
    -A "-Epo,tfo,addr,addr_gep,rf,rfx -d20 -fvictim_function_.* --spectre-v1=mode=classic --aa=transient --partial ${PHT_ARGS}"
    )
endforeach()


# file(GLOB STL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/stl*.c)

set(STLS 1 2 3 4 5 6 7 8 9 9_bis 10 11 12 13)
foreach(STL ${STLS})
  set(STL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/stl${STL}.c)
  get_filename_component(STL_NAME ${STL_SRC} NAME_WLE)
  set(STL_ARGFILE ${CMAKE_CURRENT_SOURCE_DIR}/${STL_NAME}.args)
  unset(STL_ARGS)
  if(EXISTS ${STL_ARGFILE})
    file(STRINGS ${STL_ARGFILE} STL_ARGLIST)
    list(JOIN STL_ARGLIST " " STL_ARGS)
  endif()
  add_test(NAME ${STL_NAME}
    COMMAND ${TEST_SH}
    -O ${CMAKE_CURRENT_BINARY_DIR}/${STL_NAME}
    -T ${STL_SRC}
    -R ${CMAKE_CURRENT_SOURCE_DIR}/${STL_NAME}.ref
    -L $<TARGET_FILE:lcm>
    -A "-Epo -d20 --spectre-v4 --traceback=2 --max-transient=25 --aa=transient -fcase_${STL} --partial ${STL_ARGS}"
    )
endforeach()

set(LIBSODIUM ${CMAKE_CURRENT_SOURCE_DIR}/../../libsodium/src/libsodium)
add_test(NAME spectre-v1-medium
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../run.sh
  -O ${CMAKE_CURRENT_BINARY_DIR}/spectre-v1-medium
  -L $<TARGET_FILE:lcm>
  -x "-Epo,tfo,rf,rfx,addr,data -d20 --spectre-v1=mode=classic --fast --aa=transient -fpoly1305_blocks"
  ${LIBSODIUM}/crypto_onetimeauth/poly1305/donna/poly1305_donna.c -I ${LIBSODIUM}/include/sodium
  )

set(SPECTRE_V1_LARGE_FILE crypto_core/ed25519/core_h2c.c)
set(SPECTRE_V1_LARGE_FUNC _sodium_core_h2c_string_to_hash)
# set(SPECTRE_V1_LARGE_FILE crypto_shorthash/siphash24/ref/shorthash_siphashx24_ref.c)
# set(SPECTRE_V1_LARGE_FUNC crypto_shorthash_siphashx24)

add_test(NAME spectre-v1-large
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../run.sh
  -O ${CMAKE_CURRENT_BINARY_DIR}/spectre-v1-large
  -L $<TARGET_FILE:lcm>
  -x "--spectre-v1=mode=classic --max-transient=25 --fast --aa=transient -f${SPECTRE_V1_LARGE_FUNC} --stb=56"
  ${LIBSODIUM}/${SPECTRE_V1_LARGE_FILE} -I ${LIBSODIUM}/include/sodium
  )

add_test(NAME spectre-v4-large
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../run.sh
  -O ${CMAKE_CURRENT_SOURCE_DIR}/spectre-v4-large
  -L $<TARGET_FILE:lcm>
  -x "--spectre-v4 --fast --max-transient=25 --aa=transient  -f_sodium_blake2b_init_key --stb=56"
  ${LIBSODIUM}/crypto_generichash/blake2b/ref/blake2b-ref.c -I ${LIBSODIUM}/include/sodium
  )

add_test(NAME spectre-v1-xl
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../run.sh
  -O ${CMAKE_CURRENT_SOURCE_DIR}/spectre-v1-xl
  -L $<TARGET_FILE:lcm>
  -x "--spectre-v1=mode=classic --max-transient=25 --fast --aa=transient --stb=56"
  ${LIBSODIUM}/crypto_core/ed25519/core_h2c.c -I ${LIBSODIUM}/include/sodium
  )
