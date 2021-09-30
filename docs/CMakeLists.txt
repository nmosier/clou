option(BUILD_DOCS "Build documentation" ON)

find_package(Doxygen)
if (DOXYGEN_FOUND)

  set(DOXYGEN_IN Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

  find_program(EPSTOPDF epstopdf)
  get_filename_component(EPSTOPDF_DIR ${EPSTOPDF} DIRECTORY)

  add_custom_target(docs ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
    )
  
else (DOXYGEN_FOUND)
  message("Doxygen not found")
endif()
