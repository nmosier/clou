function(add_pass name)
  add_library(${name} MODULE
    ${name}.cc
    )
  if(APPLE)
    target_link_options(${name} PRIVATE -undefined dynamic_lookup)
  endif()
endfunction()

add_pass(count_functions)
add_pass(print_linkage)
add_pass(analyze_paths)
add_pass(print_debug)
