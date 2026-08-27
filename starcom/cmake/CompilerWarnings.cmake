# Warning and language flags for Starcom core vs tests.

function(starcom_set_core_flags tgt)
  target_compile_features(${tgt} PUBLIC cxx_std_20)
  set_target_properties(${tgt} PROPERTIES CXX_EXTENSIONS OFF)
  if(MSVC)
    target_compile_options(${tgt} PRIVATE /W4 /EHs- /EHc- /GR-)
  else()
    target_compile_options(${tgt} PRIVATE
      -Wall -Wextra -Wpedantic -Werror
      -fno-exceptions -fno-rtti)
  endif()
endfunction()

function(starcom_set_test_flags tgt)
  target_compile_features(${tgt} PRIVATE cxx_std_20)
  set_target_properties(${tgt} PROPERTIES CXX_EXTENSIONS OFF)
  if(MSVC)
    target_compile_options(${tgt} PRIVATE /W4 /EHsc)
  else()
    target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endfunction()
