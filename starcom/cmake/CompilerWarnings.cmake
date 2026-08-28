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
  if(WIN32)
    target_compile_definitions(${tgt} PRIVATE _CRT_SECURE_NO_WARNINGS)
  endif()
endfunction()

# ASan + UBSan. Off by default: this tree's MinGW g++ 15.2 has the flags
# but not libasan/libubsan. Enable on Clang/Linux: -DSTARCOM_SANITIZE=ON.
# Incompatible with STARCOM_HEAP_WRAP (--wrap=malloc).
function(starcom_apply_sanitizer tgt)
  if(NOT STARCOM_SANITIZE)
    return()
  endif()
  if(MSVC)
    message(FATAL_ERROR "STARCOM_SANITIZE is not wired for MSVC")
  endif()
  target_compile_options(${tgt} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(${tgt} PRIVATE -fsanitize=address,undefined)
endfunction()
