add_executable(starcom_tests
  unit/heap_trap.cpp
  unit/test_pltu.cpp
  unit/test_v3.cpp
  unit/test_space_packet.cpp
  unit/test_ocf.cpp
  unit/test_copp.cpp
  unit/test_uslp.cpp
  unit/test_cop1.cpp
  unit/test_loopback.cpp
  unit/test_version.cpp
  unit/test_fuzz.cpp
  unit/test_size.cpp
  unit/test_mac.cpp
  unit/test_user_defined.cpp
  unit/test_host_io.cpp
  unit/test_radio_bus.cpp
  unit/test_pio_port.cpp
  unit/test_phy.cpp
  unit/test_coding.cpp
)

target_include_directories(starcom_tests PRIVATE unit)
target_link_libraries(starcom_tests PRIVATE Starcom::starcom Starcom::adapters_host Starcom::adapters_rp2350)
target_compile_definitions(starcom_tests PRIVATE STARCOM_ROOT="${STARCOM_ROOT}")
starcom_set_test_flags(starcom_tests)
starcom_apply_sanitizer(starcom_tests)

if(NOT MSVC AND NOT STARCOM_SANITIZE)
  target_compile_definitions(starcom_tests PRIVATE STARCOM_HEAP_WRAP)
  target_link_options(starcom_tests PRIVATE
    -Wl,--wrap=malloc
    -Wl,--wrap=calloc
    -Wl,--wrap=realloc)
endif()

add_test(NAME starcom.unit COMMAND starcom_tests)

# Known-bad tidy probe. The sources are not a build target.
find_program(STARCOM_CLANG_TIDY NAMES clang-tidy clang-tidy.exe
  HINTS "C:/Program Files/LLVM/bin")
# The test is always registered, so a missing tool shows in the CTest result.
# The script exits 2 when clang-tidy 21.1.x is not found; CTest reports that
# as "Skipped", not as a pass.
find_program(STARCOM_BASH NAMES bash
  HINTS "C:/Program Files/Git/bin" "C:/Program Files/Git/usr/bin")
if(STARCOM_BASH)
  add_test(NAME starcom.tidy_selftest
    COMMAND "${STARCOM_BASH}"
            "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/tidy_selftest.sh")
  if(STARCOM_CLANG_TIDY)
    set_property(TEST starcom.tidy_selftest PROPERTY ENVIRONMENT
      "CLANG_TIDY=${STARCOM_CLANG_TIDY}")
  else()
    set_property(TEST starcom.tidy_selftest PROPERTY ENVIRONMENT
      "CLANG_TIDY=clang-tidy-not-found")
  endif()
  set_property(TEST starcom.tidy_selftest PROPERTY SKIP_RETURN_CODE 2)
else()
  add_test(NAME starcom.tidy_selftest COMMAND "${CMAKE_COMMAND}" -E echo "no bash")
  set_property(TEST starcom.tidy_selftest PROPERTY DISABLED TRUE)
  message(STATUS "tidy_selftest disabled: no bash")
endif()
