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
set(_starcom_tidy_selftest OFF)
if(STARCOM_CLANG_TIDY)
  execute_process(
    COMMAND "${STARCOM_CLANG_TIDY}" --version
    RESULT_VARIABLE _starcom_tidy_rc
    OUTPUT_VARIABLE _starcom_tidy_ver
    ERROR_VARIABLE _starcom_tidy_ver_err
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(_starcom_tidy_ver_all "${_starcom_tidy_ver}${_starcom_tidy_ver_err}")
  if(_starcom_tidy_rc EQUAL 0 AND _starcom_tidy_ver_all MATCHES "21\\.1\\.[0-9]+")
    find_program(STARCOM_BASH NAMES bash
      HINTS "C:/Program Files/Git/bin" "C:/Program Files/Git/usr/bin")
    if(STARCOM_BASH)
      set(_starcom_tidy_selftest ON)
    endif()
  endif()
endif()
if(_starcom_tidy_selftest)
  add_test(NAME starcom.tidy_selftest
    COMMAND "${STARCOM_BASH}"
            "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/tidy_selftest.sh")
  set_property(TEST starcom.tidy_selftest PROPERTY ENVIRONMENT
    "CLANG_TIDY=${STARCOM_CLANG_TIDY}")
else()
  message(STATUS "tidy_selftest skipped: no pinned clang-tidy 21.1.x")
endif()
