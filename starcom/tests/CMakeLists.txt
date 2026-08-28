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
  unit/test_mac.cpp
  unit/test_user_defined.cpp
  unit/test_host_io.cpp
  unit/test_radio_bus.cpp
  unit/test_pio_port.cpp
  unit/test_phy.cpp
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
