add_library(starcom_adapters_rp2350 STATIC
  radio_bus.cpp
  pio_port.cpp
  phy.cpp
)
add_library(Starcom::adapters_rp2350 ALIAS starcom_adapters_rp2350)

target_link_libraries(starcom_adapters_rp2350 PUBLIC Starcom::starcom)
starcom_set_core_flags(starcom_adapters_rp2350)
starcom_apply_sanitizer(starcom_adapters_rp2350)

# IVP 16 inspection: no Pico SDK includes in public headers.
file(GLOB_RECURSE _sc_hdrs "${CMAKE_CURRENT_SOURCE_DIR}/../../include/starcom/*")
foreach(_h IN LISTS _sc_hdrs)
  if(NOT _h MATCHES "\\.(hpp|h)$")
    continue()
  endif()
  file(READ "${_h}" _txt)
  if(_txt MATCHES "#include[^\n]*(pico\\.h|hardware/spi|hardware/gpio|pico/stdlib|hardware/pio|sx1276)")
    message(FATAL_ERROR "Pico SDK or named radio include in public header: ${_h} (IVP 16/17)")
  endif()
endforeach()
