add_library(starcom_adapters_host STATIC
  loopback.cpp
  file_replay.cpp
  udp.cpp
)
add_library(Starcom::adapters_host ALIAS starcom_adapters_host)

target_link_libraries(starcom_adapters_host PUBLIC Starcom::starcom)
if(WIN32)
  target_link_libraries(starcom_adapters_host PRIVATE ws2_32)
  # Clang/GNU-cmd uses the MSVC CRT; fopen is marked deprecated there.
  target_compile_definitions(starcom_adapters_host PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
starcom_set_core_flags(starcom_adapters_host)
starcom_apply_sanitizer(starcom_adapters_host)

# IVP 15 inspection: core must not link sockets.
get_target_property(_starcom_ll starcom LINK_LIBRARIES)
if(_starcom_ll AND NOT _starcom_ll MATCHES "NOTFOUND")
  foreach(_lib IN LISTS _starcom_ll)
    if(_lib MATCHES "ws2_32|socket|nsl")
      message(FATAL_ERROR
        "IVP 15: Starcom core must not link sockets (found ${_lib})")
    endif()
  endforeach()
endif()
