add_library(starcom_adapters_host STATIC
  loopback.cpp
)
add_library(Starcom::adapters_host ALIAS starcom_adapters_host)

target_link_libraries(starcom_adapters_host PUBLIC Starcom::starcom)
starcom_set_core_flags(starcom_adapters_host)
starcom_apply_sanitizer(starcom_adapters_host)
