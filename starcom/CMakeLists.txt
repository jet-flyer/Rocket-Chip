cmake_minimum_required(VERSION 3.20)

# Peek STARCOM_VERSION so project() is not a second literal. Full generate
# (git identity) is cmake/starcom_version.cmake after project().
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/STARCOM_VERSION" _sc_peek)
set(_sc_maj "0")
set(_sc_min "0")
set(_sc_pat "0")
foreach(_line IN LISTS _sc_peek)
  if(_line MATCHES "^VERSION_MAJOR[ \t]*=[ \t]*([0-9]+)")
    set(_sc_maj "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "^VERSION_MINOR[ \t]*=[ \t]*([0-9]+)")
    set(_sc_min "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "^PATCHLEVEL[ \t]*=[ \t]*([0-9]+)")
    set(_sc_pat "${CMAKE_MATCH_1}")
  endif()
endforeach()
project(starcom VERSION ${_sc_maj}.${_sc_min}.${_sc_pat} LANGUAGES CXX)

option(STARCOM_BUILD_TESTS "Build host unit tests and ctest" ON)
option(STARCOM_BUILD_HOST_ADAPTERS "UDP/file/loopback host adapters" ON)
option(STARCOM_SANITIZE "ASan+UBSan on core, adapters, and tests (not MSVC; needs libasan)" OFF)

include(cmake/CompilerWarnings.cmake)
include(cmake/starcom_version.cmake)

add_library(starcom STATIC
  src/ccsds/crc32.cpp
  src/ccsds/pltu.cpp
  src/ccsds/v3.cpp
  src/ccsds/space_packet.cpp
  src/ccsds/plcw.cpp
  src/ccsds/clcw.cpp
  src/ccsds/copp.cpp
  src/ccsds/uslp.cpp
  src/ccsds/cop1.cpp
  src/ccsds/mac.cpp
  src/ccsds/conv.cpp
  src/ccsds/ldpc.cpp
)
add_library(Starcom::starcom ALIAS starcom)

target_include_directories(starcom
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
    $<BUILD_INTERFACE:${STARCOM_VERSION_GENERATED_DIR}>
    $<INSTALL_INTERFACE:include>
)

starcom_set_core_flags(starcom)
starcom_apply_sanitizer(starcom)

if(STARCOM_BUILD_HOST_ADAPTERS)
  add_subdirectory(adapters/host)
endif()
add_subdirectory(adapters/rp2350)

if(STARCOM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
