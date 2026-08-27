cmake_minimum_required(VERSION 3.20)
project(starcom VERSION 0.0.0 LANGUAGES CXX)

option(STARCOM_BUILD_TESTS "Build host unit tests and ctest" ON)

include(cmake/CompilerWarnings.cmake)

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
)
add_library(Starcom::starcom ALIAS starcom)

target_include_directories(starcom
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
    $<INSTALL_INTERFACE:include>
)

starcom_set_core_flags(starcom)

add_subdirectory(adapters/host)

if(STARCOM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
