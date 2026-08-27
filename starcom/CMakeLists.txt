cmake_minimum_required(VERSION 3.20)
project(starcom VERSION 0.0.0 LANGUAGES CXX)

option(STARCOM_BUILD_TESTS "Build host unit tests and ctest" ON)

include(cmake/CompilerWarnings.cmake)

add_library(starcom STATIC
  src/ccsds/crc32.cpp
  src/ccsds/pltu.cpp
  src/ccsds/v3.cpp
)
add_library(Starcom::starcom ALIAS starcom)

target_include_directories(starcom
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
    $<INSTALL_INTERFACE:include>
)

starcom_set_core_flags(starcom)

if(STARCOM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
