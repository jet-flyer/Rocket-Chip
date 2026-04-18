cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD "adafruit_fruit_jam" CACHE STRING "Board")

include(pico_sdk_import.cmake)

project(i2c_bare_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(i2c_bare_test i2c_bare_test.c)

target_link_libraries(i2c_bare_test
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

pico_enable_stdio_usb(i2c_bare_test 1)
pico_enable_stdio_uart(i2c_bare_test 0)
pico_add_extra_outputs(i2c_bare_test)
