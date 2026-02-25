# Ground Station Builds
#
# Standalone Pico SDK programs for the ground station hardware.
# Separate from the flight firmware build.
#
# Build:
#   cmake -B build_gs -S ground_station -G Ninja \
#         -DPICO_BOARD=adafruit_fruit_jam
#   cmake --build build_gs
#
# Output: build_gs/radio_rx.uf2

cmake_minimum_required(VERSION 3.13)

# Board configuration — Fruit Jam (RP2350B)
set(PICO_BOARD "adafruit_fruit_jam" CACHE STRING "Board type")

# Pico VS Code Extension Integration
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# Pull in Pico SDK (pico_sdk_import.cmake is in parent directory)
include(${CMAKE_CURRENT_LIST_DIR}/../pico_sdk_import.cmake)

project(rocketchip_ground_station C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

# ============================================================================
# Radio RX — LoRa receiver (Fruit Jam + RFM95W breakout)
# ============================================================================

add_executable(radio_rx
    radio_rx.cpp
    gs_spi.cpp                              # SPI1 for Fruit Jam (replaces flight spi_bus.cpp)
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/rfm95w.cpp  # Reuse flight radio driver
)

target_include_directories(radio_rx PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers   # spi_bus.h, rfm95w.h
    ${CMAKE_CURRENT_LIST_DIR}/../include       # rocketchip/config.h (for kSpiBusFreqHz)
    ${CMAKE_CURRENT_LIST_DIR}/../src           # for any transitive includes
)

# Generate ws2812.pio.h from SDK's PIO program (for NeoPixel signal strength)
pico_generate_pio_header(radio_rx
    ${PICO_SDK_PATH}/src/rp2_common/pico_status_led/ws2812.pio
)

target_link_libraries(radio_rx
    pico_stdlib
    hardware_gpio
    hardware_spi
    hardware_pio
)

pico_enable_stdio_usb(radio_rx 1)
pico_enable_stdio_uart(radio_rx 0)

pico_add_extra_outputs(radio_rx)

# Print build info
message(STATUS "")
message(STATUS "=== Ground Station Build ===")
message(STATUS "Board: ${PICO_BOARD}")
message(STATUS "Target: radio_rx")
message(STATUS "")
