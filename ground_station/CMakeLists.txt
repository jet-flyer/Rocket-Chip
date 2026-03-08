# Ground Station Build — Fruit Jam (Stage J)
#
# Builds the SAME flight firmware source for the Fruit Jam board.
# Board abstraction (board.h) selects correct pin definitions at compile time.
# Absent sensors (no IMU/baro on Fruit Jam) are handled by probe-first
# detection — those subsystems simply don't activate.
#
# [N2] Full flight codebase inclusion is intentional. Dormant subsystems
# activate when hardware is present (e.g., plug in an IMU via Qwiic).
# A relay node, balloon GCS, or mothership all run the same binary
# with different hardware and different Mission Profile settings.
#
# Build:
#   cmake -B build_gs -S ground_station -G Ninja
#   cmake --build build_gs
#
# Output: build_gs/rocketchip.uf2

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

project(rocketchip C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

# ============================================================================
# Build Configuration (mirrors main CMakeLists.txt)
# ============================================================================

set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_definitions(DEBUG=1)
endif()

add_compile_definitions(PICO_USE_STACK_GUARDS=1)
add_compile_definitions(PICO_STACK_SIZE=0x1000)

# Station mission profile: RX mode (compile-time behavioral selection)
add_compile_definitions(ROCKETCHIP_MISSION_STATION=1)

add_compile_options(
    -Wall
    -Wextra
    -Werror
    -Wno-unused-parameter
)

# ============================================================================
# External Libraries (same paths as main build, relative to parent)
# ============================================================================

set(RUUVI_DPS310_DIR ${CMAKE_CURRENT_LIST_DIR}/../lib/ruuvi.dps310.c/src)
set(LWGPS_DIR ${CMAKE_CURRENT_LIST_DIR}/../lib/lwgps/lwgps/src)

# ============================================================================
# Main Executable — SAME sources as flight build
# ============================================================================

add_executable(rocketchip
    ${CMAKE_CURRENT_LIST_DIR}/../src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/ws2812_status.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/i2c_bus.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/icm20948.cpp
    ${RUUVI_DPS310_DIR}/dps310.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/baro_dps310.cpp
    ${LWGPS_DIR}/lwgps/lwgps.c
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/gps_pa1010d.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/gps_uart.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/calibration/calibration_data.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/calibration/calibration_manager.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/calibration/calibration_storage.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/cli/rc_os.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/math/vec3.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/math/quat.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/eskf.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/eskf_codegen.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/wmm_declination.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/mahony_ahrs.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/ud_factor.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/data_convert.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/pcm_frame.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/ring_buffer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/flight_table.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/log_decimator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/flash_flush.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/psram_init.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/spi_bus.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/rfm95w.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/telemetry/telemetry_encoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/telemetry/telemetry_service.cpp
)

# Include directories (same as main build)
target_include_directories(rocketchip PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/../src
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../src/calibration
    ${CMAKE_CURRENT_LIST_DIR}/../src/cli
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging
    ${CMAKE_CURRENT_LIST_DIR}/../src/telemetry
    ${RUUVI_DPS310_DIR}
    ${LWGPS_DIR}/include
)

# Generate WS2812 PIO header
pico_generate_pio_header(rocketchip
    ${PICO_SDK_PATH}/src/rp2_common/pico_status_led/ws2812.pio
)

# Link libraries (same as main build)
target_link_libraries(rocketchip
    pico_stdlib
    hardware_gpio
    hardware_pio
    hardware_clocks
    hardware_i2c
    hardware_uart
    hardware_flash
    pico_flash
    pico_multicore
    hardware_exception
    hardware_watchdog
    pico_rand
    hardware_spi
    hardware_xip_cache
)

pico_enable_stdio_usb(rocketchip 1)
pico_enable_stdio_uart(rocketchip 0)

target_compile_definitions(rocketchip PRIVATE
    ESKF_USE_BIERMAN=1
)

# USB configuration — LED pin is board-specific
target_compile_definitions(rocketchip PRIVATE
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
    PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED=29
)

# -Wpedantic for our sources
set(ROCKETCHIP_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/ws2812_status.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/i2c_bus.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/icm20948.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/baro_dps310.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/gps_pa1010d.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/gps_uart.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/calibration/calibration_data.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/calibration/calibration_manager.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/calibration/calibration_storage.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/cli/rc_os.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/math/vec3.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/math/quat.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/eskf.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/eskf_codegen.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/wmm_declination.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/mahony_ahrs.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/fusion/ud_factor.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/data_convert.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/pcm_frame.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/ring_buffer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/flight_table.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/log_decimator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/flash_flush.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/logging/psram_init.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/spi_bus.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/drivers/rfm95w.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/telemetry/telemetry_encoder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/telemetry/telemetry_service.cpp
)
set_source_files_properties(${ROCKETCHIP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wpedantic")

# Per-file -O2 for ESKF (same as main build)
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/../src/fusion/eskf.cpp
    PROPERTIES COMPILE_OPTIONS "-O2;-Wpedantic")
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/../src/fusion/eskf_codegen.cpp
    PROPERTIES COMPILE_OPTIONS "-O2;-Wpedantic")

pico_add_extra_outputs(rocketchip)

# Print build info
message(STATUS "")
message(STATUS "=== RocketChip Ground Station Build ===")
message(STATUS "Board: ${PICO_BOARD}")
message(STATUS "Platform: ${PICO_PLATFORM}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Same source as flight firmware — board.h selects pin defs")
message(STATUS "")
