# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Set board and platform BEFORE including pico_sdk_import.cmake
set(PICO_BOARD "adafruit_feather_rp2350")
set(PICO_PLATFORM "rp2350")

# Include the Pico SDK import
include(pico_sdk_import.cmake)

# Project definition
project(rocket_chip_freertos_validation C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Set FreeRTOS config directory BEFORE including FreeRTOS_Kernel_import.cmake
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_SOURCE_DIR}" CACHE STRING "" FORCE)

# Include FreeRTOS Kernel
include(FreeRTOS_Kernel_import.cmake)

# ============================================================================
# Common Include Directory and Debug Configuration
# ============================================================================

# Add common include path for debug.h and other shared headers
set(ROCKETCHIP_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

# Enable debug output by default (disable for release builds)
# To disable: cmake -DCONFIG_DEBUG=OFF ..
option(CONFIG_DEBUG "Enable debug output macros" ON)

# ============================================================================
# HAL Library
# ============================================================================

add_library(hal STATIC
    src/hal/Bus.cpp
    src/hal/GPIO.cpp
    src/hal/ADC.cpp
    src/hal/PWM.cpp
    src/hal/UART.cpp
    src/hal/Timing.cpp
    src/hal/PIO.cpp
    src/hal/HAL.cpp
    src/hal/Radio_RFM95W.cpp
)

target_include_directories(hal PUBLIC
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(hal PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(hal PUBLIC
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_gpio
    hardware_adc
    hardware_uart
    hardware_timer
    hardware_pio
    hardware_pwm
    hardware_watchdog
    hardware_clocks
    FreeRTOS-Kernel
)

# Note: USBSerial support requires project to link tinyusb_device
# and provide tusb_config.h. Enable with pico_enable_stdio_usb().

# ============================================================================
# ST Platform-Independent Sensor Drivers
# ============================================================================

# ISM330DHCX IMU driver (STMicroelectronics)
add_library(st_ism330dhcx INTERFACE)
target_include_directories(st_ism330dhcx INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/ism330dhcx-pid
)
target_sources(st_ism330dhcx INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/ism330dhcx-pid/ism330dhcx_reg.c
)

# LIS3MDL Magnetometer driver (STMicroelectronics)
add_library(st_lis3mdl INTERFACE)
target_include_directories(st_lis3mdl INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/lis3mdl-pid
)
target_sources(st_lis3mdl INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/lis3mdl-pid/lis3mdl_reg.c
)

# DPS310 Barometer driver (Ruuvi) - source files are in src/ subdirectory
add_library(ruuvi_dps310 INTERFACE)
target_include_directories(ruuvi_dps310 INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/ruuvi-dps310/src
)
target_sources(ruuvi_dps310 INTERFACE
    ${CMAKE_SOURCE_DIR}/lib/st_drivers/ruuvi-dps310/src/dps310.c
)

# ============================================================================
# HAL Sensors Library (C++ wrappers around ST drivers)
# ============================================================================

add_library(hal_sensors STATIC
    src/hal/IMU_ISM330DHCX.cpp
    src/hal/Mag_LIS3MDL.cpp
    src/hal/Baro_DPS310.cpp
    src/hal/GPS_PA1010D.cpp
)

target_include_directories(hal_sensors PUBLIC
    ${CMAKE_SOURCE_DIR}/src/hal
)

target_link_libraries(hal_sensors PUBLIC
    hal
    st_ism330dhcx
    st_lis3mdl
    ruuvi_dps310
)

# ============================================================================
# AP_HAL_RP2350 - ArduPilot HAL Implementation for RP2350
# ============================================================================

add_library(ap_hal_rp2350 STATIC
    lib/AP_HAL_RP2350/Scheduler.cpp
    lib/AP_HAL_RP2350/Semaphores.cpp
    lib/AP_HAL_RP2350/Util.cpp
    lib/AP_HAL_RP2350/HAL_RP2350_Class.cpp
)

target_include_directories(ap_hal_rp2350 PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/AP_HAL_RP2350
    ${CMAKE_SOURCE_DIR}/lib
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_hal_rp2350 PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_hal_rp2350 PUBLIC
    pico_stdlib
    pico_unique_id
    hardware_timer
    hardware_watchdog
    hardware_sync
    FreeRTOS-Kernel
)

# ============================================================================
# ArduPilot Compatibility Shim (Legacy - will be replaced by ap_hal_rp2350)
# ============================================================================

add_library(ap_compat STATIC
    lib/ap_compat/AP_HAL_Compat.cpp
)

target_include_directories(ap_compat PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/src/hal
    ${ROCKETCHIP_INCLUDE_DIR}
)

if(CONFIG_DEBUG)
    target_compile_definitions(ap_compat PUBLIC CONFIG_DEBUG)
endif()

target_link_libraries(ap_compat PUBLIC
    hal
)

# ============================================================================
# ArduPilot Math Library (AP_Math)
# ============================================================================

add_library(ap_math STATIC
    lib/ardupilot/libraries/AP_Math/AP_Math.cpp
    lib/ardupilot/libraries/AP_Math/vector2.cpp
    lib/ardupilot/libraries/AP_Math/vector3.cpp
    lib/ardupilot/libraries/AP_Math/matrix3.cpp
    lib/ardupilot/libraries/AP_Math/matrixN.cpp
    lib/ardupilot/libraries/AP_Math/matrix_alg.cpp
    lib/ardupilot/libraries/AP_Math/quaternion.cpp
    lib/ardupilot/libraries/AP_Math/location.cpp
    lib/ardupilot/libraries/AP_Math/polygon.cpp
    lib/ardupilot/libraries/AP_Math/crc.cpp
)

target_include_directories(ap_math PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
)

target_link_libraries(ap_math PUBLIC
    ap_compat
)

target_compile_definitions(ap_math PUBLIC
    AP_PARAM_ENABLED=0
    HAL_GCS_ENABLED=0
    HAL_WITH_EKF_DOUBLE=0
)

# ============================================================================
# ArduPilot Filter Library
# ============================================================================

add_library(ap_filter STATIC
    lib/ardupilot/libraries/Filter/LowPassFilter.cpp
    lib/ardupilot/libraries/Filter/LowPassFilter2p.cpp
    lib/ardupilot/libraries/Filter/NotchFilter.cpp
    lib/ardupilot/libraries/Filter/DerivativeFilter.cpp
    lib/ardupilot/libraries/Filter/ModeFilter.cpp
    lib/ardupilot/libraries/Filter/SlewLimiter.cpp
)

target_include_directories(ap_filter PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
)

target_link_libraries(ap_filter PUBLIC
    ap_math
)

# ============================================================================
# ArduPilot Accelerometer Calibrator
# ============================================================================

add_library(ap_calibrators STATIC
    lib/ardupilot/libraries/AP_AccelCal/AccelCalibrator.cpp
)

target_include_directories(ap_calibrators PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/ap_compat
    ${CMAKE_SOURCE_DIR}/lib/ardupilot/libraries
)

target_link_libraries(ap_calibrators PUBLIC
    ap_math
)

# ============================================================================
# Services Library (FreeRTOS Tasks)
# ============================================================================

add_library(services STATIC
    src/services/SensorTask.cpp
)

target_include_directories(services PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/services
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_link_libraries(services PUBLIC
    hal
    hal_sensors
    FreeRTOS-Kernel
)

# ============================================================================
# Production Target: rocketchip
# ============================================================================

add_executable(rocketchip
    src/main.cpp
)

target_include_directories(rocketchip PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${ROCKETCHIP_INCLUDE_DIR}
)

target_link_libraries(rocketchip
    services
    hal
    hal_sensors
    ap_compat
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_spi
    hardware_dma
    hardware_timer
    hardware_clocks
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(rocketchip 1)
pico_enable_stdio_uart(rocketchip 0)

pico_add_extra_outputs(rocketchip)

# ============================================================================
# Validation Target: freertos_validation (preserved for reference/testing)
# ============================================================================

add_executable(freertos_validation
    tests/validation/freertos_validation/main.c
    tests/validation/freertos_validation/hooks.c
)

target_link_libraries(freertos_validation
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_spi
    hardware_dma
    hardware_timer
    hardware_clocks
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(freertos_validation 1)
pico_enable_stdio_uart(freertos_validation 0)

pico_add_extra_outputs(freertos_validation)

# Simple test executable (no FreeRTOS)
add_executable(simple_test
    tests/smoke_tests/simple_test.c
)

target_link_libraries(simple_test
    pico_stdlib
)

pico_enable_stdio_usb(simple_test 1)
pico_enable_stdio_uart(simple_test 0)

pico_add_extra_outputs(simple_test)

# ============================================================================
# Smoke Tests
# ============================================================================

# IMU Qwiic Port Test - verifies ISM330DHCX + LIS3MDL FeatherWing connectivity
add_executable(smoke_imu_qwiic
    tests/smoke_tests/imu_qwiic_test.c
)

target_link_libraries(smoke_imu_qwiic
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(smoke_imu_qwiic 1)
pico_enable_stdio_uart(smoke_imu_qwiic 0)

pico_add_extra_outputs(smoke_imu_qwiic)

# HAL Validation Test - comprehensive HAL smoke test
add_executable(smoke_hal_validation
    tests/smoke_tests/hal_validation.cpp
)

target_link_libraries(smoke_hal_validation
    hal
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_hal_validation 1)
pico_enable_stdio_uart(smoke_hal_validation 0)

pico_add_extra_outputs(smoke_hal_validation)

# ST Driver Sensor Test - validates all three ST driver wrappers
add_executable(smoke_st_sensors
    tests/smoke_tests/st_sensors_test.cpp
)

target_link_libraries(smoke_st_sensors
    hal_sensors
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_st_sensors 1)
pico_enable_stdio_uart(smoke_st_sensors 0)

pico_add_extra_outputs(smoke_st_sensors)

# GPS Module Test - validates PA1010D GPS driver
add_executable(smoke_gps
    tests/smoke_tests/gps_test.cpp
)

target_link_libraries(smoke_gps
    hal_sensors
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_gps 1)
pico_enable_stdio_uart(smoke_gps 0)

pico_add_extra_outputs(smoke_gps)

# I2C Bus Scanner - debug tool to find devices
add_executable(i2c_scan
    tests/smoke_tests/i2c_scan.c
)

target_link_libraries(i2c_scan
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(i2c_scan 1)
pico_enable_stdio_uart(i2c_scan 0)

pico_add_extra_outputs(i2c_scan)

# ============================================================================
# Radio Tests
# ============================================================================

# Radio TX Test - transmits debug packets
add_executable(smoke_radio_tx
    tests/smoke_tests/radio_tx_test.cpp
)

target_link_libraries(smoke_radio_tx
    hal
    pico_stdlib
    hardware_spi
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_radio_tx 1)
pico_enable_stdio_uart(smoke_radio_tx 0)

pico_add_extra_outputs(smoke_radio_tx)

# ============================================================================
# Ground Station
# ============================================================================

# Radio RX - Ground station receiver
add_executable(radio_rx
    ground_station/radio_rx.cpp
)

target_link_libraries(radio_rx
    hal
    pico_stdlib
    hardware_spi
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(radio_rx 1)
pico_enable_stdio_uart(radio_rx 0)

pico_add_extra_outputs(radio_rx)

# ============================================================================
# Calibration Smoke Test
# ============================================================================

add_executable(smoke_calibration
    tests/smoke_tests/calibration_test.cpp
)

target_link_libraries(smoke_calibration
    hal_sensors
    ap_calibrators
    ap_math
    pico_stdlib
    hardware_i2c
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_calibration 1)
pico_enable_stdio_uart(smoke_calibration 0)

pico_add_extra_outputs(smoke_calibration)

# ============================================================================
# AP_HAL_RP2350 Smoke Test
# ============================================================================

add_executable(smoke_ap_hal
    tests/smoke_tests/ap_hal_test.cpp
)

target_link_libraries(smoke_ap_hal
    ap_hal_rp2350
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(smoke_ap_hal 1)
pico_enable_stdio_uart(smoke_ap_hal 0)

pico_add_extra_outputs(smoke_ap_hal)

# ============================================================================
# Minimal Blink Test (no FreeRTOS - for debugging boot issues)
# ============================================================================

add_executable(minimal_blink
    tests/smoke_tests/minimal_blink.cpp
)

target_link_libraries(minimal_blink
    pico_stdlib
)

pico_add_extra_outputs(minimal_blink)

# FreeRTOS Static Allocation Test - minimal test without AP_HAL
add_executable(freertos_static_test
    tests/smoke_tests/freertos_static_test.cpp
)

target_link_libraries(freertos_static_test
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_add_extra_outputs(freertos_static_test)

# FreeRTOS with AP_HAL include (but no HAL usage) - tests global object
add_executable(freertos_with_hal_include
    tests/smoke_tests/freertos_with_hal_include.cpp
)

target_link_libraries(freertos_with_hal_include
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    ap_hal_rp2350
)

pico_add_extra_outputs(freertos_with_hal_include)
