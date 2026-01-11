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

# Main executable
add_executable(freertos_validation
    src/main.c
    src/hooks.c
)

# Link libraries
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

# Enable USB serial output, disable UART
pico_enable_stdio_usb(freertos_validation 1)
pico_enable_stdio_uart(freertos_validation 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(freertos_validation)

# Simple test executable (no FreeRTOS)
add_executable(simple_test
    src/simple_test.c
)

target_link_libraries(simple_test
    pico_stdlib
)

pico_enable_stdio_usb(simple_test 1)
pico_enable_stdio_uart(simple_test 0)

pico_add_extra_outputs(simple_test)
