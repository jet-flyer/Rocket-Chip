cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350-arm-s)  # RP2350 ARM cores (for SMP)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/external/FreeRTOS-Kernel)

include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)

project(rocket_chip_firmware C CXX ASM)

pico_sdk_init()

add_executable(rocket_chip_firmware
    src/main.cpp
)

target_include_directories(rocket_chip_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src  # FreeRTOSConfig.h
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ
)

target_link_libraries(rocket_chip_firmware
    pico_stdlib
    pico_multicore  # Dual-core support
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_add_extra_outputs(rocket_chip_firmware)