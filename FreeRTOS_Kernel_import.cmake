# This is a copy of <FREERTOS_KERNEL_PATH>/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake

# This can be dropped into an external project to help locate the FreeRTOS kernel
# It should be include()ed prior to project()

if (DEFINED ENV{FREERTOS_KERNEL_PATH} AND (NOT FREERTOS_KERNEL_PATH))
    set(FREERTOS_KERNEL_PATH $ENV{FREERTOS_KERNEL_PATH})
    message("Using FREERTOS_KERNEL_PATH from environment ('${FREERTOS_KERNEL_PATH}')")
endif ()

if (NOT FREERTOS_KERNEL_PATH)
    # Check for local FreeRTOS-Kernel submodule first
    if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel/include/FreeRTOS.h")
        set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel")
        message("Using local FreeRTOS-Kernel submodule at '${FREERTOS_KERNEL_PATH}'")
    else ()
        message(FATAL_ERROR
                "FreeRTOS kernel location was not specified. Please set FREERTOS_KERNEL_PATH."
                )
    endif ()
endif ()

get_filename_component(FREERTOS_KERNEL_PATH "${FREERTOS_KERNEL_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${FREERTOS_KERNEL_PATH})
    message(FATAL_ERROR "Directory '${FREERTOS_KERNEL_PATH}' not found")
endif ()

if (NOT EXISTS ${FREERTOS_KERNEL_PATH}/include/FreeRTOS.h)
    message(FATAL_ERROR "Directory '${FREERTOS_KERNEL_PATH}' does not appear to contain the FreeRTOS kernel")
endif ()

set(FREERTOS_KERNEL_PATH ${FREERTOS_KERNEL_PATH} CACHE PATH "Path to the FreeRTOS kernel" FORCE)

# Add the FreeRTOS portable layer for RP2040/RP2350
# The kernel provides SMP support via the RP2040 port (also works for RP2350)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
