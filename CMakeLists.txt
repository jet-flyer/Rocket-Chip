# RocketChip CMake Build System
#
# Two build modes:
#   Target build (default): cmake -B build -G Ninja
#   Host tests:             cmake -B build_host -G Ninja -DBUILD_TESTS=ON
#
# The BUILD_TESTS gate selects between Pico SDK target build and
# host-side test build. They are mutually exclusive.

cmake_minimum_required(VERSION 3.13)

option(BUILD_TESTS "Build host-side tests (no Pico SDK)" OFF)

if(BUILD_TESTS)
    # ========================================================================
    # Host Build: Pure C++ libraries + Google Test (no Pico SDK)
    # ========================================================================

    project(rocketchip_tests CXX C)

    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    # Google Test (pinned v1.14.0)
    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    # Prevent gtest from overriding our compiler/linker settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # ------------------------------------------------------------------
    # Math library (no SDK dependencies)
    # Source files added as IVP-39/40 are implemented
    # ------------------------------------------------------------------
    add_library(rc_math STATIC
        src/math/vec3.cpp
        src/math/quat.cpp
    )
    target_include_directories(rc_math PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
    )

    # ------------------------------------------------------------------
    # Fusion library (no SDK dependencies)
    # Source files added as IVP-41+ are implemented
    # ------------------------------------------------------------------
    add_library(rc_fusion STATIC
        src/fusion/baro_kf.cpp
        src/fusion/eskf.cpp
    )
    target_link_libraries(rc_fusion PUBLIC rc_math)

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------
    enable_testing()
    add_subdirectory(test)

else()
    # ========================================================================
    # Target Build: Pico SDK for RP2350 (existing build — unchanged)
    # ========================================================================

    # Pico VS Code Extension Integration (handles toolchain and picotool)
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

    # Board and Platform Configuration (MUST be before pico_sdk_import)
    set(PICO_BOARD "adafruit_feather_rp2350" CACHE STRING "Board type")

    # Pull in SDK (must be before project())
    include(pico_sdk_import.cmake)

    # Project definition
    project(rocketchip C CXX ASM)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    # Initialize the SDK
    pico_sdk_init()

    # ============================================================================
    # Build Configuration
    # ============================================================================

    # Force Debug build for development (enables DBG_* macros)
    # Pico SDK defaults to Release, but we want debug output during development
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)

    # Add DEBUG define for debug builds
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        add_compile_definitions(DEBUG=1)
    endif()

    # IVP-29: SDK-level stack limit registers (belt-and-suspenders with MPU guard)
    add_compile_definitions(PICO_USE_STACK_GUARDS=1)

    # Compiler warnings — applied globally (includes SDK sources)
    add_compile_options(
        -Wall
        -Wextra
        -Werror
        -Wno-unused-parameter
    )
    # -Wpedantic applied only to our source files (SDK triggers
    # -Wpedantic violations we can't fix, e.g. void* to fn-ptr casts)
    # Added via set_source_files_properties below.

    # ============================================================================
    # External Libraries
    # ============================================================================

    # ruuvi.dps310.c - Proven DPS310 barometer driver
    set(RUUVI_DPS310_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/ruuvi.dps310.c/src)

    # lwGPS - NMEA parser
    set(LWGPS_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/lwgps/lwgps/src)

    # ============================================================================
    # Main Executable
    # ============================================================================

    add_executable(rocketchip
        # -------------------------------------------------------------------------
        # Stage 1: Foundation (IVP-01 through IVP-08)
        # -------------------------------------------------------------------------
        src/main.cpp
        src/drivers/ws2812_status.cpp
        src/drivers/i2c_bus.cpp

        # -------------------------------------------------------------------------
        # Stage 2: Single-Core Sensors (IVP-09 through IVP-18)
        # Re-enable one at a time with verification at each IVP gate
        # -------------------------------------------------------------------------
        # IVP-09: IMU driver
        src/drivers/icm20948.cpp

        # IVP-11: Barometer (requires ruuvi library)
        ${RUUVI_DPS310_DIR}/dps310.c
        src/drivers/baro_dps310.cpp

        # IVP-31: GPS (requires lwGPS library)
        ${LWGPS_DIR}/lwgps/lwgps.c
        src/drivers/gps_pa1010d.cpp

        # IVP-14: Calibration system
        src/calibration/calibration_data.cpp
        src/calibration/calibration_manager.cpp
        src/calibration/calibration_storage.cpp

        # IVP-18: RC_OS CLI
        src/cli/rc_os.cpp

        # -------------------------------------------------------------------------
        # Stage 5: Sensor Fusion Math (IVP-39 through IVP-41)
        # Pure C++ — no SDK dependencies. Compiles on host and target.
        # -------------------------------------------------------------------------
        # IVP-39: Vec3 + Quaternion
        src/math/vec3.cpp
        src/math/quat.cpp

        # IVP-41: Baro KF (first fusion code)
        src/fusion/baro_kf.cpp

        # IVP-42a: ESKF core propagation
        src/fusion/eskf.cpp
    )

    # -Wpedantic for our source files only (SDK sources are excluded)
    set(ROCKETCHIP_SOURCES
        src/main.cpp
        src/drivers/ws2812_status.cpp
        src/drivers/i2c_bus.cpp
        src/drivers/icm20948.cpp
        src/drivers/baro_dps310.cpp
        src/drivers/gps_pa1010d.cpp
        src/calibration/calibration_data.cpp
        src/calibration/calibration_manager.cpp
        src/calibration/calibration_storage.cpp
        src/cli/rc_os.cpp
        src/math/vec3.cpp
        src/math/quat.cpp
        src/fusion/baro_kf.cpp
        src/fusion/eskf.cpp
    )
    set_source_files_properties(${ROCKETCHIP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wpedantic")

    # Include directories
    target_include_directories(rocketchip PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/drivers
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src/calibration    # IVP-14
        ${CMAKE_CURRENT_LIST_DIR}/src/cli            # IVP-18
        ${RUUVI_DPS310_DIR}                            # IVP-11
        ${LWGPS_DIR}/include                             # IVP-31
    )

    # Generate WS2812 PIO header from SDK's pio file
    pico_generate_pio_header(rocketchip
        ${PICO_SDK_PATH}/src/rp2_common/pico_status_led/ws2812.pio
    )

    # Link libraries
    target_link_libraries(rocketchip
        pico_stdlib
        hardware_gpio
        hardware_pio         # For NeoPixel/WS2812 PIO driver
        hardware_clocks      # For clock_get_hz()
        hardware_i2c         # For I2C/QWIIC sensors
        # IVP-14: Calibration storage
        hardware_flash       # For flash_range_erase/program
        pico_flash           # For flash_safe_execute
        # IVP-19: Dual-core support
        pico_multicore       # For multicore_launch_core1, inter-core primitives
        # IVP-29: MPU stack guard (exception handling)
        hardware_exception
        # IVP-30: Hardware watchdog
        hardware_watchdog
        # IVP-35: Mag calibration (Fisher-Yates shuffle with hardware TRNG)
        pico_rand
    )

    # Enable USB CDC for stdio (not UART)
    pico_enable_stdio_usb(rocketchip 1)
    pico_enable_stdio_uart(rocketchip 0)

    # USB configuration
    target_compile_definitions(rocketchip PRIVATE
        PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
        PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED=7
    )

    # Generate UF2 file for easy flashing
    pico_add_extra_outputs(rocketchip)

    # ============================================================================
    # Standalone Benchmark (IVP-40 gate — not production firmware)
    # ============================================================================

    add_executable(mat_benchmark
        src/tools/mat_benchmark.cpp
        src/math/vec3.cpp          # Required by rc_math (baro_kf dependency)
        src/fusion/baro_kf.cpp
    )
    target_include_directories(mat_benchmark PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
    )
    # Benchmark must use -O2 — Debug (-O0) gives wildly pessimistic numbers.
    # Flight firmware will also run optimized (Release or RelWithDebInfo).
    target_compile_options(mat_benchmark PRIVATE -O2)
    target_link_libraries(mat_benchmark
        pico_stdlib
    )
    pico_enable_stdio_usb(mat_benchmark 1)
    pico_enable_stdio_uart(mat_benchmark 0)
    pico_add_extra_outputs(mat_benchmark)

    # ============================================================================
    # Print Build Info
    # ============================================================================

    message(STATUS "")
    message(STATUS "=== RocketChip Build Configuration ===")
    message(STATUS "Board: ${PICO_BOARD}")
    message(STATUS "Platform: ${PICO_PLATFORM}")
    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
    message(STATUS "Platform: Bare-metal Pico SDK")
    message(STATUS "")

endif()
