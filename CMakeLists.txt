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
        src/fusion/eskf_codegen.cpp
        src/fusion/ud_factor.cpp
        src/fusion/wmm_tables.cpp
        src/fusion/mahony_ahrs.cpp
        src/fusion/innovation_monitor.cpp
        src/fusion/confidence_gate.cpp
        src/fusion/eskf_brake.cpp
    )
    target_link_libraries(rc_fusion PUBLIC rc_math)

    # ------------------------------------------------------------------
    # Logging library (no SDK dependencies)
    # IVP-49/50/51: Data model, timestamps, PCM frame format
    # IVP-52b: Ring buffer
    # IVP-52c: Log decimator
    # IVP-53a: CRC-32 + Flight log table
    # ------------------------------------------------------------------
    add_library(rc_logging STATIC
        src/logging/data_convert.cpp
        src/logging/pcm_frame.cpp
        src/logging/ring_buffer.cpp
        src/logging/flight_table.cpp
        src/logging/log_decimator.cpp
    )
    target_include_directories(rc_logging PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
    )

    # ------------------------------------------------------------------
    # Telemetry library (no SDK dependencies)
    # IVP-58: CCSDS + MAVLink encoders
    # ------------------------------------------------------------------
    add_library(rc_telemetry STATIC
        src/telemetry/telemetry_encoder.cpp
        src/telemetry/mavlink_rx.cpp
    )
    target_include_directories(rc_telemetry PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/logging
        ${CMAKE_CURRENT_LIST_DIR}/lib/mavlink
    )

    # ------------------------------------------------------------------
    # QEP library (IVP-67, vendored QP/C 8.1.3 HSM dispatch engine)
    # Portable C99 — compiles on host and target.
    # ROCKETCHIP_HOST_TEST selects no-op critical sections for testing.
    # ------------------------------------------------------------------
    add_library(qep STATIC
        lib/qep/qep_hsm.c
        lib/qep/qp_app.c
        # IVP-75: QF Active Object framework + QV cooperative scheduler
        lib/qep/qf_act.c
        lib/qep/qf_actq.c
        lib/qep/qf_defer.c
        lib/qep/qf_dyn.c
        lib/qep/qf_mem.c
        lib/qep/qf_ps.c
        lib/qep/qf_qact.c
        lib/qep/qf_qeq.c
        lib/qep/qf_time.c
        lib/qep/qv.c
        lib/qep/bsp_qv.c
    )
    target_include_directories(qep PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/lib/qep
    )
    target_compile_definitions(qep PUBLIC ROCKETCHIP_HOST_TEST=1)

    # ------------------------------------------------------------------
    # Flight Director library (IVP-68, QEP-based HSM + flight state)
    # Pure C++ — no Pico SDK. Links QEP for host-side HSM dispatch tests.
    # ------------------------------------------------------------------
    add_library(rc_flight_director STATIC
        src/flight_director/flight_director.cpp
        src/flight_director/go_nogo_checks.cpp
        src/flight_director/command_handler.cpp
        src/flight_director/guard_functions.cpp
        src/flight_director/guard_evaluator.cpp
        src/flight_director/guard_combinator.cpp
        src/flight_director/action_executor.cpp
    )
    target_include_directories(rc_flight_director PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/flight_director
        ${CMAKE_CURRENT_LIST_DIR}/lib/qep
    )
    target_link_libraries(rc_flight_director PUBLIC qep)
    target_compile_definitions(rc_flight_director PUBLIC ROCKETCHIP_HOST_TEST=1)

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
    # Default PICO_BOARD depends on job role:
    #   - Station role (ROCKETCHIP_JOB_STATION=1) → Fruit Jam
    #   - Vehicle role (default)                  → Feather RP2350
    # Users can still override explicitly with -DPICO_BOARD=...
    # Rationale: prevents Frankenstein builds where station role runs with
    # Feather pin config. See AGENT_WHITEBOARD 2026-04-22, LL Entry 36.
    if(ROCKETCHIP_JOB_STATION AND NOT DEFINED PICO_BOARD AND NOT DEFINED ENV{PICO_BOARD})
        set(PICO_BOARD "adafruit_fruit_jam" CACHE STRING "Board type")
        message(STATUS "ROCKETCHIP_JOB_STATION=1 detected, defaulting PICO_BOARD=adafruit_fruit_jam")
    else()
        set(PICO_BOARD "adafruit_feather_rp2350" CACHE STRING "Board type")
    endif()

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

    # Increase USB CDC TX buffer from 64 to 1024 bytes.
    # Default 64B overflows with MAVLink telemetry (~150B per 10Hz tick).
    add_compile_definitions(CFG_TUD_CDC_TX_BUFSIZE=1024)
    add_compile_definitions(CFG_TUD_CDC_RX_BUFSIZE=256)

    # Reduce stdio USB stdout timeout from 500ms to 10ms.
    # Default 500ms blocks the QF scheduler when QGC pauses reading,
    # causing missed heartbeats and QGC "Communication Lost" after ~10s.
    add_compile_definitions(PICO_STDIO_USB_STDOUT_TIMEOUT_US=10000)

    # Core 0 stack: 4KB (SDK default is 2KB = 0x800, insufficient for ESKF call chain).
    # SCRATCH_Y is 4KB total with 0 bytes of .scratch_y data — full 4KB available.
    add_compile_definitions(PICO_STACK_SIZE=0x1000)

    # ============================================================
    # RP2350-ONLY — silicon erratum E2 workaround (SDK 2.2.0 issue).
    # ============================================================
    # Do NOT apply on RP2040 or other non-RP2350 chips — the
    # PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL flag affects per-core
    # M33_ACTLR setup during runtime_init, which is specific to
    # Cortex-M33 / RP2350. On RP2040 (Cortex-M0+) the flag has no
    # effect but is meaningless.
    #
    # RP2350-E2 (datasheet Appendix E, p.1373): SIO spinlock mirror
    # writes. SDK 2.2.0 uses SW spinlocks by default on RP2350 which
    # avoids the primary path. This additional flag closes a secondary
    # boot-deadlock path in the SW-spinlock impl where the SDK's
    # default `M33_ACTLR.EXTEXCLALL` setup interacts with LDAEXB/STREXB
    # in a way that deadlocks Core 0 in `spin_lock_blocking` before
    # main() (symptom: PC=0xeffffffe, LR=0xffffffe9, USB CDC never
    # enumerates, Core 1 stuck at bootrom 0x000000da). Workaround
    # disables EXTEXCLALL in sync_spin_lock.c:28-42.
    #
    # Affects steppings A2/A3/A4 on both RP2350A (Feather HSTX) and
    # RP2350B (Fruit Jam). Reporter-confirmed per pico-sdk #2495, #2706.
    # Known in-tree gaps R-1 (warm reboot from `picotool -f`) and R-2
    # (SWD halt during spinlock wait) are tracked in the Incident log
    # of standards/RP2350_ERRATA.md — not addressed by this flag.
    #
    # Full erratum tracking: standards/RP2350_ERRATA.md (E2 row).
    # Any new RP2350-specific compile definition added here MUST be
    # cross-referenced against that document — if it's not in the
    # matrix, verify it has no errata implications and add a row if
    # it does.
    if(PICO_PLATFORM STREQUAL "rp2350" OR PICO_PLATFORM STREQUAL "rp2350-arm-s"
       OR PICO_PLATFORM STREQUAL "rp2350-riscv")
        add_compile_definitions(PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1)
        message(STATUS "RP2350-E2 workaround enabled: PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1")
    endif()

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
        src/core1/sensor_core1.cpp
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
        src/drivers/gps_uart.cpp

        # Stage 16C IVP-142a: MCU die-temperature sensor
        src/drivers/mcu_temp.cpp

        # IVP-14: Calibration system
        src/calibration/calibration_data.cpp
        src/calibration/calibration_manager.cpp
        src/calibration/calibration_storage.cpp
        src/calibration/cal_hooks.cpp

        # Stage T IVP-T5.5: radio config persistence
        src/logging/radio_config_storage.cpp

        # IVP-18: RC_OS CLI
        src/cli/rc_os.cpp
        src/cli/rc_os_dashboard.cpp
        src/cli/rc_os_commands.cpp

        # Dev diagnostics (excluded from flight builds)
        $<$<NOT:$<BOOL:${BUILD_FOR_FLIGHT}>>:src/dev/dev_cli.cpp>
        $<$<NOT:$<BOOL:${BUILD_FOR_FLIGHT}>>:src/dev/fault_inject.cpp>
        $<$<NOT:$<BOOL:${BUILD_FOR_FLIGHT}>>:src/dev/replay_inject.cpp>
        # diag_stats.cpp has both bench and flight tier content:
        # - Always: diag_stats_t0_preconditions() for soak Frankenstein checks
        # - Bench only (internal guard): diag_stats_dump() + msp_tick()
        src/dev/diag_stats.cpp
        $<$<NOT:$<BOOL:${BUILD_FOR_FLIGHT}>>:src/dev/station_fault_inject.cpp>
        $<$<NOT:$<BOOL:${BUILD_FOR_FLIGHT}>>:src/dev/station_replay.cpp>

        # -------------------------------------------------------------------------
        # Stage 5: Sensor Fusion Math (IVP-39+)
        # Pure C++ — no SDK dependencies. Compiles on host and target.
        # -------------------------------------------------------------------------
        # IVP-39: Vec3 + Quaternion
        src/math/vec3.cpp
        src/math/quat.cpp

        # IVP-42a: ESKF core propagation
        src/fusion/eskf.cpp
        src/fusion/eskf_codegen.cpp
        src/fusion/eskf_runner.cpp
        src/fusion/eskf_brake.cpp

        # IVP-44: WMM declination lookup table
        src/fusion/wmm_tables.cpp

        # IVP-45: Mahony AHRS cross-check
        src/fusion/mahony_ahrs.cpp

        # UD factorization (Bierman measurement update path)
        src/fusion/ud_factor.cpp

        # IVP-83: Innovation monitor + IVP-84: Confidence gate
        src/fusion/innovation_monitor.cpp
        src/fusion/confidence_gate.cpp

        # IVP-88: PIO heartbeat watchdog + IVP-89: Backup timers
        src/safety/pio_watchdog.cpp
        src/safety/pio_backup_timer.cpp
        src/safety/pyro_edge_logger.cpp

        # Phase 6: Health Monitor (called from AO_FlightDirector at 10Hz)
        src/safety/health_monitor.cpp

        # -------------------------------------------------------------------------
        # Stage 6: Data Logging (IVP-49/50/51, IVP-52a/b/c, IVP-53a/b)
        # -------------------------------------------------------------------------
        src/logging/data_convert.cpp
        src/logging/pcm_frame.cpp
        src/logging/ring_buffer.cpp
        src/logging/flight_table.cpp
        src/logging/log_decimator.cpp
        src/logging/flash_flush.cpp
        src/logging/psram_init.cpp

        # -------------------------------------------------------------------------
        # Stage 7: Radio & Telemetry (IVP-57, IVP-58, IVP-59)
        # -------------------------------------------------------------------------
        src/drivers/spi_bus.cpp
        src/drivers/rfm95w.cpp
        src/telemetry/telemetry_encoder.cpp
        src/telemetry/telemetry_service.cpp
        src/telemetry/mavlink_rx.cpp

        # -------------------------------------------------------------------------
        # Stage 8: Flight Director (IVP-66+)
        # -------------------------------------------------------------------------

        # IVP-67: QEP hierarchical state machine dispatch (vendored QP/C 8.1.3)
        lib/qep/qep_hsm.c
        lib/qep/qp_app.c
        # IVP-75: QF Active Object framework + QV cooperative scheduler
        lib/qep/qf_act.c
        lib/qep/qf_actq.c
        lib/qep/qf_defer.c
        lib/qep/qf_dyn.c
        lib/qep/qf_mem.c
        lib/qep/qf_ps.c
        lib/qep/qf_qact.c
        lib/qep/qf_qeq.c
        lib/qep/qf_time.c
        lib/qep/qv.c
        lib/qep/bsp_qv.c

        # IVP-68: Flight Director HSM + phase tracking
        src/flight_director/flight_director.cpp

        # IVP-69: Go/No-Go checks + command handler
        src/flight_director/go_nogo_checks.cpp
        src/flight_director/command_handler.cpp

        # IVP-70: Guard functions + evaluator
        src/flight_director/guard_functions.cpp
        src/flight_director/guard_evaluator.cpp

        # IVP-71: Guard combinator + safety lockouts
        src/flight_director/guard_combinator.cpp

        # IVP-72: Action executor
        src/flight_director/action_executor.cpp

        # -------------------------------------------------------------------------
        # Stage 9: Active Object Architecture (IVP-76+)
        # -------------------------------------------------------------------------
        src/active_objects/ao_blinker.cpp
        src/active_objects/ao_counter.cpp
        src/active_objects/ao_led_engine.cpp
        src/active_objects/ao_flight_director.cpp
        src/active_objects/ao_health_monitor.cpp
        src/active_objects/ao_rf_manager.cpp
        src/active_objects/ao_notify.cpp
        src/notify/notify_backend_led.cpp
        src/notify/notify_backend_audio.cpp
        src/active_objects/ao_logger.cpp
        src/active_objects/ao_telemetry.cpp
        src/active_objects/ao_radio.cpp
        src/active_objects/ao_rcos.cpp

        # -------------------------------------------------------------------------
        # Stage 16C: Station Runtime Decoupling (IVP-140+)
        # -------------------------------------------------------------------------
        src/station/station_idle_tick.cpp
    )

    # -Wpedantic for our source files only (SDK sources are excluded)
    set(ROCKETCHIP_SOURCES
        src/main.cpp
        src/core1/sensor_core1.cpp
        src/drivers/ws2812_status.cpp
        src/drivers/i2c_bus.cpp
        src/drivers/icm20948.cpp
        src/drivers/baro_dps310.cpp
        src/drivers/gps_pa1010d.cpp
        src/drivers/gps_uart.cpp
        src/drivers/mcu_temp.cpp
        src/calibration/calibration_data.cpp
        src/calibration/calibration_manager.cpp
        src/calibration/calibration_storage.cpp
        src/calibration/cal_hooks.cpp
        src/logging/radio_config_storage.cpp
        src/cli/rc_os.cpp
        src/cli/rc_os_dashboard.cpp
        src/cli/rc_os_commands.cpp
        src/active_objects/ao_rcos.cpp
        src/math/vec3.cpp
        src/math/quat.cpp
        src/fusion/eskf.cpp
        src/fusion/eskf_codegen.cpp
        src/fusion/eskf_runner.cpp
        src/fusion/eskf_brake.cpp
        src/fusion/wmm_tables.cpp
        src/fusion/mahony_ahrs.cpp
        src/fusion/ud_factor.cpp
        src/logging/data_convert.cpp
        src/logging/pcm_frame.cpp
        src/logging/ring_buffer.cpp
        src/logging/flight_table.cpp
        src/logging/log_decimator.cpp
        src/logging/flash_flush.cpp
        src/logging/psram_init.cpp
        src/drivers/spi_bus.cpp
        src/drivers/rfm95w.cpp
        src/telemetry/telemetry_encoder.cpp
        src/telemetry/telemetry_service.cpp
        src/telemetry/mavlink_rx.cpp
        src/flight_director/flight_director.cpp
        src/flight_director/go_nogo_checks.cpp
        src/flight_director/command_handler.cpp
        src/flight_director/guard_functions.cpp
        src/flight_director/guard_evaluator.cpp
        src/flight_director/guard_combinator.cpp
        src/flight_director/action_executor.cpp
        src/safety/health_monitor.cpp
        src/active_objects/ao_blinker.cpp
        src/active_objects/ao_counter.cpp
        src/station/station_idle_tick.cpp
    )
    set_source_files_properties(${ROCKETCHIP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wpedantic")

    # IVP-42d / IVP-47: Per-file -O2 for ESKF propagation and codegen.
    # eskf.cpp: -O2 for stack optimization (LL Entry 1 / council R-4).
    # eskf_codegen.cpp: -O2 critical for codegen performance (199 CSE intermediates).
    set_source_files_properties(src/fusion/eskf.cpp PROPERTIES COMPILE_OPTIONS "-O2;-Wpedantic")
    set_source_files_properties(src/fusion/eskf_codegen.cpp PROPERTIES COMPILE_OPTIONS "-O2;-Wpedantic")

    # Include directories
    target_include_directories(rocketchip PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/drivers
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src/calibration    # IVP-14
        ${CMAKE_CURRENT_LIST_DIR}/src/core1           # Stage 13: Core 1 sensor loop
        ${CMAKE_CURRENT_LIST_DIR}/src/cli            # IVP-18
        ${CMAKE_CURRENT_LIST_DIR}/src/logging          # IVP-49
        ${CMAKE_CURRENT_LIST_DIR}/src/telemetry        # IVP-58
        ${CMAKE_CURRENT_LIST_DIR}/lib/mavlink            # IVP-61
        ${CMAKE_CURRENT_LIST_DIR}/lib/qep                # IVP-67
        ${CMAKE_CURRENT_LIST_DIR}/src/flight_director  # IVP-68
        ${CMAKE_CURRENT_LIST_DIR}/src/active_objects   # IVP-76
        ${RUUVI_DPS310_DIR}                            # IVP-11
        ${LWGPS_DIR}/include                             # IVP-31
    )

    # Generate WS2812 PIO header from SDK's pio file
    pico_generate_pio_header(rocketchip
        ${PICO_SDK_PATH}/src/rp2_common/pico_status_led/ws2812.pio
    )

    # IVP-88: PIO heartbeat watchdog
    pico_generate_pio_header(rocketchip
        ${CMAKE_CURRENT_LIST_DIR}/pio/heartbeat_watchdog.pio
    )

    # IVP-89: PIO backup deployment timer
    pico_generate_pio_header(rocketchip
        ${CMAKE_CURRENT_LIST_DIR}/pio/backup_timer.pio
    )

    # Link libraries
    target_link_libraries(rocketchip
        pico_stdlib
        hardware_gpio
        hardware_pio         # For NeoPixel/WS2812 PIO driver
        hardware_clocks      # For clock_get_hz()
        hardware_i2c         # For I2C/QWIIC sensors
        hardware_uart        # For UART GPS (IVP-46)
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
        # IVP-57: SPI bus for LoRa radio
        hardware_spi
        # IVP-53b: XIP cache clean for PSRAM flush (council req. #1)
        hardware_xip_cache
        # Stage 16C IVP-142a: MCU die-temperature sensor on ADC ch 4
        hardware_adc
    )

    # Always-on soak preconditions symbol — present in both bench and
    # flight binaries so soaks can validate identity/board/radio at T=0.
    target_link_options(rocketchip PRIVATE
        -Wl,--undefined=diag_stats_t0_preconditions
        -Wl,--undefined=rfm95w_read_version
    )

    # Keep fault injection symbols for GDB `call` (--gc-sections strips uncalled functions)
    if(NOT BUILD_FOR_FLIGHT)
        target_link_options(rocketchip PRIVATE
            -Wl,--undefined=fault_force_eskf_unhealthy
            -Wl,--undefined=fault_force_core0_stall
            -Wl,--undefined=fault_force_core0_stall_clear
            -Wl,--undefined=fault_force_watchdog_stall
            -Wl,--undefined=fault_force_health_fail
            -Wl,--undefined=fault_force_ao_queue_flood
            -Wl,--undefined=fault_force_pio_sm_halt
            -Wl,--undefined=replay_inject_start
            -Wl,--undefined=replay_inject_stop
            -Wl,--undefined=replay_inject_sample
            -Wl,--undefined=diag_stats_dump
            -Wl,--undefined=fault_force_station_rx_drop
            -Wl,--undefined=fault_force_station_ack_suppress
            -Wl,--undefined=fault_force_station_gps_loss
            -Wl,--undefined=fault_force_station_gps_restore
            -Wl,--undefined=station_replay_inject_bytes
            -Wl,--undefined=station_replay_get_inject_count
            -Wl,--undefined=station_replay_start
            -Wl,--undefined=station_replay_stop
            -Wl,--undefined=station_replay_active
        )
    endif()

    # Enable USB CDC for stdio (not UART)
    pico_enable_stdio_usb(rocketchip 1)
    pico_enable_stdio_uart(rocketchip 0)

    # Bierman scalar measurement update (replaces Joseph form)
    target_compile_definitions(rocketchip PRIVATE
        ESKF_USE_BIERMAN=1
    )

    # Device role selection (IVP-95)
    # Pass -DROCKETCHIP_JOB_STATION=1 or -DROCKETCHIP_JOB_RELAY=1 to cmake
    if(ROCKETCHIP_JOB_STATION)
        target_compile_definitions(rocketchip PRIVATE ROCKETCHIP_JOB_STATION=1)
        message(STATUS "Job: STATION (RX mode)")
    elseif(ROCKETCHIP_JOB_RELAY)
        target_compile_definitions(rocketchip PRIVATE ROCKETCHIP_JOB_RELAY=1)
        message(STATUS "Job: RELAY")
    else()
        message(STATUS "Job: VEHICLE (default)")
    endif()

    # Flight build: exclude dev diagnostics (bench stats, debug menu)
    # Usage: cmake -DBUILD_FOR_FLIGHT=ON ..
    option(BUILD_FOR_FLIGHT "Exclude dev diagnostics from binary" OFF)
    if(BUILD_FOR_FLIGHT)
        target_compile_definitions(rocketchip PRIVATE BUILD_FOR_FLIGHT=1)
        message(STATUS "Build tier: FLIGHT (dev diagnostics excluded)")
    else()
        message(STATUS "Build tier: BENCH (dev diagnostics included)")
    endif()

    # Stage T — RadioScheduler timing diagnostics (IVP-T1)
    # Usage: cmake -DROCKETCHIP_STAGE_T_LOGGING=ON ..
    # Emits [STAGE_T] state-transition and per-RX metadata lines on vehicle.
    # Off by default; compiled out for flight builds.
    option(ROCKETCHIP_STAGE_T_LOGGING "Enable Stage T RadioScheduler diagnostics" OFF)
    if(ROCKETCHIP_STAGE_T_LOGGING AND NOT BUILD_FOR_FLIGHT)
        target_compile_definitions(rocketchip PRIVATE ROCKETCHIP_STAGE_T_LOGGING=1)
        message(STATUS "Stage T diagnostics: ENABLED")
    endif()

    # Stage T2 — cheat-mode sync ceiling (IVP-T2)
    # Usage: cmake -DROCKETCHIP_STAGE_T2_CHEAT=ON -DROCKETCHIP_JOB_STATION=1 ..
    # Queues X presses on station and fires on next vehicle RX — measures the
    # upper bound on achievable success rate with perfect sync. THROWAWAY
    # DIAGNOSTIC. Revert after data collection.
    option(ROCKETCHIP_STAGE_T2_CHEAT "Enable Stage T2 station cheat-mode sync" OFF)
    if(ROCKETCHIP_STAGE_T2_CHEAT AND NOT BUILD_FOR_FLIGHT)
        target_compile_definitions(rocketchip PRIVATE ROCKETCHIP_STAGE_T2_CHEAT=1)
        message(STATUS "Stage T2 cheat-mode: ENABLED")
    endif()

    # Stage T3 — CCSDS vs MAVLink protocol A/B (IVP-T3)
    # Usage: cmake -DROCKETCHIP_STAGE_T3_MAVLINK=ON ..
    # When ON, kDefaultRocketRadioConfig.protocol = kMavlink instead of kCcsds.
    # Must rebuild AND reflash BOTH vehicle and station with the same setting.
    option(ROCKETCHIP_STAGE_T3_MAVLINK "Use MAVLink framing over LoRa (T3 A/B)" OFF)
    if(ROCKETCHIP_STAGE_T3_MAVLINK)
        target_compile_definitions(rocketchip PRIVATE ROCKETCHIP_STAGE_T3_MAVLINK=1)
        message(STATUS "Stage T3 protocol: MAVLink")
    endif()

    # Stage T6 is now a RUNTIME sweep via IVP-T5.5's SET_RADIO_CONFIG command,
    # NOT compile-flag rebuilds. See docs/plans/STAGE_T_FIX_PLAN.md §IVP-T6.

    # Git hash injection for version.h
    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_HASH_RAW
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT GIT_HASH_RAW)
        set(GIT_HASH_RAW "unknown")
    endif()
    target_compile_definitions(rocketchip PRIVATE GIT_HASH="${GIT_HASH_RAW}")

    # USB configuration
    # BOOTSEL LED uses PICO_DEFAULT_LED_PIN (SDK provides per board)
    target_compile_definitions(rocketchip PRIVATE
        PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
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
    # UD Factorization Benchmark (numerical stability investigation)
    # ============================================================================

    add_executable(ud_benchmark
        src/benchmark/ud_benchmark.cpp
        src/fusion/ud_factor.cpp
        src/fusion/eskf.cpp
        src/fusion/eskf_codegen.cpp
        src/fusion/innovation_monitor.cpp
        src/math/vec3.cpp
        src/math/quat.cpp
        src/fusion/baro_kf.cpp
    )
    target_include_directories(ud_benchmark PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/src
    )
    # Benchmark must use -O2 — Debug (-O0) gives wildly pessimistic numbers.
    target_compile_options(ud_benchmark PRIVATE -O2)
    target_link_libraries(ud_benchmark
        pico_stdlib
        hardware_timer
    )
    pico_enable_stdio_usb(ud_benchmark 1)
    pico_enable_stdio_uart(ud_benchmark 0)
    pico_add_extra_outputs(ud_benchmark)

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
