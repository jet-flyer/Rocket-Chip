# RocketChip CMake Build System
#
# Two build modes:
#   Target build (default): cmake -B build -G Ninja
#   Host tests:             cmake -B build_host -G Ninja -DBUILD_TESTS=ON
#
# The BUILD_TESTS gate selects between Pico SDK target build and
# host-side test build. They are mutually exclusive.

cmake_minimum_required(VERSION 3.25)

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
    # rc_log library — bounded log channel replacing <stdio.h>
    # R-5 Unit B (2026-05-16). See:
    #   docs/decisions/STDIO_REPLACEMENT_PLAN.md (why)
    #   C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md (how)
    #   docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md (spec list)
    # Council-approved 3-persona Approach A (2026-05-15): hand-rolled
    # printf-subset formatter dispatching to etl::to_string. ETL 20.47.1
    # vendored at EXTERNAL/etl-20.47.1/ — acceptance-criterion verified
    # (no vsnprintf/snprintf/<stdio.h> in format+to_string subtree).
    # ------------------------------------------------------------------
    add_library(rc_log STATIC
        src/log/rc_log.cpp
    )
    target_include_directories(rc_log PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/EXTERNAL/etl-20.47.1
    )
    target_compile_definitions(rc_log PUBLIC ROCKETCHIP_HOST_TEST)
    # Treat the vendored ETL as SYSTEM headers so its warnings (mostly
    # in third-party-style code paths) don't fail our pedantic gates.
    target_include_directories(rc_log SYSTEM PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/EXTERNAL/etl-20.47.1
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
    #
    # NOT DUPLICATION — the same flight_director sources also appear in
    # the target build's add_executable(rocketchip ...) source list below
    # (around line 420 onward). Host and target builds are in different
    # if(BUILD_TESTS) branches and compile these files in different
    # contexts: host defines ROCKETCHIP_HOST_TEST=1 and links pure C++
    # only; target compiles against the full Pico SDK. DO NOT collapse
    # into a shared source-list variable without preserving that split.
    # See docs/BUILD_SYSTEM_AUDIT.md "Host/target CMake split" entry.
    # ------------------------------------------------------------------
    add_library(rc_flight_director STATIC
        src/flight_director/flight_director.cpp
        src/flight_director/go_nogo_checks.cpp
        src/flight_director/command_handler.cpp
        src/flight_director/guard_functions.cpp
        src/flight_director/guard_evaluator.cpp
        src/flight_director/guard_combinator.cpp
        src/flight_director/action_executor.cpp
        src/safety/test_mode.cpp           # R-25-exec: runtime gate (host-test build uses ROCKETCHIP_HOST_TEST shim)
        src/safety/flight_in_progress.cpp  # Fault-recovery 2026-05-14: sentinel set on ARM, cleared on LANDED
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
        src/shared_state.cpp
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
        src/calibration/lm_solver.cpp     # R-6c (2026-05-07 audit): extracted from calibration_manager.cpp

        # Stage T IVP-T5.5: radio config persistence
        src/logging/radio_config_storage.cpp

        # IVP-18: RC_OS CLI
        src/cli/rc_os.cpp
        src/cli/rc_os_dashboard.cpp
        src/cli/rc_os_commands.cpp
        src/cli/rc_os_debug.cpp                 # R-25-exec step 2: migrated from src/dev/dev_cli.cpp; test-mode-gated

        # R-25-exec step 8 (2026-05-13, council Approach A): the parallel
        # dev/bench tier was retired. Migrated modules now live in the
        # single flight binary:
        #   src/safety/fault_inject.cpp        (step 3, test-mode-gated)
        #   src/diag/diag_stats.cpp            (step 4, always-on reads)
        #   src/safety/station_fault_inject.cpp (step 6, test-mode-gated)
        # DELETED in steps 5 + 6 (council amendment #4, replay went host-side):
        #   src/dev/replay_inject.{cpp,h}
        #   src/dev/station_replay.{cpp,h}
        src/diag/diag_stats.cpp

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
        src/safety/fault_protection.cpp  # OPT-IVP-01: shared MPU + fault handlers
        src/safety/crash_record.cpp      # R-3 audit 2026-05-07: capture-state-then-reset
        src/safety/flight_in_progress.cpp  # Fault-recovery 2026-05-14: boot sentinel for anomalous-boot gate
        src/safety/anomalous_boot.cpp    # Fault-recovery 2026-05-14: confidence gate at boot
        src/safety/core1_i2c_pause.cpp   # R-17 audit 2026-05-07: cooperative pause around flash ops
        src/safety/test_mode.cpp         # R-25-exec: runtime gate for test/fault-injection affordances (Approach A 2026-05-13)
        src/safety/fault_inject.cpp      # R-25-exec step 3: migrated from src/dev/, gated by test_mode
        src/safety/station_fault_inject.cpp  # R-25-exec step 6: migrated from src/dev/, gated by test_mode (no-op on vehicle role)

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

        # -------------------------------------------------------------------------
        # R-5 Unit B (2026-05-16): bounded log channel replacing <stdio.h>
        # See docs/decisions/STDIO_REPLACEMENT_PLAN.md + parsed-soaring-popcorn.md
        # -------------------------------------------------------------------------
        src/log/rc_log.cpp
    )

    # -Wpedantic applied per-source-file to our code only. Pico SDK .c
    # sources (hardware_irq, TinyUSB) get pulled in as INTERFACE sources
    # of the linked SDK libraries and compile under the rocketchip target
    # options — target-level -Wpedantic would incorrectly pedantic-check
    # those too. Per-file scoping via set_source_files_properties is the
    # only pattern that works here.
    #
    # Vendor headers our files pull in directly (mavlink, qep, ruuvi,
    # lwgps) are classified as SYSTEM below — GCC suppresses pedantic in
    # system headers, so including them in pedantic-checked files is fine.
    #
    # ANY new .cpp added under src/ MUST be added to this list. The
    # audit in docs/BUILD_SYSTEM_AUDIT.md catches drift.
    set(ROCKETCHIP_SOURCES
        src/main.cpp
        src/shared_state.cpp
        src/core1/sensor_core1.cpp
        src/drivers/ws2812_status.cpp
        src/drivers/i2c_bus.cpp
        src/drivers/icm20948.cpp
        src/drivers/baro_dps310.cpp
        src/drivers/gps_pa1010d.cpp
        src/drivers/gps_uart.cpp
        src/drivers/mcu_temp.cpp
        src/drivers/spi_bus.cpp
        src/drivers/rfm95w.cpp
        src/calibration/calibration_data.cpp
        src/calibration/calibration_manager.cpp
        src/calibration/calibration_storage.cpp
        src/calibration/cal_hooks.cpp
        src/calibration/lm_solver.cpp     # R-6c (2026-05-07 audit): extracted from calibration_manager.cpp
        src/logging/radio_config_storage.cpp
        src/logging/data_convert.cpp
        src/logging/pcm_frame.cpp
        src/logging/ring_buffer.cpp
        src/logging/flight_table.cpp
        src/logging/log_decimator.cpp
        src/logging/flash_flush.cpp
        src/logging/psram_init.cpp
        src/cli/rc_os.cpp
        src/cli/rc_os_dashboard.cpp
        src/cli/rc_os_commands.cpp
        src/cli/rc_os_debug.cpp                 # R-25-exec step 2: migrated from src/dev/dev_cli.cpp; test-mode-gated
        src/active_objects/ao_rcos.cpp
        src/active_objects/ao_flight_director.cpp
        src/active_objects/ao_health_monitor.cpp
        src/active_objects/ao_led_engine.cpp        # F-2026-05-13-004: pedantic-gate restoration (Tier 2.4 P2)
        src/active_objects/ao_logger.cpp
        src/active_objects/ao_notify.cpp            # F-2026-05-13-004: pedantic-gate restoration (Tier 2.4 P2)
        src/active_objects/ao_radio.cpp
        src/active_objects/ao_rf_manager.cpp
        src/active_objects/ao_telemetry.cpp
        src/diag/diag_stats.cpp                     # R-25-exec step 4
        src/math/vec3.cpp
        src/math/quat.cpp
        src/fusion/eskf.cpp
        src/fusion/eskf_codegen.cpp
        src/fusion/eskf_runner.cpp
        src/fusion/eskf_brake.cpp
        src/fusion/wmm_tables.cpp
        src/fusion/mahony_ahrs.cpp
        src/fusion/ud_factor.cpp
        src/fusion/confidence_gate.cpp
        src/fusion/innovation_monitor.cpp
        src/fusion/baro_kf.cpp                      # F-2026-05-13-004: pedantic-gate restoration (Tier 2.4 P2)
        src/notify/notify_backend_led.cpp
        src/notify/notify_backend_audio.cpp
        src/safety/pio_watchdog.cpp
        src/safety/pio_backup_timer.cpp
        src/safety/pyro_edge_logger.cpp
        src/safety/fault_protection.cpp  # OPT-IVP-01: shared MPU + fault handlers
        src/safety/crash_record.cpp      # R-3 audit 2026-05-07: capture-state-then-reset
        src/safety/flight_in_progress.cpp  # Fault-recovery 2026-05-14: boot sentinel for anomalous-boot gate
        src/safety/anomalous_boot.cpp    # Fault-recovery 2026-05-14: confidence gate at boot
        src/safety/core1_i2c_pause.cpp   # R-17 audit 2026-05-07: cooperative pause around flash ops
        src/safety/test_mode.cpp         # R-25-exec: runtime gate for test/fault-injection affordances (Approach A 2026-05-13)
        src/safety/fault_inject.cpp      # R-25-exec step 3: migrated from src/dev/, gated by test_mode
        src/safety/station_fault_inject.cpp  # R-25-exec step 6: migrated from src/dev/, gated by test_mode
        src/safety/health_monitor.cpp
        src/telemetry/telemetry_encoder.cpp
        src/telemetry/mavlink_rx.cpp
        src/flight_director/flight_director.cpp
        src/flight_director/go_nogo_checks.cpp
        src/flight_director/command_handler.cpp
        src/flight_director/guard_functions.cpp
        src/flight_director/guard_evaluator.cpp
        src/flight_director/guard_combinator.cpp
        src/flight_director/action_executor.cpp
        src/station/station_idle_tick.cpp
        src/log/rc_log.cpp                      # R-5 Unit B (2026-05-16): bounded log channel replacing stdio
    )
    set_source_files_properties(${ROCKETCHIP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wpedantic")

    # IVP-42d / IVP-47: Per-file -O2 for ESKF propagation and codegen.
    # eskf.cpp: -O2 for stack optimization (LL Entry 1 / council R-4).
    # eskf_codegen.cpp: -O2 critical for codegen performance (199 CSE intermediates).
    set_source_files_properties(src/fusion/eskf.cpp PROPERTIES COMPILE_OPTIONS "-O2;-Wpedantic")
    set_source_files_properties(src/fusion/eskf_codegen.cpp PROPERTIES COMPILE_OPTIONS "-O2;-Wpedantic")

    # Include directories — split into our code (-I, pedantic-checked) and
    # vendored third-party code (-isystem, pedantic-suppressed).
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
        ${CMAKE_CURRENT_LIST_DIR}/src/flight_director  # IVP-68
        ${CMAKE_CURRENT_LIST_DIR}/src/active_objects   # IVP-76
    )
    # Vendored third-party — SYSTEM suppresses -Wpedantic from their headers.
    target_include_directories(rocketchip SYSTEM PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/lib/mavlink          # IVP-61 — third-party telemetry lib
        ${CMAKE_CURRENT_LIST_DIR}/lib/qep              # IVP-67 — vendored QP/C 8.1.3
        ${RUUVI_DPS310_DIR}                            # IVP-11 — Ruuvi DPS310 driver
        ${LWGPS_DIR}/include                           # IVP-31 — lwgps NMEA parser
        ${CMAKE_CURRENT_LIST_DIR}/EXTERNAL/etl-20.47.1 # R-5 Unit B: ETL for rc_log formatter
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

    # Pico SDK + vendored-in-SDK library INTERFACE_INCLUDE_DIRECTORIES get
    # marked SYSTEM so downstream consumers (rocketchip) see them as
    # -isystem. GCC suppresses -Wpedantic in SYSTEM headers, so vendor code
    # (TinyUSB packed descriptors with flexible array members) doesn't
    # surface as errors when our pedantic-checked sources include them.
    # Requires CMake >= 3.25 (enforced at cmake_minimum_required).
    # See docs/BUILD_SYSTEM_AUDIT.md for the pattern.
    #
    # Hit list was discovered empirically — the most common SDK lib API
    # faces (pico_stdlib, hardware_*) plus the deep vendor targets that
    # supply tinyusb's includes via INTERFACE chains that bypass SDK-
    # public lib properties. `tinyusb_common_base` is inside the SDK's
    # vendored lib/tinyusb tree; `tinyusb_device_unmarked` and
    # `tinyusb_common` are the SDK-public wrappers. Marking them all
    # belt-and-suspenders ensures tinyusb never leaks -I into user code.
    set(PICO_SDK_INTERFACE_LIBS
        pico_stdlib hardware_gpio hardware_pio hardware_clocks hardware_i2c
        hardware_uart hardware_flash pico_flash pico_multicore
        hardware_exception hardware_watchdog pico_rand hardware_spi
        hardware_xip_cache hardware_adc
        # TinyUSB chain — marks the deep INTERFACE targets so transitive
        # includes don't leak through without SYSTEM classification.
        tinyusb_common_base tinyusb_common tinyusb_device_unmarked
        tinyusb_device tinyusb_device_base
    )
    foreach(lib ${PICO_SDK_INTERFACE_LIBS})
        if(TARGET ${lib})
            set_target_properties(${lib} PROPERTIES SYSTEM TRUE)
        endif()
    endforeach()

    # R-25-exec (2026-05-13, Approach A): always-on symbols in the single
    # flight binary. T=0 preconditions + diag_stats_dump (read-only,
    # always available) + fault_force_* (test-mode-gated at entry).
    target_link_options(rocketchip PRIVATE
        -Wl,--undefined=diag_stats_t0_preconditions
        -Wl,--undefined=diag_stats_dump
        -Wl,--undefined=rfm95w_read_version
        -Wl,--undefined=fault_force_eskf_unhealthy
        -Wl,--undefined=fault_force_core0_stall
        -Wl,--undefined=fault_force_core0_stall_clear
        -Wl,--undefined=fault_force_watchdog_stall
        -Wl,--undefined=fault_force_health_fail
        -Wl,--undefined=fault_force_launch_abort
        -Wl,--undefined=fault_force_radio_dropout
        -Wl,--undefined=fault_force_ao_queue_flood
        -Wl,--undefined=fault_force_pio_sm_halt
        -Wl,--undefined=fault_force_hardfault
        # R-25-exec step 6 (2026-05-13): station fault-injection symbols
        # joined the always-on block; test-mode-gated at entry (Approach A).
        -Wl,--undefined=fault_force_station_rx_drop
        -Wl,--undefined=fault_force_station_ack_suppress
        -Wl,--undefined=fault_force_station_gps_loss
        -Wl,--undefined=fault_force_station_gps_restore
    )

    # R-25-exec steps 5+6: replay_inject_* + station_replay_* symbols
    # DELETED per council amendment #4 (replay coverage moves host-side).
    # The NOT_CERTIFIED_FOR_FLIGHT-only --undefined block is now empty
    # and ready for collapse in step 8.

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

    # ============================================================================
    # Build tier — single flight binary (R-25-exec step 8, 2026-05-13)
    # ============================================================================
    #
    # Per council-APPROVED Approach A
    # (docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md), the project
    # produces ONE firmware tier per role (vehicle / station). Test
    # affordances (fault_force_*, debug submenu state-mutating commands)
    # live in the same binary, gated at entry by rc::test_mode_active()
    # — runtime-partitioning per SWE-133. There is no parallel dev/bench
    # build configuration; NOT_CERTIFIED_FOR_FLIGHT was retired here.
    #
    # Build directories:
    #   build_flight/          — vehicle flight (no ROCKETCHIP_JOB_STATION)
    #   build_station_flight/  — station flight (-DROCKETCHIP_JOB_STATION=1)
    # ============================================================================
    message(STATUS "Build tier: single flight binary (R-25-exec Approach A, 2026-05-13)")

    # Dev tools — hierarchical gate. Off in production builds (all four
    # standard presets). See docs/BUILD_SYSTEM_AUDIT.md for the full model.
    #
    #   ROCKETCHIP_BUILD_DEV_TOOLS (parent)  — gate for dev-tool targets
    #     └─ ROCKETCHIP_STAGE_ARCHIVE (child) — momentary stage-bounded diagnostics
    #         └─ ROCKETCHIP_STAGE_T_LOGGING   (individual, opt-in per flag)
    #         └─ ROCKETCHIP_STAGE_T2_CHEAT    (individual, opt-in per flag)
    #         └─ ROCKETCHIP_STAGE_T3_MAVLINK  (individual, opt-in per flag)
    #
    # Parent + archive child on: stage-bounded options become *visible* but
    # still default OFF — you must name the specific flag to light it up.
    # Nested structure prevents accidental activation of archived stage flags
    # during normal dev work.
    option(ROCKETCHIP_BUILD_DEV_TOOLS "Parent gate — exposes dev-tool targets and the stage archive" OFF)
    if(ROCKETCHIP_BUILD_DEV_TOOLS)
        message(STATUS "Dev tools: ENABLED (regular dev targets available)")

        # Child gate — stage-bounded archive. Unlock the specific individual
        # stage flags below. Off even with DEV_TOOLS=ON, by design.
        option(ROCKETCHIP_STAGE_ARCHIVE "Child gate — expose archived stage-bounded diagnostic flags" OFF)
        if(ROCKETCHIP_STAGE_ARCHIVE)
            message(STATUS "Stage archive: UNLOCKED (individual stage flags available, still opt-in per flag)")

            # Stage T — RadioScheduler timing diagnostics (IVP-T1)
            # Emits [STAGE_T] state-transition and per-RX metadata lines on vehicle.
            # Use when replaying Stage T RadioScheduler investigation.
            # Usage: cmake -DROCKETCHIP_BUILD_DEV_TOOLS=ON -DROCKETCHIP_STAGE_ARCHIVE=ON -DROCKETCHIP_STAGE_T_LOGGING=ON ..
            option(ROCKETCHIP_STAGE_T_LOGGING "Enable Stage T RadioScheduler diagnostics" OFF)
            if(ROCKETCHIP_STAGE_T_LOGGING)
                # R-25-exec step 8 (2026-05-13): NOT_CERTIFIED_FOR_FLIGHT retired.
                # Stage archive flags now stand alone behind the parent + archive
                # gates (ROCKETCHIP_BUILD_DEV_TOOLS + ROCKETCHIP_STAGE_ARCHIVE).
                target_compile_definitions(rocketchip PRIVATE ROCKETCHIP_STAGE_T_LOGGING=1)
                message(STATUS "Stage T diagnostics: ENABLED")
            endif()

            # Stage T2 — cheat-mode sync ceiling (IVP-T2)
            # Queues X presses on station and fires on next vehicle RX — measures the
            # upper bound on achievable success rate with perfect sync. Kept for
            # possible replay during post-CCSDS command-layer work.
            # Usage: add -DROCKETCHIP_STAGE_T2_CHEAT=ON -DROCKETCHIP_JOB_STATION=1 (alongside parent+archive).
            option(ROCKETCHIP_STAGE_T2_CHEAT "Enable Stage T2 station cheat-mode sync" OFF)
            if(ROCKETCHIP_STAGE_T2_CHEAT)
                # R-25-exec step 8 (2026-05-13): NOT_CERTIFIED_FOR_FLIGHT retired.
                target_compile_definitions(rocketchip PRIVATE ROCKETCHIP_STAGE_T2_CHEAT=1)
                message(STATUS "Stage T2 cheat-mode: ENABLED")
            endif()

            # Stage T3 — CCSDS vs MAVLink protocol A/B (IVP-T3)
            # When ON, kDefaultRocketRadioConfig.protocol = kMavlink instead of kCcsds.
            # Must rebuild AND reflash BOTH vehicle and station with the same setting.
            # Usage: add -DROCKETCHIP_STAGE_T3_MAVLINK=ON (alongside parent+archive).
            option(ROCKETCHIP_STAGE_T3_MAVLINK "Use MAVLink framing over LoRa (T3 A/B)" OFF)
            if(ROCKETCHIP_STAGE_T3_MAVLINK)
                target_compile_definitions(rocketchip PRIVATE ROCKETCHIP_STAGE_T3_MAVLINK=1)
                message(STATUS "Stage T3 protocol: MAVLink")
            endif()

            # Stage T6 is a RUNTIME sweep (SET_RADIO_CONFIG command), not a
            # compile-flag. See docs/plans/STAGE_T_FIX_PLAN.md §IVP-T6.
        endif()
    endif()

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
