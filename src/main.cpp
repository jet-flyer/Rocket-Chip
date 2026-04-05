// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file main.cpp
 * @brief RocketChip main entry point
 *
 * Hardware init, Core 1 sensor loop (~1kHz IMU, ~50Hz baro, ~10Hz GPS),
 * Core 0 CLI dispatch, dual-core watchdog, MPU stack guard.
 *
 * IVP test code (Stages 1-4) stripped after hardware verification.
 * See git history for IVP gate checks, soak ticks, and test dispatchers.
 */

#include "rocketchip/config.h"
#include "rocketchip/sensor_seqlock.h"
#include "core1/sensor_core1.h"
#include "rocketchip/led_patterns.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "drivers/ws2812_status.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include "drivers/baro_dps310.h"
#include "drivers/gps_pa1010d.h"
#include "drivers/gps_uart.h"
#include "calibration/calibration_storage.h"
#include "calibration/calibration_manager.h"
#include "calibration/cal_hooks.h"
#include "fusion/eskf.h"
#include "fusion/eskf_runner.h"
#include "fusion/confidence_gate.h"
#include "safety/pio_watchdog.h"
#include "safety/pio_backup_timer.h"
#include "fusion/mahony_ahrs.h"
#include "fusion/wmm_declination.h"
#include "drivers/spi_bus.h"
#include "drivers/rfm95w.h"
#include "logging/psram_init.h"
#include "logging/ring_buffer.h"
#include "logging/flash_flush.h"
#include "logging/flight_table.h"
#include "logging/crc32.h"
#include "rocketchip/pcm_frame.h"
#include "rocketchip/fused_state.h"
#include "rocketchip/telemetry_service.h"
#include "cli/rc_os.h"
#include "cli/ansi_dashboard.h"
#include "cli/cli_commands.h"
#include "active_objects/ao_rcos.h"
#include "watchdog/watchdog_recovery.h"
#include "flight_director/flight_director.h"
#include "flight_director/command_handler.h"
#include "rocketchip/ao_signals.h"  // IVP-76: system-wide signal catalog
#include "ao_blinker.h"            // IVP-76: demo AO (heartbeat LED)
#include "ao_counter.h"            // IVP-76: demo AO (jitter measurement)
#include "ao_led_engine.h"         // IVP-77: NeoPixel LED AO (incremental test)
#include "ao_flight_director.h"    // IVP-78: Flight Director AO (incremental test)
#include "ao_logger.h"             // IVP-79: Logger AO (incremental test)
#include "ao_telemetry.h"          // IVP-80: Telemetry AO
#include "ao_radio.h"              // IVP-93: Radio AO
#include "qp_port.h"   // QP/C QEP (IVP-67): Q_onError, QHsm types
#include "qsafe.h"     // QP/C FuSa assertions
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/exception.h"
#include "hardware/watchdog.h"
#include "hardware/structs/mpu.h"
#include <atomic>
#include <stdio.h>
#include <string.h>
#include <math.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr uint kNeoPixelPin = board::kNeoPixelPin;

// MPU stack guard
static constexpr uint32_t kMpuGuardSizeBytes = 64;              // Guard region at bottom of stack

// Fault handler blink pattern
static constexpr int32_t kFaultBlinkFastLoops = 200000;         // ~100ms at 150MHz
static constexpr int32_t kFaultBlinkSlowLoops = 800000;         // ~400ms at 150MHz
static constexpr uint8_t kFaultFastBlinks = 3;                  // Fast blinks before slow

// Watchdog
static constexpr uint32_t kWatchdogTimeoutMs = 5000;            // 5 second timeout

// CLI display constants moved to cli_commands.cpp (Phase 7).
// ESKF constants moved to eskf_runner.cpp (Phase 2).
// DPS310 data-ready constants moved to sensor_core1.cpp (Phase 1).

// Calibration constants moved to src/calibration/cal_hooks.cpp (Phase 8).

// Sensor power-up settling time (generous margin over ICM-20948 11ms + DPS310 40ms)
static constexpr uint32_t kSensorSettleMs = 200;

// NeoPixel timing moved to sensor_core1.cpp (Core 1 private).

// NeoPixel pattern constants moved to include/rocketchip/led_patterns.h (Phase 0B).
// Backward-compat aliases (kCalNeo*, kRxNeo*, kFdNeo*) provided by that header.

// NeoPixel mode/color tracking moved to sensor_core1.cpp (Core 1 private).


// ============================================================================
// Global State
// ============================================================================

bool g_neopixelInitialized = false;           // Non-static: Core 1 reads (sensor_core1.cpp)
bool g_i2cInitialized = false;  // Non-static: cli_commands.cpp reads (Phase 7)
bool g_imuInitialized = false;                // Non-static: Core 1 reads (sensor_core1.cpp)
bool g_baroInitialized = false;               // Non-static: Core 1 reads/writes (sensor_core1.cpp)
bool g_baroContinuous = false;                // Non-static: Core 1 reads (sensor_core1.cpp)
bool g_gpsInitialized = false;                // Non-static: Core 1 reads/writes (sensor_core1.cpp)
gps_transport_t g_gpsTransport = GPS_TRANSPORT_NONE;  // Non-static: Core 1 reads
bool g_spiInitialized = false;  // Non-static: cli_commands.cpp reads (Phase 7)
bool g_radioInitialized = false;  // Non-static: AO_Radio reads (IVP-93 transitional)
rfm95w_t g_radio;                 // Non-static: AO_Radio borrows (IVP-93 transitional)

// Radio/telemetry globals removed (IVP-94). State now in AO_Telemetry + AO_Radio.

size_t g_psramSize = 0;                  // Non-static: cli_commands.cpp reads (Phase 7)
bool g_psramSelfTestPassed = false;      // Non-static: cli_commands.cpp reads (Phase 7)
bool g_psramFlashSafePassed = false;     // Non-static: cli_commands.cpp reads (Phase 7)

// Logging subsystem (IVP-52c) — moved to ao_logger.cpp (Phase 4).
// Ring buffer, decimator, flight table, SRAM fallback buffer, decimation
// constants all owned by AO_Logger. Access via ao_logger.h accessors.

// Transport-neutral GPS function pointers — set once during init_sensors().
// Avoids if/else on every Core 1 GPS poll cycle.
// Non-static: Core 1 calls via sensor_core1.cpp.
bool (*g_gpsFnUpdate)()                    = nullptr;
bool (*g_gpsFnGetData)(gps_data_t*)       = nullptr;
bool (*g_gpsFnHasFix)()                    = nullptr;

// best_gps_fix_t, g_bestGpsFix, g_bestGpsValid moved to sensor_core1.h/.cpp.
// Extern declarations provided by core1/sensor_core1.h.

// IVP-66: Watchdog recovery policy — persists flight state across WDT resets.
// Tracks reboot count, safe-mode lockout, ESKF failure backoff.
// Non-static: eskf_runner reads g_recovery.eskf_disabled for backoff.
rc::WatchdogRecovery g_recovery;

// IVP-68: Flight Director HSM (Stage 8)
// Moved to AO_FlightDirector (Phase 3). Access via ao_flight_director.h.
// g_director and g_directorInitialized are now internal to ao_flight_director.cpp.

// ESKF globals moved to src/fusion/eskf_runner.cpp (Phase 2).
// Access via eskf_runner_get_eskf(), eskf_runner_get_mahony(), etc.
// g_eskf and g_eskfInitialized are extern-declared in sensor_core1.h
// and defined in eskf_runner.cpp.

// Sensor data types, seqlock protocol, and cross-core signaling
// are defined in include/rocketchip/sensor_seqlock.h (Phase 0A extraction).
// Global instances defined here, will migrate to owning modules in later phases.

sensor_seqlock_t g_sensorSeqlock;

std::atomic<bool> g_startSensorPhase{false};
std::atomic<bool> g_sensorPhaseDone{false};
std::atomic<bool> g_calReloadPending{false};
std::atomic<bool> g_core1PauseI2C{false};
std::atomic<bool> g_core1I2CPaused{false};
std::atomic<bool> g_core1LockoutReady{false};

// g_calNeoPixelOverride removed (Phase 5). CLI uses AO_LedEngine_post_override(),
// FD uses AO_LedEngine_post_pattern(). No cross-core atomic needed.

// Dual-core watchdog kick flags — std::atomic per MULTICORE_RULES.md
// volatile is NOT sufficient for cross-core visibility on ARM (no hardware barrier)
// IVP-90: SDK watchdog globals removed. PIO heartbeat is sole health monitor.
// g_wdtCore0Alive, g_wdtCore1Alive, g_watchdogEnabled — deleted.

// log_flight_event moved to AO_Logger (Phase 4). Use AO_Logger_log_event().
bool g_watchdogReboot = false;  // Non-static: cli_commands.cpp reads (Phase 7)

// Sensor phase active — set by Core 0 when Core 1 enters sensor loop.
// Read by Core 0 (cal hooks, print_sensor_status, eskf_runner). Plain bool is
// correct — single-core write/read, no cross-core visibility needed.
// Non-static: eskf_runner reads for tick gating.
bool g_sensorPhaseActive = false;

// Calibration storage state
bool g_calStorageInitialized = false;  // Non-static: cli_commands.cpp reads (Phase 7)


// IMU device handle (non-static: Core 1 reads via sensor_core1.cpp, per LL Entry 1)
icm20948_t g_imu;

// Seqlock read/write API moved to sensor_seqlock.h (Phase 0A).

// ============================================================================
// MemManage / HardFault Handler
// ============================================================================
// Fires when MPU stack guard is hit (stack overflow).
// Must NOT use stack (it may be overflowed). Uses direct GPIO register writes.
// Blink pattern: 3 fast + 1 slow on red LED, forever.

static void memmanage_fault_handler() {
    __asm volatile ("cpsid i");  // Disable interrupts

    // Direct GPIO register writes — no SDK calls that might use stack
    const uint32_t ledMask = 1U << board::kLedPin;

    // Active-low LED: SET register turns pin HIGH (off), CLR turns LOW (on)
    // Active-high LED: SET turns on, CLR turns off
    io_rw_32 *ledOn  = board::kLedActiveHigh ? &sio_hw->gpio_set : &sio_hw->gpio_clr;
    io_rw_32 *ledOff = board::kLedActiveHigh ? &sio_hw->gpio_clr : &sio_hw->gpio_set;

    while (true) {
        for (uint8_t i = 0; i < kFaultFastBlinks; i++) {
            *ledOn = ledMask;
            for (int32_t d = 0; d < kFaultBlinkFastLoops; d++) { __asm volatile(""); }
            *ledOff = ledMask;
            for (int32_t d = 0; d < kFaultBlinkFastLoops; d++) { __asm volatile(""); }
        }
        *ledOn = ledMask;
        for (int32_t d = 0; d < kFaultBlinkSlowLoops; d++) { __asm volatile(""); }
        *ledOff = ledMask;
        for (int32_t d = 0; d < kFaultBlinkSlowLoops; d++) { __asm volatile(""); }
    }
}

// ============================================================================
// QP/C Assertion Handler (IVP-67)
// ============================================================================
// Called by QEP when a state machine invariant is violated (null state handler,
// nesting depth overflow, etc.). Logs the failure and halts — the watchdog
// will reset the device and the recovery policy (IVP-66) will track it.
extern "C" Q_NORETURN Q_onError(
    char const * const module,
    int_t const id)
{
    __asm volatile("cpsid i");  // Disable interrupts
    printf("[QP ASSERT] module=%s, id=%d\n", module, id);
    // Spin until watchdog resets us — recovery scratch registers are fresh
    // from the last watchdog_kick_tick().
    while (true) {
        __asm volatile("nop");
    }
}

// ============================================================================
// MPU Stack Guard Setup (per-core, PMSAv8)
// ============================================================================
// Configures MPU region 0 as a no-access guard at the bottom of the stack.
// Each core has its own MPU — call from the core being protected.

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" uint32_t __StackBottom;     // Core 0 stack bottom (SCRATCH_Y, linker-defined)
// __StackOneBottom declaration moved to sensor_core1.cpp (Core 1 uses it).

static void mpu_setup_stack_guard(uint32_t stackBottom) {
    // Disable MPU during configuration
    mpu_hw->ctrl = 0;
    __dsb();
    __isb();

    // NOLINTBEGIN(readability-magic-numbers) — PMSAv8 MPU register bit fields per ARMv8-M Architecture Reference Manual
    // Region 0: Stack guard (no access, execute-never)
    // PMSAv8 RBAR: [31:5]=BASE, [4:3]=SH(0=non-shareable), [2:1]=AP(0=priv no-access), [0]=XN(1)
    mpu_hw->rnr = 0;
    mpu_hw->rbar = (stackBottom & ~0x1FU)
                  | (0U << 3)   // SH: Non-shareable
                  | (0U << 1)   // AP: Privileged no-access
                  | (1U << 0);  // XN: Execute-never

    // PMSAv8 RLAR: [31:5]=LIMIT, [3:1]=ATTRINDX(0), [0]=EN(1)
    mpu_hw->rlar = ((stackBottom + kMpuGuardSizeBytes - 1) & ~0x1FU)
                  | (0U << 1)   // ATTRINDX: 0 (uses MAIR0 attr 0)
                  | (1U << 0);  // EN: Enable region

    // MAIR0 attr 0 = 0x00 = Device-nGnRnE (strictest, no caching)
    mpu_hw->mair[0] = 0;

    // Enable MPU with PRIVDEFENA=1 (default memory map for unprogrammed regions)
    mpu_hw->ctrl = (1U << 2)   // PRIVDEFENA
                 | (1U << 0);  // ENABLE
    // NOLINTEND(readability-magic-numbers)
    __dsb();
    __isb();
}

// ============================================================================
// Core 1 sensor loop extracted to src/core1/sensor_core1.cpp (Phase 1).
// Public entry point: core1_entry() — declared in core1/sensor_core1.h.
// ============================================================================

// Calibration hooks moved to src/calibration/cal_hooks.cpp (Phase 8).

// Display functions moved to src/cli/cli_commands.cpp (Phase 7).


// ============================================================================
// Init: Hardware (fault handlers, MPU, GPIO, NeoPixel, Core 1, I2C, sensors)
// Returns true if previous reboot was caused by watchdog.
// ============================================================================

// Initialize I2C sensors (IMU, baro, GPS). Requires I2C bus already initialized.
static void init_sensors() {
    // Probe-first peripheral detection: only init drivers for devices that
    // are physically present on the bus. Prevents wasted init attempts and
    // avoids bus side-effects from absent devices (LL Entry 28).
    //
    // Init order matters: IMU + baro FIRST, GPS LAST.
    // In bypass mode, AK09916 at 0x0C shares the external I2C bus.
    // Probing the GPS (0x10) triggers NMEA streaming which can corrupt
    // AK09916 init transactions. Defer GPS probe until after IMU bypass
    // mode is fully established.
    bool imuDetected  = i2c_bus_probe(kIcm20948AddrDefault);
    bool baroDetected = i2c_bus_probe(kBaroDps310AddrDefault);

    // Sensor power-up settling time
    // ICM-20948 datasheet: 11ms, DPS310: 40ms, generous margin
    sleep_ms(kSensorSettleMs);

    // Init IMU first — establishes bypass mode for AK09916 at 0x0C
    if (imuDetected) {
        g_imuInitialized = icm20948_init(&g_imu, kIcm20948AddrDefault);
    }

    if (baroDetected) {
        g_baroInitialized = baro_dps310_init(kBaroDps310AddrDefault);
        if (g_baroInitialized) {
            g_baroContinuous = baro_dps310_start_continuous();
        }
    }

    // GPS detection — UART first (FeatherWing on GPIO0/1), I2C fallback.
    // UART GPS has no I2C bus contention (LL Entry 24), preferred for production.
    // [M3] UART GPS unavailable on Fruit Jam (GPIO 0/1 = Boot button + USB Host D+)
    if (board::kUartGpsAvailable && gps_uart_init()) {
        g_gpsInitialized = true;
        g_gpsTransport = GPS_TRANSPORT_UART;
        g_gpsFnUpdate   = gps_uart_update;
        g_gpsFnGetData = gps_uart_get_data;
        g_gpsFnHasFix  = gps_uart_has_fix;
    } else {
        // I2C fallback — probe + init AFTER IMU bypass mode is stable.
        // PA1010D streams NMEA autonomously after any I2C read (LL Entry 20).
        bool gpsDetected = i2c_bus_probe(kGpsPa1010dAddr);
        if (gpsDetected) {
            uint8_t gpsDrain[255];
            i2c_bus_read(kGpsPa1010dAddr, gpsDrain, sizeof(gpsDrain));
            if (gps_pa1010d_init()) {
                g_gpsInitialized = true;
                g_gpsTransport = GPS_TRANSPORT_I2C;
                g_gpsFnUpdate   = gps_pa1010d_update;
                g_gpsFnGetData = gps_pa1010d_get_data;
                g_gpsFnHasFix  = gps_pa1010d_has_fix;
            }
        }
    }
}

// Initialize USB CDC without blocking. Terminal connection is handled
// by rc_os_update() which prints banner on first connect (LL Entry 15).
static void init_usb() {
    stdio_init_all();
}

// Watchdog sentinel in scratch[0]. We write this before watchdog_enable() and
// check it at boot. scratch[0] survives watchdog resets (by design) but is
// cleared by POR and SWD resets. Neither the bootrom nor the SDK touches
// scratch[0-3] — only scratch[4-7] are used for reboot-to-address.
//
// Why not use the SDK functions:
// - watchdog_caused_reboot(): false positives on SWD `monitor reset run` because
//   the reason register persists across warm resets and rom_get_last_boot_type()
//   returns BOOT_TYPE_NORMAL for SWD-loaded boots after a watchdog timeout.
// - watchdog_enable_caused_reboot(): false negatives on real watchdog timeouts
//   because the bootrom overwrites scratch[4] during boot.
static constexpr uint32_t kWatchdogSentinel = 0x52435754;  // "RCWT"

static bool check_watchdog_reboot() {
    // Check for genuine watchdog timeout: reason register set AND our sentinel
    // present in scratch[0]. Clear sentinel immediately to avoid stale reads.
    bool watchdogReboot = (watchdog_hw->reason != 0) &&
                          (watchdog_hw->scratch[0] == kWatchdogSentinel);
    watchdog_hw->scratch[0] = 0;
    return watchdogReboot;
}

static void init_early_hw() {
    // Register fault handlers early (before any MPU config)
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, memmanage_fault_handler);
    exception_set_exclusive_handler(MEMMANAGE_EXCEPTION, memmanage_fault_handler);
    mpu_setup_stack_guard(reinterpret_cast<uint32_t>(&__StackBottom));

    // Red LED GPIO init
    gpio_init(board::kLedPin);
    gpio_set_dir(board::kLedPin, true);  // GPIO_OUT

    // NeoPixel init
    g_neopixelInitialized = ws2812_status_init(pio0, kNeoPixelPin,
                                                board::kNeoPixelCount);
}

static void init_peripherals() {
    // SPI bus + radio init (before USB per LL Entry 4/12)
    // Optional peripheral: absent FeatherWing detected at init time
    g_spiInitialized = spi_bus_init();
    if (g_spiInitialized) {
        g_radioInitialized = rfm95w_init(&g_radio,
            rocketchip::pins::kRadioCs,
            rocketchip::pins::kRadioRst,
            rocketchip::pins::kRadioIrq);
    }

    // Telemetry service init removed (IVP-94) — AO_Telemetry + AO_Radio own this now.
    // Radio init stays in init_peripherals (SPI bus + rfm95w_init).
    // AO_Radio borrows g_radio handle at startup (IVP-93 transitional).

    // Calibration storage init (before USB per LL Entry 4/12)
    g_calStorageInitialized = calibration_storage_init();
    calibration_manager_init();

    // USB CDC init (after I2C/flash per LL Entry 4/12). Non-blocking —
    // rc_os_update() handles terminal connect/disconnect and prints boot
    // banner on first connection.
    init_usb();
}

static bool init_hardware() {
    bool watchdogReboot = check_watchdog_reboot();
    rc::watchdog_recovery_init(&g_recovery, watchdogReboot);
    init_early_hw();

    // PSRAM init — MUST be before Core 1 launch because psram_init()
    // manipulates QMI registers that control XIP flash execution.
    // Core 1 must not be running from flash during QMI reconfiguration.
    // flash_safe_test also uses flash_safe_execute() which needs
    // multicore_lockout — safe only after Core 1 is launched. So:
    // init + self-test before Core 1, flash-safe test deferred to after.
    g_psramSize = rc::psram_init(rocketchip::pins::kPsramCs);
    if (g_psramSize > 0) {
        g_psramSelfTestPassed = rc::psram_self_test(g_psramSize);
    }

    // Launch Core 1 early so NeoPixel blinks immediately (no USB dependency)
    multicore_launch_core1(core1_entry);

    // IVP-66: Safe mode — solid red NeoPixel to signal critical state
    if (g_recovery.launch_abort && g_neopixelInitialized) {
        ws2812_set_mode(WS2812_MODE_SOLID, kColorRed);
    }

    // I2C bus init (before USB per LL Entry 4/12)
    g_i2cInitialized = i2c_bus_init();
    if (g_i2cInitialized) {
        init_sensors();
    }

    // PSRAM flash-safe test deferred to after g_startSensorPhase, where
    // Core 1 has called multicore_lockout_victim_init().

    init_peripherals();
    return watchdogReboot;
}

// print_boot_status moved to src/cli/cli_commands.cpp (Phase 7).


// ============================================================================
// Init: RC_OS setup, Core 1 sensor phase start, watchdog enable
// ============================================================================

// CLI forward declarations removed � functions now in cli_commands.cpp (Phase 7).


static void init_rc_os_hooks() {
    rc_os_init();
    rc_os_imu_available = g_imuInitialized;
    rc_os_baro_available = g_baroContinuous;
    if constexpr (job::kRadioModeRx) {
        rc_os_print_sensor_status = cli_print_station_status;
    } else {
        rc_os_print_sensor_status = cli_print_sensor_status;
    }
    rc_os_print_boot_summary = cli_print_hw_status;
    rc_os_print_boot_status = cli_print_boot_status;
    rc_os_read_accel = cal_read_accel;
    rc_os_read_mag = cal_read_mag;
    rc_os_reset_mag_staleness = cal_reset_mag_staleness;
    rc_os_cal_pre_hook = cal_pre_hook;
    rc_os_cal_post_hook = cal_post_hook;
    rc_os_set_cal_neo = cal_set_neo_override;
    rc_os_feed_cal = cal_feed_active;
    rc_os_print_eskf_live = cli_print_eskf_live;
    rc_os_on_unhandled_key = cli_handle_unhandled_key;
    rc_os_dispatch_flight_signal = AO_FlightDirector_dispatch_signal;
    rc_os_print_flight_status = AO_FlightDirector_print_status;
    rc_os_process_flight_command = AO_FlightDirector_process_command;
}

// init_logging_ring() moved to ao_logger.cpp (Phase 4).
// Called from AO_Logger_start().

// init_flight_director() moved to AO_FlightDirector_start() (Phase 3).

// IVP-88/89: Initialize PIO safety systems on PIO2
static void init_pio_safety() {
    // Heartbeat watchdog — IRQ-based, no GPIO
    if (!rc::pio_watchdog_init()) {
        DBG_ERROR("PIO watchdog init failed — PIO2 SM unavailable");
    }
    // Backup deployment timers (drogue=GPIO12, main=GPIO13)
    // Bench test pins — not connected to pyro hardware yet
    static constexpr uint8_t kPioDroguePin = 12;
    static constexpr uint8_t kPioMainPin = 13;
    if (!rc::pio_backup_timer_init(kPioDroguePin, kPioMainPin)) {
        DBG_ERROR("PIO backup timer init failed");
    }
}

static void init_application(bool watchdogReboot) {
    g_watchdogReboot = watchdogReboot;
    init_rc_os_hooks();

    // Signal Core 1 to start sensor phase — Vehicle only.
    // Station/Relay have no sensors on Core 1. Keep I2C available for GPS.
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        g_sensorPhaseActive = true;
        g_startSensorPhase.store(true, std::memory_order_release);
        rc_os_i2c_scan_allowed = false;  // LL Entry 23: prevent CLI I2C scan

        // Wait for Core 1 to call multicore_lockout_victim_init() — required
        // before any flash_safe_execute() call.
        while (!g_core1LockoutReady.load(std::memory_order_acquire)) {
            sleep_ms(1);
        }
    } else {
        // Station/Relay: Core 1 idle, I2C scan allowed, no sensor phase
        rc_os_i2c_scan_allowed = true;
    }

    // PSRAM flash-safe test (deferred from init_hardware).
    // Core 1 has called multicore_lockout_victim_init(),
    // so flash_safe_execute() is safe to use.
    if (g_psramSize > 0 && g_psramSelfTestPassed) {
        g_psramFlashSafePassed = rc::psram_flash_safe_test();
    }

    // Logging ring buffer and flight table init moved to AO_Logger_start() (Phase 4).

    // Auto-calibrate baro ground reference at boot.
    if (g_baroContinuous) {
        calibration_start_baro();
    }

    // init_flight_director() moved to AO_FlightDirector_start() (Phase 3).
    // AO_FlightDirector_start() is called from the AO startup sequence below.

    // Phase 2: Initialize ESKF runner with mission profile and event log callback.
    // Phase 4: callback routes to AO_Logger_log_event (ao_logger.cpp).
    eskf_runner_init(&rc::kDefaultRocketProfile,
                     [](uint8_t id, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
                         AO_Logger_log_event(static_cast<rc::LogEventId>(id), d0, d1, d2, d3);
                     });

    // IVP-90: SDK hardware watchdog REMOVED from production.
    // PIO heartbeat watchdog (init_pio_safety) is the sole health monitor.
    // No automatic MCU reset — ever — without user command.
    // Recovery scratch still written for boot diagnostics.
    watchdog_hw->scratch[0] = kWatchdogSentinel;

    init_pio_safety();
}

// ============================================================================
// Main Loop Tick Functions
// ============================================================================
// Each tick function manages one subsystem. nowMs is computed once per loop
// iteration and passed to all ticks to prevent temporal skew.

// Council recommendation: track which tick was running when watchdog fires.
static const char* g_lastTickFunction = "init";

// heartbeat_tick() removed — AO_Blinker owns LED heartbeat (IVP-76)

// IVP-90: PIO watchdog feed replaces SDK watchdog.
// No MCU reset — PIO heartbeat is the sole health monitor.
// Recovery scratch still updated for boot diagnostics.
static void watchdog_kick_tick() {
    rc::watchdog_recovery_update_scratch(&g_recovery);
    rc::pio_watchdog_feed();
}

// CLI and ANSI dashboard moved to AO_RCOS (Stage 12B Phase 2).
// Calibration bridge: blocking wizards stay here, triggered by g_pending_cal.

// ESKF tick functions moved to src/fusion/eskf_runner.cpp (Phase 2).
// eskf_runner_tick() is called from qv_idle_bridge() below.

// Flight Director tick, CLI callbacks, and init moved to
// ao_flight_director.cpp (Phase 3). Access via ao_flight_director.h.
// CLI command handlers moved to src/cli/cli_commands.cpp (Phase 7).


// ============================================================================
// Logging Tick, populate_fused_state, log_flight_event — moved to
// ao_logger.cpp (Phase 4). AO_Logger owns ring buffer, decimator,
// FusedState builder, and event logging.
// ============================================================================

// ============================================================================
// mavlink_direct_tick and telemetry_radio_tick removed (IVP-94)
// Now handled by AO_Telemetry (protocol) and AO_Radio (hardware).
// ============================================================================

// ============================================================================
// QF+QV Active Object Infrastructure (IVP-76)
//
// Pub-sub subscriber array sized to system-wide signal catalog.
// QV_onIdle bridge: runs existing tick functions during migration.
// ============================================================================

static QSubscrList g_subscrList[rc::SIG_AO_MAX];

// QV_onIdle bridge — called from bsp_qv.c when all AO queues are empty.
// During Stage 9 migration (IVP-76 through IVP-80), this runs the existing
// tick functions. As modules migrate to AOs, their calls are removed here.
//
// Council A2: watchdog_kick_tick() stays here PERMANENTLY — never an AO.
// Council A4: No __wfi() — tick functions require polling every iteration.
//             __wfi() only after IVP-81 when all work is event-driven.
extern "C" void qv_idle_bridge(void) {
    // Watchdog (Council A2: permanent), ESKF (seqlock bridge), CLI (polled).
    // FD → AO_FlightDirector (IVP-78)
    // Logger → AO_Logger (IVP-79)
    // Telemetry → AO_Telemetry (IVP-80)
    // LED → AO_LedEngine (IVP-77)

    g_lastTickFunction = "watchdog";
    g_recovery.current_tick_fn = rc::TickFnId::kWatchdog;
    watchdog_kick_tick();

    g_lastTickFunction = "eskf";
    g_recovery.current_tick_fn = rc::TickFnId::kEskf;
    eskf_runner_tick();

    // CLI key dispatch + calibration wizards: still in idle bridge.
    // AO_RCOS handles output mode + ANSI render only.
    // rc_os_update() stays here because blocking cal wizards run inside it.
    g_lastTickFunction = "cli";
    g_recovery.current_tick_fn = rc::TickFnId::kCli;
    if (AO_RCOS_get_output_mode() == StationOutputMode::kMenu ||
        AO_RCOS_get_output_mode() == StationOutputMode::kCsv) {
        rc_os_update();
        cal_feed_active();
    }

    g_lastTickFunction = "idle";
    g_recovery.current_tick_fn = rc::TickFnId::kSleep;
    // WFI: sleep until next interrupt (100Hz QF tick, USB CDC, etc.).
    // Correct QV idle pattern per Samek. Tick functions above run once per
    // idle call, then WFI suspends until next event source fires.
    // Power: ~45mW (WFI) vs ~140mW (busy loop). Significant for ground ops.
    // Previous sleep_ms(1) was a workaround for misdiagnosed OpenOCD
    // interference — WFI is safe with USB CDC (USB IRQs wake the core).
    __wfi();
}

// ============================================================================
// Main
// ============================================================================

int main() {
    bool watchdogReboot = init_hardware();
    init_application(watchdogReboot);

    // --- IVP-76: QF+QV Active Object initialization ---
    QF_init();
    QActive_psInit(g_subscrList, Q_DIM(g_subscrList));

    // Start Active Objects — incremental add, one at a time
    // Start Active Objects based on device role (IVP-95)
    // Vehicle: all AOs. Station: no FD/Logger. Relay: Radio + LED only.
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        AO_FlightDirector_start(5U); // 100Hz
        AO_Logger_start(4U, g_psramSize, g_psramSelfTestPassed);  // 50Hz
    }
    if constexpr (job::kRole != job::DeviceRole::kRelay) {
        AO_Telemetry_start(3U);      // 10Hz (Vehicle + Station, not Relay)
    }
    AO_Radio_start(6U);             // 100Hz — all roles
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        AO_LedEngine_start(2U);     // 33Hz — Vehicle only (flight phase patterns)
    }
    // Station/Relay: AO_Radio owns NeoPixels exclusively (RSSI bar).
    // AO_LedEngine disabled to prevent PIO contention (LL Entry 32 pattern).
    AO_RCOS_start(1U);              // 20Hz — CLI/dashboard, all roles
    // AO_Counter: jitter measurement diagnostic. Disabled — prints every 5s,
    // clutters serial. Re-enable for scheduler debugging.
    // AO_Counter_start(1U);

    // QF_run() replaces while(true) — never returns.
    // QV cooperative scheduler dispatches AOs, calls QV_onIdle() (which runs
    // the bridge tick functions) when all queues are empty.
    return static_cast<int>(QF_run());
}
