// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// RocketChip entry: HW init, Core 1 sensors, Core 0 CLI, watchdog, MPU.

#include "rocketchip/rc_debug.h"
#include "rocketchip/job.h"
#include "rocketchip/rc_log.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/shared_state.h"
#include "core1/sensor_core1.h"
#include "station/station_idle_tick.h"
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
#include "drivers/mcu_temp.h"
#include "drivers/gps_uart.h"
#include "calibration/calibration_storage.h"
#include "logging/radio_config_storage.h"  // persisted radio config
#include "calibration/calibration_manager.h"
#include "fusion/eskf.h"
#include "fusion/eskf_runner.h"
#include "fusion/confidence_gate.h"
#include "safety/pio_watchdog.h"
#include "safety/pio_backup_timer.h"
#include "safety/fault_protection.h"
#include "safety/inject_arm_gate.h"    // R-25-exec inject-arm gate
#include "safety/anomalous_boot.h"     // Fault-recovery 2026-05-14: confidence gate at boot
#include "diag/diag_stats.h"
#include "fusion/mahony_ahrs.h"
#include "fusion/wmm_tables.h"
#include "drivers/spi_bus.h"
#include "drivers/rfm95w.h"
#include "logging/psram_init.h"
#include "logging/ring_buffer.h"
#include "logging/flash_flush.h"
#include "logging/flight_table.h"
#include "logging/crc32.h"
#include "rocketchip/pcm_frame.h"
#include "rocketchip/fused_state.h"
#include "cli/rc_os.h"
#include "cli/rc_os_dashboard.h"
#include "cli/rc_os_commands.h"
#include "active_objects/ao_rcos.h"
#include "flight_director/flight_director.h"
#include "flight_director/command_handler.h"
#include "rocketchip/ao_signals.h"
#include "ao_led_engine.h"
#include "ao_flight_director.h"
#include "ao_logger.h"
#include "ao_telemetry.h"
#include "ao_radio.h"
#include "ao_health_monitor.h"
#include "ao_rf_manager.h"
#include "ao_notify.h"
#include "qp_port.h"
#include "qsafe.h"     // QP/C FuSa assertions
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/exception.h"
#include "hardware/watchdog.h"
#include <atomic>
#include <string.h>
#include <math.h>

#include "rocketchip/linker_symbols.h"  // __StackBottom (linker-defined, MPU guard)

// ============================================================================
// Constants
// ============================================================================

static constexpr uint kNeoPixelPin = board::kNeoPixelPin;

// Watchdog timeout/feed lives in pio_watchdog.

// Sensor power-up settling time (generous margin over ICM-20948 11ms + DPS310 40ms)
static constexpr uint32_t kSensorSettleMs = 200;

// All global state is declared in include/rocketchip/shared_state.h and
// defined in src/shared_state.cpp.

namespace {

void bind_gps_uart_backend() {
    g_gpsTransport = GPS_TRANSPORT_UART;
}

void bind_gps_i2c_backend() {
    g_gpsTransport = GPS_TRANSPORT_I2C;
}

}  // namespace

// ============================================================================
// Init: Hardware (fault handlers, MPU, GPIO, NeoPixel, Core 1, I2C, sensors)
// Returns true if previous reboot was caused by watchdog.
// ============================================================================

// Initialize I2C sensors (IMU, baro, GPS). Requires I2C bus already initialized.
static void init_gps() {
    // UART GPS when the pack exposes it (board::kUartGpsAvailable).
    // I2C fallback otherwise. UART has no I2C bus contention (LL 24).
    if (board::kUartGpsAvailable) {
        g_gpsInitAttempted = true;
        if (gps_uart_init()) {
            g_gpsInitialized = true;
            bind_gps_uart_backend();
            return;
        }
    }
    // I2C fallback — probe + init AFTER IMU bypass mode is stable.
    // PA1010D streams NMEA autonomously after any I2C read (LL Entry 20).
    if (!i2c_bus_probe(kGpsPa1010dAddr)) {
        return;
    }
    g_gpsInitAttempted = true;
    uint8_t gps_drain[255];
    // Drain the auto-streamed NMEA so gps_pa1010d_init() starts clean; the
    // bytes are deliberately discarded (LL 20). (void) is the honest marker.
    (void)i2c_bus_read(kGpsPa1010dAddr, gps_drain, sizeof(gps_drain));
    if (gps_pa1010d_init()) {
        g_gpsInitialized = true;
        bind_gps_i2c_backend();
    }
}

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
    bool imu_detected  = i2c_bus_probe(kIcm20948AddrDefault);
    if (!imu_detected) {
        imu_detected = icm20948_stuck_slave_recovery(kIcm20948AddrDefault);
    }
    bool baro_detected = i2c_bus_probe(kBaroDps310AddrDefault);
    // Sensor power-up settling time
    // ICM-20948 datasheet: 11ms, DPS310: 40ms, generous margin
    sleep_ms(kSensorSettleMs);

    // Init IMU first — establishes bypass mode for AK09916 at 0x0C
    if (imu_detected) {
        g_imuInitAttempted = true;
        g_imuInitialized = icm20948_init(&g_imu, kIcm20948AddrDefault);
    }

    if (baro_detected) {
        g_baroInitAttempted = true;
        g_baroInitialized = baro_dps310_init(kBaroDps310AddrDefault);
        if (g_baroInitialized) {
            g_baroContinuous = baro_dps310_start_continuous();
        }
    }

    if (!g_gpsInitialized) {
        init_gps();
    }
}

// Initialize USB CDC without blocking. Terminal connection is handled
// by rc_os_update() which prints banner on first connect (LL Entry 15).
static void init_usb() {
    stdio_init_all();
    // Disable CR/LF translation for binary MAVLink output.
    // Without this, every 0x0A in a MAVLink frame gets expanded to 0x0D 0x0A,
    // corrupting the binary protocol and causing QGC to fail parsing.
    stdio_set_translate_crlf(&stdio_usb, false);
}

// Boot ordering helper: fault handlers + MPU + test-mode init. Must run
// before any C++ static constructor that touches .uninitialized_data.
static void init_fault_recovery() {
    // Confidence gate FIRST — anomalous_boot_init reads .uninitialized_data
    // sentinel + POWMAN_CHIP_RESET. Downstream baro auto-zero gate
    // (init_application) consumes anomalous_boot_verdict().
    rc::anomalous_boot_init();

    // Shared fault handlers + MPU stack guard.
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, memmanage_fault_handler);
    exception_set_exclusive_handler(MEMMANAGE_EXCEPTION, memmanage_fault_handler);
    mpu_setup_stack_guard(reinterpret_cast<uintptr_t>(&__StackBottom));

    // R-25-exec: latches probe-armed test mode + clears the SRAM magic.
    rc::test_mode_init();
}

// Aggressive early GPS bring-up. MT3333 has a brief I2C slave window after
// cold boot; init_sensors() later misses it by hundreds of ms. Grok triage.
static void init_gps_early() {
    g_i2cInitialized = i2c_bus_init();
    if (!g_i2cInitialized) { return; }
    g_gpsInitAttempted = true;
    if (gps_pa1010d_init()) {
        g_gpsInitialized = true;
        bind_gps_i2c_backend();
    }
}

static void init_early_hw() {
    init_fault_recovery();

    gpio_init(board::kLedPin);
    gpio_set_dir(board::kLedPin, true);

    // Pack may release a shared peripheral RESET before I2C (no-op on Feather).
    board::board_release_peripheral_reset();

    init_gps_early();

    g_neopixelInitialized = ws2812_status_init(pio0, kNeoPixelPin,
                                                board::kNeoPixelCount);
    (void)rc::mcu_temp_init();
}

static void init_peripherals() {
    // SPI bus + radio init (before USB per LL Entry 4/12)
    // Optional peripheral: absent FeatherWing detected at init time
    g_spiInitialized = spi_bus_init();
    // Radio hardware init moved to AO_Radio_start() — owns its own lifecycle.

    // Calibration storage init (before USB per LL Entry 4/12)
    g_calStorageInitialized = calibration_storage_init();
    calibration_manager_init();

    // Radio config storage init (before USB, same flash-safety rule).
    // Sector scan here; RadioAo_initial later calls radio_config_storage_read().
    radio_config_storage_init();

    // USB CDC init (after I2C/flash per LL Entry 4/12). Non-blocking —
    // rc_os_update() handles terminal connect/disconnect and prints boot
    // banner on first connection.
    init_usb();
}

static void init_hardware() {
    init_early_hw();

    // PSRAM init — MUST be before Core 1 launch because psram_init()
    // manipulates QMI registers that control XIP flash execution.
    // Core 1 must not be running from flash during QMI reconfiguration.
    // flash_safe_test also uses flash_safe_execute() which needs
    // multicore_lockout — safe only after Core 1 is launched. So:
    // init + self-test before Core 1, flash-safe test deferred to after.
    g_psramSize = rc::psram_init(board::kPsramCsPin);
    if (g_psramSize > 0) {
        g_psramSelfTestPassed = rc::psram_self_test(g_psramSize);
    }

    // R-19: PSM reset Core 1 before launch (post-AIRCR wedge fix).
    // See docs/decisions/CORE1_PSM_RESET_BEFORE_LAUNCH.md.
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // Launch abort visual — solid red NeoPixel if flag is latched
    // (e.g. previous boot within same powered session detected a critical
    // fault while ARMED). Power cycle clears the flag; see
    // docs/USER_GUIDE.md "Safety State Model".
    if (rc::flight_director_launch_abort() && g_neopixelInitialized) {
        ws2812_set_mode(WS2812_MODE_SOLID, kColorRed);
    }

    // I2C bus init (before USB per LL Entry 4/12)
    if (!g_i2cInitialized) {
        g_i2cInitialized = i2c_bus_init();
    }
    if (g_i2cInitialized) {
        init_sensors();
    }

    // PSRAM flash-safe test deferred to after g_startSensorPhase, where
    // Core 1 has called multicore_lockout_victim_init().

    init_peripherals();
}

// ============================================================================
// Init: RC_OS setup, Core 1 sensor phase start, watchdog enable
// ============================================================================


static void init_rc_os_hooks() {
    rc_os_init();
    rc_os_imu_available = g_imuInitialized;
    rc_os_baro_available = g_baroContinuous;
}

// Initialize PIO safety systems on PIO2
static void init_pio_safety() {
    // Heartbeat watchdog — IRQ-based, no GPIO
    if (!rc::pio_watchdog_init()) {
        DBG_ERROR("PIO watchdog init failed — PIO2 SM unavailable");
    }
    // Backup deployment timers (drogue=GPIO12, main=GPIO13)
    // Bench test pins — not connected to pyro hardware yet
    if (!rc::pio_backup_timer_init(board::kPyroDroguePin, board::kPyroMainPin)) {
        DBG_ERROR("PIO backup timer init failed");
    }
    // pyro_edge_logger_init() is WIP (WN-274 / rem WB R-14). Do not arm the
    // GPIO IRQ from flight boot until pyro HW is on these pins and a real
    // log consumer exists.
}

// Vehicle: signal Core 1 to start sensor phase + wait for lockout.
// Station/Relay: Core 1 stays idle, init station_idle_tick if RX role.
static void init_core1_role() {
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        g_sensorPhaseActive = true;
        g_startSensorPhase.store(true, std::memory_order_release);
        rc_os_i2c_scan_allowed = false;  // LL Entry 23
        // Wait for Core 1's multicore_lockout_victim_init() (required for
        // any flash_safe_execute() to follow).
        while (!g_core1LockoutReady.load(std::memory_order_acquire)) {
            sleep_ms(1);
        }
    } else {
        rc_os_i2c_scan_allowed = true;
        if constexpr (job::kRadioModeRx) {
            rc::station_idle_tick_init();
        }
    }
}

// Auto-zero baro ground reference, suppressed if anomalous-boot gate
// returned PROBABLY_MID_FLIGHT (re-zero at altitude during an in-flight
// reboot would cause main deploy at the wrong altitude).
static void init_baro_auto_zero() {
    if (!g_baroContinuous) { return; }
    const rc::BootVerdict boot_verdict = rc::anomalous_boot_verdict();
    if (boot_verdict == rc::BootVerdict::kProbablyOnPad) {
        calibration_start_baro();
        while (calibration_is_active()) {
            sleep_ms(50);
        }
        calibration_reset_state();
        return;
    }
    // PROBABLY_MID_FLIGHT — log so the operator sees baro auto-zero was
    // skipped and the prior ground reference remains in use.
    const rc::BootSignals& sig = rc::anomalous_boot_signals();
    DBG_PRINT("BOOT: auto-zero-baro SUPPRESSED (verdict=%s sentinel=%d "
              "non_por=%d bor=%d prior_uptime_ms=%lu)",
              rc::anomalous_boot_verdict_name(),
              static_cast<int>(sig.sentinel_was_set),
              static_cast<int>(sig.had_any_non_por),
              static_cast<int>(sig.had_bor),
              static_cast<unsigned long>(sig.prior_uptime_ms));
}

static void init_application() {
    init_rc_os_hooks();
    init_core1_role();

    // PSRAM flash-safe test (deferred until Core 1 lockout is ready).
    if (g_psramSize > 0 && g_psramSelfTestPassed) {
        g_psramFlashSafePassed = rc::psram_flash_safe_test();
    }

    init_baro_auto_zero();

    eskf_runner_init(&rc::kDefaultRocketProfile);

    init_pio_safety();
}

// ============================================================================
// Main Loop Tick Functions
// ============================================================================
// Each tick function manages one subsystem. nowMs is computed once per loop
// iteration and passed to all ticks to prevent temporal skew.

// PIO watchdog feed replaces SDK watchdog. The PIO heartbeat is
// the sole health monitor — sets an IRQ flag on timeout, never resets
// the chip. No MCU reset ever without user command.
static void watchdog_kick_tick() {
    rc::pio_watchdog_feed();
}

// ============================================================================
// QF+QV Active Object Infrastructure
//
// Pub-sub subscriber array sized to system-wide signal catalog.
// ============================================================================

static QSubscrList g_subscrList[rc::SIG_AO_MAX];

// QV_onIdle bridge — called from bsp_qv.c when all AO queues are empty.
// Runs watchdog feed and ESKF propagation (seqlock bridge).
//
// watchdog_kick_tick stays here permanently — never an AO.
extern "C" void qv_idle_bridge(void) {
    // The fault_force_* writers in src/safety/fault_inject.cpp check
    // rc::test_mode_active() at entry and no-op on production boots, so
    // g_fault_core0_stall and g_fault_watchdog_skip are always 0 unless
    // test mode is armed. The branches below are dead in production
    // (volatile-load cost is ~2 cycles per idle tick, immaterial).
    extern volatile bool g_fault_core0_stall;
    if (g_fault_core0_stall) { return; }
    extern volatile uint32_t g_fault_watchdog_skip;
    if (g_fault_watchdog_skip > 0) {
        g_fault_watchdog_skip = g_fault_watchdog_skip - 1;
    } else {
        watchdog_kick_tick();
    }

    eskf_runner_tick();

    // Station: command ACK retry + GPS/MCU-temp poll.
    if constexpr (job::kRadioModeRx) {
        AO_Telemetry_cmd_retry_tick(to_ms_since_boot(get_absolute_time()));
        rc::station_idle_tick();
    }

    diag_stats_msp_tick();

    // R-5 Unit B drain wiring: pump the rc_log ring buffer to USB CDC
    // from the idle path. Non-blocking; the drain has a ring-empty
    // fast-path so most idle passes return immediately without
    // touching TinyUSB (added 2026-05-16 after the empty-ring drain
    // was empirically shown to interfere with Core 1's I2C — see LL
    // Entry 39 / commit message of this fix).
    rc_log_drain_to_cdc();

    // WFI: sleep until next interrupt (100Hz QF tick, USB CDC, etc.).
    // Correct QV idle pattern per Samek. Tick functions above run once per
    // idle call, then WFI suspends until next event source fires.
    // Power: ~45mW (WFI) vs ~140mW (busy loop). Significant for ground ops.
    // Previous sleep_ms(1) was a workaround for misdiagnosed OpenOCD
    // interference — WFI is safe with USB CDC (USB IRQs wake the core).
    __wfi();
}

// ============================================================================
// Active Object startup
//
// Start all AOs in priority order (highest first). Priority layout
// documented in lib/qep/qp_config.h. Roles (Vehicle/Station/Relay) mask
// which AOs run — see job::DeviceRole gating below.
// ============================================================================

static void start_active_objects() {
    // FD=9 (top: phase transitions feed every consumer), Radio=8 (hardware),
    // RfManager=7 (consumes SIG_RADIO_RX, gates station TX — between Radio
    // and HealthMon so it drains same cooperative pass as Radio's posts),
    // HealthMon=6, Notify=5, Logger=4, Telem=3, LED=2, RCOS=1.
    // FD=9 so RfManager can sit at 7 (between Radio and HealthMon).
    AO_Radio_start(8U, g_spiInitialized);  // 100Hz — owns radio hardware init
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        AO_FlightDirector_start(9U); // 100Hz — top priority; station has no FD
    }
    // HealthMonitor on every role with a health pipeline. Station uses
    // job.h capability masks so unused sensors report kHealthAbsent.
    // Relay stays out — link-layer only.
    if constexpr (job::kRole != job::DeviceRole::kRelay) {
        AO_HealthMonitor_start(6U);  // 10Hz — between FD and Notify
    }
    // AO_RfManager: RF link health + station TX anchored to vehicle RxDone.
    // Vehicle and station; not Relay.
    if constexpr (job::kRole != job::DeviceRole::kRelay) {
        // Initial nav_period_ms = 200 (5 Hz default). Updated via
        // AO_RfManager_set_nav_period_ms() on SET_RADIO_CONFIG apply.
        rc::AO_RfManager_start(7U, 200U);  // 10Hz
    }
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        AO_Notify_start(5U);         // 33Hz — notification intent hub
        // PSRAM ring only if addressing self-test AND flash-safe (QMI
        // restore across erase) both passed. Else SRAM fallback.
        AO_Logger_start(4U, g_psramSize,
                        g_psramSelfTestPassed && g_psramFlashSafePassed);
    }
    if constexpr (job::kRole != job::DeviceRole::kRelay) {
        AO_Telemetry_start(3U);      // 10Hz (Vehicle + Station, not Relay)
    }
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        AO_LedEngine_start(2U);     // 33Hz — Vehicle only (flight phase patterns)
    }
    // Station/Relay: AO_Radio owns NeoPixels exclusively (RSSI bar).
    // AO_LedEngine disabled to prevent PIO contention (LL Entry 32 pattern).
    AO_RCOS_start(1U);              // 20Hz — CLI/dashboard, all roles
}

// ============================================================================
// Main
// ============================================================================

int main() {
    init_hardware();
    init_application();

    // --- QF+QV Active Object initialization ---
    QF_init();
    QActive_psInit(g_subscrList, Q_DIM(g_subscrList));

    start_active_objects();

    // QF_run() replaces while(true) — never returns.
    // QV cooperative scheduler dispatches AOs, calls QV_onIdle() (which runs
    // the bridge tick functions) when all queues are empty.
    return static_cast<int>(QF_run());
}
