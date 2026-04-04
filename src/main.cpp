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
static constexpr const char* kBuildTag = "ivp74-profile-1";

// Heartbeat constants removed — AO_Blinker owns LED heartbeat (IVP-76)
// Core 1 sensor loop constants moved to src/core1/sensor_core1.cpp (Phase 1).

// DPS310 MEAS_CFG register (0x08): data-ready bits
// Per DPS310 datasheet Section 7, PRS_RDY=bit4, TMP_RDY=bit5
static constexpr uint8_t kDps310MeasCfgReg = 0x08;
static constexpr uint8_t kDps310PrsRdy = 0x10;                  // Bit 4
static constexpr uint8_t kDps310TmpRdy = 0x20;                  // Bit 5

// Core 1 pause ACK timeout — also used by Core 0 CLI wait loop
static constexpr uint32_t kCore1PauseAckMaxMs = 100;

// MPU stack guard
static constexpr uint32_t kMpuGuardSizeBytes = 64;              // Guard region at bottom of stack
// MPU region config: ARM v8-M attribute index for Device-nGnRE
static constexpr uint32_t kMpuAttrIndex = 7;                    // MAIR slot for guard region
static constexpr uint32_t kMpuAttrDeviceNgnre = 0x00;           // Device-nGnRE: fault on stack overflow
// RP2350 SRAM base (10MB region containing stack)
static constexpr uint32_t kSramRegionBase = 0x20000000U;
static constexpr uint32_t kSramRegionLimit = 0x20FFFFFFU;       // 16MB minus 1

// Fault handler blink pattern
static constexpr int32_t kFaultBlinkFastLoops = 200000;         // ~100ms at 150MHz
static constexpr int32_t kFaultBlinkSlowLoops = 800000;         // ~400ms at 150MHz
static constexpr uint8_t kFaultFastBlinks = 3;                  // Fast blinks before slow

// Watchdog
static constexpr uint32_t kWatchdogTimeoutMs = 5000;            // 5 second timeout

// GPS coordinate bounds moved to sensor_core1.cpp (only used by Core 1).
// kGpsCoordScale retained — used by Core 0 CLI display.
static constexpr double kGpsCoordScale = 1e7;                   // Degrees to 1e-7 degree integers

// PA1010D SDA settling delay moved to sensor_core1.cpp.

// Seqlock parameters: kSeqlockMaxRetries moved to sensor_seqlock.h

// ESKF propagation rate (Hz) — derived from IMU rate / divider
static constexpr uint32_t kEskfRateHz = 200;

// IVP-42d: ESKF propagation — every 5th IMU sample = 200Hz
// Note: count divider discards 4/5 IMU samples. For flight with high vibration,
// compound integration of all samples or higher rate propagation may be needed (R-5).
static constexpr uint32_t kEskfImuDivider = 5;

// Rad/deg conversion for CLI display and geodetic conversion
static constexpr float kRadToDeg = 180.0F / 3.14159265F;
static constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
static constexpr float kFullCircleDeg = 360.0F;           // Heading wrap-around

// IMU config readback bitmask: DLPF_CFG is bits [5:3] of ACCEL/GYRO_CONFIG register
static constexpr uint8_t kDlpfCfgMask = 0x07U;

// Flight director tick period moved to ao_flight_director.cpp (Phase 3).

// Erase confirmation input
static constexpr int32_t kEraseInputMaxChars = 7;               // Max input chars before enter
static constexpr uint32_t kEraseInputTimeoutUs = 10000000;       // 10 second input timeout

// Flight number input timeout
static constexpr uint32_t kFlightNumTimeoutUs = 3000000;         // 3 second timeout

// CRC-32 init/finalize value
static constexpr uint32_t kCrc32InitXor = 0xFFFFFFFFU;

// Microsecond-to-second conversion
static constexpr float kUsToSec = 1e-6F;
static constexpr float kGps1e7ToDegreesF = 1e-7F;         // GPS 1e-7 degree integers to degrees (float)
static constexpr double kGps1e7ToDegrees = 1e-7;           // GPS 1e-7 degree integers to degrees (double)

// Calibration read rate (~100Hz for accel cal)
static constexpr uint32_t kCalReadDelayMs = 10;

// Mag diagnostic print modulus (every N failures)
static constexpr uint32_t kMagDiagPrintModulus = 200;

// Sensor power-up settling time (generous margin over ICM-20948 11ms + DPS310 40ms)
static constexpr uint32_t kSensorSettleMs = 200;

// Core 1 startup delay moved to sensor_core1.cpp.

// GPS session NIS sentinel (larger than any valid NIS)
static constexpr float kGpsNisSentinel = 1e9F;

// I2C alternate addresses for device identification
static constexpr uint8_t kI2cAddrIcm20948Alt = 0x68;      // ICM-20948 with AD0=LOW
static constexpr uint8_t kI2cAddrDps310Alt   = 0x76;      // DPS310 alternate address

// ESKF state buffer: 200Hz × 5s = 1000 samples × 68B = 68KB (SRAM)
static constexpr uint32_t kEskfBufferSamples = 1000;

// ESKF dt sanity bounds (reject if too fast or too slow)
static constexpr uint32_t kEskfMinDtUs = 1000;     // 1ms
static constexpr uint32_t kEskfMaxDtUs = 100000;    // 100ms

// NeoPixel timing moved to sensor_core1.cpp (Core 1 private).

// NeoPixel pattern constants moved to include/rocketchip/led_patterns.h (Phase 0B).
// Backward-compat aliases (kCalNeo*, kRxNeo*, kFdNeo*) provided by that header.

// NeoPixel mode/color tracking moved to sensor_core1.cpp (Core 1 private).


// ============================================================================
// Global State
// ============================================================================

bool g_neopixelInitialized = false;           // Non-static: Core 1 reads (sensor_core1.cpp)
static bool g_i2cInitialized = false;
bool g_imuInitialized = false;                // Non-static: Core 1 reads (sensor_core1.cpp)
bool g_baroInitialized = false;               // Non-static: Core 1 reads/writes (sensor_core1.cpp)
bool g_baroContinuous = false;                // Non-static: Core 1 reads (sensor_core1.cpp)
bool g_gpsInitialized = false;                // Non-static: Core 1 reads/writes (sensor_core1.cpp)
gps_transport_t g_gpsTransport = GPS_TRANSPORT_NONE;  // Non-static: Core 1 reads
static bool g_spiInitialized = false;
bool g_radioInitialized = false;  // Non-static: AO_Radio reads (IVP-93 transitional)
rfm95w_t g_radio;                 // Non-static: AO_Radio borrows (IVP-93 transitional)

// Radio/telemetry globals removed (IVP-94). State now in AO_Telemetry + AO_Radio.

static size_t g_psramSize = 0;
static bool g_psramSelfTestPassed = false;
static bool g_psramFlashSafePassed = false;

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
static bool g_watchdogReboot = false;

// Sensor phase active — set by Core 0 when Core 1 enters sensor loop.
// Read by Core 0 (cal hooks, print_sensor_status, eskf_runner). Plain bool is
// correct — single-core write/read, no cross-core visibility needed.
// Non-static: eskf_runner reads for tick gating.
bool g_sensorPhaseActive = false;

// Calibration storage state
static bool g_calStorageInitialized = false;


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

// ============================================================================
// Accel Read Callback (for 6-pos calibration via CLI)
// ============================================================================

static bool read_accel_for_cal(float* ax, float* ay, float* az, float* tempC) {
    sleep_ms(kCalReadDelayMs);  // ~100Hz sampling rate
    // Use full icm20948_read() instead of icm20948_read_accel() — the accel-only
    // read (6 bytes from ACCEL_XOUT_H) does NOT read through TEMP_OUT_L, so the
    // data-ready flag is never cleared. After ~200 reads the output registers
    // stop updating (all zeros). The full 14-byte read clears data-ready.
    icm20948_data_t data;
    if (!icm20948_read(&g_imu, &data) || !data.accel_valid) {
        return false;
    }
    *ax = data.accel.x;
    *ay = data.accel.y;
    *az = data.accel.z;
    *tempC = data.temperature_c;
    return true;
}

// ============================================================================
// Mag Read Callback (for compass calibration via CLI)
// ============================================================================
// Reads from seqlock — Core 1 keeps running, no I2C contention.
// Bus-agnostic: reads from shared data regardless of sensor transport.

// Staleness gate — only accept mag data with a new mag_read_count.
// Reset before each calibration run so 2nd attempt doesn't inherit stale counter.
static uint32_t g_lastMagReadCount = 0;

// Diagnostic counters for mag read failures (debug mag cal freeze)
static uint32_t g_magDiagSeqlockFail = 0;
static uint32_t g_magDiagNotValid = 0;
static uint32_t g_magDiagStale = 0;
static uint32_t g_magDiagLastSeenCount = 0;

static void reset_mag_read_staleness() {
    g_lastMagReadCount = 0;
    g_magDiagSeqlockFail = 0;
    g_magDiagNotValid = 0;
    g_magDiagStale = 0;
    g_magDiagLastSeenCount = 0;
}

static bool read_mag_from_seqlock(float* mx, float* my, float* mz) {
    shared_sensor_data_t snap = {};
    if (!seqlock_read(&g_sensorSeqlock, &snap)) {
        g_magDiagSeqlockFail++;
        return false;
    }
    g_magDiagLastSeenCount = snap.mag_read_count;
    if (!snap.mag_valid) {
        g_magDiagNotValid++;
        // Periodic diagnostic — print every 200 failures to show what's happening
        if (g_magDiagNotValid == 1 || g_magDiagNotValid % kMagDiagPrintModulus == 0) {
            (void)printf("  [mag_valid=false, mag_read_count=%lu, lastAccepted=%lu]\n",
                        (unsigned long)snap.mag_read_count,
                        (unsigned long)g_lastMagReadCount);
            (void)fflush(stdout);
        }
        return false;
    }
    // Only return genuinely new mag data — AK09916 updates at ~100Hz,
    // but the seqlock is written at ~1kHz. Without this check, the mag cal
    // feeds identical samples that get REJECTED_CLOSE by the angular filter.
    if (snap.mag_read_count == g_lastMagReadCount) {
        g_magDiagStale++;
        return false;
    }
    g_lastMagReadCount = snap.mag_read_count;

    // Return RAW mag data for calibration — the ellipsoid solver needs
    // uncorrected samples to compute offset/scale/offdiag corrections.
    *mx = snap.mag_raw_x;
    *my = snap.mag_raw_y;
    *mz = snap.mag_raw_z;
    return true;
}

// ============================================================================
// Calibration Hooks — pause Core 1 during rapid accel reads
// ============================================================================
// Bypass mode eliminates the I2C master race condition (LL Entry 21).
// Hooks only need to pause/unpause Core 1 for bus ownership.

static void cal_pre_hook() {
    // If Core 1 is running sensors, pause it and take I2C ownership
    if (g_sensorPhaseActive && !g_core1I2CPaused.load(std::memory_order_acquire)) {
        g_core1PauseI2C.store(true, std::memory_order_release);
        // Wait for Core 1 to acknowledge pause (max 100ms)
        for (uint32_t i = 0; i < kCore1PauseAckMaxMs; i++) {
            if (g_core1I2CPaused.load(std::memory_order_acquire)) {
                break;
            }
            sleep_ms(1);
        }
    }
}

static void cal_post_hook() {
    // Tell Core 1 to reload calibration and resume sensors
    if (g_sensorPhaseActive) {
        g_calReloadPending.store(true, std::memory_order_release);
        g_core1PauseI2C.store(false, std::memory_order_release);
    }
}

// ============================================================================
// NeoPixel calibration override callback (Phase 5)
// Routes CLI calibration overlays to AO_LedEngine's calibration layer.
// ============================================================================

static void set_cal_neo_override(uint8_t mode) {
    AO_LedEngine_post_override(mode);
}

// ============================================================================
// Calibration Sensor Feed Helper (no-op — Core 1 feeds directly)
// ============================================================================
// Async calibration samples (gyro/level/baro) are now fed by Core 1 in
// core1_read_imu() and core1_read_baro(), using raw sensor data with no
// I2C bus contention. This function exists as the rc_os_feed_cal callback
// target — the wizard's wait loop calls it for watchdog kicking.
// Core 0 must NOT call icm20948_read() or baro_dps310_read() while Core 1
// owns the I2C bus (per MULTICORE_RULES.md invariant at core1_sensor_loop).

static void feed_active_calibration() {
    // Core 1 handles all sensor feeds. Nothing to do here.
}

// ============================================================================
// Sensor Status Callback (for RC_OS CLI 's' command)
// ============================================================================

// Helpers for print_seqlock_sensors() — extracted to reduce CC
static void print_imu_status(const shared_sensor_data_t& snap) {
    if (snap.accel_valid) {
        float aMag = sqrtf(snap.accel_x*snap.accel_x + snap.accel_y*snap.accel_y
                           + snap.accel_z*snap.accel_z);
        printf("Accel (m/s^2): X=%7.3f Y=%7.3f Z=%7.3f |A|=%.3f\n",
               (double)snap.accel_x, (double)snap.accel_y, (double)snap.accel_z,
               (double)aMag);
    } else {
        printf("Accel: invalid\n");
    }
    if (snap.gyro_valid) {
        printf("Gyro  (rad/s): X=%7.4f Y=%7.4f Z=%7.4f\n",
               (double)snap.gyro_x, (double)snap.gyro_y, (double)snap.gyro_z);
    } else {
        printf("Gyro: invalid\n");
    }
    if (snap.mag_valid) {
        float magMag = sqrtf(snap.mag_x*snap.mag_x + snap.mag_y*snap.mag_y + snap.mag_z*snap.mag_z);
        printf("Mag     (uT):  X=%6.1f  Y=%6.1f  Z=%6.1f  |M|=%.1f\n",
               (double)snap.mag_x, (double)snap.mag_y, (double)snap.mag_z, (double)magMag);
        float heading = atan2f(-snap.mag_y, snap.mag_x) * kRadToDeg;
        if (heading < 0.0F) { heading += kFullCircleDeg; }
        printf("Heading: %.1f deg (level only)\n", (double)heading);
    } else {
        printf("Mag: not ready\n");
    }
    if (snap.baro_valid) {
        float alt = calibration_get_altitude_agl(snap.pressure_pa);
        printf("Baro: %.1f Pa, %.2f C, AGL=%.2f m\n",
               (double)snap.pressure_pa, (double)snap.baro_temperature_c,
               (double)alt);
    } else {
        printf("Baro: no data yet\n");
    }
}

// NOLINTBEGIN(readability-magic-numbers) — ESKF P indices are state layout
static void print_eskf_gates_and_diags() {
    printf("      gate: bA=%lu/%lu mA=%lu/%lu mR=%lu gA=%lu/%lu zA=%lu/%lu\n",
           (unsigned long)g_eskf.baro_total_accepts_,
           (unsigned long)(g_eskf.baro_total_accepts_ + g_eskf.baro_total_rejects_),
           (unsigned long)g_eskf.mag_total_accepts_,
           (unsigned long)(g_eskf.mag_total_accepts_ + g_eskf.mag_total_rejects_),
           (unsigned long)g_eskf.mag_resets_,
           (unsigned long)g_eskf.gps_pos_total_accepts_,
           (unsigned long)(g_eskf.gps_pos_total_accepts_ + g_eskf.gps_pos_total_rejects_),
           (unsigned long)g_eskf.zupt_total_accepts_,
           (unsigned long)(g_eskf.zupt_total_accepts_ + g_eskf.zupt_total_rejects_));
    printf("      Pvel=%.4f,%.4f,%.4f  Pab=%.6f  Pgb=%.6f\n",
           (double)g_eskf.P(6, 6), (double)g_eskf.P(7, 7), (double)g_eskf.P(8, 8),
           (double)g_eskf.P(9, 9), (double)g_eskf.P(12, 12));
    // 24-state inhibit flags + conditional extended state display
    printf("      inhib: mag=%c wind=%c bbias=%c\n",
           g_eskf.inhibit_mag_states_ ? 'Y' : 'N',
           g_eskf.inhibit_wind_states_ ? 'Y' : 'N',
           g_eskf.inhibit_baro_bias_ ? 'Y' : 'N');
    if (!g_eskf.inhibit_mag_states_) {
        printf("      eMag=%.1f,%.1f,%.1f bMag=%.1f,%.1f,%.1f\n",
               (double)g_eskf.earth_mag.x, (double)g_eskf.earth_mag.y,
               (double)g_eskf.earth_mag.z,
               (double)g_eskf.body_mag_bias.x, (double)g_eskf.body_mag_bias.y,
               (double)g_eskf.body_mag_bias.z);
    }
    if (!g_eskf.inhibit_wind_states_) {
        printf("      wind=%.2f,%.2f m/s\n",
               (double)g_eskf.wind_n_, (double)g_eskf.wind_e_);
    }
    if (!g_eskf.inhibit_baro_bias_) {
        printf("      bBias=%.3f m\n", (double)g_eskf.baro_bias_);
    }
    // IVP-83/84: Phase Q/R + confidence gate status
    if (g_eskf.phase_qr_) {
        printf("      phQ: att=%.1f vel=%.1f  innov: b=%.2f m=%.2f gp=%.2f gv=%.2f\n",
               (double)g_eskf.q_active_.attitude,
               (double)g_eskf.q_active_.velocity,
               (double)g_eskf.innov_baro_.alpha,
               (double)g_eskf.innov_mag_.alpha,
               (double)g_eskf.innov_gps_pos_.alpha,
               (double)g_eskf.innov_gps_vel_.alpha);
    }
    const rc::ConfidenceState* conf = eskf_runner_get_confidence();
    printf("      conf=%c div=%.1f° unc=%lums\n",
           conf->confident ? 'Y' : 'N',
           (double)conf->ahrs_divergence_deg,
           (unsigned long)conf->time_since_confident_ms);
}
// NOLINTEND(readability-magic-numbers)

static void print_eskf_status() {
    if (g_eskfInitialized && g_eskf.healthy()) {
        rc::Vec3 euler = g_eskf.q.to_euler();
        float rollDeg  = euler.x * kRadToDeg;
        float pitchDeg = euler.y * kRadToDeg;
        float yawDeg   = euler.z * kRadToDeg;
        float patt = g_eskf.P(0, 0);
        if (g_eskf.P(1, 1) > patt) { patt = g_eskf.P(1, 1); }
        if (g_eskf.P(2, 2) > patt) { patt = g_eskf.P(2, 2); }
        printf("ESKF: R=%6.2f P=%6.2f Y=%6.2f deg  Patt=%.4f  qnorm=%.6f\n",
               (double)rollDeg, (double)pitchDeg, (double)yawDeg,
               (double)patt, (double)g_eskf.q.norm());
        printf("      vel=%.3f,%.3f,%.3f m/s  bNIS=%.2f mNIS=%.2f\n",
               (double)g_eskf.v.x, (double)g_eskf.v.y, (double)g_eskf.v.z,
               (double)g_eskf.last_baro_nis_,
               (double)g_eskf.last_mag_nis_);
        print_eskf_gates_and_diags();
        const rc::MahonyAHRS* mahony = eskf_runner_get_mahony();
        if (eskf_runner_is_mahony_initialized() && mahony->healthy()) {
            rc::Vec3 meuler = mahony->q.to_euler();
            float mdivDeg = rc::MahonyAHRS::divergence_rad(g_eskf.q, mahony->q) * kRadToDeg;
            printf("Mahony: R=%6.2f P=%6.2f Y=%6.2f deg  Mdiv=%.1f deg\n",
                   (double)(meuler.x * kRadToDeg),
                   (double)(meuler.y * kRadToDeg),
                   (double)(meuler.z * kRadToDeg),
                   (double)mdivDeg);
        }
        {
            uint32_t benchAvg = 0, benchMin = 0, benchMax = 0, benchCount = 0;
            eskf_runner_get_bench(&benchAvg, &benchMin, &benchMax, &benchCount);
            if (benchCount > 0) {
                printf("      predict: %luus avg, %luus min, %luus max (%lu calls)\n",
                       (unsigned long)benchAvg, (unsigned long)benchMin,
                       (unsigned long)benchMax, (unsigned long)benchCount);
            }
        }
        printf("      buf: %lu/%lu samples\n",
               (unsigned long)eskf_runner_get_buffer_count(), (unsigned long)kEskfBufferSamples);
    } else if (g_eskfInitialized) {
        printf("ESKF: UNHEALTHY (stopped, awaiting re-init)\n");
    } else {
        printf("ESKF: waiting for stationary init...\n");
    }
}

static void print_gps_status(const shared_sensor_data_t& snap) {
    // GPS — show transport label
    const char* gpsLabel = "???";
    if (g_gpsTransport == GPS_TRANSPORT_UART) {
        gpsLabel = "UART";
    } else if (g_gpsTransport == GPS_TRANSPORT_I2C) {
        gpsLabel = "I2C";
    }
    if (snap.gps_read_count > 0) {
        if (snap.gps_valid) {
            printf("GPS (%s): %.7f, %.7f, %.1f m MSL\n",
                   gpsLabel,
                   snap.gps_lat_1e7 / kGpsCoordScale,
                   snap.gps_lon_1e7 / kGpsCoordScale,
                   (double)snap.gps_alt_msl_m);
            printf("     Fix=%u Sats=%u Speed=%.1f m/s HDOP=%.2f VDOP=%.2f\n",
                   snap.gps_fix_type, snap.gps_satellites,
                   (double)snap.gps_ground_speed_mps,
                   (double)snap.gps_hdop, (double)snap.gps_vdop);
        } else {
            printf("GPS (%s): no fix (%u sats)\n",
                   gpsLabel, snap.gps_satellites);
            printf("     RMC=%c GGA=%u GSA=%u",
                   snap.gps_rmc_valid ? 'A' : 'V',
                   snap.gps_gga_fix,
                   snap.gps_gsa_fix_mode);
            if (g_gpsTransport == GPS_TRANSPORT_UART) {
                printf("  rxOvf=%lu",
                       (unsigned long)gps_uart_get_overflow_count());
            }
            printf("\n");
            if (g_bestGpsValid.load(std::memory_order_relaxed)) {
                printf("     Best: %.7f, %.7f  Sats=%u HDOP=%.2f\n",
                       g_bestGpsFix.lat_1e7 / kGpsCoordScale,
                       g_bestGpsFix.lon_1e7 / kGpsCoordScale,
                       g_bestGpsFix.satellites,
                       (double)g_bestGpsFix.hdop);
            } else if (snap.gps_lat_1e7 != 0 || snap.gps_lon_1e7 != 0) {
                printf("     Last fix: %.7f, %.7f\n",
                       snap.gps_lat_1e7 / kGpsCoordScale,
                       snap.gps_lon_1e7 / kGpsCoordScale);
            }
        }
    } else if (g_gpsInitialized) {
        printf("GPS (%s): initialized, no reads yet\n", gpsLabel);
    } else {
        printf("GPS: not detected\n");
    }
}

static void print_sensor_counts(const shared_sensor_data_t& snap) {
    printf("Reads: I=%lu M=%lu B=%lu G=%lu  "
           "Errors: I=%lu B=%lu G=%lu\n",
           (unsigned long)snap.imu_read_count,
           (unsigned long)snap.mag_read_count,
           (unsigned long)snap.baro_read_count,
           (unsigned long)snap.gps_read_count,
           (unsigned long)snap.imu_error_count,
           (unsigned long)snap.baro_error_count,
           (unsigned long)snap.gps_error_count);
    if (AO_Logger_is_initialized()) {
        const rc::RingBuffer* ring = AO_Logger_get_ring();
        printf("  Log: %lu frames stored, %lu capacity\n",
               (unsigned long)rc::ring_stored_count(ring),
               (unsigned long)rc::ring_capacity_frames(ring));
    }
    {
        const rc::FlightTableState* ft = AO_Logger_get_flight_table();
        if (ft->loaded) {
            printf("  Flash: %lu flights, %.1f%% used\n",
                   (unsigned long)rc::flight_table_count(ft),
                   static_cast<double>(rc::flight_table_used_pct(ft)));
        }
    }
}

// Print sensor data from seqlock snapshot (Core 1 driving sensors)
static void print_seqlock_sensors(const shared_sensor_data_t& snap) {
    print_imu_status(snap);
    print_eskf_status();
    print_gps_status(snap);
    print_sensor_counts(snap);
}

// Compact ESKF live output (1Hz, one line per update)
static void print_eskf_live() {
    if (!g_eskfInitialized) {
        printf("ESKF: waiting for init...\n");
        return;
    }
    if (!g_eskf.healthy()) {
        printf("ESKF: UNHEALTHY\n");
        return;
    }
    // Read seqlock for baro_read_count
    shared_sensor_data_t snap = {};
    seqlock_read(&g_sensorSeqlock, &snap);

    float alt = -g_eskf.p.z;
    float vz = g_eskf.v.z;
    float vh = sqrtf(g_eskf.v.x * g_eskf.v.x + g_eskf.v.y * g_eskf.v.y);
    float patt = g_eskf.P(0, 0);
    if (g_eskf.P(1, 1) > patt) { patt = g_eskf.P(1, 1); }
    if (g_eskf.P(2, 2) > patt) { patt = g_eskf.P(2, 2); }
    // NOLINTNEXTLINE(readability-magic-numbers) — ESKF P(5,5) = position-down variance
    float ppos = g_eskf.P(5, 5);

    rc::Vec3 euler = g_eskf.q.to_euler();
    float yawDeg = euler.z * kRadToDeg;

    const rc::MahonyAHRS* mahony_live = eskf_runner_get_mahony();
    float mdivDeg = (eskf_runner_is_mahony_initialized() && mahony_live->healthy())
                    ? rc::MahonyAHRS::divergence_rad(g_eskf.q, mahony_live->q) * kRadToDeg
                    : -1.0F;
    printf("alt=%.2f vz=%.2f vh=%.2f Y=%.1f Patt=%.4f Pp=%.4f bNIS=%.2f mNIS=%.2f mA=%lu/%lu Z=%c zNIS=%.2f G=%c gNIS=%.2f Mdiv=%.1f B=%lu\n",
           (double)alt, (double)vz, (double)vh, (double)yawDeg,
           (double)patt, (double)ppos,
           (double)g_eskf.last_baro_nis_,
           (double)g_eskf.last_mag_nis_,
           (unsigned long)g_eskf.mag_total_accepts_,
           (unsigned long)(g_eskf.mag_total_accepts_ + g_eskf.mag_total_rejects_),
           g_eskf.last_zupt_active_ ? 'Y' : 'N',
           (double)g_eskf.last_zupt_nis_,
           g_eskf.has_origin_ ? 'Y' : 'N',
           (double)g_eskf.last_gps_pos_nis_,
           (double)mdivDeg,
           (unsigned long)snap.baro_read_count);
}

// Print sensor data from direct I2C reads (Core 0 owns bus, pre-sensor-phase)
static void print_direct_sensors() {
    if (g_imuInitialized) {
        icm20948_data_t data;
        if (icm20948_read(&g_imu, &data)) {
            float gx = 0.0F;
            float gy = 0.0F;
            float gz = 0.0F;
            float ax = 0.0F;
            float ay = 0.0F;
            float az = 0.0F;
            calibration_apply_gyro(data.gyro.x, data.gyro.y, data.gyro.z,
                                   &gx, &gy, &gz);
            calibration_apply_accel(data.accel.x, data.accel.y, data.accel.z,
                                    &ax, &ay, &az);
            printf("Accel (m/s^2): X=%7.3f Y=%7.3f Z=%7.3f\n",
                   (double)ax, (double)ay, (double)az);
            printf("Gyro  (rad/s): X=%7.4f Y=%7.4f Z=%7.4f\n",
                   (double)gx, (double)gy, (double)gz);
            if (data.mag_valid) {
                float mxCal = 0.0F;
                float myCal = 0.0F;
                float mzCal = 0.0F;
                calibration_apply_mag(data.mag.x, data.mag.y, data.mag.z,
                                      &mxCal, &myCal, &mzCal);
                float magMag = sqrtf(mxCal*mxCal + myCal*myCal + mzCal*mzCal);
                printf("Mag     (uT):  X=%6.1f  Y=%6.1f  Z=%6.1f  |M|=%.1f\n",
                       (double)mxCal, (double)myCal, (double)mzCal, (double)magMag);
                float heading = atan2f(-myCal, mxCal) * kRadToDeg;
                if (heading < 0.0F) { heading += kFullCircleDeg; }
                printf("Heading: %.1f deg (level only)\n", (double)heading);
            } else {
                printf("Mag: not ready\n");
            }
            printf("Temp: %.1f C\n", (double)data.temperature_c);
        } else {
            printf("IMU: read failed\n");
        }
    } else {
        printf("IMU: not initialized\n");
    }

    if (g_baroContinuous) {
        baro_dps310_data_t bdata;
        if (baro_dps310_read(&bdata) && bdata.valid) {
            float alt = calibration_get_altitude_agl(bdata.pressure_pa);
            printf("Baro: %.1f Pa, %.2f C, AGL=%.2f m\n",
                   (double)bdata.pressure_pa, (double)bdata.temperature_c,
                   (double)alt);
        } else {
            printf("Baro: read failed\n");
        }
    } else {
        printf("Baro: not initialized\n");
    }
}

static void print_cal_params() {
    const calibration_store_t* cal = calibration_manager_get();
    uint16_t flags = cal->cal_flags;
    printf("  Cal Flags: 0x%04X [%s%s%s%s%s]\n", flags,
           ((flags & CAL_STATUS_LEVEL) != 0) ? "Lv " : "",
           ((flags & CAL_STATUS_ACCEL_6POS) != 0) ? "6P " : "",
           ((flags & CAL_STATUS_GYRO) != 0) ? "Gy " : "",
           ((flags & CAL_STATUS_MAG) != 0) ? "Mg " : "",
           ((flags & CAL_STATUS_BARO) != 0) ? "Ba " : "");
    printf("  Accel: off=[%.4f, %.4f, %.4f]\n",
           (double)cal->accel.offset.x, (double)cal->accel.offset.y,
           (double)cal->accel.offset.z);
    printf("         scl=[%.4f, %.4f, %.4f]\n",
           (double)cal->accel.scale.x, (double)cal->accel.scale.y,
           (double)cal->accel.scale.z);
    printf("         odg=[%.4f, %.4f, %.4f]\n",
           (double)cal->accel.offdiag.x, (double)cal->accel.offdiag.y,
           (double)cal->accel.offdiag.z);
    printf("  Gyro:  bias=[%.6f, %.6f, %.6f]\n",
           (double)cal->gyro.bias.x, (double)cal->gyro.bias.y,
           (double)cal->gyro.bias.z);
    if ((flags & CAL_STATUS_MAG) != 0) {
        printf("  Mag:   off=[%.1f, %.1f, %.1f]\n",
               (double)cal->mag.offset.x, (double)cal->mag.offset.y,
               (double)cal->mag.offset.z);
        printf("         scl=[%.4f, %.4f, %.4f]\n",
               (double)cal->mag.scale.x, (double)cal->mag.scale.y,
               (double)cal->mag.scale.z);
    }
    const float* rotMat = cal->board_rotation.m;
    // NOLINTBEGIN(readability-magic-numbers) — row-major DCM indices [0..8]
    printf("  Rot:   [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]\n",
           (double)rotMat[0], (double)rotMat[1], (double)rotMat[2],
           (double)rotMat[3], (double)rotMat[4], (double)rotMat[5],
           (double)rotMat[6], (double)rotMat[7], (double)rotMat[8]);
    // NOLINTEND(readability-magic-numbers)
}

static void print_sensor_status() {
    printf("\n========================================\n");
    printf("  Sensor Readings (calibrated)\n");
    printf("========================================\n");

    // Once Core 1 owns I2C, read from seqlock to prevent bus contention.
    // Before sensor phase, fall back to direct I2C reads.
    if (g_sensorPhaseActive) {
        shared_sensor_data_t snap = {};
        if (seqlock_read(&g_sensorSeqlock, &snap)) {
            print_seqlock_sensors(snap);
        } else {
            printf("Seqlock read failed (retries exhausted)\n");
        }
    } else {
        print_direct_sensors();
    }

    print_cal_params();
    printf("========================================\n\n");
}

// ============================================================================
// Hardware Status (boot banner + CLI 'b' command callback)
// ============================================================================

static const char* get_device_name(uint8_t addr) {
    switch (addr) {
        case kI2cAddrIcm20948:  return "ICM-20948";
        case kI2cAddrAk09916:   return "AK09916 (mag)";
        case kI2cAddrDps310:    return "DPS310";
        case kI2cAddrPa1010d:   return "PA1010D GPS";
        case kI2cAddrIcm20948Alt: return "ICM-20948 (AD0=LOW)";
        case kI2cAddrDps310Alt:   return "DPS310 (alt)";
        default:                 return "Unknown";
    }
}

// Print I2C device detection results
static void hw_validate_i2c_devices() {
    // Skip I2C probes when Core 1 owns the bus (LL Entry 23: probe collision)
    if (rc_os_i2c_scan_allowed) {
        static constexpr uint8_t kExpected[] = {
            kI2cAddrAk09916,   // 0x0C (AK09916 visible via bypass mode)
            kI2cAddrIcm20948,  // 0x69
            kI2cAddrDps310,    // 0x77
        };
        int foundCount = 0;
        for (const auto& addr : kExpected) {
            bool found = i2c_bus_probe(addr);
            printf("[----] I2C 0x%02X (%s): %s\n",
                   addr, get_device_name(addr),
                   found ? "FOUND" : "NOT FOUND");
            if (found) {
                foundCount++;
            }
        }
        printf("[INFO] Sensors found: %d/%zu expected\n",
               foundCount, sizeof(kExpected) / sizeof(kExpected[0]));
    } else {
        printf("[INFO] I2C probe skipped (Core 1 owns bus)\n");
    }
}

// Print sensor initialization results
static void hw_validate_sensors() {
    if (g_imuInitialized) {
        printf("[PASS] ICM-20948 init (WHO_AM_I=0xEA)\n");
        printf("[%s] AK09916 magnetometer %s\n",
               g_imu.mag_initialized ? "PASS" : "WARN",
               g_imu.mag_initialized ? "ready" : "not ready");

        // Pre-Stage 6 check: DLPF/ODR readback from Bank 2
        uint8_t accelCfg = 0;
        uint8_t gyroCfg1 = 0;
        uint8_t gyroDiv = 0;
        if (icm20948_read_config_registers(&g_imu, &accelCfg, &gyroCfg1, &gyroDiv)) {
            // ACCEL_CONFIG[5:3]=DLPF_CFG, [2:1]=FS_SEL, [0]=DLPF_EN
            // GYRO_CONFIG_1[5:3]=DLPF_CFG, [2:1]=FS_SEL, [0]=DLPF_EN
            uint8_t accelDlpf = (accelCfg >> 3) & kDlpfCfgMask;
            bool accelDlpfEn  = (accelCfg & 0x01U) != 0;
            uint8_t gyroDlpf  = (gyroCfg1 >> 3) & kDlpfCfgMask;
            bool gyroDlpfEn   = (gyroCfg1 & 0x01U) != 0;
            printf("  IMU config: accelDLPF=%u(%s) gyroDLPF=%u(%s) gyroDiv=%u\n",
                   accelDlpf, accelDlpfEn ? "on" : "off",
                   gyroDlpf, gyroDlpfEn ? "on" : "off",
                   gyroDiv);
        }
    } else {
        printf("[FAIL] ICM-20948 init failed\n");
    }

    if (g_baroInitialized && g_baroContinuous) {
        printf("[PASS] DPS310 init OK, continuous mode active\n");
    } else if (g_baroInitialized) {
        printf("[WARN] DPS310 init OK, continuous mode failed\n");
    } else {
        printf("[FAIL] DPS310 init failed\n");
    }

    if (g_gpsInitialized) {
        if (g_gpsTransport == GPS_TRANSPORT_UART) {
            printf("[PASS] GPS init (UART on GPIO0/1, 9600 baud)\n");
        } else {
            printf("[PASS] GPS init (I2C at 0x10, 500us settling delay)\n");
        }
    } else {
        printf("[----] GPS not detected (UART or I2C)\n");
    }
}

static void print_psram_status() {
    if (g_psramSize > 0) {
        printf("[%s] PSRAM: %luMB at 0x%08lX",
               g_psramSelfTestPassed ? "PASS" : "FAIL",
               (unsigned long)(g_psramSize / (1024 * 1024)),
               (unsigned long)rc::kPsramCachedBase);
        if (g_psramSelfTestPassed) {
            printf(" self-test OK");
        } else {
            printf(" self-test FAIL");
        }
        if (g_psramFlashSafePassed) {
            printf(", flash-safe OK\n");
        } else {
            printf(", flash-safe %s\n",
                   g_psramSelfTestPassed ? "FAIL" : "skipped");
        }
    } else {
        printf("[----] PSRAM: not detected (GPIO %d)\n",
               rocketchip::pins::kPsramCs);
    }
}

static void print_logging_status() {
    if (AO_Logger_is_initialized()) {
        bool isPsram = (g_psramSize > 0 && g_psramSelfTestPassed);
        // Display rate: 200Hz / 4 = 50Hz (PSRAM), 200Hz / 8 = 25Hz (SRAM)
        uint32_t rate = isPsram ? 50U : 25U;
        const rc::RingBuffer* ring = AO_Logger_get_ring();
        printf("[PASS] Logging: %s ring, %luHz, %lu frames capacity, %lu stored\n",
               isPsram ? "PSRAM" : "SRAM",
               (unsigned long)rate,
               (unsigned long)rc::ring_capacity_frames(ring),
               (unsigned long)rc::ring_stored_count(ring));
    } else {
        printf("[----] Logging: not initialized\n");
    }
}

static void print_flash_status() {
    const rc::FlightTableState* ft = AO_Logger_get_flight_table();
    if (ft->loaded) {
        uint32_t nFlights = rc::flight_table_count(ft);
        float usedPct = rc::flight_table_used_pct(ft);
        uint32_t freeSectors = rc::flight_table_capacity_sectors() -
                               rc::flight_table_used_sectors(ft);
        uint32_t freeMB = (freeSectors * rc::kFlashSectorSize) / (1024U * 1024U);
        printf("[PASS] Flash: %.1f%% used (%lu flights, %luMB free)\n",
               static_cast<double>(usedPct),
               (unsigned long)nFlights,
               (unsigned long)freeMB);
    } else {
        printf("[INFO] Flash: flight table empty (fresh)\n");
    }
}

static void print_gps_status() {
    if (g_bestGpsValid.load(std::memory_order_acquire)) {
        printf("  Best GPS: Fix=%u Sats=%u HDOP=%.2f\n",
               g_bestGpsFix.fix_type, g_bestGpsFix.satellites,
               (double)g_bestGpsFix.hdop);
        printf("            %.7f, %.7f, %.1f m MSL\n",
               g_bestGpsFix.lat_1e7 / kGpsCoordScale,
               g_bestGpsFix.lon_1e7 / kGpsCoordScale,
               (double)g_bestGpsFix.alt_msl_m);
    } else if (g_gpsInitialized) {
        printf("  Best GPS: no fix acquired yet\n");
    }

    const gps_session_stats_t* gpsSess = eskf_runner_get_gps_session();
    if (gpsSess->gps_updates > 0) {
        printf("  GPS session: %lu updates, max_dist=%.1fm, last_dist=%.1fm\n",
               (unsigned long)gpsSess->gps_updates,
               (double)gpsSess->max_dist_from_origin_m,
               (double)gpsSess->last_dist_from_origin_m);
        printf("              last_pos N=%.1f E=%.1f m  gNIS=[%.2f, %.2f]\n",
               (double)gpsSess->last_pos_n_m,
               (double)gpsSess->last_pos_e_m,
               (double)gpsSess->min_gps_nis,
               (double)gpsSess->max_gps_nis);
    }
}

static void print_hw_status() {
    printf("\n=== Hardware Status ===\n");
    printf("  Build: %s (%s %s)\n", kBuildTag, __DATE__, __TIME__);

    printf("[PASS] Build + boot (you're reading this)\n");
    printf("[PASS] Red LED GPIO initialized (pin %d, %s)\n",
           board::kLedPin, board::kLedActiveHigh ? "active-high" : "active-low");
    printf("[%s] NeoPixel PIO initialized (pin %d)\n",
           g_neopixelInitialized ? "PASS" : "FAIL", kNeoPixelPin);
    printf("[PASS] USB CDC connected\n");

    uint32_t ts = time_us_32();
    printf("[%s] Debug macros functional (timestamp=%lu us)\n",
           ts > 0 ? "PASS" : "FAIL", (unsigned long)ts);

    if (g_i2cInitialized) {
        printf("[PASS] I2C bus initialized at %lukHz (SDA=%d, SCL=%d)\n",
               (unsigned long)(kI2cBusFreqHz / 1000), kI2cBusSdaPin, kI2cBusSclPin);
    } else {
        printf("[FAIL] I2C bus failed to initialize\n");
    }

    hw_validate_i2c_devices();
    hw_validate_sensors();

    if (g_radioInitialized) {
        printf("[PASS] Radio: RFM95W LoRa 915 MHz SF7 20dBm (CS=%d RST=%d IRQ=%d)\n",
               rocketchip::pins::kRadioCs, rocketchip::pins::kRadioRst,
               rocketchip::pins::kRadioIrq);
        if constexpr (kRadioModeRx) {
            auto mode = AO_RCOS_get_output_mode();
            const char* mode_name = (mode == StationOutputMode::kAnsi) ? "ANSI dashboard" :
                                    (mode == StationOutputMode::kCsv)  ? "CSV" : "MAVLink";
            printf("[MODE] RX — %s output ('m' to cycle)\n", mode_name);
        } else {
            printf("[MODE] TX CCSDS %dB%s\n",
                   static_cast<int>(rc::ccsds::kNavPacketLen),
                   AO_Telemetry_get_mavlink_output() ? " + USB MAVLink" : "");
        }
    } else if (g_spiInitialized) {
        printf("[----] Radio: not detected (FeatherWing not stacked?)\n");
    }

    print_psram_status();
    print_logging_status();
    print_flash_status();
    print_gps_status();

    printf("=== Status Complete ===\n\n");
}

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

// ============================================================================
// Init: Boot banner and hardware status output
// ============================================================================

static void print_boot_status() {
    printf("\n");
    printf("==============================================\n");
    printf("  RocketChip v%s  Build: ivp74-profile-1\n", kVersionString);
    printf("  Board: %s\n", board::kBoardName);
    printf("  Profile: %s\n", rc::kDefaultRocketProfile.name);
    printf("==============================================\n\n");

    if (g_watchdogReboot) {
        printf("[WARN] *** PREVIOUS REBOOT WAS CAUSED BY WATCHDOG RESET ***\n");
        if (g_recovery.boot_state.valid) {
            printf("[WARN] Reboot #%u, last tick: %u, flight phase: %u\n",
                   static_cast<unsigned>(g_recovery.boot_state.reboot_count),
                   static_cast<unsigned>(g_recovery.boot_state.last_tick_fn),
                   static_cast<unsigned>(g_recovery.boot_state.flight_phase));
        }
        if (g_recovery.launch_abort) {
            printf("[WARN] *** LAUNCH_ABORT: too many rapid reboots (safe mode) ***\n");
        }
        printf("[INFO] Re-enabling watchdog (%lu ms)\n\n",
               (unsigned long)kWatchdogTimeoutMs);
    }

    print_hw_status();
}

// ============================================================================
// Init: RC_OS setup, Core 1 sensor phase start, watchdog enable
// ============================================================================

// Forward declarations for CLI callbacks
static void handle_unhandled_key(int key);
// Flight Director CLI callbacks moved to ao_flight_director.cpp (Phase 3).
// populate_fused_state moved to ao_logger.cpp (Phase 4). Use AO_Logger_populate_fused_state().

// Forward declaration — defined with other station commands below (IVP-99)
[[maybe_unused]] static void print_station_status();

static void init_rc_os_hooks() {
    rc_os_init();
    rc_os_imu_available = g_imuInitialized;
    rc_os_baro_available = g_baroContinuous;
    if constexpr (job::kRadioModeRx) {
        rc_os_print_sensor_status = print_station_status;
    } else {
        rc_os_print_sensor_status = print_sensor_status;
    }
    rc_os_print_boot_summary = print_hw_status;
    rc_os_print_boot_status = print_boot_status;
    rc_os_read_accel = read_accel_for_cal;
    rc_os_read_mag = read_mag_from_seqlock;
    rc_os_reset_mag_staleness = reset_mag_read_staleness;
    rc_os_cal_pre_hook = cal_pre_hook;
    rc_os_cal_post_hook = cal_post_hook;
    rc_os_set_cal_neo = set_cal_neo_override;
    rc_os_feed_cal = feed_active_calibration;
    rc_os_print_eskf_live = print_eskf_live;
    rc_os_on_unhandled_key = handle_unhandled_key;
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

// ============================================================================
// Logging Tick (IVP-52c)
// ============================================================================
// Flash Flush + Erase (IVP-53b)
// ============================================================================

// Watchdog kick callback for flash operations.
// During multi-sector flash writes, the main loop is blocked.
// Feed PIO watchdog between sectors (~45ms each) to prevent timeout.
static void flush_kick_watchdog() {
    rc::pio_watchdog_feed();
}

static void cmd_flush_log() {
    if (!AO_Logger_is_initialized()) {
        printf("Logging not initialized.\n");
        return;
    }
    rc::FlightTableState* ft = AO_Logger_get_flight_table_mut();
    if (!ft->loaded) {
        printf("Flight table not loaded.\n");
        return;
    }

    rc::RingBuffer* ring = AO_Logger_get_ring_mut();
    uint32_t stored = rc::ring_stored_count(ring);
    if (stored == 0) {
        printf("No frames in ring buffer.\n");
        return;
    }

    bool isPsram = (g_psramSize > 0 && g_psramSelfTestPassed);
    // Display rate: 200Hz / 4 = 50Hz (PSRAM), 200Hz / 8 = 25Hz (SRAM)
    uint8_t rate = isPsram ? 50U : 25U;

    printf("Flushing %lu frames to flash...\n", (unsigned long)stored);

    // Build summary from current state (minimal — no flight state machine yet)
    rc::FlightMetadata meta = {};
    rc::FlightSummary summ = {};
    summ.frame_count = stored;

    rc::FlushResult result = rc::flush_ring_to_flash(
        ring, ft, &meta, &summ, rate, flush_kick_watchdog);

    switch (result) {
    case rc::FlushResult::kOk:
        printf("Flush OK. Flight #%lu saved (%lu frames, %lu sectors).\n",
               (unsigned long)rc::flight_table_count(ft),
               (unsigned long)stored,
               (unsigned long)(stored * rc::kPcmFrameStandardSize + rc::kFlashSectorSize - 1)
                   / rc::kFlashSectorSize);
        break;
    case rc::FlushResult::kFlashFull:
        printf("Flash full. Erase flights with 'x' command.\n");
        break;
    case rc::FlushResult::kTableFull:
        printf("Flight table full (32 flights). Erase with 'x'.\n");
        break;
    case rc::FlushResult::kEraseError:
        printf("Flash erase error.\n");
        break;
    case rc::FlushResult::kWriteError:
        printf("Flash write error.\n");
        break;
    case rc::FlushResult::kTableSaveError:
        printf("Flight table save error.\n");
        break;
    default:
        printf("Flush error: %d\n", static_cast<int>(result));
        break;
    }
}

static void cmd_erase_all_flights() {
    rc::FlightTableState* ft = AO_Logger_get_flight_table_mut();
    if (!ft->loaded) {
        printf("Flight table not loaded.\n");
        return;
    }

    uint32_t count = rc::flight_table_count(ft);
    if (count == 0) {
        printf("No flights to erase.\n");
        return;
    }

    printf("Erase ALL %lu flights? Type 'yes' + Enter to confirm: ",
           (unsigned long)count);

    // Read up to 8 chars with 10s timeout (blocking — acceptable for destructive op)
    char buf[8] = {};
    int pos = 0;
    while (pos < kEraseInputMaxChars) {
        int ch = getchar_timeout_us(kEraseInputTimeoutUs);
        if (ch == PICO_ERROR_TIMEOUT) {
            break;
        }
        if (ch == '\r' || ch == '\n') {
            break;
        }
        buf[pos++] = static_cast<char>(ch);
        putchar(ch);  // echo
    }
    buf[pos] = '\0';
    printf("\n");

    if (pos != 3 || buf[0] != 'y' || buf[1] != 'e' || buf[2] != 's') {
        printf("Erase cancelled.\n");
        return;
    }

    printf("Erasing flight log sectors...\n");
    if (!rc::flight_log_erase_all(ft, flush_kick_watchdog)) {
        printf("Flash erase error.\n");
        return;
    }

    // Erase flight table in flash
    if (!rc::flight_table_erase_flash()) {
        printf("Table erase error.\n");
        return;
    }

    // Reset in-memory table
    rc::flight_table_erase_all(ft);
    ft->loaded = true;

    // Save fresh empty table
    rc::flight_table_save(ft);

    printf("All flights erased.\n");
}

// ============================================================================
// IVP-54a: Flight list + binary download
// ============================================================================

static void cmd_list_flights() {
    const rc::FlightTableState* ft = AO_Logger_get_flight_table();
    if (!ft->loaded) {
        printf("Flight table not loaded.\n");
        return;
    }
    uint32_t count = rc::flight_table_count(ft);
    if (count == 0) {
        printf("No flights stored.\n");
        return;
    }
    printf("\n  #  Frames  Rate  Sectors  Size(KB)\n");
    printf("  -- ------  ----  -------  --------\n");
    for (uint32_t i = 0; i < count; ++i) {
        rc::FlightLogEntry entry = {};
        if (rc::flight_table_get_entry(ft, i, &entry)) {
            uint32_t sizeKb = (entry.sector_count * rc::kFlashSectorSize) / 1024;
            printf("  %2lu %6lu  %3uHz  %5lu  %6lu\n",
                   (unsigned long)(i + 1),
                   (unsigned long)entry.frame_count,
                   (unsigned)entry.log_rate_hz,
                   (unsigned long)entry.sector_count,
                   (unsigned long)sizeKb);
        }
    }
    printf("\n  Flash: %.1f%% used (%lu flights)\n\n",
           static_cast<double>(rc::flight_table_used_pct(ft)),
           (unsigned long)count);
}

// Read multi-digit flight number from USB (blocks up to 3s)
static int read_flight_number() {
    int num = 0;
    bool gotDigit = false;
    for (int i = 0; i < 3; ++i) {  // max 2 digits + enter
        int c = getchar_timeout_us(kFlightNumTimeoutUs);
        if (c == PICO_ERROR_TIMEOUT) {
            break;
        }
        if (c >= '0' && c <= '9') {
            num = num * 10 + (c - '0');
            gotDigit = true;
        } else if (c == '\r' || c == '\n') {
            break;
        }
    }
    return gotDigit ? num : -1;
}

// Stream raw binary frames from flash via XIP, skipping sector padding.
// Returns finalized CRC-32 of all binary data sent.
static uint32_t stream_flight_binary(const rc::FlightLogEntry& entry) {
    uint32_t frame_size = entry.frame_size;
    uint32_t frames_per_sector = rc::kFlashSectorSize / frame_size;
    uint32_t flash_base = entry.start_sector * rc::kFlashSectorSize;
    const uint8_t* xip_base = reinterpret_cast<const uint8_t*>(
        XIP_BASE + flash_base);

    // Disable CRLF translation for raw binary output — SDK converts 0x0A→0x0D0A
    stdio_set_translate_crlf(&stdio_usb, false);

    uint32_t running_crc = kCrc32InitXor;
    uint32_t frames_sent = 0;

    while (frames_sent < entry.frame_count) {
        uint32_t sector_idx = frames_sent / frames_per_sector;
        uint32_t frame_in_sector = frames_sent % frames_per_sector;
        uint32_t offset = sector_idx * rc::kFlashSectorSize +
                          frame_in_sector * frame_size;

        uint32_t remaining_in_sector = frames_per_sector - frame_in_sector;
        uint32_t remaining_total = entry.frame_count - frames_sent;
        uint32_t batch = (remaining_in_sector < remaining_total)
                             ? remaining_in_sector : remaining_total;
        uint32_t batch_bytes = batch * frame_size;

        fwrite(xip_base + offset, 1, batch_bytes, stdout);
        fflush(stdout);

        running_crc = rc::crc32_update(running_crc,
                                       xip_base + offset, batch_bytes);
        frames_sent += batch;
        rc::pio_watchdog_feed();
    }

    stdio_set_translate_crlf(&stdio_usb, true);
    return running_crc ^ kCrc32InitXor;
}

static void cmd_download_flight() {
    const rc::FlightTableState* ft = AO_Logger_get_flight_table();
    if (!ft->loaded) {
        printf("Flight table not loaded.\n");
        return;
    }
    uint32_t count = rc::flight_table_count(ft);
    if (count == 0) {
        printf("No flights stored.\n");
        return;
    }

    printf("Flight # (1-%lu): ", (unsigned long)count);
    int num = read_flight_number();
    if (num < 1 || static_cast<uint32_t>(num) > count) {
        printf("\nInvalid flight number.\n");
        return;
    }
    printf("%d\n", num);

    rc::FlightLogEntry entry = {};
    if (!rc::flight_table_get_entry(ft, static_cast<uint32_t>(num - 1),
                                     &entry)) {
        printf("Failed to read flight entry.\n");
        return;
    }

    // Text header
    printf("RCBIN:%d:%lu:%u:%lu\n",
           num,
           (unsigned long)entry.frame_count,
           (unsigned)entry.log_rate_hz,
           (unsigned long)entry.frame_size);

    uint32_t crc = stream_flight_binary(entry);
    printf("RCEND:%08lX\n", (unsigned long)crc);
}

static void cmd_radio_status() {
    const auto* rs = AO_Radio_get_state();
    if (!rs->initialized) {
        printf("Radio not initialized.\n");
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if constexpr (kRadioModeRx) {
        // Station/Relay RX display
        uint32_t gap = (rs->rx_count > 0) ? (now - rs->last_rx_ms) : 0;
        printf("RX: %lu pkts  seq=%u  %ddBm  %ddB SNR  %lu CRC err\n",
               (unsigned long)rs->rx_count,
               static_cast<unsigned>(rs->last_rx_seq),
               static_cast<int>(rs->last_rx_rssi),
               static_cast<int>(rs->last_rx_snr),
               (unsigned long)rs->rx_crc_errors);
        printf("    last=%lu.%lus ago  phase=%d\n",
               (unsigned long)(gap / 1000),
               (unsigned long)((gap % 1000) / 100),
               static_cast<int>(rs->scheduler.phase));
        if constexpr (job::kRole == job::DeviceRole::kRelay) {
            printf("    relayed=%lu\n", (unsigned long)rs->relay_count);
        }
    } else {
        // Vehicle TX display
        printf("TX: %lu sent  %u fail  phase=%d\n",
               (unsigned long)rs->tx_count,
               static_cast<unsigned>(rs->tx_consec_fail),
               static_cast<int>(rs->scheduler.phase));
    }
}

// ============================================================================
// Station RX Telemetry Display (IVP-99: Station CLI)
// ============================================================================

// Print decoded vehicle telemetry fields (helper for print_station_status)
static void print_station_rx_fields(const rc::TelemetryState& t,
                                     const RadioAoState* rs,
                                     uint32_t met_ms, uint16_t seq) {
    static constexpr float kMmToM  = 0.001f;
    static constexpr float kMmToFt = 0.00328084f;
    static constexpr float kCmsToMs = 0.01f;

    float alt_m  = static_cast<float>(t.baro_alt_mm) * kMmToM;
    float alt_ft = static_cast<float>(t.baro_alt_mm) * kMmToFt;
    float vvel   = static_cast<float>(t.baro_vvel_cms) * kCmsToMs;
    uint8_t fix  = (t.gps_fix_sats >> 4) & 0x0F;
    uint8_t sats = t.gps_fix_sats & 0x0F;
    bool eskf_ok = (t.health & rc::kHealthEskfHealthy) != 0;

    uint32_t age_ms = to_ms_since_boot(get_absolute_time()) - rs->last_rx_ms;
    uint32_t lost = 0;
    if (rs->rx_count > 1) {
        uint32_t expected = static_cast<uint32_t>(rs->last_rx_seq) + 1;
        if (expected > rs->rx_count) { lost = expected - rs->rx_count; }
    }

    const char* phase = rc::flight_phase_name(
        static_cast<rc::FlightPhase>(t.flight_state));

    printf("State: %-8s  Pkts: %lu (%lu lost)\n", phase,
           (unsigned long)rs->rx_count, (unsigned long)lost);
    printf("Alt:   %.1f m (%.0f ft)  Vvel: %.1f m/s\n",
           static_cast<double>(alt_m), static_cast<double>(alt_ft),
           static_cast<double>(vvel));
    printf("RSSI:  %d dBm  SNR: %d dB\n",
           static_cast<int>(rs->last_rx_rssi),
           static_cast<int>(rs->last_rx_snr));
    printf("GPS:   fix=%u sats=%u  Batt: %.2f V\n",
           static_cast<unsigned>(fix), static_cast<unsigned>(sats),
           static_cast<double>(t.battery_mv) * 0.001);
    printf("ESKF:  %s  Temp: %d C\n",
           eskf_ok ? "HEALTHY" : "UNHEALTHY",
           static_cast<int>(t.temperature_c));
    printf("Last:  %lu.%lus ago  MET: %lu.%lus  seq=%u\n",
           (unsigned long)(age_ms / 1000), (unsigned long)((age_ms % 1000) / 100),
           (unsigned long)(met_ms / 1000), (unsigned long)((met_ms % 1000) / 100),
           static_cast<unsigned>(seq));
    if (rs->rx_crc_errors > 0) {
        printf("CRC errors: %lu\n", (unsigned long)rs->rx_crc_errors);
    }
}

[[maybe_unused]]
static void print_station_status() {
    const auto* rs = AO_Radio_get_state();
    const auto* rx = AO_Telemetry_get_rx_state();

    printf("\n=== RocketChip Station ===\n");
    if (!rs->initialized) {
        printf("Radio: not initialized\n");
        return;
    }
    if (!rx->valid) {
        printf("Waiting for vehicle packets...\n");
        printf("RX: %lu pkts  %lu CRC err\n",
               (unsigned long)rs->rx_count, (unsigned long)rs->rx_crc_errors);
        return;
    }
    print_station_rx_fields(rx->telem, rs, rx->met_ms, rx->seq);
}

// Station-only CLI commands (IVP-97)

// Haversine distance (meters) between two lat/lon points in 1e-7 degrees
[[maybe_unused]]
static float haversine_m(int32_t lat1_e7, int32_t lon1_e7,
                          int32_t lat2_e7, int32_t lon2_e7) {
    static constexpr float kDegToRad = 3.14159265f / 180.0f;
    static constexpr float kEarthR   = 6371000.0f;
    static constexpr float kScale    = 1e-7f;

    float lat1 = static_cast<float>(lat1_e7) * kScale * kDegToRad;
    float lat2 = static_cast<float>(lat2_e7) * kScale * kDegToRad;
    float dlat = lat2 - lat1;
    float dlon = (static_cast<float>(lon2_e7 - lon1_e7)) * kScale * kDegToRad;

    float a = sinf(dlat * 0.5f) * sinf(dlat * 0.5f)
            + cosf(lat1) * cosf(lat2) * sinf(dlon * 0.5f) * sinf(dlon * 0.5f);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return kEarthR * c;
}

static void cmd_station_gps() {
    if constexpr (!kRadioModeRx) { return; }
    if (!g_gpsInitialized) {
        printf("Station GPS: not connected\n");
        return;
    }
    printf("Station GPS: fix=%u sats=%u hdop=%.1f\n",
           g_bestGpsFix.fix_type, g_bestGpsFix.satellites,
           static_cast<double>(g_bestGpsFix.hdop));
    printf("  Lat=%.7f Lon=%.7f Alt=%.1fm\n",
           static_cast<double>(g_bestGpsFix.lat_1e7) / 1e7,
           static_cast<double>(g_bestGpsFix.lon_1e7) / 1e7,
           static_cast<double>(g_bestGpsFix.alt_msl_m));
}

static void cmd_station_distance() {
    if constexpr (!kRadioModeRx) { return; }
    if (!g_gpsInitialized || g_bestGpsFix.fix_type < 2) {
        printf("Distance: station GPS has no fix\n");
        return;
    }
    // TODO: read last received vehicle position from AO_Telemetry
    // For now, show station position only
    printf("Station: %.7f, %.7f, %.1fm MSL\n",
           static_cast<double>(g_bestGpsFix.lat_1e7) / 1e7,
           static_cast<double>(g_bestGpsFix.lon_1e7) / 1e7,
           static_cast<double>(g_bestGpsFix.alt_msl_m));
    printf("Vehicle distance: needs received position (IVP-97c)\n");
}

static void handle_unhandled_key(int key) {
    switch (key) {
    case 'l': case 'L': cmd_flush_log(); break;
    case 'x': case 'X': cmd_erase_all_flights(); break;
    case 'd': case 'D':
        if constexpr (kRadioModeRx) { cmd_station_distance(); }
        else { cmd_download_flight(); }
        break;
    case 'g': case 'G':
        if constexpr (kRadioModeRx) { cmd_station_gps(); }
        else { cmd_list_flights(); }
        break;
    case 't': case 'T': cmd_radio_status(); break;
    case 'r':
        if (AO_Radio_get_state()->initialized && !kRadioModeRx) {
            uint8_t newRate = AO_Telemetry_cycle_rate();
            printf("[TX] Rate changed to %dHz\n", static_cast<int>(newRate));
        }
        break;
    case 'm': case 'M':
        AO_RCOS_cycle_output_mode();
        {
            auto mode = AO_RCOS_get_output_mode();
            const char* name = (mode == StationOutputMode::kAnsi) ? "ANSI" :
                               (mode == StationOutputMode::kCsv)  ? "CSV" : "MAVLink";
            if (mode == StationOutputMode::kAnsi) {
                printf("\033[2J\033[H");
            } else {
                printf("\n[RX] Output: %s\n", name);
            }
        }
        break;
    default: break;
    }
}

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
        feed_active_calibration();
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
