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
#include "fusion/mahony_ahrs.h"
#include "fusion/wmm_declination.h"
#include "drivers/spi_bus.h"
#include "drivers/rfm95w.h"
#include "logging/psram_init.h"
#include "logging/log_decimator.h"
#include "logging/ring_buffer.h"
#include "logging/data_convert.h"
#include "logging/flash_flush.h"
#include "logging/flight_table.h"
#include "logging/crc32.h"
#include "rocketchip/pcm_frame.h"
#include "rocketchip/fused_state.h"
#include "rocketchip/telemetry_service.h"
#include "cli/rc_os.h"
#include "watchdog/watchdog_recovery.h"
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
static constexpr const char* kBuildTag = "ivp66-recovery-1";

// Heartbeat: 100ms on, 900ms off
static constexpr uint32_t kHeartbeatOnMs = 100;
static constexpr uint32_t kHeartbeatOffMs = 900;
static constexpr uint32_t kHeartbeatPeriodMs = kHeartbeatOnMs + kHeartbeatOffMs;

// Core 1 sensor loop timing
static constexpr uint32_t kCore1TargetCycleUs = 1000;           // ~1kHz target
static constexpr uint32_t kCore1BaroDivider = 32;               // Baro at ~31Hz (DPS310 continuous = 32 SPS)
static constexpr uint32_t kCore1GpsDivider = 100;               // GPS at ~10Hz
static constexpr uint32_t kGpsMinIntervalUs = 2000;             // MT3333 buffer refill time
static constexpr uint32_t kCore1CalFeedDivider = 10;            // Cal feed at ~100Hz
static constexpr uint32_t kCore1ConsecFailBusRecover = 10;      // I2C bus recovery threshold
static constexpr uint32_t kCore1ConsecFailDevReset = 50;        // ICM-20948 device reset threshold
static constexpr uint32_t kBaroMaxReinitAttempts = 3;           // Max baro re-inits before declaring dead
// Minimum plausible accel magnitude: 9.8 × cos(72°) = 3.0 m/s².
// Below any valid sensor reading — all-zeros output means device is in sleep/reset state.
// Source: gravity projection floor when tilted 72° off vertical.
static constexpr float kAccelMinHealthyMag = 3.0F;

// DPS310 MEAS_CFG register (0x08): data-ready bits
// Per DPS310 datasheet Section 7, PRS_RDY=bit4, TMP_RDY=bit5
static constexpr uint8_t kDps310MeasCfgReg = 0x08;
static constexpr uint8_t kDps310PrsRdy = 0x10;                  // Bit 4
static constexpr uint8_t kDps310TmpRdy = 0x20;                  // Bit 5

// Core 1 pause ACK timeout (for calibration I2C handoff)
static constexpr uint32_t kCore1PauseAckMaxMs = 100;

// MPU stack guard
static constexpr uint32_t kMpuGuardSizeBytes = 64;              // Guard region at bottom of stack

// Fault handler blink pattern
static constexpr int32_t kFaultBlinkFastLoops = 200000;         // ~100ms at 150MHz
static constexpr int32_t kFaultBlinkSlowLoops = 800000;         // ~400ms at 150MHz
static constexpr uint8_t kFaultFastBlinks = 3;                  // Fast blinks before slow

// Watchdog
static constexpr uint32_t kWatchdogTimeoutMs = 5000;            // 5 second timeout

// GPS coordinate bounds (WGS-84)
static constexpr double kLatMaxDeg  =  90.0;
static constexpr double kLatMinDeg  = -90.0;
static constexpr double kLonMaxDeg  = 180.0;
static constexpr double kLonMinDeg  = -180.0;
static constexpr double kGpsCoordScale = 1e7;                   // Degrees to 1e-7 degree integers

// PA1010D SDA settling delay (LL Entry 24)
static constexpr uint32_t kGpsSdaSettleUs = 500;

// Seqlock parameters
static constexpr uint32_t kSeqlockMaxRetries = 4;

// IVP-42d: ESKF propagation — every 5th IMU sample = 200Hz
// Note: count divider discards 4/5 IMU samples. For flight with high vibration,
// compound integration of all samples or higher rate propagation may be needed (R-5).
static constexpr uint32_t kEskfImuDivider = 5;

// Rad/deg conversion for CLI display and geodetic conversion
static constexpr float kRadToDeg = 180.0F / 3.14159265F;
static constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
static constexpr float kFullCircleDeg = 360.0F;           // Heading wrap-around

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

// Core 1 startup delay
static constexpr uint32_t kCore1StartupDelayMs = 500;

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

// NeoPixel timing (Core 1 sensor phase)
static constexpr uint32_t kSensorPhaseTimeoutMs = 300000;       // 5 min — switch to magenta

// NeoPixel calibration override values (set by Core 0 CLI, read by Core 1)
// Core 1 checks this atomic; when non-zero, overrides normal NeoPixel status.
static constexpr uint8_t kCalNeoOff         = 0;  // Normal NeoPixel behavior
static constexpr uint8_t kCalNeoGyro        = 1;  // Blue breathe (keep still)
static constexpr uint8_t kCalNeoLevel       = 2;  // Blue breathe (keep flat)
static constexpr uint8_t kCalNeoBaro        = 3;  // Cyan breathe (sampling)
static constexpr uint8_t kCalNeoAccelWait   = 4;  // Yellow blink (position board)
static constexpr uint8_t kCalNeoAccelSample = 5;  // Yellow solid (hold still)
static constexpr uint8_t kCalNeoMag         = 6;  // Rainbow (rotate freely)
static constexpr uint8_t kCalNeoSuccess     = 7;  // Green solid (step passed)
static constexpr uint8_t kCalNeoFail        = 8;  // Red blink fast (step failed)
// RX mode NeoPixel overlay (set by Core 0 RX tick, read by Core 1)
static constexpr uint8_t kRxNeoReceiving   = 9;  // Green solid (packets arriving)
static constexpr uint8_t kRxNeoGap         = 10; // Yellow blink (>1s gap)
static constexpr uint8_t kRxNeoLost        = 11; // Red blink fast (>5s gap)

// Tracks last NeoPixel mode/color so we only call ws2812_set_mode()
// on transitions, not every cycle (which would reset animation state).
static uint8_t g_lastCalNeoOverride = kCalNeoOff;
static ws2812_mode_t  g_lastNeoMode  = WS2812_MODE_OFF;
static ws2812_rgb_t   g_lastNeoColor = kColorOff;


// ============================================================================
// Global State
// ============================================================================

static bool g_neopixelInitialized = false;
static bool g_i2cInitialized = false;
static bool g_imuInitialized = false;
static bool g_baroInitialized = false;
static bool g_baroContinuous = false;
static bool g_gpsInitialized = false;
static gps_transport_t g_gpsTransport = GPS_TRANSPORT_NONE;
static bool g_spiInitialized = false;
static bool g_radioInitialized = false;
static rfm95w_t g_radio;

// Runtime BW mode: 0=BW125, 1=BW250, 2=BW500 (switchable via RX command)
static uint8_t g_txBwMode = 0;
static const uint8_t kTxBwValues[3] = {
    rfm95w::kBw125, rfm95w::kBw250, rfm95w::kBw500
};
static const char* const kTxBwLabels[3] = { "BW125", "BW250", "BW500" };

// Telemetry service (IVP-59) — replaces radio_test_tx_tick()
static rc::TelemetryServiceState g_telemService;
static rc::TelemetryState g_latestTelem = {};      // Updated by logging_tick()
static bool g_latestTelemValid = false;

static size_t g_psramSize = 0;
static bool g_psramSelfTestPassed = false;
static bool g_psramFlashSafePassed = false;

// Logging subsystem (IVP-52c)
static rc::RingBuffer g_ringBuffer;
static rc::LogDecimator g_decimator;
static bool g_loggingInitialized = false;
// SRAM fallback: 200KB ring buffer at 25Hz if PSRAM unavailable.
// 200KB / 55B = 3636 frames / 25Hz = 145 seconds.
static constexpr uint32_t kSramRingSize = 200U * 1024U;
static uint8_t g_sramRingBuf[kSramRingSize];
// Decimation: 4:1 for PSRAM (200→50Hz), 8:1 for SRAM (200→25Hz)
static constexpr uint32_t kDecimationPsram = 4;
static constexpr uint32_t kDecimationSram = 8;
// Header sync: every 50 frames (~1 second at 50Hz)
static constexpr uint32_t kHeaderSyncDiv = 50;

// Flight table (IVP-53b) — flash-persistent flight log index
static rc::FlightTableState g_flightTable;
static bool g_flightTableLoaded = false;

// Transport-neutral GPS function pointers — set once during init_sensors().
// Avoids if/else on every Core 1 GPS poll cycle.
static bool (*g_gpsFnUpdate)()                    = nullptr;
static bool (*g_gpsFnGetData)(gps_data_t*)       = nullptr;
static bool (*g_gpsFnHasFix)()                    = nullptr;

// Best GPS fix diagnostic: captures the highest-quality fix seen this session.
// Written by Core 1, read by Core 0 CLI. Atomic flag guards visibility
// (not struct consistency — benign for diagnostics, not flight-critical).
struct best_gps_fix_t {
    int32_t lat_1e7;
    int32_t lon_1e7;
    float alt_msl_m;
    float hdop;
    uint8_t satellites;
    uint8_t fix_type;
};
static best_gps_fix_t g_bestGpsFix = {};
static std::atomic<bool> g_bestGpsValid{false};

// IVP-46 outdoor session stats — accumulated while GPS is active, printed on
// reconnect via 's'. Lets the user verify movement gates without a live terminal.
struct gps_session_stats_t {
    float max_dist_from_origin_m;    // Furthest ESKF position from origin (10m gate)
    float last_pos_n_m;              // ESKF N position at last GPS fix (square gate)
    float last_pos_e_m;              // ESKF E position at last GPS fix (square gate)
    float last_dist_from_origin_m;   // Distance from origin at last GPS fix (closure)
    uint32_t gps_updates;            // Total GPS position updates applied
    float min_gps_nis;               // Min GPS NIS seen (innovation health)
    float max_gps_nis;               // Max GPS NIS seen
};
static gps_session_stats_t g_gpsSess = {
    0.0F, 0.0F, 0.0F, 0.0F, 0, kGpsNisSentinel, 0.0F
};

// IVP-66: Watchdog recovery policy — persists flight state across WDT resets.
// Tracks reboot count, safe-mode lockout, ESKF failure backoff.
static rc::WatchdogRecovery g_recovery;

// IVP-42d: ESKF 15-state error-state Kalman filter (Core 0 at 200Hz)
// Static per LL Entry 1: ESKF struct is ~970 bytes (Mat15 P = 900 bytes).
static rc::ESKF g_eskf;
static bool g_eskfInitialized = false;
static uint32_t g_lastEskfImuCount = 0;
static uint32_t g_lastEskfTimestampUs = 0;
static uint32_t g_eskfEpoch = 0;        // Incremented on each ESKF propagation

// IVP-45: Mahony AHRS cross-check (~33 bytes, no stack overflow risk)
static rc::MahonyAHRS g_mahony;
static bool g_mahonyInitialized = false;
static uint32_t g_lastMahonyTimestampUs = 0;

// Compact ESKF state for circular buffer (no P matrix — R-6 council requirement)
// 68 bytes per sample. At 200Hz for 5s = 1000 samples = 68KB static in SRAM.
struct eskf_state_snap_t {
    uint32_t timestamp_us;          // 4B
    float qw, qx, qy, qz;         // 16B — quaternion
    float px, py, pz;              // 12B — position NED (m)
    float vx, vy, vz;              // 12B — velocity NED (m/s)
    float abx, aby, abz;           // 12B — accel bias (m/s²)
    float gbx, gby, gbz;           // 12B — gyro bias (rad/s)
};
static_assert(sizeof(eskf_state_snap_t) == 68, "ESKF snap size changed"); // NOLINT(readability-magic-numbers)

static eskf_state_snap_t g_eskfBuffer[kEskfBufferSamples];
static uint32_t g_eskfBufferIndex = 0;
static uint32_t g_eskfBufferCount = 0;

// ESKF benchmark timing (wall-clock us per predict() call)
static uint32_t g_eskfBenchMin = UINT32_MAX;
static uint32_t g_eskfBenchMax = 0;
static uint32_t g_eskfBenchSum = 0;
static uint32_t g_eskfBenchCount = 0;
// BENCH: predict timing histogram — comment out for production builds
// static constexpr uint32_t kHistBuckets = 8;
// static constexpr uint32_t kHistStep = 100;  // µs per bucket
// static uint32_t g_eskfBenchHist[kHistBuckets] = {};

// Shared sensor data struct (per SEQLOCK_DESIGN.md, council-approved)
// All values calibration-applied, body frame, SI units. 128 bytes in SRAM.
struct shared_sensor_data_t {
    // IMU (68 bytes: 13 floats + 3 uint32_t + 3 bool + 1 pad)
    float accel_x;                          // m/s^2
    float accel_y;
    float accel_z;
    float gyro_x;                           // rad/s
    float gyro_y;
    float gyro_z;
    float mag_x;                            // uT calibrated (when mag_valid)
    float mag_y;
    float mag_z;
    float mag_raw_x;                        // uT raw (for mag recalibration)
    float mag_raw_y;
    float mag_raw_z;
    float imu_temperature_c;
    uint32_t imu_timestamp_us;
    uint32_t imu_read_count;                // Monotonic
    uint32_t mag_read_count;                // Increments only on new mag data
    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
    uint8_t _pad_imu;

    // Barometer (20 bytes)
    float pressure_pa;
    float baro_temperature_c;
    uint32_t baro_timestamp_us;
    uint32_t baro_read_count;
    bool baro_valid;
    uint8_t _pad_baro[3];

    // GPS (32 bytes)
    int32_t gps_lat_1e7;
    int32_t gps_lon_1e7;
    float gps_alt_msl_m;
    float gps_ground_speed_mps;
    float gps_course_deg;
    uint32_t gps_timestamp_us;
    uint32_t gps_read_count;
    uint8_t gps_fix_type;
    uint8_t gps_satellites;
    bool gps_valid;
    // Diagnostic: raw lwGPS fields for debugging fix detection
    uint8_t gps_gga_fix;       // GGA fix quality (0=none, 1=GPS, 2=DGPS)
    uint8_t gps_gsa_fix_mode;  // GSA fix mode (1=none, 2=2D, 3=3D)
    bool gps_rmc_valid;        // RMC status ('A')
    uint8_t _pad_gps[2];
    float gps_hdop;            // Horizontal DOP (0 = unknown)
    float gps_vdop;            // Vertical DOP (0 = unknown)

    // Health (16 bytes)
    uint32_t imu_error_count;
    uint32_t baro_error_count;
    uint32_t gps_error_count;
    uint32_t core1_loop_count;              // For watchdog/stall detection
};

static_assert(sizeof(shared_sensor_data_t) == 148, "Struct size changed — update SEQLOCK_DESIGN.md"); // NOLINT(readability-magic-numbers)
static_assert(sizeof(shared_sensor_data_t) % 4 == 0, "Struct must be 4-byte aligned for memcpy");

// Seqlock wrapper — single buffer with sequence counter
struct sensor_seqlock_t {
    std::atomic<uint32_t> sequence{0};      // Odd = write in progress
    shared_sensor_data_t data = {};
};

static sensor_seqlock_t g_sensorSeqlock;

// Cross-core signaling (atomic flags — FIFO reserved by multicore_lockout)
static std::atomic<bool> g_startSensorPhase{false};
static std::atomic<bool> g_sensorPhaseDone{false};
static std::atomic<bool> g_calReloadPending{false};
static std::atomic<bool> g_core1PauseI2C{false};
static std::atomic<bool> g_core1I2CPaused{false};
static std::atomic<bool> g_core1LockoutReady{false};  // Core 1 has called multicore_lockout_victim_init()

// INTERIM: NeoPixel calibration override (Phase M.5).
// Replace with proper AP_Notify-style status state machine when implemented.
// Core 0 (CLI) writes, Core 1 reads in core1_neopixel_update().
static std::atomic<uint8_t> g_calNeoPixelOverride{kCalNeoOff};

// Dual-core watchdog kick flags — std::atomic per MULTICORE_RULES.md
// volatile is NOT sufficient for cross-core visibility on ARM (no hardware barrier)
static std::atomic<bool> g_wdtCore0Alive{false};
static std::atomic<bool> g_wdtCore1Alive{false};
static bool g_watchdogEnabled = false;
static bool g_watchdogReboot = false;

// Sensor phase active — set by Core 0 when Core 1 enters sensor loop.
// Read only by Core 0 (cal hooks, print_sensor_status). Plain bool is correct —
// single-core write/read, no cross-core visibility needed.
static bool g_sensorPhaseActive = false;

// Calibration storage state
static bool g_calStorageInitialized = false;


// IMU device handle (static per LL Entry 1 — avoid large objects on stack)
static icm20948_t g_imu;

// ============================================================================
// Seqlock Read/Write API
// ============================================================================
// Per SEQLOCK_DESIGN.md: explicit __dmb() required because memory_order_release
// only orders the atomic store itself, not the non-atomic memcpy data.

static void seqlock_write(sensor_seqlock_t* sl, const shared_sensor_data_t* src) {
    uint32_t seq = sl->sequence.load(std::memory_order_relaxed);
    // Signal write-in-progress (odd)
    sl->sequence.store(seq + 1, std::memory_order_release);
    __dmb();  // Ensure odd counter visible before data writes
    memcpy(&sl->data, src, sizeof(shared_sensor_data_t));
    __dmb();  // Ensure all data writes complete before even counter
    sl->sequence.store(seq + 2, std::memory_order_release);
}

static bool seqlock_read(sensor_seqlock_t* sl, shared_sensor_data_t* dst) {
    for (uint32_t attempt = 0; attempt < kSeqlockMaxRetries; attempt++) {
        uint32_t seq1 = sl->sequence.load(std::memory_order_acquire);
        if ((seq1 & 1U) != 0U) {
            continue;  // Write in progress, retry
        }
        __dmb();  // Ensure counter read committed before data reads
        memcpy(dst, &sl->data, sizeof(shared_sensor_data_t));
        __dmb();  // Ensure all data loads complete before re-reading counter
        uint32_t seq2 = sl->sequence.load(std::memory_order_acquire);
        if (seq1 == seq2) {
            return true;  // Consistent snapshot
        }
    }
    return false;  // All retries collided — caller uses previous data
}

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
// MPU Stack Guard Setup (per-core, PMSAv8)
// ============================================================================
// Configures MPU region 0 as a no-access guard at the bottom of the stack.
// Each core has its own MPU — call from the core being protected.

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" uint32_t __StackBottom;     // Core 0 stack bottom (SCRATCH_Y, linker-defined)
// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" uint32_t __StackOneBottom;  // Core 1 stack bottom (SCRATCH_X, linker-defined)

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
// Core 1: Sensor Read Helpers
// ============================================================================
// Each helper writes into localData (passed by pointer). The seqlock_write()
// call stays in the sensor loop — NOT inside these helpers (council rule).

// IMU error path: increment fail counter, invalidate accel/gyro, attempt recovery.
static void core1_imu_error_recovery(uint32_t* imuConsecFail,
                                      shared_sensor_data_t* localData) {
    (*imuConsecFail)++;
    localData->accel_valid = false;
    localData->gyro_valid = false;
    localData->imu_error_count++;
    if (*imuConsecFail >= kCore1ConsecFailDevReset) {
        icm20948_init(&g_imu, kIcm20948AddrDefault);
        *imuConsecFail = 0;
    } else if (*imuConsecFail >= kCore1ConsecFailBusRecover
               && *imuConsecFail % kCore1ConsecFailBusRecover == 0) {
        i2c_bus_recover();
    }
}

// Apply calibration to raw IMU data and write calibrated values into localData.
static void core1_apply_imu_cal(const icm20948_data_t& imuData,
                                 shared_sensor_data_t* localData,
                                 calibration_store_t* localCal) {
    float ax = 0.0F;
    float ay = 0.0F;
    float az = 0.0F;
    float gx = 0.0F;
    float gy = 0.0F;
    float gz = 0.0F;
    calibration_apply_accel_with(localCal,
        imuData.accel.x, imuData.accel.y, imuData.accel.z,
        &ax, &ay, &az);
    calibration_apply_gyro_with(localCal,
        imuData.gyro.x, imuData.gyro.y, imuData.gyro.z,
        &gx, &gy, &gz);

    localData->accel_x = ax;
    localData->accel_y = ay;
    localData->accel_z = az;
    localData->gyro_x = gx;
    localData->gyro_y = gy;
    localData->gyro_z = gz;
    localData->imu_timestamp_us = time_us_32();
    localData->imu_read_count++;
    localData->accel_valid = true;
    localData->gyro_valid = true;

    if (imuData.mag_valid) {
        float mxCal = 0.0F;
        float myCal = 0.0F;
        float mzCal = 0.0F;
        calibration_apply_mag_with(localCal,
            imuData.mag.x, imuData.mag.y, imuData.mag.z,
            &mxCal, &myCal, &mzCal);
        localData->mag_x = mxCal;
        localData->mag_y = myCal;
        localData->mag_z = mzCal;
        localData->mag_raw_x = imuData.mag.x;
        localData->mag_raw_y = imuData.mag.y;
        localData->mag_raw_z = imuData.mag.z;
        localData->mag_valid = true;
        localData->mag_read_count++;
    }
}

static void core1_read_imu(shared_sensor_data_t* localData,
                            calibration_store_t* localCal,
                            uint32_t* imuConsecFail,
                            bool feedCal) {
    icm20948_data_t imuData;
    bool imuOk = icm20948_read(&g_imu, &imuData);
    if (!imuOk) {
        core1_imu_error_recovery(imuConsecFail, localData);
        return;
    }

    // Sanity-check raw accel magnitude. A working sensor in ANY orientation always
    // measures at least 3 m/s² (gravity floor at 72° tilt: 9.8×cos(72°)=3.0).
    // All-zeros = ICM-20948 silent reset to sleep state (LL Entry 29).
    const float rawAccelMag = sqrtf(
        imuData.accel.x * imuData.accel.x +
        imuData.accel.y * imuData.accel.y +
        imuData.accel.z * imuData.accel.z);
    if (rawAccelMag < kAccelMinHealthyMag) {
        core1_imu_error_recovery(imuConsecFail, localData);
        return;
    }

    *imuConsecFail = 0;

    // Feed calibration with RAW data (before cal apply) — Core 1 owns I2C,
    // so no bus contention. Core 0 must NOT do concurrent icm20948_read().
    if (feedCal) {
        cal_state_t calState = calibration_manager_get_state();
        if (calState == CAL_STATE_GYRO_SAMPLING) {
            calibration_feed_gyro(imuData.gyro.x, imuData.gyro.y,
                                  imuData.gyro.z, imuData.temperature_c);
        } else if (calState == CAL_STATE_ACCEL_LEVEL_SAMPLING) {
            calibration_feed_accel(imuData.accel.x, imuData.accel.y,
                                   imuData.accel.z, imuData.temperature_c);
        }
    }

    core1_apply_imu_cal(imuData, localData, localCal);
}

static void core1_read_baro(shared_sensor_data_t* localData) {
    static uint32_t baroConsecFail = 0;
    static uint32_t baroReinitAttempts = 0;

    // Read directly via ruuvi driver — no MEAS_CFG pre-check.
    // The DPS310 in continuous mode alternates PRS_RDY and TMP_RDY;
    // requiring both simultaneously is too strict and produces false errors.
    // The ruuvi driver reads raw registers; I2C failure returns !valid.
    baro_dps310_data_t baroData;
    if (baro_dps310_read(&baroData) && baroData.valid) {
        localData->pressure_pa = baroData.pressure_pa;
        localData->baro_temperature_c = baroData.temperature_c;
        localData->baro_timestamp_us = time_us_32();
        localData->baro_read_count++;
        localData->baro_valid = true;
        baroConsecFail = 0;
        baroReinitAttempts = 0;  // Successful read proves sensor is alive

        // Feed baro calibration with raw data from Core 1
        if (calibration_manager_get_state() == CAL_STATE_BARO_SAMPLING) {
            calibration_feed_baro(baroData.pressure_pa,
                                  baroData.temperature_c);
        }
        return;
    }

    // I2C failure or driver error — escalate (mirrors IMU pattern)
    baroConsecFail++;
    localData->baro_error_count++;
    if (baroConsecFail >= kCore1ConsecFailDevReset) {
        if (baroReinitAttempts < kBaroMaxReinitAttempts) {
            baro_dps310_start_continuous();
            baroReinitAttempts++;
        } else {
            g_baroInitialized = false;  // Declare baro dead
        }
        baroConsecFail = 0;
    } else if (baroConsecFail >= kCore1ConsecFailBusRecover
               && baroConsecFail % kCore1ConsecFailBusRecover == 0) {
        i2c_bus_recover();
    }
}

// Update best-fix diagnostic when satellite count or HDOP improves.
static void core1_update_best_gps_fix(const shared_sensor_data_t* localData) {
    if (localData->gps_valid && localData->gps_fix_type >= 2) {
        bool better = !g_bestGpsValid.load(std::memory_order_relaxed)
                      || (localData->gps_satellites > g_bestGpsFix.satellites)
                      || (localData->gps_satellites == g_bestGpsFix.satellites &&
                          localData->gps_hdop > 0.0F &&
                          localData->gps_hdop < g_bestGpsFix.hdop);
        if (better) {
            g_bestGpsFix.lat_1e7     = localData->gps_lat_1e7;
            g_bestGpsFix.lon_1e7     = localData->gps_lon_1e7;
            g_bestGpsFix.alt_msl_m   = localData->gps_alt_msl_m;
            g_bestGpsFix.hdop        = localData->gps_hdop;
            g_bestGpsFix.satellites  = localData->gps_satellites;
            g_bestGpsFix.fix_type    = localData->gps_fix_type;
            g_bestGpsValid.store(true, std::memory_order_release);
        }
    }
}

static constexpr uint32_t kGpsStalenessTimeoutUs = 10000000;  // 10s
// Velocity threshold for "probably flying" heuristic (flight state gate).
// Prevents UART reinit mid-flight. Replaced by real state machine in IVP-67.
static constexpr float kGpsFlyingVelocityThreshold = 5.0F;  // m/s

// GPS UART staleness watchdog. Reinits UART if no valid parse for 10s,
// gated on flight state (don't reinit mid-flight). Blocks up to 2s.
static void core1_gps_staleness_check(bool parsed, uint32_t nowUs) {
    static uint32_t lastValidGpsUs = 0;

    if (parsed) {
        lastValidGpsUs = nowUs;
        return;
    }

    if (g_gpsTransport != GPS_TRANSPORT_UART || lastValidGpsUs == 0) {
        return;
    }
    if (nowUs - lastValidGpsUs <= kGpsStalenessTimeoutUs) {
        return;
    }

    // Flight state gate: proxy heuristic until real state machine in IVP-67.
    bool probablyFlying = g_eskfInitialized &&
                          g_eskf.v.norm() > kGpsFlyingVelocityThreshold;
    if (probablyFlying) {
        return;
    }

    if (!gps_uart_reinit()) {
        g_gpsInitialized = false;  // GPS dead — stop polling
    }
    lastValidGpsUs = time_us_32();  // Reset timer after attempt
}

static void core1_read_gps(shared_sensor_data_t* localData,
                            uint32_t* lastGpsReadUs) {
    uint32_t gpsNowUs = time_us_32();
    if (gpsNowUs - *lastGpsReadUs < kGpsMinIntervalUs) {
        return;
    }

    *lastGpsReadUs = gpsNowUs;

    // Transport-neutral poll via function pointers (set once in init_sensors)
    bool parsed = g_gpsFnUpdate();
    if (g_gpsTransport == GPS_TRANSPORT_I2C) {
        busy_wait_us(kGpsSdaSettleUs);  // SDA settling delay (LL Entry 24)
    }

    gps_data_t d;
    g_gpsFnGetData(&d);

    // gps_read_count always increments — counts driver polls, not valid fixes.
    localData->gps_read_count++;
    localData->gps_timestamp_us = gpsNowUs;

    // Hold-on-valid pattern (ArduPilot GPS backend `new_data` flag):
    // Between 1Hz NMEA bursts at 10Hz polling, 9/10 cycles have d.valid==false.
    // Only overwrite position/velocity fields when the driver reports valid data.
    // localData retains last-valid values between bursts.
    if (d.valid) {
        double lat = d.latitude;
        double lon = d.longitude;
        if (lat > kLatMaxDeg) { lat = kLatMaxDeg; }
        if (lat < kLatMinDeg) { lat = kLatMinDeg; }
        if (lon > kLonMaxDeg) { lon = kLonMaxDeg; }
        if (lon < kLonMinDeg) { lon = kLonMinDeg; }

        localData->gps_lat_1e7 = static_cast<int32_t>(lat * kGpsCoordScale);
        localData->gps_lon_1e7 = static_cast<int32_t>(lon * kGpsCoordScale);
        localData->gps_alt_msl_m = d.altitudeM;
        localData->gps_ground_speed_mps = d.speedMps;
        localData->gps_course_deg = d.courseDeg;
        localData->gps_valid = true;
        localData->gps_hdop = d.hdop;
        localData->gps_vdop = d.vdop;
    }

    // Always update diagnostic fields (satellites, fix type, raw sentence flags)
    localData->gps_fix_type = static_cast<uint8_t>(d.fix);
    localData->gps_satellites = d.satellites;
    localData->gps_gga_fix = d.ggaFix;
    localData->gps_gsa_fix_mode = d.gsaFixMode;
    localData->gps_rmc_valid = d.rmcValid;

    if (!parsed) {
        localData->gps_error_count++;
    }

    core1_gps_staleness_check(parsed, gpsNowUs);
    core1_update_best_gps_fix(localData);
}

// ============================================================================
// Core 1: NeoPixel State Update
// ============================================================================

// Helper: call ws2812_set_mode() only when mode or color changes.
// Calling ws2812_set_mode() every tick resets phaseStartMs, blinkState,
// and rainbowHue, which prevents breathe/blink/rainbow from animating.
static void neo_set_if_changed(ws2812_mode_t mode, ws2812_rgb_t color) {
    if (mode != g_lastNeoMode ||
        color.r != g_lastNeoColor.r ||
        color.g != g_lastNeoColor.g ||
        color.b != g_lastNeoColor.b) {
        g_lastNeoMode  = mode;
        g_lastNeoColor = color;
        ws2812_set_mode(mode, color);
    }
}

static void core1_neopixel_update(shared_sensor_data_t* localData,
                                   uint32_t nowMs, uint32_t sensorPhaseStartMs) {
    // INTERIM: Calibration NeoPixel override (Phase M.5).
    // When Core 0 CLI is running a calibration, it sets the override to
    // indicate the desired LED pattern. Replace with AP_Notify state machine.
    uint8_t calOverride = g_calNeoPixelOverride.load(std::memory_order_relaxed);
    if (calOverride != kCalNeoOff) {
        if (calOverride != g_lastCalNeoOverride) {
            g_lastCalNeoOverride = calOverride;
            switch (calOverride) {
                case kCalNeoGyro:        // fall through
                case kCalNeoLevel:       neo_set_if_changed(WS2812_MODE_BREATHE, kColorBlue); break;
                case kCalNeoBaro:        neo_set_if_changed(WS2812_MODE_BREATHE, kColorCyan); break;
                case kCalNeoAccelWait:   neo_set_if_changed(WS2812_MODE_BLINK, kColorYellow); break;
                case kCalNeoAccelSample: neo_set_if_changed(WS2812_MODE_SOLID, kColorYellow); break;
                case kCalNeoMag:         neo_set_if_changed(WS2812_MODE_RAINBOW, kColorWhite); break;
                case kCalNeoSuccess:     neo_set_if_changed(WS2812_MODE_SOLID, kColorGreen); break;
                case kCalNeoFail:        neo_set_if_changed(WS2812_MODE_BLINK_FAST, kColorRed); break;
                case kRxNeoReceiving:    neo_set_if_changed(WS2812_MODE_SOLID, kColorGreen); break;
                case kRxNeoGap:          neo_set_if_changed(WS2812_MODE_BLINK, kColorYellow); break;
                case kRxNeoLost:         neo_set_if_changed(WS2812_MODE_BLINK_FAST, kColorRed); break;
                default:                 break;
            }
        }
        ws2812_update();
        return;
    }
    g_lastCalNeoOverride = kCalNeoOff;

    // Normal status NeoPixel logic — use neo_set_if_changed() so animations
    // aren't reset every tick.
    if ((nowMs - sensorPhaseStartMs) >= kSensorPhaseTimeoutMs) {
        neo_set_if_changed(WS2812_MODE_SOLID, kColorMagenta);
    } else if (!g_eskfInitialized) {
        // ESKF waiting for stationary init — fast red blink ("hold still").
        neo_set_if_changed(WS2812_MODE_BLINK_FAST, kColorRed);
    } else if (g_gpsInitialized) {
        if (localData->gps_fix_type >= 3) {
            // 3D fix — solid green
            neo_set_if_changed(WS2812_MODE_SOLID, kColorGreen);
        } else if (localData->gps_fix_type == 2) {
            // 2D fix — blink green
            neo_set_if_changed(WS2812_MODE_BLINK, kColorGreen);
        } else if (localData->gps_read_count > 0) {
            // Searching (NMEA flowing, no fix) — blink yellow
            neo_set_if_changed(WS2812_MODE_BLINK, kColorYellow);
        } else {
            // GPS init but no NMEA yet — fast blink cyan
            neo_set_if_changed(WS2812_MODE_BLINK_FAST, kColorCyan);
        }
    } else if (g_sensorPhaseDone.load(std::memory_order_acquire)) {
        neo_set_if_changed(WS2812_MODE_SOLID, kColorGreen);
    } else {
        // ESKF running, no GPS — slow blink blue
        neo_set_if_changed(WS2812_MODE_BLINK, kColorBlue);
    }
    ws2812_update();
}

// ============================================================================
// Core 1: Sensor Loop
// ============================================================================
// INVARIANT: Core 0 must NOT call icm20948_*() or baro_dps310_*() unless
// g_core1I2CPaused == true. Core 1 owns I2C during this phase.
// Seqlock write stays here — read helpers write into localData by pointer.

// Check calibration reload request and I2C pause from Core 0. Returns true
// if the caller should skip the rest of the loop iteration (continue).
static bool core1_check_pause_and_reload(calibration_store_t* localCal) {
    if (g_calReloadPending.load(std::memory_order_acquire)) {
        calibration_load_into(localCal);
        g_calReloadPending.store(false, std::memory_order_release);
    }

    if (g_core1PauseI2C.load(std::memory_order_acquire)) {
        g_core1I2CPaused.store(true, std::memory_order_release);
        ws2812_set_mode(WS2812_MODE_SOLID, kColorOrange);
        ws2812_update();
        while (g_core1PauseI2C.load(std::memory_order_acquire)) {
            sleep_ms(1);
        }
        g_core1I2CPaused.store(false, std::memory_order_release);
        ws2812_set_mode(WS2812_MODE_SOLID, kColorBlue);
        ws2812_update();
        return true;
    }
    return false;
}

// Load calibration from flash, or set identity defaults if unavailable.
static void core1_load_cal_or_defaults(calibration_store_t* localCal) {
    if (!calibration_load_into(localCal)) {
        memset(localCal, 0, sizeof(*localCal));
        localCal->accel.scale.x = 1.0F;
        localCal->accel.scale.y = 1.0F;
        localCal->accel.scale.z = 1.0F;
        // NOLINTBEGIN(readability-magic-numbers) — 3x3 identity matrix diagonal indices
        localCal->board_rotation.m[0] = 1.0F;
        localCal->board_rotation.m[4] = 1.0F;
        localCal->board_rotation.m[8] = 1.0F;
        // NOLINTEND(readability-magic-numbers)
    }
}

static void core1_sensor_loop() {
    calibration_store_t localCal;
    core1_load_cal_or_defaults(&localCal);

    shared_sensor_data_t localData = {};
    uint32_t loopCount = 0;
    uint32_t baroCycle = 0;
    uint32_t calFeedCycle = 0;
    uint32_t imuConsecFail = 0;
    uint32_t gpsCycle = 0;
    uint32_t lastGpsReadUs = 0;
    uint32_t sensorPhaseStartMs = to_ms_since_boot(get_absolute_time());

    while (true) {
        uint32_t cycleStartUs = time_us_32();
        loopCount++;

        if (core1_check_pause_and_reload(&localCal)) {
            continue;
        }

        // Sensor reads — each writes into localData by pointer
        calFeedCycle++;
        bool feedCal = (calFeedCycle >= kCore1CalFeedDivider);
        if (feedCal) {
            calFeedCycle = 0;
        }
        if (g_imuInitialized) {
            core1_read_imu(&localData, &localCal, &imuConsecFail, feedCal);
        }

        baroCycle++;
        if (baroCycle >= kCore1BaroDivider && g_baroInitialized) {
            baroCycle = 0;
            core1_read_baro(&localData);
        }

        gpsCycle++;
        if (gpsCycle >= kCore1GpsDivider && g_gpsInitialized
            && !rc_os_mag_cal_active) {
            gpsCycle = 0;
            core1_read_gps(&localData, &lastGpsReadUs);
        }

        // Seqlock publish (always write, even on IMU failure — council mod #4)
        localData.core1_loop_count = loopCount;
        seqlock_write(&g_sensorSeqlock, &localData);

        g_wdtCore1Alive.store(true, std::memory_order_relaxed);

        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        core1_neopixel_update(&localData, nowMs, sensorPhaseStartMs);

        uint32_t elapsed = time_us_32() - cycleStartUs;
        if (elapsed < kCore1TargetCycleUs) {
            busy_wait_us(kCore1TargetCycleUs - elapsed);
        }
    }
}

// ============================================================================
// Core 1 Entry Point
// ============================================================================

static void core1_entry() {
    mpu_setup_stack_guard(reinterpret_cast<uint32_t>(&__StackOneBottom));

    // Wait for Core 0 to signal sensor phase start
    while (!g_startSensorPhase.load(std::memory_order_acquire)) {
        sleep_ms(1);
    }

    multicore_lockout_victim_init();
    g_core1LockoutReady.store(true, std::memory_order_release);
    core1_sensor_loop();
}

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
// INTERIM: NeoPixel override callback (Phase M.5)
// Replace with AP_Notify state machine when full status module is implemented.
// ============================================================================

static void set_cal_neo_override(uint8_t mode) {
    g_calNeoPixelOverride.store(mode, std::memory_order_relaxed);
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
        if (g_mahonyInitialized && g_mahony.healthy()) {
            rc::Vec3 meuler = g_mahony.q.to_euler();
            float mdivDeg = rc::MahonyAHRS::divergence_rad(g_eskf.q, g_mahony.q) * kRadToDeg;
            printf("Mahony: R=%6.2f P=%6.2f Y=%6.2f deg  Mdiv=%.1f deg\n",
                   (double)(meuler.x * kRadToDeg),
                   (double)(meuler.y * kRadToDeg),
                   (double)(meuler.z * kRadToDeg),
                   (double)mdivDeg);
        }
        if (g_eskfBenchCount > 0) {
            uint32_t avg = g_eskfBenchSum / g_eskfBenchCount;
            printf("      predict: %luus avg, %luus min, %luus max (%lu calls)\n",
                   (unsigned long)avg, (unsigned long)g_eskfBenchMin,
                   (unsigned long)g_eskfBenchMax, (unsigned long)g_eskfBenchCount);
            // BENCH: histogram display — uncomment with declarations above
            // printf("      hist(us): ");
            // for (uint32_t i = 0; i < kHistBuckets; i++) {
            //     if (i < kHistBuckets - 1) {
            //         printf("<%lu=%lu ", (unsigned long)((i + 1) * kHistStep),
            //                (unsigned long)g_eskfBenchHist[i]);
            //     } else {
            //         printf(">%lu=%lu", (unsigned long)(i * kHistStep),
            //                (unsigned long)g_eskfBenchHist[i]);
            //     }
            // }
            // printf("\n");
        }
        printf("      buf: %lu/%lu samples\n",
               (unsigned long)g_eskfBufferCount, (unsigned long)kEskfBufferSamples);
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
    if (g_loggingInitialized) {
        printf("  Log: %lu frames stored, %lu capacity\n",
               (unsigned long)rc::ring_stored_count(&g_ringBuffer),
               (unsigned long)rc::ring_capacity_frames(&g_ringBuffer));
    }
    if (g_flightTable.loaded) {
        printf("  Flash: %lu flights, %.1f%% used\n",
               (unsigned long)rc::flight_table_count(&g_flightTable),
               static_cast<double>(rc::flight_table_used_pct(&g_flightTable)));
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

    float mdivDeg = (g_mahonyInitialized && g_mahony.healthy())
                    ? rc::MahonyAHRS::divergence_rad(g_eskf.q, g_mahony.q) * kRadToDeg
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
            uint8_t accelDlpf = (accelCfg >> 3) & 0x07U;
            bool accelDlpfEn  = (accelCfg & 0x01U) != 0;
            uint8_t gyroDlpf  = (gyroCfg1 >> 3) & 0x07U;
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
    if (g_loggingInitialized) {
        bool isPsram = (g_psramSize > 0 && g_psramSelfTestPassed);
        uint32_t rate = isPsram ? (200 / kDecimationPsram) : (200 / kDecimationSram);
        printf("[PASS] Logging: %s ring, %luHz, %lu frames capacity, %lu stored\n",
               isPsram ? "PSRAM" : "SRAM",
               (unsigned long)rate,
               (unsigned long)rc::ring_capacity_frames(&g_ringBuffer),
               (unsigned long)rc::ring_stored_count(&g_ringBuffer));
    } else {
        printf("[----] Logging: not initialized\n");
    }
}

static void print_flash_status() {
    if (g_flightTable.loaded) {
        uint32_t nFlights = rc::flight_table_count(&g_flightTable);
        float usedPct = rc::flight_table_used_pct(&g_flightTable);
        uint32_t freeSectors = rc::flight_table_capacity_sectors() -
                               rc::flight_table_used_sectors(&g_flightTable);
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

    if (g_gpsSess.gps_updates > 0) {
        printf("  GPS session: %lu updates, max_dist=%.1fm, last_dist=%.1fm\n",
               (unsigned long)g_gpsSess.gps_updates,
               (double)g_gpsSess.max_dist_from_origin_m,
               (double)g_gpsSess.last_dist_from_origin_m);
        printf("              last_pos N=%.1f E=%.1f m  gNIS=[%.2f, %.2f]\n",
               (double)g_gpsSess.last_pos_n_m,
               (double)g_gpsSess.last_pos_e_m,
               (double)g_gpsSess.min_gps_nis,
               (double)g_gpsSess.max_gps_nis);
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
        printf("[PASS] Radio: RFM95W LoRa 915 MHz SF7/%s 20dBm (CS=%d RST=%d IRQ=%d)\n",
               kTxBwLabels[g_txBwMode],
               rocketchip::pins::kRadioCs, rocketchip::pins::kRadioRst,
               rocketchip::pins::kRadioIrq);
        if constexpr (kRadioModeRx) {
            printf("[MODE] RX — %s output ('m' to toggle)\n",
                   g_telemService.mavlink_output ? "MAVLink" : "CSV");
        } else {
            printf("[MODE] TX %dHz CCSDS %dB%s\n",
                   static_cast<int>(g_telemService.rate_hz),
                   static_cast<int>(rc::ccsds::kNavPacketLen),
                   g_telemService.mavlink_output ? " + USB MAVLink" : "");
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

    // Telemetry service init (IVP-59/60/61)
    // Radio mode is compile-time: kRadioModeRx selects RX (ground station),
    // default is TX (flight downlink). Will be set by Mission Profile.
    if (g_radioInitialized) {
        rc::telemetry_service_init(&g_telemService, &g_radio, 2);
        if constexpr (kRadioModeRx) {
            rc::telemetry_service_start_rx(&g_telemService);
        }
    }

    // MAVLink encoder init — needed for both RX relay and vehicle direct output.
    // Init regardless of radio — direct USB MAVLink doesn't need LoRa.
    g_telemService.mav_encoder.init();
    g_telemService.mavlink_output = kDefaultMavlinkOutput;

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
    printf("  RocketChip v%s\n", kVersionString);
    printf("  Board: %s\n", board::kBoardName);
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

// Forward declaration for CLI key handler (defined after logging_tick)
static void handle_unhandled_key(int key);

static void init_rc_os_hooks() {
    rc_os_init();
    rc_os_imu_available = g_imuInitialized;
    rc_os_baro_available = g_baroContinuous;
    rc_os_print_sensor_status = print_sensor_status;
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
}

static void init_logging_ring() {
    // Initialize logging ring buffer (IVP-52c).
    // Uses PSRAM if available (8MB at 50Hz = ~48 min), SRAM fallback otherwise
    // (200KB at 25Hz = ~145 sec). Ring buffer init writes header to memory.
    uint8_t* ringMem = nullptr;
    uint32_t ringSize = 0;
    uint32_t decRatio = kDecimationSram;

    if (g_psramSize > 0 && g_psramSelfTestPassed) {
        ringMem = rc::psram_base_ptr();
        ringSize = static_cast<uint32_t>(g_psramSize);
        decRatio = kDecimationPsram;
    } else {
        ringMem = g_sramRingBuf;
        ringSize = kSramRingSize;
        decRatio = kDecimationSram;
    }

    if (ringMem != nullptr) {
        g_loggingInitialized =
            rc::ring_init(&g_ringBuffer, ringMem, ringSize,
                          rc::kPcmFrameStandardSize, kHeaderSyncDiv);
        if (g_loggingInitialized) {
            rc::decimator_init(&g_decimator, decRatio);
        }
    }
}

static void init_application(bool watchdogReboot) {
    g_watchdogReboot = watchdogReboot;
    init_rc_os_hooks();

    // Signal Core 1 to start sensor phase
    g_sensorPhaseActive = true;
    g_startSensorPhase.store(true, std::memory_order_release);
    rc_os_i2c_scan_allowed = false;  // LL Entry 23: prevent CLI I2C scan from corrupting bus

    // Wait for Core 1 to call multicore_lockout_victim_init() — required before
    // any flash_safe_execute() call. Deterministic flag instead of sleep_ms().
    while (!g_core1LockoutReady.load(std::memory_order_acquire)) {
        sleep_ms(1);
    }

    // PSRAM flash-safe test (deferred from init_hardware).
    // Core 1 has called multicore_lockout_victim_init(),
    // so flash_safe_execute() is safe to use.
    if (g_psramSize > 0 && g_psramSelfTestPassed) {
        g_psramFlashSafePassed = rc::psram_flash_safe_test();
    }

    init_logging_ring();

    // Load flight table from flash (IVP-53b).
    g_flightTableLoaded = rc::flight_table_load(&g_flightTable);

    // Auto-calibrate baro ground reference at boot.
    if (g_baroContinuous) {
        calibration_start_baro();
    }

    // Enable watchdog (council critical fix: must be unconditional).
    // Write sentinel to scratch[0] so next boot can distinguish a genuine
    // watchdog timeout from SWD/picotool resets (see kWatchdogSentinel comment).
    g_watchdogEnabled = true;
    watchdog_hw->scratch[0] = kWatchdogSentinel;
    watchdog_enable(kWatchdogTimeoutMs, true);
}

// ============================================================================
// Main Loop Tick Functions
// ============================================================================
// Each tick function manages one subsystem. nowMs is computed once per loop
// iteration and passed to all ticks to prevent temporal skew.

// Council recommendation: track which tick was running when watchdog fires.
static const char* g_lastTickFunction = "init";

static void heartbeat_tick(uint32_t nowMs) {
    static bool g_ledState = false;
    uint32_t phase = nowMs % kHeartbeatPeriodMs;
    bool shouldBeOn = (phase < kHeartbeatOnMs);
    if (shouldBeOn != g_ledState) {
        g_ledState = shouldBeOn;
        board::board_led_set(g_ledState);
    }
}

static void watchdog_kick_tick() {
    if (!g_watchdogEnabled) {
        return;
    }

    // IVP-66: Update scratch registers with current state before kicking.
    // If the next kick is missed, recovery data is fresh.
    rc::watchdog_recovery_update_scratch(&g_recovery);

    g_wdtCore0Alive.store(true, std::memory_order_relaxed);
    if (g_wdtCore0Alive.load(std::memory_order_relaxed) &&
        g_wdtCore1Alive.load(std::memory_order_relaxed)) {
        watchdog_update();
        g_wdtCore0Alive.store(false, std::memory_order_relaxed);
        g_wdtCore1Alive.store(false, std::memory_order_relaxed);
    }
}

static void cli_update_tick() {
    // In MAVLink output mode, suppress all CLI text (banner, status, help)
    // to avoid corrupting the binary stream. Only 'm' key accepted.
    // Drain ALL pending USB input — GCS sends heartbeats, param requests,
    // etc. that must be consumed or the USB CDC TX buffer backs up and
    // the host's serial writes time out.
    if (g_telemService.mavlink_output) {
        if (stdio_usb_connected()) {
            int ch;
            while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
                if (ch == 'm' || ch == 'M') {
                    g_telemService.mavlink_output = false;
                    printf("\n[%s] Output: CLI\n",
                           kRadioModeRx ? "RX" : "USB");
                    return;
                }
            }
        }
        return;
    }
    rc_os_update();
    feed_active_calibration();
}

// INTERIM: Adafruit ICM-20948 breakout has sensor Z-up convention.
// ESKF expects NED (Z-down). Negate Z for accel, gyro, and mag at
// the ESKF feed boundary. Proper fix: set board_rotation matrix with
// Z-negate (needs level cal redo and council review for axis mapping).
// X→X, Y→Y, Z→-Z preserves right-handedness for rotation about Z.
static rc::Vec3 sensor_to_ned_accel(const shared_sensor_data_t& snap) {
    return rc::Vec3(snap.accel_x, snap.accel_y, -snap.accel_z);
}
static rc::Vec3 sensor_to_ned_gyro(const shared_sensor_data_t& snap) {
    return rc::Vec3(snap.gyro_x, snap.gyro_y, -snap.gyro_z);
}
static rc::Vec3 sensor_to_ned_mag(const shared_sensor_data_t& snap) {
    return rc::Vec3(snap.mag_x, snap.mag_y, -snap.mag_z);
}

// IVP-42d: ESKF init from first stable accel/gyro reading.
// IVP-44: If mag is available, set initial yaw from tilt-compensated heading.
// Returns true if init succeeded.
static bool eskf_try_init(const shared_sensor_data_t& snap) {
    rc::Vec3 accel = sensor_to_ned_accel(snap);
    rc::Vec3 gyro = sensor_to_ned_gyro(snap);

    if (!g_eskf.init(accel, gyro)) {
        return false;  // Stationarity check failed — try again next tick
    }

    // IVP-44: Set initial yaw from magnetometer heading if available.
    // Without this, yaw starts at 0° and the mag gate rejects updates
    // if actual heading is far from 0° (innovation >> 3σ).
    // Same approach as ArduPilot EKF3 InitialiseFilterBootstrap().
    if (snap.mag_valid) {
        rc::Vec3 mag = sensor_to_ned_mag(snap);
        // Tilt-compensate using roll/pitch from current quaternion (yaw=0)
        rc::Vec3 euler = g_eskf.q.to_euler();
        rc::Quat qZeroYaw = rc::Quat::from_euler(euler.x, euler.y, 0.0F);
        rc::Vec3 magLevel = qZeroYaw.rotate(mag);
        float initialYaw = -atan2f(magLevel.y, magLevel.x);
        // Rebuild quaternion with mag-derived yaw
        g_eskf.q = rc::Quat::from_euler(euler.x, euler.y, initialYaw);
        g_eskf.q.normalize();
    }

    g_eskfInitialized = true;
    g_eskf.reset_p_growth_baseline();
    g_lastEskfTimestampUs = snap.imu_timestamp_us;  // CR-2: set for first predict dt
    return true;
}

// IVP-42d: ESKF predict step with benchmark + state buffer write.
static void eskf_run_predict(const shared_sensor_data_t& snap) {
    // Compute dt from IMU timestamps (unsigned subtraction handles 32-bit wrap)
    uint32_t dtUs = snap.imu_timestamp_us - g_lastEskfTimestampUs;
    g_lastEskfTimestampUs = snap.imu_timestamp_us;  // CR-3: always update timestamp

    // Sanity-check dt — reject if too fast or too slow
    if (dtUs < kEskfMinDtUs || dtUs > kEskfMaxDtUs) {
        return;
    }

    float dt = static_cast<float>(dtUs) * kUsToSec;

    rc::Vec3 accel = sensor_to_ned_accel(snap);
    rc::Vec3 gyro = sensor_to_ned_gyro(snap);

    // Benchmark: wall-clock time for predict()
    uint32_t t0 = time_us_32();
    g_eskf.predict(accel, gyro, dt);
    uint32_t elapsed = time_us_32() - t0;

    // CR-1: stop propagation if filter diverges
    if (!g_eskf.healthy()) {
        g_eskfInitialized = false;
        rc::watchdog_recovery_eskf_failed(&g_recovery);  // IVP-66: backoff
        return;
    }

    // Update benchmark stats
    if (elapsed < g_eskfBenchMin) { g_eskfBenchMin = elapsed; }
    if (elapsed > g_eskfBenchMax) { g_eskfBenchMax = elapsed; }
    g_eskfBenchSum += elapsed;
    g_eskfBenchCount++;
    // BENCH: histogram recording — uncomment with declarations above
    // uint32_t bucket = elapsed / kHistStep;
    // if (bucket >= kHistBuckets) { bucket = kHistBuckets - 1; }
    // g_eskfBenchHist[bucket]++;

    // Write compact state to circular buffer
    eskf_state_snap_t& s = g_eskfBuffer[g_eskfBufferIndex];
    s.timestamp_us = snap.imu_timestamp_us;
    s.qw = g_eskf.q.w;  s.qx = g_eskf.q.x;
    s.qy = g_eskf.q.y;  s.qz = g_eskf.q.z;
    s.px = g_eskf.p.x;  s.py = g_eskf.p.y;  s.pz = g_eskf.p.z;
    s.vx = g_eskf.v.x;  s.vy = g_eskf.v.y;  s.vz = g_eskf.v.z;
    s.abx = g_eskf.accel_bias.x;  s.aby = g_eskf.accel_bias.y;
    s.abz = g_eskf.accel_bias.z;
    s.gbx = g_eskf.gyro_bias.x;   s.gby = g_eskf.gyro_bias.y;
    s.gbz = g_eskf.gyro_bias.z;
    g_eskfBufferIndex = (g_eskfBufferIndex + 1) % kEskfBufferSamples;
    if (g_eskfBufferCount < kEskfBufferSamples) {
        g_eskfBufferCount++;
    }
}

// IVP-43: Baro altitude measurement update (~32Hz DPS310 rate, on new data)
static void eskf_tick_baro(const shared_sensor_data_t& snap) {
    if (snap.baro_valid && g_baroContinuous) {
        static uint32_t g_lastEskfBaroCount = 0;
        if (snap.baro_read_count != g_lastEskfBaroCount) {
            g_lastEskfBaroCount = snap.baro_read_count;
            float alt = calibration_get_altitude_agl(snap.pressure_pa);
            g_eskf.update_baro(alt);
        }
    }
}

// IVP-44: Mag heading measurement update (~10Hz from AK09916 via seqlock)
static void eskf_tick_mag(const shared_sensor_data_t& snap) {
    if (snap.mag_valid) {
        static uint32_t g_lastEskfMagCount = 0;
        if (snap.mag_read_count != g_lastEskfMagCount) {
            g_lastEskfMagCount = snap.mag_read_count;
            rc::Vec3 magBody = sensor_to_ned_mag(snap);
            // Get expected magnitude from calibration for interference detection.
            // If mag not calibrated (expected_radius == 0), skip interference check.
            const calibration_store_t* cal = calibration_manager_get();
            float expectedMag = ((cal->cal_flags & CAL_STATUS_MAG) != 0)
                                ? cal->mag.expected_radius : 0.0F;
            // WMM declination: use GPS position if available, else 0 (magnetic heading).
            float declinationRad = 0.0F;
            if (snap.gps_valid && snap.gps_fix_type >= 2) {
                float latDeg = static_cast<float>(snap.gps_lat_1e7) * kGps1e7ToDegreesF;
                float lonDeg = static_cast<float>(snap.gps_lon_1e7) * kGps1e7ToDegreesF;
                declinationRad = rc::wmm_get_declination(latDeg, lonDeg);
            }
            g_eskf.update_mag_heading(magBody, expectedMag, declinationRad);
        }
    }
}

// Accumulate GPS session stats for post-session review via 's'.
static void eskf_tick_gps_stats() {
    g_gpsSess.gps_updates++;
    float dist = sqrtf(g_eskf.p.x * g_eskf.p.x +
                       g_eskf.p.y * g_eskf.p.y);
    if (dist > g_gpsSess.max_dist_from_origin_m) {
        g_gpsSess.max_dist_from_origin_m = dist;
    }
    g_gpsSess.last_pos_n_m = g_eskf.p.x;
    g_gpsSess.last_pos_e_m = g_eskf.p.y;
    g_gpsSess.last_dist_from_origin_m = dist;
    float nis = g_eskf.last_gps_pos_nis_;
    if (nis < g_gpsSess.min_gps_nis) { g_gpsSess.min_gps_nis = nis; }
    if (nis > g_gpsSess.max_gps_nis) { g_gpsSess.max_gps_nis = nis; }
}

// IVP-46: GPS position + velocity measurement update.
// Gated on 3D fix with new data. On first quality fix, sets NED origin
// and resets p/v to zero. Subsequent fixes convert geodetic to NED and
// inject position + velocity. Velocity gated on speed >= 0.5 m/s to
// suppress noisy updates at rest (known limitation until IVP-67 state machine).
static void eskf_tick_gps(const shared_sensor_data_t& snap) {
    if (snap.gps_valid && snap.gps_fix_type >= 3) {
        static uint32_t g_lastGpsCount = 0;
        if (snap.gps_read_count != g_lastGpsCount) {
            g_lastGpsCount = snap.gps_read_count;

            double latRad = static_cast<double>(snap.gps_lat_1e7) * kGps1e7ToDegrees * kDeg2Rad;
            double lonRad = static_cast<double>(snap.gps_lon_1e7) * kGps1e7ToDegrees * kDeg2Rad;
            float altM = snap.gps_alt_msl_m;
            float hdop = snap.gps_hdop;

            // First 3D fix: set origin + reset p/v
            if (!g_eskf.has_origin_) {
                if (g_eskf.set_origin(latRad, lonRad, altM, hdop)) {
                    g_eskf.p = rc::Vec3();
                    g_eskf.v = rc::Vec3();
                }
            } else {
                // Re-center origin if position drifts > 10km (HAB flights)
                float distXy = sqrtf(g_eskf.p.x * g_eskf.p.x +
                                     g_eskf.p.y * g_eskf.p.y);
                if (distXy > rc::ESKF::kOriginResetDistance) {
                    g_eskf.reset_origin(latRad, lonRad, altM);
                }

                // GPS position update (3 sequential scalar updates N/E/D)
                rc::Vec3 gpsNed = g_eskf.geodetic_to_ned(latRad, lonRad, altM);
                g_eskf.update_gps_position(gpsNed, hdop);

                // GPS velocity update — only when moving (>= 0.5 m/s)
                if (snap.gps_ground_speed_mps >= rc::ESKF::kGpsMinSpeedForVel) {
                    float courseRad = snap.gps_course_deg * static_cast<float>(kDeg2Rad);
                    float vNorth = snap.gps_ground_speed_mps * cosf(courseRad);
                    float vEast  = snap.gps_ground_speed_mps * sinf(courseRad);
                    g_eskf.update_gps_velocity(vNorth, vEast);
                }

                eskf_tick_gps_stats();
            }
        }
    }
}

// IVP-45: Mahony AHRS cross-check — independent attitude estimator.
// Runs at same 200Hz tick as ESKF. Uses its own dt tracking so it
// remains independent of the ESKF timestamp variable.
static void eskf_tick_mahony(const shared_sensor_data_t& snap) {
    rc::Vec3 accel = sensor_to_ned_accel(snap);
    rc::Vec3 gyro  = sensor_to_ned_gyro(snap);

    if (!g_mahonyInitialized) {
        rc::Vec3 magBody = snap.mag_valid ? sensor_to_ned_mag(snap) : rc::Vec3();
        if (g_mahony.init(accel, magBody)) {
            g_mahonyInitialized = true;
            g_lastMahonyTimestampUs = snap.imu_timestamp_us;
        }
    } else {
        uint32_t dtUs = snap.imu_timestamp_us - g_lastMahonyTimestampUs;
        g_lastMahonyTimestampUs = snap.imu_timestamp_us;
        if (dtUs >= kEskfMinDtUs && dtUs <= kEskfMaxDtUs) {
            float dt = static_cast<float>(dtUs) * kUsToSec;
            rc::Vec3 magBody = snap.mag_valid ? sensor_to_ned_mag(snap) : rc::Vec3();
            const calibration_store_t* cal = calibration_manager_get();
            float expectedMag = ((cal->cal_flags & CAL_STATUS_MAG) != 0)
                                ? cal->mag.expected_radius : 0.0F;
            bool magCalValid = (cal->cal_flags & CAL_STATUS_MAG) != 0;
            g_mahony.update(accel, gyro, magBody, expectedMag, magCalValid, dt);
            if (!g_mahony.healthy()) {
                g_mahonyInitialized = false;
            }
        }
    }
}

// IVP-42d: ESKF tick — runs at 200Hz via IMU count divider.
static void eskf_tick() {
    if (!g_sensorPhaseActive) {
        return;
    }

    // IVP-66: ESKF failure backoff — skip if disabled after too many failures
    if (g_recovery.eskf_disabled) {
        return;
    }

    // CR-4: seqlock failure is a separate early return from data validity
    shared_sensor_data_t snap = {};
    if (!seqlock_read(&g_sensorSeqlock, &snap)) {
        return;  // Seqlock contention — skip this cycle
    }

    if (!snap.accel_valid || !snap.gyro_valid) {
        return;
    }

    // Run every kEskfImuDivider-th new IMU sample (200Hz at 1kHz IMU rate)
    uint32_t newSamples = snap.imu_read_count - g_lastEskfImuCount;
    if (newSamples < kEskfImuDivider) {
        return;
    }
    g_lastEskfImuCount = snap.imu_read_count;

    if (!g_eskfInitialized) {
        eskf_try_init(snap);
        return;
    }

    eskf_run_predict(snap);

    // P-growth check: catch slow divergence before velocity hits 500 m/s
    if (!g_eskf.check_p_growth(time_us_32())) {
        g_eskfInitialized = false;  // CR-1 reset
        rc::watchdog_recovery_eskf_failed(&g_recovery);  // IVP-66: backoff
        return;
    }

    eskf_tick_baro(snap);
    eskf_tick_mag(snap);

    // IVP-44b: Zero-velocity pseudo-measurement (ZUPT).
    // Runs at predict rate (200Hz). Stationarity check is cheap — the ESKF
    // internally checks accel magnitude ≈ g and gyro < threshold.
    // When stationary, injects v=[0,0,0] to prevent horizontal velocity
    // divergence that occurs without GPS or velocity aiding.
    {
        rc::Vec3 accel = sensor_to_ned_accel(snap);
        rc::Vec3 gyro = sensor_to_ned_gyro(snap);
        g_eskf.update_zupt(accel, gyro);
    }

    eskf_tick_gps(snap);
    eskf_tick_mahony(snap);

    g_eskfEpoch++;
}

// ============================================================================
// Logging Tick (IVP-52c)
// ============================================================================
// Flash Flush + Erase (IVP-53b)
// ============================================================================

// Watchdog kick callback for flash operations.
// During multi-sector flash writes, the main loop is blocked.
// The watchdog (5s) must be kicked between sectors (~45ms each).
static void flush_kick_watchdog() {
    watchdog_update();
}

static void cmd_flush_log() {
    if (!g_loggingInitialized) {
        printf("Logging not initialized.\n");
        return;
    }
    if (!g_flightTable.loaded) {
        printf("Flight table not loaded.\n");
        return;
    }

    uint32_t stored = rc::ring_stored_count(&g_ringBuffer);
    if (stored == 0) {
        printf("No frames in ring buffer.\n");
        return;
    }

    bool isPsram = (g_psramSize > 0 && g_psramSelfTestPassed);
    uint8_t rate = isPsram
        ? static_cast<uint8_t>(200U / kDecimationPsram)
        : static_cast<uint8_t>(200U / kDecimationSram);

    printf("Flushing %lu frames to flash...\n", (unsigned long)stored);

    // Build summary from current state (minimal — no flight state machine yet)
    rc::FlightMetadata meta = {};
    rc::FlightSummary summ = {};
    summ.frame_count = stored;

    rc::FlushResult result = rc::flush_ring_to_flash(
        &g_ringBuffer, &g_flightTable, &meta, &summ, rate, flush_kick_watchdog);

    switch (result) {
    case rc::FlushResult::kOk:
        printf("Flush OK. Flight #%lu saved (%lu frames, %lu sectors).\n",
               (unsigned long)rc::flight_table_count(&g_flightTable),
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
    if (!g_flightTable.loaded) {
        printf("Flight table not loaded.\n");
        return;
    }

    uint32_t count = rc::flight_table_count(&g_flightTable);
    if (count == 0) {
        printf("No flights to erase.\n");
        return;
    }

    printf("Erase ALL %lu flights? Type 'yes' + Enter to confirm: ",
           (unsigned long)count);

    // Read up to 8 chars with 10s timeout (blocking — acceptable for destructive op)
    char buf[8] = {};
    int pos = 0;
    while (pos < 7) {
        int ch = getchar_timeout_us(10000000);  // 10s timeout
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
    if (!rc::flight_log_erase_all(&g_flightTable, flush_kick_watchdog)) {
        printf("Flash erase error.\n");
        return;
    }

    // Erase flight table in flash
    if (!rc::flight_table_erase_flash()) {
        printf("Table erase error.\n");
        return;
    }

    // Reset in-memory table
    rc::flight_table_erase_all(&g_flightTable);
    g_flightTable.loaded = true;

    // Save fresh empty table
    rc::flight_table_save(&g_flightTable);

    printf("All flights erased.\n");
}

// ============================================================================
// IVP-54a: Flight list + binary download
// ============================================================================

static void cmd_list_flights() {
    if (!g_flightTable.loaded) {
        printf("Flight table not loaded.\n");
        return;
    }
    uint32_t count = rc::flight_table_count(&g_flightTable);
    if (count == 0) {
        printf("No flights stored.\n");
        return;
    }
    printf("\n  #  Frames  Rate  Sectors  Size(KB)\n");
    printf("  -- ------  ----  -------  --------\n");
    for (uint32_t i = 0; i < count; ++i) {
        rc::FlightLogEntry entry = {};
        if (rc::flight_table_get_entry(&g_flightTable, i, &entry)) {
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
           static_cast<double>(rc::flight_table_used_pct(&g_flightTable)),
           (unsigned long)count);
}

// Read multi-digit flight number from USB (blocks up to 3s)
static int read_flight_number() {
    int num = 0;
    bool gotDigit = false;
    for (int i = 0; i < 3; ++i) {  // max 2 digits + enter
        int c = getchar_timeout_us(3000000);
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

    uint32_t running_crc = 0xFFFFFFFFU;
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
        watchdog_update();
    }

    stdio_set_translate_crlf(&stdio_usb, true);
    return running_crc ^ 0xFFFFFFFFU;
}

static void cmd_download_flight() {
    if (!g_flightTable.loaded) {
        printf("Flight table not loaded.\n");
        return;
    }
    uint32_t count = rc::flight_table_count(&g_flightTable);
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
    if (!rc::flight_table_get_entry(&g_flightTable, static_cast<uint32_t>(num - 1),
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
    if (!g_radioInitialized) {
        printf("Radio not initialized.\n");
        return;
    }
    if (g_telemService.mode == rc::RadioMode::kRx) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint32_t gap = (g_telemService.rx_count > 0) ?
            (now - g_telemService.last_rx_ms) : 0;
        printf("RX: %lu pkts, %ddBm, %d.%ddB SNR, %lu CRC err, last %lu.%lus ago\n",
               (unsigned long)g_telemService.rx_count,
               static_cast<int>(g_telemService.last_rx_rssi),
               static_cast<int>(g_telemService.last_rx_snr),
               0,
               (unsigned long)g_telemService.rx_crc_errors,
               (unsigned long)(gap / 1000),
               (unsigned long)((gap % 1000) / 100));
    } else {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint8_t duty = rc::telemetry_service_duty_pct(&g_telemService, now);
        printf("TX: %dHz CCSDS %dB %lums/pkt duty=%u%% seq=%lu drop=%lu\n",
               static_cast<int>(g_telemService.rate_hz),
               static_cast<int>(rc::ccsds::kNavPacketLen),
               (unsigned long)g_telemService.last_airtime_us / 1000,
               static_cast<unsigned>(duty),
               (unsigned long)g_telemService.tx_count,
               (unsigned long)g_telemService.tx_fail_count);
    }
}

static void handle_unhandled_key(int key) {
    switch (key) {
    case 'l': case 'L': cmd_flush_log(); break;
    case 'x': case 'X': cmd_erase_all_flights(); break;
    case 'f': case 'F': cmd_list_flights(); break;
    case 'd': case 'D': cmd_download_flight(); break;
    case 't': case 'T': cmd_radio_status(); break;
    case 'r':
        if (g_radioInitialized && !kRadioModeRx) {
            uint8_t newRate = rc::telemetry_service_cycle_rate(&g_telemService);
            printf("[TX] Rate changed to %dHz\n", static_cast<int>(newRate));
        }
        break;
    case 'm': case 'M':
        g_telemService.mavlink_output = !g_telemService.mavlink_output;
        printf("[%s] Output: %s\n",
               kRadioModeRx ? "RX" : "USB",
               g_telemService.mavlink_output ? "MAVLink" : "CLI");
        break;
    default: break;
    }
}

// ============================================================================
// Logging Tick (IVP-52c)
// ============================================================================
// Runs at main loop rate (~1kHz), but only produces output when ESKF has
// advanced (200Hz). Decimator further reduces to 50Hz (PSRAM) or 25Hz (SRAM).

static uint32_t g_lastLogEpoch = 0;

static void fused_copy_eskf_state(rc::FusedState& fused) {
    fused.q_w = g_eskf.q.w;
    fused.q_x = g_eskf.q.x;
    fused.q_y = g_eskf.q.y;
    fused.q_z = g_eskf.q.z;

    fused.pos_n = g_eskf.p.x;
    fused.pos_e = g_eskf.p.y;
    fused.pos_d = g_eskf.p.z;

    fused.vel_n = g_eskf.v.x;
    fused.vel_e = g_eskf.v.y;
    fused.vel_d = g_eskf.v.z;

    fused.accel_bias_x = g_eskf.accel_bias.x;
    fused.accel_bias_y = g_eskf.accel_bias.y;
    fused.accel_bias_z = g_eskf.accel_bias.z;

    fused.gyro_bias_x = g_eskf.gyro_bias.x;
    fused.gyro_bias_y = g_eskf.gyro_bias.y;
    fused.gyro_bias_z = g_eskf.gyro_bias.z;

    // Covariance diagnostics (max diagonal in each block)
    float patt = g_eskf.P(0, 0);
    if (g_eskf.P(1, 1) > patt) { patt = g_eskf.P(1, 1); }
    if (g_eskf.P(2, 2) > patt) { patt = g_eskf.P(2, 2); }
    fused.sig_att = patt;

    float ppos = g_eskf.P(3, 3);
    if (g_eskf.P(4, 4) > ppos) { ppos = g_eskf.P(4, 4); }
    if (g_eskf.P(5, 5) > ppos) { ppos = g_eskf.P(5, 5); }
    fused.sig_pos = ppos;

    float pvel = g_eskf.P(6, 6);
    if (g_eskf.P(7, 7) > pvel) { pvel = g_eskf.P(7, 7); }
    if (g_eskf.P(8, 8) > pvel) { pvel = g_eskf.P(8, 8); }
    fused.sig_vel = pvel;

    fused.eskf_healthy = g_eskf.healthy();
    fused.zupt_active = g_eskf.last_zupt_active_;
}

static void populate_fused_state(rc::FusedState& fused,
                                 const shared_sensor_data_t& snap) {
    fused_copy_eskf_state(fused);

    // Baro AGL and vertical velocity
    if (snap.baro_valid) {
        fused.baro_alt_agl = calibration_get_altitude_agl(snap.pressure_pa);
        fused.baro_vvel = g_eskf.v.z;  // NED down = positive descent
    }

    fused.baro_temperature_c = snap.baro_temperature_c;
    fused.imu_temperature_c = snap.imu_temperature_c;

    // Mahony divergence
    fused.mahony_div_deg = (g_mahonyInitialized && g_mahony.healthy())
        ? rc::MahonyAHRS::divergence_rad(g_eskf.q, g_mahony.q) * kRadToDeg
        : -1.0f;

    // GPS from sensor snapshot
    fused.gps_lat_1e7 = snap.gps_lat_1e7;
    fused.gps_lon_1e7 = snap.gps_lon_1e7;
    fused.gps_alt_msl_m = snap.gps_alt_msl_m;
    fused.gps_ground_speed_mps = snap.gps_ground_speed_mps;
    fused.gps_fix_type = snap.gps_fix_type;
    fused.gps_satellites = snap.gps_satellites;

    fused.flight_state = 0;  // IDLE until IVP-67
    fused.met_ms = to_ms_since_boot(get_absolute_time());
}

static void logging_tick() {
    if (!g_loggingInitialized || !g_eskfInitialized) {
        return;
    }

    // Only run when ESKF has produced a new propagation
    if (g_eskfEpoch == g_lastLogEpoch) {
        return;
    }
    g_lastLogEpoch = g_eskfEpoch;

    // Read sensor snapshot for GPS/baro/temp fields
    shared_sensor_data_t snap = {};
    if (!seqlock_read(&g_sensorSeqlock, &snap)) {
        return;  // Seqlock contention — skip this cycle
    }

    rc::FusedState fused = {};
    populate_fused_state(fused, snap);

    // Feed to decimator
    rc::FusedState averaged = {};
    if (!rc::decimator_push(&g_decimator, fused, averaged)) {
        return;  // Not enough samples yet
    }

    // Convert to wire format and push to ring buffer
    rc::TelemetryState telem = {};
    rc::fused_to_telemetry(averaged, telem);

    rc::PcmFrameStandard frame = {};
    rc::pcm_encode_standard(telem, averaged.met_ms, frame);

    rc::ring_push(&g_ringBuffer, &frame);

    // Stash for telemetry service (IVP-59)
    g_latestTelem = telem;
    g_latestTelemValid = true;
}

// ============================================================================
// Direct USB MAVLink Output (IVP-61)
// ============================================================================
// Vehicle direct mode: encode TelemetryState → MAVLink v2 frames → USB.
// Same as station relay but skips LoRa — like plugging an ArduPilot board
// directly into QGC/Mission Planner via USB.

static constexpr uint32_t kMavDirectIntervalMs = 100;  // 10 Hz output
static constexpr uint8_t  kMavFrameBufSize     = 64;
static uint32_t g_lastMavDirectMs = 0;

static void mavlink_direct_tick(uint32_t nowMs) {
    if (!g_telemService.mavlink_output) { return; }
    if constexpr (kRadioModeRx) { return; }  // Station uses RX path
    if (!g_latestTelemValid) { return; }
    if (!stdio_usb_connected()) { return; }

    // Rate limit — 10 Hz nav + 1 Hz heartbeat/sys_status
    uint8_t frame[kMavFrameBufSize];
    uint16_t len = 0;

    // 1 Hz heartbeat + SYS_STATUS
    if (nowMs - g_telemService.last_heartbeat_ms >= 1000) {
        g_telemService.last_heartbeat_ms = nowMs;
        len = g_telemService.mav_encoder.encode_heartbeat(
            g_latestTelem.flight_state, frame);
        fwrite(frame, 1, len, stdout);

        len = g_telemService.mav_encoder.encode_sys_status(
            g_latestTelem, frame);
        fwrite(frame, 1, len, stdout);
    }

    // 10 Hz ATTITUDE + GLOBAL_POSITION_INT
    if (nowMs - g_lastMavDirectMs < kMavDirectIntervalMs) { return; }
    g_lastMavDirectMs = nowMs;

    len = g_telemService.mav_encoder.encode_attitude(
        g_latestTelem, nowMs, frame);
    fwrite(frame, 1, len, stdout);

    len = g_telemService.mav_encoder.encode_global_pos(
        g_latestTelem, nowMs, frame);
    fwrite(frame, 1, len, stdout);

    fflush(stdout);
}

// ============================================================================
// Telemetry Radio Tick (IVP-59 TX, IVP-60 RX)
// ============================================================================

// RX NeoPixel gap thresholds
static constexpr uint32_t kRxGapWarningMs = 1000;   // Yellow blink after 1s
static constexpr uint32_t kRxGapLostMs    = 5000;   // Red blink after 5s

static void telemetry_radio_tick(uint32_t nowMs) {
    if (!g_radioInitialized) { return; }

    if (g_telemService.mode == rc::RadioMode::kTx) {
        if (!g_latestTelemValid) { return; }
        rc::telemetry_service_tick(&g_telemService, &g_latestTelem, nowMs);
        // Clear RX NeoPixel override when in TX mode
        uint8_t cur = g_calNeoPixelOverride.load(std::memory_order_relaxed);
        if (cur >= kRxNeoReceiving && cur <= kRxNeoLost) {
            g_calNeoPixelOverride.store(kCalNeoOff, std::memory_order_relaxed);
        }
    } else {
        rc::telemetry_service_rx_tick(&g_telemService, nowMs);

        // RX NeoPixel overlay — signal link quality to Core 1
        uint32_t gap = nowMs - g_telemService.last_rx_ms;
        uint8_t neoVal;
        if (g_telemService.rx_count == 0 || gap >= kRxGapLostMs) {
            neoVal = kRxNeoLost;
        } else if (gap >= kRxGapWarningMs) {
            neoVal = kRxNeoGap;
        } else {
            neoVal = kRxNeoReceiving;
        }
        g_calNeoPixelOverride.store(neoVal, std::memory_order_relaxed);
    }
}

// ============================================================================
// Main
// ============================================================================

int main() {
    bool watchdogReboot = init_hardware();
    init_application(watchdogReboot);

    while (true) {
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());

        g_lastTickFunction = "heartbeat";
        g_recovery.current_tick_fn = rc::TickFnId::kHeartbeat;
        heartbeat_tick(nowMs);

        g_lastTickFunction = "watchdog";
        g_recovery.current_tick_fn = rc::TickFnId::kWatchdog;
        watchdog_kick_tick();

        g_lastTickFunction = "eskf";
        g_recovery.current_tick_fn = rc::TickFnId::kEskf;
        eskf_tick();

        g_lastTickFunction = "logging";
        g_recovery.current_tick_fn = rc::TickFnId::kLogging;
        logging_tick();

        g_lastTickFunction = "radio";
        g_recovery.current_tick_fn = rc::TickFnId::kRadio;
        telemetry_radio_tick(nowMs);

        g_lastTickFunction = "mavlink";
        g_recovery.current_tick_fn = rc::TickFnId::kMavlink;
        mavlink_direct_tick(nowMs);

        g_lastTickFunction = "cli";
        g_recovery.current_tick_fn = rc::TickFnId::kCli;
        cli_update_tick();

        g_lastTickFunction = "sleep";
        g_recovery.current_tick_fn = rc::TickFnId::kSleep;
        sleep_ms(1);
    }

    return 0;
}
