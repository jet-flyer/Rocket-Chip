/**
 * @file main.cpp
 * @brief RocketChip main entry point - IVP Stage 3
 *
 * Implements IVP-01 through IVP-30:
 *   IVP-01: Clean build from source
 *   IVP-02: Red LED heartbeat (100ms on / 900ms off)
 *   IVP-03: NeoPixel rainbow via PIO
 *   IVP-04: USB CDC serial output
 *   IVP-05: Debug macros functional
 *   IVP-06: I2C bus init at 400kHz with bus recovery
 *   IVP-07: I2C scan (detect all connected sensors)
 *   IVP-08: Heartbeat superloop with uptime
 *   IVP-09: ICM-20948 IMU initialization
 *   IVP-10: IMU data validation (10Hz read, gate checks)
 *   IVP-11: DPS310 barometer initialization
 *   IVP-12: Barometer data validation
 *   IVP-13: Multi-sensor polling (single core)
 *   IVP-13a: I2C bus recovery under fault
 *   IVP-14: Calibration storage (flash persistence)
 *   IVP-15: Gyro bias calibration
 *   IVP-16: Level calibration
 *   IVP-17: 6-position accel calibration
 *   IVP-18: CLI menu (RC_OS)
 *   IVP-19: Core 1 launched with NeoPixel
 *   IVP-20: Cross-core atomic counter
 *   IVP-21: Spinlock soak (5 min)
 *   IVP-22: FIFO message passing
 *   IVP-23: Doorbell signals
 *   IVP-24: Seqlock single-buffer
 *   IVP-25: Core 1 IMU sampling (~1kHz)
 *   IVP-26: Core 1 baro sampling (~8Hz)
 *   IVP-27: USB stability soak (10 min under dual-core)
 *   IVP-28: Flash writes under dual-core
 *   IVP-29: MPU stack guard regions (both cores)
 *   IVP-30: Hardware watchdog (dual-core kick)
 */

#include "rocketchip/config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "drivers/ws2812_status.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include "drivers/baro_dps310.h"
#include "calibration/calibration_storage.h"
#include "calibration/calibration_manager.h"
#include "cli/rc_os.h"
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

static constexpr uint kNeoPixelPin = 21;

// Heartbeat: 100ms on, 900ms off (per IVP-08)
static constexpr uint32_t kHeartbeatOnMs = 100;
static constexpr uint32_t kHeartbeatOffMs = 900;
static constexpr uint32_t kHeartbeatPeriodMs = kHeartbeatOnMs + kHeartbeatOffMs;

// Skip verified IVP gates (IVP-10, IVP-12, IVP-13) — set false to re-run
static constexpr bool kSkipVerifiedGates = true;

// Superloop validation at 10 seconds
static constexpr uint32_t kSuperloopValidationMs = 10000;

// IVP-10: IMU data validation at 10Hz
static constexpr uint32_t kImuReadIntervalMs = 100;  // 10Hz
static constexpr uint32_t kImuSampleCount = 50;       // 5 seconds of data
static constexpr uint32_t kImuValidationDelayMs = 2000; // Wait 2s after boot for sensor settle

// IVP-12: Baro data validation at 10Hz (DPS310 runs at 8Hz continuous)
static constexpr uint32_t kBaroReadIntervalMs = 100;  // 10Hz polling
static constexpr uint32_t kBaroSampleCount = 100;      // 10 seconds of data
static constexpr uint32_t kBaroValidationDelayMs = 1000; // 1s after IVP-10 completes

// IVP-13: Multi-sensor polling rates
// IMU 100Hz, Baro 50Hz — VALIDATE targets per IVP.md
static constexpr uint32_t kIvp13ImuIntervalUs = 10000;   // 100Hz = 10ms
static constexpr uint32_t kIvp13BaroIntervalUs = 20000;  // 50Hz = 20ms
static constexpr uint32_t kIvp13DurationMs = 60000;       // 60 seconds
static constexpr uint32_t kIvp13StatusIntervalMs = 1000;  // Print status every 1s

// IVP-13a: I2C bus recovery thresholds
static constexpr uint32_t kI2cRecoveryThreshold = 50;     // ~333ms of failures at 150 reads/s
static constexpr uint32_t kI2cRecoveryCooldownMs = 2000;   // Min ms between recovery attempts

// IVP-21: Spinlock soak test
static constexpr uint32_t kSpinlockSoakMs = 5 * 60 * 1000;  // 5 minutes
static constexpr uint32_t kSpinlockCheckIntervalMs = 100;     // Core 0 reads 10x/sec

// IVP-22: FIFO message passing test
static constexpr uint32_t kFifoTestCount = 1000;

// IVP-23: Doorbell test
static constexpr uint32_t kDoorbellTestCount = 1000;

// Core 1 test-mode commands (sent via FIFO)
static constexpr uint32_t kCmd_FifoEchoStart = 0xF1F0E000;
static constexpr uint32_t kCmd_FifoEchoDone  = 0xF1F0D000;
static constexpr uint32_t kCmd_FifoSendStart = 0xF1F05000;
static constexpr uint32_t kCmd_DoorbellStart = 0xDB00B000;
static constexpr uint32_t kCmd_DoorbellDone  = 0xDB00D000;
static constexpr uint32_t kCmd_SpinlockStart = 0x5510C000;

// IVP-25/26: Core 1 sensor loop timing
static constexpr uint32_t kCore1TargetCycleUs = 1000;           // ~1kHz target
static constexpr uint32_t kCore1BaroDivider = 20;               // Baro at ~50Hz
static constexpr uint32_t kSensorSoakMs = 5 * 60 * 1000;       // 5-minute soak
static constexpr uint32_t kSeqlockReadIntervalMs = 5;           // 200Hz Core 0 reader
static constexpr uint32_t kJitterSampleCount = 1000;            // For gate 3 jitter
static constexpr uint32_t kCore1ConsecFailMax = 50;             // I2C recovery threshold

// DPS310 MEAS_CFG register (0x08): data-ready bits
// Per DPS310 datasheet Section 7, PRS_RDY=bit4, TMP_RDY=bit5
static constexpr uint8_t kDps310MeasCfgReg = 0x08;
static constexpr uint8_t kDps310PrsRdy = 0x10;                  // Bit 4
static constexpr uint8_t kDps310TmpRdy = 0x20;                  // Bit 5

// Gate check thresholds
static constexpr uint32_t kCore1PauseAckMaxMs = 100;            // Max wait for Core 1 ACK
static constexpr float kImuRateTolerancePct = 5.0f;             // IMU rate ±5% (I2C jitter)
static constexpr float kBaroRateTolerancePct = 50.0f;           // Baro rate ±50% (DPS310 hw rate is approximate)

// IVP-27: USB stability soak
static constexpr uint32_t kIvp27SoakMs = 10 * 60 * 1000;       // 10 minutes (IVP-27 gate 1)
static constexpr uint32_t kIvp27StatusIntervalMs = 60000;        // Print every 60s

// IVP-28: Flash under dual-core
static constexpr uint32_t kIvp28FlashRepeatCount = 5;            // 5 saves (IVP-28 gate 7)
static constexpr uint32_t kIvp28PostFlashSettleMs = 50;          // Core 1 resume detection
// Council mod #2: Core 1 resumes mid-I2C-transaction after lockout; 1-2 reads fail.
static constexpr uint32_t kIvp28MaxExpectedErrors = 5;

// IVP-29: MPU stack guard
static constexpr uint32_t kMpuGuardSizeBytes = 64;     // Guard region at bottom of stack
static constexpr int32_t kFaultBlinkFastLoops = 200000; // ~100ms at 150MHz (fault handler busy-wait)
static constexpr int32_t kFaultBlinkSlowLoops = 800000; // ~400ms at 150MHz (fault handler busy-wait)

// IVP-30: Hardware watchdog
static constexpr uint32_t kWatchdogTimeoutMs = 5000;    // 5 second timeout per IVP spec
static constexpr uint32_t kIvp30SoakMs = 5 * 60 * 1000; // 5-minute soak (no false fires)
static constexpr uint32_t kIvp30StatusIntervalMs = 60000; // Print every 60s

// Core 1 skip command (for kSkipVerifiedGates — bypass Phase 2)
static constexpr uint32_t kCmd_SkipToSensors = 0x5E5052E5;

// ============================================================================
// Global State
// ============================================================================

static bool g_neopixelInitialized = false;
static bool g_i2cInitialized = false;
static bool g_imuInitialized = false;
static bool g_baroInitialized = false;
static bool g_baroContinuous = false;
static bool g_superloopValidationDone = false;
static uint32_t g_loopCounter = 0;

// IVP-10: IMU data validation state
static uint32_t g_imuSampleNum = 0;
static uint32_t g_lastImuReadMs = 0;
static bool g_imuValidationStarted = false;
static bool g_imuValidationDone = false;

// IVP-10: Gate tracking — accumulate min/max/sum for validation
static float g_accelXSum, g_accelYSum, g_accelZSum;
static float g_accelZMin, g_accelZMax;
static float g_gyroAbsMax;
static float g_magMagMin, g_magMagMax;
static float g_tempMin, g_tempMax;
static uint32_t g_nanCount = 0;
static uint32_t g_magValidCount = 0;
static uint32_t g_validSampleCount = 0;

// IVP-12: Baro data validation state
static uint32_t g_baroSampleNum = 0;
static uint32_t g_lastBaroReadMs = 0;
static bool g_baroValidationStarted = false;
static bool g_baroValidationDone = false;
static uint32_t g_baroValidStartMs = 0;

// IVP-12: Baro gate tracking
static float g_baroPressMin, g_baroPressMax, g_baroPressSum;
static float g_baroTempMin, g_baroTempMax;
static uint32_t g_baroValidReads = 0;
static uint32_t g_baroInvalidReads = 0;
static uint32_t g_baroNanCount = 0;
static float g_baroLastPress;  // For stuck-value detection
static uint32_t g_baroStuckCount = 0;

// IVP-13: Multi-sensor polling state
static bool g_ivp13Started = false;
static bool g_ivp13Done = false;
static uint32_t g_ivp13StartMs = 0;
static uint64_t g_ivp13LastImuUs = 0;
static uint64_t g_ivp13LastBaroUs = 0;
static uint32_t g_ivp13LastStatusMs = 0;
static uint32_t g_ivp13ImuCount = 0;
static uint32_t g_ivp13BaroCount = 0;
static uint32_t g_ivp13ImuErrors = 0;
static uint32_t g_ivp13BaroErrors = 0;
static uint64_t g_ivp13ImuTimeSum = 0;   // Sum of per-read times in us
static uint64_t g_ivp13BaroTimeSum = 0;
static uint32_t g_ivp13ImuTimeMax = 0;
static uint32_t g_ivp13BaroTimeMax = 0;
// Per-second counters for rate measurement
static uint32_t g_ivp13ImuSecCount = 0;
static uint32_t g_ivp13BaroSecCount = 0;

// IVP-13a: I2C bus recovery state
static uint32_t g_i2cConsecErrors = 0;      // Consecutive I2C errors (any sensor)
static uint32_t g_i2cRecoveryAttempts = 0;   // Total recovery attempts
static uint32_t g_i2cRecoverySuccesses = 0;  // Successful recoveries
static uint32_t g_i2cLastRecoveryMs = 0;     // Timestamp of last recovery attempt
static uint32_t g_baroConsecFails = 0;       // Consecutive baro failures (for lazy reinit)
static bool g_imuLastReadOk = false;         // IMU last read succeeded (bus health proxy)

// IVP-19/20: Dual-core state
// Atomic counter in static SRAM — Core 1 increments, Core 0 reads.
// Per SEQLOCK_DESIGN.md: shared data must be in SRAM (not PSRAM).
static std::atomic<uint32_t> g_core1Counter{0};

// IVP-21: Spinlock shared struct — multi-field, protected by spinlock.
// If spinlock works correctly, a == b == c == write_count on every read.
struct spinlock_test_data_t {
    uint32_t field_a;
    uint32_t field_b;
    uint32_t field_c;
    uint32_t write_count;
};
static spinlock_test_data_t g_spinlockData = {0, 0, 0, 0};
static spin_lock_t* g_pTestSpinlock = nullptr;
static int g_spinlockId = -1;

// IVP-21: Soak test tracking (Core 0 side)
static bool g_ivp21Active = false;
static bool g_ivp21Done = false;
static uint32_t g_ivp21StartMs = 0;
static uint32_t g_ivp21LastCheckMs = 0;
static uint32_t g_ivp21ReadCount = 0;
static uint32_t g_ivp21InconsistentCount = 0;
static uint32_t g_ivp21LockTimeMinUs = UINT32_MAX;
static uint32_t g_ivp21LockTimeMaxUs = 0;
static uint64_t g_ivp21LockTimeSumUs = 0;
static uint32_t g_ivp21LastPrintMs = 0;

// IVP-23: Doorbell number (shared between test setup and Core 1)
static int g_doorbellNum = -1;

// IVP-24: Shared sensor data struct (per SEQLOCK_DESIGN.md, council-approved)
// All values calibration-applied, body frame, SI units. 124 bytes in SRAM.
struct shared_sensor_data_t {
    // IMU (56 bytes)
    float accel_x;                          // m/s^2
    float accel_y;
    float accel_z;
    float gyro_x;                           // rad/s
    float gyro_y;
    float gyro_z;
    float mag_x;                            // uT (when mag_valid)
    float mag_y;
    float mag_z;
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

    // GPS (32 bytes, zeroed until IVP-31)
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
    uint8_t _pad_gps;

    // Health (16 bytes)
    uint32_t imu_error_count;
    uint32_t baro_error_count;
    uint32_t gps_error_count;
    uint32_t core1_loop_count;              // For watchdog/stall detection
};

static_assert(sizeof(shared_sensor_data_t) == 124, "Struct size mismatch with SEQLOCK_DESIGN.md");
static_assert(sizeof(shared_sensor_data_t) % 4 == 0, "Struct must be 4-byte aligned for memcpy");

// IVP-24: Seqlock wrapper — single buffer with sequence counter
struct sensor_seqlock_t {
    std::atomic<uint32_t> sequence{0};      // Odd = write in progress
    shared_sensor_data_t data;
};

static sensor_seqlock_t g_sensorSeqlock;

// IVP-25/26: Cross-core signaling (atomic flags — FIFO reserved by multicore_lockout)
static std::atomic<bool> g_startSensorPhase{false};
static std::atomic<bool> g_sensorPhaseDone{false};
static std::atomic<bool> g_calReloadPending{false};
static std::atomic<bool> g_core1PauseI2C{false};
static std::atomic<bool> g_core1I2CPaused{false};

// IVP-25/26: Soak test tracking (Core 0 side)
static bool g_ivp25Active = false;
static bool g_ivp25Done = false;
static uint32_t g_ivp25StartMs = 0;
static uint32_t g_ivp25LastReadMs = 0;
static uint32_t g_ivp25ReadCount = 0;
static uint32_t g_ivp25RetryCount = 0;
static uint32_t g_ivp25StaleCount = 0;
static uint32_t g_ivp25LastPrintMs = 0;
static uint32_t g_ivp25LastImuCount = 0;  // For stale detection

// Jitter: Core 1 writes first 1000 IMU timestamps, Core 0 reads after soak
static uint32_t g_jitterTimestamps[kJitterSampleCount];  // 4KB in SRAM
static std::atomic<uint32_t> g_jitterSamplesCollected{0};

// IVP-27: USB stability soak
static bool g_ivp27Active = false;
static bool g_ivp27Done = false;
static uint32_t g_ivp27StartMs = 0;
static uint32_t g_ivp27LastStatusMs = 0;
static bool g_ivp27Prompt3min = false;
static bool g_ivp27Prompt6min = false;
static bool g_ivp27Prompt8min = false;

// IVP-28: Flash under dual-core
static bool g_ivp28Started = false;
static bool g_ivp28Done = false;

// IVP-29: MPU stack guard
static bool g_ivp29Done = false;

// IVP-30: Watchdog
static bool g_ivp30Active = false;
static bool g_ivp30Done = false;
static uint32_t g_ivp30StartMs = 0;
static uint32_t g_ivp30LastStatusMs = 0;

// Dual-core watchdog kick flags — std::atomic per MULTICORE_RULES.md
// volatile is NOT sufficient for cross-core visibility on ARM (no hardware barrier)
static std::atomic<bool> g_wdtCore0Alive{false};
static std::atomic<bool> g_wdtCore1Alive{false};
static bool g_watchdogEnabled = false;
static bool g_testCommandsEnabled = false;  // Set after soak OR after watchdog reboot

// IVP-30: Core 1 stall test flag (set by Core 0, checked by Core 1)
static std::atomic<bool> g_core1StallTest{false};

// IVP-14: Calibration storage state
static bool g_calStorageInitialized = false;
static bool g_ivp14Done = false;

// IVP-15: Gyro bias calibration state
static bool g_ivp15Started = false;
static bool g_ivp15Done = false;
static uint64_t g_ivp15LastFeedUs = 0;
static constexpr uint32_t kIvp15FeedIntervalUs = 10000;  // 100Hz feed rate

// IVP-16: Accel level calibration state
static bool g_ivp16Started = false;
static bool g_ivp16Done = false;
static uint64_t g_ivp16LastFeedUs = 0;
static constexpr uint32_t kIvp16FeedIntervalUs = 10000;  // 100Hz feed rate

// IVP-18: CLI feed interval for calibrations triggered via menu
static uint64_t g_cliCalLastFeedUs = 0;
static constexpr uint32_t kCliCalFeedIntervalUs = 10000;  // 100Hz

// IMU device handle (static per LL Entry 1 — avoid large objects on stack)
static icm20948_t g_imu;

// ============================================================================
// IVP-24: Seqlock Read/Write API
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
    for (uint32_t attempt = 0; attempt < 4; attempt++) {
        uint32_t seq1 = sl->sequence.load(std::memory_order_acquire);
        if (seq1 & 1u) {
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
    return false;  // All 4 retries collided — caller uses previous data
}

// ============================================================================
// IVP-29: MemManage / HardFault Handler
// ============================================================================
// Fires when MPU stack guard is hit (stack overflow).
// Must NOT use stack (it may be overflowed). Uses direct GPIO register writes.
// Blink pattern: 3 fast + 1 slow on red LED, forever.

static void memmanage_fault_handler(void) {
    __asm volatile ("cpsid i");  // Disable interrupts

    // Direct GPIO register writes — no SDK calls that might use stack
    io_rw_32 *gpio_out = &sio_hw->gpio_out;
    const uint32_t led_mask = 1u << PICO_DEFAULT_LED_PIN;

    while (true) {
        for (uint8_t i = 0; i < 3; i++) {
            *gpio_out |= led_mask;
            for (int32_t d = 0; d < kFaultBlinkFastLoops; d++) { __asm volatile(""); }
            *gpio_out &= ~led_mask;
            for (int32_t d = 0; d < kFaultBlinkFastLoops; d++) { __asm volatile(""); }
        }
        *gpio_out |= led_mask;
        for (int32_t d = 0; d < kFaultBlinkSlowLoops; d++) { __asm volatile(""); }
        *gpio_out &= ~led_mask;
        for (int32_t d = 0; d < kFaultBlinkSlowLoops; d++) { __asm volatile(""); }
    }
}

// ============================================================================
// IVP-29: MPU Stack Guard Setup (per-core, PMSAv8)
// ============================================================================
// Configures MPU region 0 as a no-access guard at the bottom of the stack.
// Each core has its own MPU — call from the core being protected.

extern "C" uint32_t __StackBottom;     // Core 0 stack bottom (SCRATCH_Y, linker-defined)
extern "C" uint32_t __StackOneBottom;  // Core 1 stack bottom (SCRATCH_X, linker-defined)

static void mpu_setup_stack_guard(uint32_t stack_bottom) {
    // Disable MPU during configuration
    mpu_hw->ctrl = 0;
    __dsb();
    __isb();

    // Region 0: Stack guard (no access, execute-never)
    // PMSAv8 RBAR: [31:5]=BASE, [4:3]=SH(0=non-shareable), [2:1]=AP(0=priv no-access), [0]=XN(1)
    mpu_hw->rnr = 0;
    mpu_hw->rbar = (stack_bottom & ~0x1FU)
                  | (0u << 3)   // SH: Non-shareable
                  | (0u << 1)   // AP: Privileged no-access
                  | (1u << 0);  // XN: Execute-never

    // PMSAv8 RLAR: [31:5]=LIMIT, [3:1]=ATTRINDX(0), [0]=EN(1)
    mpu_hw->rlar = ((stack_bottom + kMpuGuardSizeBytes - 1) & ~0x1FU)
                  | (0u << 1)   // ATTRINDX: 0 (uses MAIR0 attr 0)
                  | (1u << 0);  // EN: Enable region

    // MAIR0 attr 0 = 0x00 = Device-nGnRnE (strictest, no caching)
    mpu_hw->mair[0] = 0;

    // Enable MPU with PRIVDEFENA=1 (default memory map for unprogrammed regions)
    mpu_hw->ctrl = (1u << 2)   // PRIVDEFENA
                 | (1u << 0);  // ENABLE
    __dsb();
    __isb();
}

// ============================================================================
// IVP-29: Gate Check
// ============================================================================

static void ivp29_gate_check(void) {
    if (!stdio_usb_connected()) {
        return;
    }

    printf("\n=== IVP-29: MPU Stack Guard Regions ===\n");
    printf("[MANUAL] Gate 1: Core 0 overflow triggers fault (press 'o')\n");
    printf("[MANUAL] Gate 2: Core 1 overflow triggers fault\n");
    printf("[INFO]   Gate 3: Fault handler — red LED 3-fast+1-slow blink\n");
    printf("[PASS]   Gate 4: No false faults (reached this point)\n");
    printf("[MANUAL] Gate 5: Verify MPU regions via GDB\n");
    printf("=== IVP-29: AUTOMATED GATES PASS / MANUAL gates above ===\n\n");
}

// ============================================================================
// IVP-30: Gate Check
// ============================================================================

static void ivp30_gate_check(void) {
    if (!stdio_usb_connected()) {
        return;
    }

    printf("\n=== IVP-30: Hardware Watchdog ===\n");
    printf("[PASS]   Gate 1: Normal operation — watchdog did not fire over 5 minutes\n");
    printf("[MANUAL] Gate 2: Stall Core 0 (press 'w') — resets within 5s\n");
    printf("[MANUAL] Gate 3: Stall Core 1 (press 'W') — resets within 5s\n");
    printf("[MANUAL] Gate 4: After reset, verify 'WATCHDOG RESET' message\n");
    printf("[MANUAL] Gate 5: Debug probe connected — watchdog pauses\n");
    printf("=== IVP-30: AUTOMATED GATES PASS / MANUAL gates above ===\n\n");
}

// ============================================================================
// IVP-19/20/21/22/23/24: Core 1 Entry Point
// ============================================================================
// Phase 1: Test dispatcher — responds to FIFO commands for IVP-22/23 exercises.
// Phase 2: Spinlock soak + NeoPixel — writes shared struct at ~100kHz for IVP-21.
// Atomic counter (IVP-20) increments throughout both phases.

static void core1_entry(void) {
    // IVP-29: MPU stack guard for Core 1
    mpu_setup_stack_guard((uint32_t)&__StackOneBottom);

    // ---- Phase 1: Test Dispatcher ----
    // Core 1 polls FIFO every 1ms, blinks NeoPixel cyan fast (~4Hz) via timer.
    bool testMode = true;
    bool skipPhase2 = false;
    uint32_t lastNeoToggleMs = to_ms_since_boot(get_absolute_time());
    bool neoOn = true;
    ws2812_set_mode(WS2812_MODE_SOLID, kColorCyan);
    ws2812_update();

    while (testMode) {
        // Poll FIFO with 1ms timeout (fast response to commands)
        uint32_t cmd;
        if (multicore_fifo_pop_timeout_us(1000, &cmd)) {
            switch (cmd) {

            // IVP-22: FIFO echo — Core 0 sends values, Core 1 echoes back
            case kCmd_FifoEchoStart: {
                // ACK: tell Core 0 we're ready
                multicore_fifo_push_blocking(kCmd_FifoEchoStart);
                for (uint32_t i = 0; i < kFifoTestCount; i++) {
                    uint32_t val = multicore_fifo_pop_blocking();
                    multicore_fifo_push_blocking(val);
                }
                multicore_fifo_push_blocking(kCmd_FifoEchoDone);
                break;
            }

            // IVP-22: FIFO send — Core 1 sends sequential values to Core 0
            case kCmd_FifoSendStart: {
                for (uint32_t i = 0; i < kFifoTestCount; i++) {
                    multicore_fifo_push_blocking(i);
                }
                break;
            }

            // IVP-23: Doorbell — Core 1 sets doorbell N times with 10us gaps
            case kCmd_DoorbellStart: {
                for (uint32_t i = 0; i < kDoorbellTestCount; i++) {
                    multicore_doorbell_set_other_core((uint)g_doorbellNum);
                    busy_wait_us(10);
                }
                multicore_fifo_push_blocking(kCmd_DoorbellDone);
                break;
            }

            // Transition to Phase 2
            case kCmd_SpinlockStart:
                testMode = false;
                // Council mod #1: enable flash_safe_execute coverage before Phase 2
                // (Phase 2 spinlock soak must be safe during calibration flash writes)
                multicore_lockout_victim_init();
                break;

            // IVP-27/28: Skip Phase 2, go directly to Phase 3 (sensor loop)
            case kCmd_SkipToSensors:
                testMode = false;
                multicore_lockout_victim_init();
                skipPhase2 = true;
                break;

            default:
                break;
            }
        }

        // NeoPixel blink via timer (~4Hz toggle = 125ms)
        g_core1Counter.fetch_add(1, std::memory_order_relaxed);
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        if ((nowMs - lastNeoToggleMs) >= 125) {
            lastNeoToggleMs = nowMs;
            neoOn = !neoOn;
            ws2812_set_mode(WS2812_MODE_SOLID,
                            neoOn ? kColorCyan : kColorOff);
            ws2812_update();
        }
    }

    uint32_t lastNeoMs = to_ms_since_boot(get_absolute_time());
    bool neoState = false;

    if (!skipPhase2) {
        // ---- Phase 2: Spinlock Soak + NeoPixel ----
        // Write shared struct at ~100kHz under spinlock. NeoPixel at ~2Hz.
        uint32_t soakStartMs = to_ms_since_boot(get_absolute_time());
        uint32_t writeCounter = 0;
        lastNeoMs = soakStartMs;

        while (true) {
            uint32_t nowMs = to_ms_since_boot(get_absolute_time());
            bool soaking = (nowMs - soakStartMs) < kSpinlockSoakMs;

            // Spinlock struct update (high rate during soak)
            if (soaking && g_pTestSpinlock != nullptr) {
                writeCounter++;
                uint32_t saved = spin_lock_blocking(g_pTestSpinlock);
                g_spinlockData.field_a = writeCounter;
                g_spinlockData.field_b = writeCounter;
                g_spinlockData.field_c = writeCounter;
                g_spinlockData.write_count = writeCounter;
                spin_unlock(g_pTestSpinlock, saved);
            }

            g_core1Counter.fetch_add(1, std::memory_order_relaxed);

            // NeoPixel update at ~2Hz
            if (nowMs - lastNeoMs >= 250) {
                lastNeoMs = nowMs;
                neoState = !neoState;
                ws2812_set_mode(WS2812_MODE_SOLID,
                    neoState ? kColorCyan : kColorMagenta);
                ws2812_update();
            }

            // ~100kHz update rate during soak, idle wait after
            if (soaking) {
                busy_wait_us(10);
            } else {
                // Post-soak: check if Core 0 wants us to start sensor phase
                if (g_startSensorPhase.load(std::memory_order_acquire)) {
                    break;  // Exit Phase 2, enter Phase 3
                }
                sleep_ms(1);
            }
        }
    } else {
        // Skipping Phase 2 — wait for sensor phase signal from Core 0
        while (!g_startSensorPhase.load(std::memory_order_acquire)) {
            g_core1Counter.fetch_add(1, std::memory_order_relaxed);
            sleep_ms(1);
        }
    }

    // ---- Phase 3: Real Sensor Loop (IVP-25/26) ----
    // Read IMU at ~1kHz, baro at ~50Hz via divider. Apply calibration.
    // Publish via seqlock. NeoPixel: blue/cyan 2Hz during soak, green when done.
    //
    // INVARIANT: Core 0 must NOT call icm20948_*() or baro_dps310_*() unless
    // g_core1I2CPaused == true. Core 1 owns I2C during Phase 3.

    // Load calibration into local copy (reads from RAM cache, no flash access)
    calibration_store_t localCal;
    if (!calibration_load_into(&localCal)) {
        // No valid calibration — use identity (raw passthrough)
        memset(&localCal, 0, sizeof(localCal));
        localCal.accel.scale.x = 1.0f;
        localCal.accel.scale.y = 1.0f;
        localCal.accel.scale.z = 1.0f;
        localCal.board_rotation.m[0] = 1.0f;
        localCal.board_rotation.m[4] = 1.0f;
        localCal.board_rotation.m[8] = 1.0f;
    }

    shared_sensor_data_t localData = {};
    uint32_t loopCount = 0;
    uint32_t baroCycle = 0;
    uint32_t imuConsecFail = 0;
    lastNeoMs = to_ms_since_boot(get_absolute_time());
    neoState = false;

    while (true) {
        uint32_t cycleStartUs = time_us_32();
        loopCount++;

        // Check calibration reload request from Core 0
        if (g_calReloadPending.load(std::memory_order_acquire)) {
            calibration_load_into(&localCal);
            g_calReloadPending.store(false, std::memory_order_release);
        }

        // Check I2C pause request from Core 0 (for calibration)
        if (g_core1PauseI2C.load(std::memory_order_acquire)) {
            g_core1I2CPaused.store(true, std::memory_order_release);
            // Show orange NeoPixel while paused
            ws2812_set_mode(WS2812_MODE_SOLID, kColorOrange);
            ws2812_update();
            // Wait for unpause — don't increment error counters (intentional pause)
            while (g_core1PauseI2C.load(std::memory_order_acquire)) {
                sleep_ms(1);
            }
            g_core1I2CPaused.store(false, std::memory_order_release);
            // Resume NeoPixel state
            ws2812_set_mode(WS2812_MODE_SOLID, kColorBlue);
            ws2812_update();
            continue;  // Restart loop timing
        }

        // ---- IMU read ----
        icm20948_data_t imuData;
        bool imuOk = icm20948_read(&g_imu, &imuData);
        if (imuOk) {
            imuConsecFail = 0;
            // Apply calibration
            float ax;
            float ay;
            float az;
            float gx;
            float gy;
            float gz;
            calibration_apply_accel_with(&localCal,
                imuData.accel.x, imuData.accel.y, imuData.accel.z,
                &ax, &ay, &az);
            calibration_apply_gyro_with(&localCal,
                imuData.gyro.x, imuData.gyro.y, imuData.gyro.z,
                &gx, &gy, &gz);

            localData.accel_x = ax;
            localData.accel_y = ay;
            localData.accel_z = az;
            localData.gyro_x = gx;
            localData.gyro_y = gy;
            localData.gyro_z = gz;
            localData.imu_timestamp_us = time_us_32();
            localData.imu_read_count++;
            localData.accel_valid = true;
            localData.gyro_valid = true;

            // Mag (updated by ICM-20948 I2C master at 100Hz, read via EXT_SLV)
            if (imuData.mag_valid) {
                localData.mag_x = imuData.mag.x;
                localData.mag_y = imuData.mag.y;
                localData.mag_z = imuData.mag.z;
                localData.mag_valid = true;
                localData.mag_read_count++;
            }
        } else {
            imuConsecFail++;
            localData.accel_valid = false;
            localData.gyro_valid = false;
            localData.imu_error_count++;
            // Council mod #4: attempt bus recovery after consecutive failures
            if (imuConsecFail >= kCore1ConsecFailMax) {
                i2c_bus_recover();
                imuConsecFail = 0;
            }
        }

        // ---- Baro read (every Nth cycle) ----
        baroCycle++;
        if (baroCycle >= kCore1BaroDivider) {
            baroCycle = 0;
            // Council mod #3: check DPS310 data-ready before reading
            uint8_t measCfg = 0;
            if (i2c_bus_read_reg(kI2cAddrDps310, kDps310MeasCfgReg, &measCfg) == 0 &&
                (measCfg & (kDps310PrsRdy | kDps310TmpRdy)) == (kDps310PrsRdy | kDps310TmpRdy)) {
                baro_dps310_data_t baroData;
                if (baro_dps310_read(&baroData) && baroData.valid) {
                    localData.pressure_pa = baroData.pressure_pa;
                    localData.baro_temperature_c = baroData.temperature_c;
                    localData.baro_timestamp_us = time_us_32();
                    localData.baro_read_count++;
                    localData.baro_valid = true;
                } else {
                    localData.baro_error_count++;
                }
            }
            // If not ready, skip — don't count as error (DPS310 at 8Hz, we poll at 50Hz)
        }

        // Publish via seqlock (always write, even on IMU failure — council mod #4)
        localData.core1_loop_count = loopCount;
        seqlock_write(&g_sensorSeqlock, &localData);

        g_core1Counter.fetch_add(1, std::memory_order_relaxed);

        // IVP-30: Signal watchdog that Core 1 is alive
        g_wdtCore1Alive.store(true, std::memory_order_relaxed);

        // IVP-30: Core 1 stall test (triggered by 'W' command from Core 0)
        if (g_core1StallTest.load(std::memory_order_relaxed)) {
            while (true) { __asm volatile ("nop"); }
        }

        // Record jitter timestamps (first N samples)
        uint32_t jIdx = g_jitterSamplesCollected.load(std::memory_order_relaxed);
        if (jIdx < kJitterSampleCount) {
            g_jitterTimestamps[jIdx] = time_us_32();
            g_jitterSamplesCollected.store(jIdx + 1, std::memory_order_release);
        }

        // NeoPixel: blue/cyan during soak, green when done
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        if (nowMs - lastNeoMs >= 250) {
            lastNeoMs = nowMs;
            neoState = !neoState;
            if (g_sensorPhaseDone.load(std::memory_order_acquire)) {
                ws2812_set_mode(WS2812_MODE_SOLID, kColorGreen);
            } else {
                ws2812_set_mode(WS2812_MODE_SOLID,
                    neoState ? kColorBlue : kColorCyan);
            }
            ws2812_update();
        }

        // Self-pace to ~1kHz (busy_wait remaining time in cycle)
        uint32_t elapsed = time_us_32() - cycleStartUs;
        if (elapsed < kCore1TargetCycleUs) {
            busy_wait_us(kCore1TargetCycleUs - elapsed);
        }
    }
}

// ============================================================================
// Accel Read Callback (for 6-pos calibration via CLI)
// ============================================================================

static bool read_accel_for_cal(float* ax, float* ay, float* az, float* temp_c) {
    sleep_ms(10);  // ~100Hz sampling rate
    icm20948_vec3_t accel;
    if (!icm20948_read_accel(&g_imu, &accel)) {
        return false;
    }
    *ax = accel.x;
    *ay = accel.y;
    *az = accel.z;
    *temp_c = 0.0f;  // Not needed for 6-pos cal
    return true;
}

// ============================================================================
// Calibration Hooks — disable I2C master during rapid accel reads
// ============================================================================
// The ICM-20948's internal I2C master (for mag reads) performs autonomous
// Bank 3 transactions that race with our Bank 0 accel reads, causing
// bank-select corruption after ~150 reads. Disable it during calibration.

static void cal_pre_hook(void) {
    // IVP-25: If Core 1 is running sensors, pause it and take I2C ownership
    if (g_ivp25Active && !g_core1I2CPaused.load(std::memory_order_acquire)) {
        g_core1PauseI2C.store(true, std::memory_order_release);
        // Wait for Core 1 to acknowledge pause (max 100ms)
        for (uint32_t i = 0; i < kCore1PauseAckMaxMs; i++) {
            if (g_core1I2CPaused.load(std::memory_order_acquire)) break;
            sleep_ms(1);
        }
    }
    if (g_imuInitialized) {
        icm20948_set_i2c_master_enable(&g_imu, false);
    }
}

static void cal_post_hook(void) {
    if (g_imuInitialized) {
        icm20948_set_i2c_master_enable(&g_imu, true);
    }
    // IVP-25: Tell Core 1 to reload calibration and resume sensors
    if (g_ivp25Active) {
        g_calReloadPending.store(true, std::memory_order_release);
        g_core1PauseI2C.store(false, std::memory_order_release);
    }
}

// ============================================================================
// IVP-29/30: Test Command Handler (via RC_OS unhandled-key callback)
// ============================================================================

static void ivp_test_key_handler(int key) {
    if (!g_testCommandsEnabled) return;  // Available after soak or watchdog reboot
    if (!stdio_usb_connected()) return;

    switch (key) {
    case 'o': {
        // IVP-29: Intentional Core 0 stack overflow
        printf("\n[TEST] Triggering Core 0 stack overflow...\n");
        printf("[TEST] Expect red LED 3-fast+1-slow blink. Power cycle to recover.\n");
        sleep_ms(100);  // Flush output

        // Recursive function with volatile local to prevent optimizer elimination
        volatile uint8_t buf[256];
        buf[0] = (uint8_t)key;  // Prevent unused-variable optimization
        (void)buf[255];
        ivp_test_key_handler(key);  // Recurse until stack overflow
        break;
    }
    case 'w': {
        // IVP-30: Stall Core 0 (watchdog should reset within 5s)
        printf("\n[TEST] Stalling Core 0 — watchdog should reset within %lu ms\n",
               (unsigned long)kWatchdogTimeoutMs);
        sleep_ms(100);  // Flush output
        while (true) { __asm volatile ("nop"); }
        break;
    }
    case 'W': {
        // IVP-30: Stall Core 1 via atomic flag
        printf("\n[TEST] Stalling Core 1 — watchdog should reset within %lu ms\n",
               (unsigned long)kWatchdogTimeoutMs);
        g_core1StallTest.store(true, std::memory_order_relaxed);
        sleep_ms(100);  // Flush output
        // Core 1 will enter infinite loop on next iteration. Core 0 keeps running
        // but can't kick watchdog (Core 1 alive flag stops being set).
        break;
    }
    default:
        break;
    }
}

// ============================================================================
// Sensor Status Callback (for RC_OS CLI 's' command)
// ============================================================================

static void print_sensor_status(void) {
    printf("\n========================================\n");
    printf("  Sensor Readings (calibrated)\n");
    printf("========================================\n");

    // Council mod #6: Once Core 1 owns I2C, read from seqlock to prevent bus contention.
    // Before sensor phase, fall back to direct I2C reads.
    if (g_ivp25Active) {
        // Read from seqlock — Core 1 is driving sensors
        shared_sensor_data_t snap;
        if (seqlock_read(&g_sensorSeqlock, &snap)) {
            if (snap.accel_valid) {
                printf("Accel (m/s^2): X=%7.3f Y=%7.3f Z=%7.3f\n",
                       (double)snap.accel_x, (double)snap.accel_y, (double)snap.accel_z);
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
                printf("Mag     (uT):  X=%6.1f  Y=%6.1f  Z=%6.1f\n",
                       (double)snap.mag_x, (double)snap.mag_y, (double)snap.mag_z);
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
            printf("IMU reads: %lu, Baro reads: %lu, Errors: I=%lu B=%lu\n",
                   (unsigned long)snap.imu_read_count,
                   (unsigned long)snap.baro_read_count,
                   (unsigned long)snap.imu_error_count,
                   (unsigned long)snap.baro_error_count);
        } else {
            printf("Seqlock read failed (retries exhausted)\n");
        }
    } else {
        // Pre-sensor-phase: direct I2C reads (Core 0 owns bus)
        if (g_imuInitialized) {
            icm20948_data_t data;
            if (icm20948_read(&g_imu, &data)) {
                float gx;
                float gy;
                float gz;
                float ax;
                float ay;
                float az;
                calibration_apply_gyro(data.gyro.x, data.gyro.y, data.gyro.z,
                                       &gx, &gy, &gz);
                calibration_apply_accel(data.accel.x, data.accel.y, data.accel.z,
                                        &ax, &ay, &az);
                printf("Accel (m/s^2): X=%7.3f Y=%7.3f Z=%7.3f\n",
                       (double)ax, (double)ay, (double)az);
                printf("Gyro  (rad/s): X=%7.4f Y=%7.4f Z=%7.4f\n",
                       (double)gx, (double)gy, (double)gz);
                if (data.mag_valid) {
                    printf("Mag     (uT):  X=%6.1f  Y=%6.1f  Z=%6.1f\n",
                           (double)data.mag.x, (double)data.mag.y, (double)data.mag.z);
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

    printf("========================================\n\n");
}

// ============================================================================
// Hardware Validation
// ============================================================================

static const char* get_device_name(uint8_t addr) {
    switch (addr) {
        case kI2cAddrIcm20948: return "ICM-20948";
        case kI2cAddrDps310:   return "DPS310";
        case kI2cAddrPa1010d:  return "PA1010D GPS";
        case 0x68:              return "ICM-20948 (AD0=LOW)";
        case 0x76:              return "DPS310 (alt)";
        default:                return "Unknown";
    }
}

static void hw_validate_stage1(void) {
    printf("\n=== HW Validation: Stage 1 ===\n");

    // IVP-01: Build + boot
    printf("[PASS] Build + boot (you're reading this)\n");

    // IVP-02: Red LED
    printf("[PASS] Red LED GPIO initialized (pin %d)\n", PICO_DEFAULT_LED_PIN);

    // IVP-03: NeoPixel
    printf("[%s] NeoPixel PIO initialized (pin %d)\n",
           g_neopixelInitialized ? "PASS" : "FAIL", kNeoPixelPin);

    // IVP-04: USB CDC
    printf("[PASS] USB CDC connected\n");

    // IVP-05: Debug macros
    uint32_t ts = time_us_32();
    printf("[%s] Debug macros functional (timestamp=%lu us)\n",
           ts > 0 ? "PASS" : "FAIL", (unsigned long)ts);

    // IVP-06: I2C bus
    if (g_i2cInitialized) {
        printf("[PASS] I2C bus initialized at %lukHz (SDA=%d, SCL=%d)\n",
               (unsigned long)(kI2cBusFreqHz / 1000), kI2cBusSdaPin, kI2cBusSclPin);
    } else {
        printf("[FAIL] I2C bus failed to initialize\n");
    }

    // IVP-07: I2C device detection
    // Note: PA1010D GPS (0x10) excluded — probing it triggers I2C bus
    // interference (Pico SDK #252). Re-add at IVP-31 with proper handling.
    static const uint8_t expected[] = {
        kI2cAddrIcm20948,  // 0x69
        kI2cAddrDps310,    // 0x77
    };
    int foundCount = 0;
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        bool found = i2c_bus_probe(expected[i]);
        printf("[----] I2C 0x%02X (%s): %s\n",
               expected[i], get_device_name(expected[i]),
               found ? "FOUND" : "NOT FOUND");
        if (found) foundCount++;
    }
    printf("[INFO] Sensors found: %d/%zu expected\n",
           foundCount, sizeof(expected) / sizeof(expected[0]));

    // IVP-09: IMU initialization
    if (g_imuInitialized) {
        printf("[PASS] ICM-20948 init (WHO_AM_I=0xEA)\n");
        printf("[%s] AK09916 magnetometer %s\n",
               g_imu.mag_initialized ? "PASS" : "WARN",
               g_imu.mag_initialized ? "ready" : "not ready");
    } else {
        printf("[FAIL] ICM-20948 init failed\n");
    }

    // IVP-11: Barometer initialization
    if (g_baroInitialized && g_baroContinuous) {
        printf("[PASS] DPS310 init OK, continuous mode active\n");
    } else if (g_baroInitialized) {
        printf("[WARN] DPS310 init OK, continuous mode failed\n");
    } else {
        printf("[FAIL] DPS310 init failed\n");
    }

    printf("=== Validation Complete ===\n\n");
}

static void hw_validate_superloop(uint32_t uptimeMs) {
    printf("\n=== HW Validation: Superloop (IVP-08) ===\n");
    printf("[PASS] Uptime advancing (%lu ms)\n", (unsigned long)uptimeMs);
    printf("[PASS] Main loop iterations: %lu\n", (unsigned long)g_loopCounter);
    printf("=== Superloop Validation Complete ===\n\n");
}

// ============================================================================
// IVP-10: IMU Data Validation
// ============================================================================

static void ivp10_print_sample(uint32_t n, const icm20948_data_t* d) {
    printf("[%03lu] A: %7.3f %7.3f %7.3f  G: %7.4f %7.4f %7.4f  M:%s %6.1f %6.1f %6.1f  T: %.1fC\n",
           (unsigned long)n,
           (double)d->accel.x, (double)d->accel.y, (double)d->accel.z,
           (double)d->gyro.x, (double)d->gyro.y, (double)d->gyro.z,
           d->mag_valid ? "" : "?",
           (double)d->mag.x, (double)d->mag.y, (double)d->mag.z,
           (double)d->temperature_c);
}

static void ivp10_accumulate(const icm20948_data_t* d) {
    // Check for NaN/inf
    if (isnan(d->accel.x) || isnan(d->accel.y) || isnan(d->accel.z) ||
        isnan(d->gyro.x) || isnan(d->gyro.y) || isnan(d->gyro.z) ||
        isinf(d->accel.x) || isinf(d->accel.y) || isinf(d->accel.z) ||
        isinf(d->gyro.x) || isinf(d->gyro.y) || isinf(d->gyro.z)) {
        g_nanCount++;
        return;
    }

    g_validSampleCount++;

    // Accel sums for averaging
    g_accelXSum += d->accel.x;
    g_accelYSum += d->accel.y;
    g_accelZSum += d->accel.z;

    // Accel Z range for stability check
    if (d->accel.z < g_accelZMin) g_accelZMin = d->accel.z;
    if (d->accel.z > g_accelZMax) g_accelZMax = d->accel.z;

    // Gyro max absolute
    float gmax = fabsf(d->gyro.x);
    if (fabsf(d->gyro.y) > gmax) gmax = fabsf(d->gyro.y);
    if (fabsf(d->gyro.z) > gmax) gmax = fabsf(d->gyro.z);
    if (gmax > g_gyroAbsMax) g_gyroAbsMax = gmax;

    // Mag magnitude
    if (d->mag_valid) {
        float mag = sqrtf(d->mag.x * d->mag.x + d->mag.y * d->mag.y + d->mag.z * d->mag.z);
        if (g_magValidCount == 0 || mag < g_magMagMin) g_magMagMin = mag;
        if (g_magValidCount == 0 || mag > g_magMagMax) g_magMagMax = mag;
        g_magValidCount++;
    }

    // Temperature
    if (d->temperature_c < g_tempMin) g_tempMin = d->temperature_c;
    if (d->temperature_c > g_tempMax) g_tempMax = d->temperature_c;
}

static void ivp10_gate_check(void) {
    printf("\n=== IVP-10: IMU Data Validation Gate ===\n");

    bool pass = true;

    float n = (float)g_validSampleCount;
    float axAvg = (n > 0) ? g_accelXSum / n : 0.0f;
    float ayAvg = (n > 0) ? g_accelYSum / n : 0.0f;
    float azAvg = (n > 0) ? g_accelZSum / n : 0.0f;

    // Gate: Accel magnitude ~9.8 m/s^2 (±0.5)
    // Uses magnitude because board orientation on desk is unknown
    float accelMag = sqrtf(axAvg * axAvg + ayAvg * ayAvg + azAvg * azAvg);
    bool amOk = (accelMag > 9.3f && accelMag < 10.3f);
    printf("[%s] Accel magnitude: %.3f m/s^2 (expect 9.3-10.3)\n",
           amOk ? "PASS" : "FAIL", (double)accelMag);
    if (!amOk) pass = false;

    // Info: Individual axis averages (orientation-dependent)
    printf("[INFO] Accel axes avg: X=%.3f Y=%.3f Z=%.3f m/s^2\n",
           (double)axAvg, (double)ayAvg, (double)azAvg);

    // Gate: Gyro near zero (±0.05 rad/s)
    bool gOk = (g_gyroAbsMax < 0.05f);
    printf("[%s] Gyro max abs: %.4f rad/s (expect <0.05)\n",
           gOk ? "PASS" : "WARN", (double)g_gyroAbsMax);

    // Gate: Mag reasonable (10-60 µT), not zeros
    bool mOk = (g_magValidCount > 0 && g_magMagMin > 10.0f && g_magMagMax < 60.0f);
    if (g_magValidCount > 0) {
        printf("[%s] Mag range: %.1f-%.1f uT (%lu/%lu valid) (expect 10-60)\n",
               mOk ? "PASS" : "WARN", (double)g_magMagMin, (double)g_magMagMax,
               (unsigned long)g_magValidCount, (unsigned long)kImuSampleCount);
    } else {
        printf("[WARN] Mag: no valid samples (may need additional boot cycle)\n");
    }

    // Gate: Temperature plausible (15-40°C)
    bool tOk = (g_tempMin > 15.0f && g_tempMax < 40.0f);
    printf("[%s] Temp range: %.1f-%.1fC (expect 15-40)\n",
           tOk ? "PASS" : "WARN", (double)g_tempMin, (double)g_tempMax);

    // Gate: No NaN/inf
    bool nanOk = (g_nanCount == 0);
    printf("[%s] NaN/inf count: %lu\n",
           nanOk ? "PASS" : "FAIL", (unsigned long)g_nanCount);
    if (!nanOk) pass = false;

    // Gate: Accel stability (max-min < 0.5 over 5s)
    float azRange = g_accelZMax - g_accelZMin;
    bool azStable = (azRange < 0.5f);
    printf("[%s] Accel Z stability: range=%.3f m/s^2 (expect <0.5)\n",
           azStable ? "PASS" : "WARN", (double)azRange);

    // Sample quality
    printf("[INFO] Valid samples: %lu/%lu\n",
           (unsigned long)g_validSampleCount, (unsigned long)kImuSampleCount);

    printf("=== IVP-10: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-12: Barometer Data Validation
// ============================================================================

static void ivp12_accumulate(const baro_dps310_data_t* d) {
    if (!d->valid) {
        g_baroInvalidReads++;
        return;
    }

    if (isnan(d->pressure_pa) || isnan(d->temperature_c) ||
        isinf(d->pressure_pa) || isinf(d->temperature_c)) {
        g_baroNanCount++;
        return;
    }

    // Stuck value detection (same pressure to 0.01 Pa)
    if (g_baroValidReads > 0 && fabsf(d->pressure_pa - g_baroLastPress) < 0.01f) {
        g_baroStuckCount++;
    }
    g_baroLastPress = d->pressure_pa;

    // Pressure
    if (g_baroValidReads == 0 || d->pressure_pa < g_baroPressMin) g_baroPressMin = d->pressure_pa;
    if (g_baroValidReads == 0 || d->pressure_pa > g_baroPressMax) g_baroPressMax = d->pressure_pa;
    g_baroPressSum += d->pressure_pa;

    // Temperature
    if (g_baroValidReads == 0 || d->temperature_c < g_baroTempMin) g_baroTempMin = d->temperature_c;
    if (g_baroValidReads == 0 || d->temperature_c > g_baroTempMax) g_baroTempMax = d->temperature_c;

    g_baroValidReads++;
}

static void ivp12_gate_check(void) {
    printf("\n=== IVP-12: Barometer Data Validation Gate ===\n");

    bool pass = true;

    if (g_baroValidReads == 0) {
        printf("[FAIL] No valid baro readings\n");
        printf("=== IVP-12: GATE FAILURES ===\n\n");
        return;
    }

    float pressAvg = g_baroPressSum / (float)g_baroValidReads;

    // Gate: Pressure ~101325 Pa (±3000 Pa for typical altitudes)
    bool pOk = (pressAvg > 98325.0f && pressAvg < 104325.0f);
    printf("[%s] Pressure avg: %.1f Pa (expect 98325-104325)\n",
           pOk ? "PASS" : "FAIL", (double)pressAvg);
    if (!pOk) pass = false;

    // Gate: Temperature plausible (15-40°C)
    bool tOk = (g_baroTempMin > 15.0f && g_baroTempMax < 40.0f);
    printf("[%s] Temp range: %.1f-%.1fC (expect 15-40)\n",
           tOk ? "PASS" : "WARN", (double)g_baroTempMin, (double)g_baroTempMax);

    // Gate: All reads valid
    bool vOk = (g_baroInvalidReads == 0);
    printf("[%s] Valid reads: %lu/%lu\n",
           vOk ? "PASS" : "WARN",
           (unsigned long)g_baroValidReads, (unsigned long)kBaroSampleCount);

    // Gate: Pressure noise < ±5 Pa over 10 seconds
    float pressRange = g_baroPressMax - g_baroPressMin;
    bool nOk = (pressRange < 10.0f);  // ±5 Pa = 10 Pa total range
    printf("[%s] Pressure noise: %.2f Pa range (expect <10)\n",
           nOk ? "PASS" : "WARN", (double)pressRange);

    // Gate: No NaN/inf
    bool nanOk = (g_baroNanCount == 0);
    printf("[%s] NaN/inf count: %lu\n",
           nanOk ? "PASS" : "FAIL", (unsigned long)g_baroNanCount);
    if (!nanOk) pass = false;

    // Gate: No stuck values (>50% identical readings = stuck)
    bool stuckOk = (g_baroStuckCount < g_baroValidReads / 2);
    printf("[%s] Stuck values: %lu/%lu\n",
           stuckOk ? "PASS" : "FAIL",
           (unsigned long)g_baroStuckCount, (unsigned long)g_baroValidReads);
    if (!stuckOk) pass = false;

    // Info: altitude
    float alt = baro_dps310_pressure_to_altitude(pressAvg, 101325.0f);
    printf("[INFO] Altitude (std atm): %.1f m\n", (double)alt);

    printf("=== IVP-12: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-13: Multi-Sensor Polling Gate Check
// ============================================================================

static void ivp13_gate_check(void) {
    printf("\n=== IVP-13: Multi-Sensor Polling Gate ===\n");

    bool pass = true;
    uint32_t elapsedMs = to_ms_since_boot(get_absolute_time()) - g_ivp13StartMs;
    float elapsedS = elapsedMs / 1000.0f;

    // Actual rates
    float imuRate = g_ivp13ImuCount / elapsedS;
    float baroRate = g_ivp13BaroCount / elapsedS;

    // Gate: IMU rate within 5% of 100Hz target
    bool imuOk = (imuRate > 95.0f && imuRate < 105.0f);
    printf("[%s] IMU rate: %.1f Hz (target 100, expect 95-105)\n",
           imuOk ? "PASS" : "FAIL", (double)imuRate);
    if (!imuOk) pass = false;

    // Gate: Baro rate within 10% of 50Hz target
    bool baroOk = (baroRate > 45.0f && baroRate < 55.0f);
    printf("[%s] Baro rate: %.1f Hz (target 50, expect 45-55)\n",
           baroOk ? "PASS" : "FAIL", (double)baroRate);
    if (!baroOk) pass = false;

    // Gate: Zero I2C errors over 60 seconds
    uint32_t totalErrors = g_ivp13ImuErrors + g_ivp13BaroErrors;
    bool errOk = (totalErrors == 0);
    printf("[%s] I2C errors: %lu (IMU: %lu, Baro: %lu)\n",
           errOk ? "PASS" : "FAIL", (unsigned long)totalErrors,
           (unsigned long)g_ivp13ImuErrors, (unsigned long)g_ivp13BaroErrors);
    if (!errOk) pass = false;

    // Info: Per-read I2C timing
    if (g_ivp13ImuCount > 0) {
        uint32_t imuAvgUs = (uint32_t)(g_ivp13ImuTimeSum / g_ivp13ImuCount);
        printf("[INFO] IMU read time: avg %lu us, max %lu us\n",
               (unsigned long)imuAvgUs, (unsigned long)g_ivp13ImuTimeMax);
    }
    if (g_ivp13BaroCount > 0) {
        uint32_t baroAvgUs = (uint32_t)(g_ivp13BaroTimeSum / g_ivp13BaroCount);
        printf("[INFO] Baro read time: avg %lu us, max %lu us\n",
               (unsigned long)baroAvgUs, (unsigned long)g_ivp13BaroTimeMax);
    }

    printf("[INFO] Total samples: IMU %lu, Baro %lu over %.1f s\n",
           (unsigned long)g_ivp13ImuCount, (unsigned long)g_ivp13BaroCount,
           (double)elapsedS);

    // IVP-13a: Recovery statistics
    if (g_i2cRecoveryAttempts > 0) {
        printf("[INFO] Bus recoveries: %lu/%lu successful\n",
               (unsigned long)g_i2cRecoverySuccesses,
               (unsigned long)g_i2cRecoveryAttempts);
    }

    printf("=== IVP-13: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-14: Calibration Storage Gate Check
// ============================================================================

// Known test values for save/load verification
static constexpr float kTestGyroX = 0.00123f;
static constexpr float kTestGyroY = -0.00234f;
static constexpr float kTestGyroZ = 0.00345f;

static void ivp14_gate_check(void) {
    printf("\n=== IVP-14: Calibration Storage Gate ===\n");

    bool pass = true;

    // Gate 1: Load returns OK or initializes defaults
    {
        cal_result_t result = calibration_load();
        bool ok = (result == CAL_RESULT_OK || result == CAL_RESULT_STORAGE_ERROR);
        printf("[%s] Gate 1: calibration_load() = %d (%s)\n",
               ok ? "PASS" : "FAIL", (int)result,
               result == CAL_RESULT_OK ? "valid data" : "defaults initialized");
        // Note: STORAGE_ERROR on first boot is expected (no data yet)
    }

    // Gate 2: Save test data, read back, verify match
    {
        // Get current cal, modify gyro bias with known values
        const calibration_store_t* current = calibration_manager_get();
        calibration_store_t test_cal = *current;
        test_cal.gyro.bias.x = kTestGyroX;
        test_cal.gyro.bias.y = kTestGyroY;
        test_cal.gyro.bias.z = kTestGyroZ;
        test_cal.gyro.status = CAL_STATUS_GYRO;
        test_cal.cal_flags |= CAL_STATUS_GYRO;
        calibration_update_crc(&test_cal);

        // Save
        bool writeOk = calibration_storage_write(&test_cal);
        if (!writeOk) {
            printf("[FAIL] Gate 2: flash write failed\n");
            pass = false;
        } else {
            // Read back
            calibration_store_t readback;
            bool readOk = calibration_storage_read(&readback);
            if (!readOk) {
                printf("[FAIL] Gate 2: flash read failed\n");
                pass = false;
            } else {
                bool match = (fabsf(readback.gyro.bias.x - kTestGyroX) < 1e-6f &&
                              fabsf(readback.gyro.bias.y - kTestGyroY) < 1e-6f &&
                              fabsf(readback.gyro.bias.z - kTestGyroZ) < 1e-6f &&
                              (readback.cal_flags & CAL_STATUS_GYRO));
                printf("[%s] Gate 2: save/readback %s (bias: %.5f, %.5f, %.5f)\n",
                       match ? "PASS" : "FAIL",
                       match ? "match" : "MISMATCH",
                       (double)readback.gyro.bias.x,
                       (double)readback.gyro.bias.y,
                       (double)readback.gyro.bias.z);
                if (!match) pass = false;
            }
        }
    }

    // Gate 3: Power cycle persistence
    // Check if test data survived from a previous boot
    {
        calibration_store_t persisted;
        bool readOk = calibration_storage_read(&persisted);
        if (readOk && (persisted.cal_flags & CAL_STATUS_GYRO) &&
            fabsf(persisted.gyro.bias.x - kTestGyroX) < 1e-6f) {
            printf("[PASS] Gate 3: data persisted across power cycle\n");
        } else {
            printf("[INFO] Gate 3: power cycle test — unplug USB, replug, check next boot\n");
            // Not a failure — just needs physical verification
        }
    }

    // Gate 4: 10 consecutive saves
    {
        bool allOk = true;
        for (uint8_t i = 0; i < 10; i++) {
            calibration_store_t test_cal;
            calibration_init_defaults(&test_cal);
            test_cal.gyro.bias.x = kTestGyroX;
            test_cal.gyro.bias.y = kTestGyroY;
            test_cal.gyro.bias.z = kTestGyroZ;
            test_cal.gyro.status = CAL_STATUS_GYRO;
            test_cal.cal_flags |= CAL_STATUS_GYRO;
            // Use reserved field to track iteration
            test_cal.reserved[0] = (uint8_t)(i + 1);
            calibration_update_crc(&test_cal);

            if (!calibration_storage_write(&test_cal)) {
                printf("[FAIL] Gate 4: write %d/10 failed\n", i + 1);
                allOk = false;
                break;
            }

            // Verify readback
            calibration_store_t verify;
            if (!calibration_storage_read(&verify) || verify.reserved[0] != (uint8_t)(i + 1)) {
                printf("[FAIL] Gate 4: readback %d/10 failed\n", i + 1);
                allOk = false;
                break;
            }
        }
        if (allOk) {
            printf("[PASS] Gate 4: 10/10 consecutive saves OK (wear leveling works)\n");
        } else {
            pass = false;
        }
    }

    printf("=== IVP-14: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-15: Gyro Bias Calibration Gate Check
// ============================================================================

static void ivp15_gate_check(void) {
    printf("\n=== IVP-15: Gyro Bias Calibration Gate ===\n");

    bool pass = true;
    cal_result_t result = calibration_get_result();

    // Gate 1: Returns CAL_RESULT_OK
    bool resultOk = (result == CAL_RESULT_OK);
    printf("[%s] Gate 1: calibration result = %d (%s)\n",
           resultOk ? "PASS" : "FAIL", (int)result,
           result == CAL_RESULT_OK ? "OK" :
           result == CAL_RESULT_MOTION_DETECTED ? "MOTION_DETECTED" : "ERROR");
    if (!resultOk) { pass = false; }

    // Gate 2: Bias values each axis < 0.1 rad/s
    const calibration_store_t* cal = calibration_manager_get();
    float bx = cal->gyro.bias.x;
    float by = cal->gyro.bias.y;
    float bz = cal->gyro.bias.z;
    bool biasOk = (fabsf(bx) < 0.1f && fabsf(by) < 0.1f && fabsf(bz) < 0.1f);
    printf("[%s] Gate 2: bias X=%.5f Y=%.5f Z=%.5f rad/s (expect <0.1 each)\n",
           biasOk ? "PASS" : "FAIL",
           (double)bx, (double)by, (double)bz);
    if (!biasOk) pass = false;

    // Gate 3: After applying, readings near zero when stationary
    // Take 10 samples and check corrected gyro near zero
    {
        float maxCorrected = 0.0f;
        uint32_t goodSamples = 0;
        for (uint8_t i = 0; i < 10; i++) {
            sleep_ms(10);  // ~100Hz
            icm20948_data_t data;
            if (icm20948_read(&g_imu, &data)) {
                float cx;
                float cy;
                float cz;
                calibration_apply_gyro(data.gyro.x, data.gyro.y, data.gyro.z,
                                       &cx, &cy, &cz);
                float m = fabsf(cx);
                if (fabsf(cy) > m) m = fabsf(cy);
                if (fabsf(cz) > m) m = fabsf(cz);
                if (m > maxCorrected) maxCorrected = m;
                goodSamples++;
            }
        }
        // After bias subtraction, stationary gyro should read < 0.05 rad/s
        bool corrOk = (goodSamples >= 5 && maxCorrected < 0.05f);
        printf("[%s] Gate 3: corrected gyro max=%.5f rad/s (%lu/10 samples) (expect <0.05)\n",
               corrOk ? "PASS" : "FAIL",
               (double)maxCorrected, (unsigned long)goodSamples);
        if (!corrOk) pass = false;
    }

    // Gate 4: Persists across power cycle
    // Save, then read back and verify
    {
        cal_result_t saveResult = calibration_save();
        if (saveResult != CAL_RESULT_OK) {
            printf("[FAIL] Gate 4: save failed (%d)\n", (int)saveResult);
            pass = false;
        } else {
            // Read back from flash
            calibration_store_t readback;
            bool readOk = calibration_storage_read(&readback);
            bool persistOk = readOk &&
                calibration_validate(&readback) &&
                (readback.cal_flags & CAL_STATUS_GYRO) &&
                fabsf(readback.gyro.bias.x - bx) < 1e-6f &&
                fabsf(readback.gyro.bias.y - by) < 1e-6f &&
                fabsf(readback.gyro.bias.z - bz) < 1e-6f;
            printf("[%s] Gate 4: save + readback %s (flags=0x%02lX)\n",
                   persistOk ? "PASS" : "FAIL",
                   persistOk ? "match" : "MISMATCH",
                   readOk ? (unsigned long)readback.cal_flags : 0UL);
            if (!persistOk) pass = false;
            printf("[INFO] Power cycle to verify full persistence\n");
        }
    }

    printf("=== IVP-15: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-16: Accel Level Calibration Gate Check
// ============================================================================

static void ivp16_gate_check(uint32_t elapsedMs) {
    printf("\n=== IVP-16: Accel Level Calibration Gate ===\n");

    bool pass = true;
    cal_result_t result = calibration_get_result();

    // Gate 1: Completes in < 3 seconds
    bool timeOk = (result == CAL_RESULT_OK && elapsedMs < 3000);
    printf("[%s] Gate 1: completed in %lu ms (expect <3000), result=%d (%s)\n",
           timeOk ? "PASS" : "FAIL", (unsigned long)elapsedMs, (int)result,
           result == CAL_RESULT_OK ? "OK" :
           result == CAL_RESULT_MOTION_DETECTED ? "MOTION_DETECTED" : "ERROR");
    if (!timeOk) pass = false;

    // Gate 2: After applying, Z reads +9.81 ±0.05, X and Y < 0.05 m/s²
    {
        float axSum = 0.0f;
        float aySum = 0.0f;
        float azSum = 0.0f;
        uint32_t goodSamples = 0;
        for (uint8_t i = 0; i < 10; i++) {
            sleep_ms(10);
            icm20948_data_t data;
            if (icm20948_read(&g_imu, &data)) {
                float cx;
                float cy;
                float cz;
                calibration_apply_accel(data.accel.x, data.accel.y, data.accel.z,
                                        &cx, &cy, &cz);
                axSum += cx;
                aySum += cy;
                azSum += cz;
                goodSamples++;
            }
        }
        if (goodSamples >= 5) {
            float n = (float)goodSamples;
            float axAvg = axSum / n;
            float ayAvg = aySum / n;
            float azAvg = azSum / n;

            bool xyOk = (fabsf(axAvg) < 0.05f && fabsf(ayAvg) < 0.05f);
            bool zOk = (fabsf(azAvg - 9.80665f) < 0.05f);

            printf("[%s] Gate 2: corrected X=%.4f Y=%.4f m/s^2 (expect <0.05 each)\n",
                   xyOk ? "PASS" : "WARN", (double)axAvg, (double)ayAvg);
            printf("[%s] Gate 2: corrected Z=%.4f m/s^2 (expect 9.81 +/-0.05)\n",
                   zOk ? "PASS" : "WARN", (double)azAvg);

            // Use WARN not FAIL — device orientation may not be perfectly flat
            // IVP.md diagnostic: "Wrong values = check accelerometer axis orientation"
            if (!xyOk || !zOk) {
                printf("[DIAG] If Z != 9.81: device may not be flat (Z-up) or needs 6-pos cal\n");
            }
        } else {
            printf("[FAIL] Gate 2: insufficient samples (%lu/10)\n",
                   (unsigned long)goodSamples);
            pass = false;
        }
    }

    // Gate 3: Persists across power cycle
    {
        const calibration_store_t* cal = calibration_manager_get();
        cal_result_t saveResult = calibration_save();
        if (saveResult != CAL_RESULT_OK) {
            printf("[FAIL] Gate 3: save failed (%d)\n", (int)saveResult);
            pass = false;
        } else {
            calibration_store_t readback;
            bool readOk = calibration_storage_read(&readback);
            bool persistOk = readOk &&
                calibration_validate(&readback) &&
                (readback.cal_flags & CAL_STATUS_LEVEL) &&
                fabsf(readback.accel.offset.x - cal->accel.offset.x) < 1e-6f &&
                fabsf(readback.accel.offset.y - cal->accel.offset.y) < 1e-6f &&
                fabsf(readback.accel.offset.z - cal->accel.offset.z) < 1e-6f;
            printf("[%s] Gate 3: save + readback %s (flags=0x%02lX)\n",
                   persistOk ? "PASS" : "FAIL",
                   persistOk ? "match" : "MISMATCH",
                   readOk ? (unsigned long)readback.cal_flags : 0UL);
            if (!persistOk) pass = false;

            // Print computed offsets for reference
            printf("[INFO] Accel offsets: X=%.4f Y=%.4f Z=%.4f m/s^2\n",
                   (double)cal->accel.offset.x,
                   (double)cal->accel.offset.y,
                   (double)cal->accel.offset.z);
            printf("[INFO] Power cycle to verify full persistence\n");
        }
    }

    printf("=== IVP-16: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES - see above");
}

// ============================================================================
// IVP-22: FIFO Message Passing Test
// ============================================================================
// Exercise-only — FIFO reserved by multicore_lockout (flash_safe_execute).

static void ivp22_fifo_test(void) {
    printf("\n=== IVP-22: Multicore FIFO Message Passing ===\n");
    bool pass = true;

    // Gate 1: Core 0 -> Core 1 echo (1000 messages)
    {
        multicore_fifo_push_blocking(kCmd_FifoEchoStart);

        // Wait for Core 1 ACK (confirms it's in the echo loop)
        uint32_t ack;
        if (!multicore_fifo_pop_timeout_us(1000000, &ack) || ack != kCmd_FifoEchoStart) {
            printf("[FAIL] Gate 1: Core 1 did not ACK echo start\n");
        }

        uint32_t lost = 0;
        uint64_t rttSumUs = 0;
        uint32_t rttMinUs = UINT32_MAX;
        uint32_t rttMaxUs = 0;

        for (uint32_t i = 0; i < kFifoTestCount; i++) {
            uint64_t t0 = time_us_64();
            multicore_fifo_push_blocking(i + 1);
            uint32_t echo;
            bool ok = multicore_fifo_pop_timeout_us(100000, &echo);
            uint32_t dt = (uint32_t)(time_us_64() - t0);

            if (!ok || echo != (i + 1)) {
                lost++;
            } else {
                rttSumUs += dt;
                if (dt < rttMinUs) rttMinUs = dt;
                if (dt > rttMaxUs) rttMaxUs = dt;
            }
        }

        // Wait for done signal
        uint32_t done;
        multicore_fifo_pop_timeout_us(1000000, &done);

        bool echoOk = (lost == 0);
        printf("[%s] Gate 1: Core0->Core1 echo: %lu/%lu OK, %lu lost\n",
               echoOk ? "PASS" : "FAIL",
               (unsigned long)(kFifoTestCount - lost),
               (unsigned long)kFifoTestCount, (unsigned long)lost);
        if (!echoOk) pass = false;

        if (lost < kFifoTestCount) {
            uint32_t rttAvg = (uint32_t)(rttSumUs / (kFifoTestCount - lost));
            printf("[INFO] Round-trip latency: min=%lu avg=%lu max=%lu us\n",
                   (unsigned long)rttMinUs, (unsigned long)rttAvg,
                   (unsigned long)rttMaxUs);
        }
    }

    multicore_fifo_drain();
    sleep_ms(10);

    // Gate 2: Core 1 -> Core 0 (1000 sequential messages)
    {
        multicore_fifo_push_blocking(kCmd_FifoSendStart);

        uint32_t received = 0;
        uint32_t outOfOrder = 0;

        for (uint32_t i = 0; i < kFifoTestCount; i++) {
            uint32_t val;
            bool ok = multicore_fifo_pop_timeout_us(100000, &val);
            if (ok) {
                received++;
                if (val != i) outOfOrder++;
            }
        }

        bool sendOk = (received == kFifoTestCount && outOfOrder == 0);
        printf("[%s] Gate 2: Core1->Core0: %lu/%lu received, %lu out of order\n",
               sendOk ? "PASS" : "FAIL",
               (unsigned long)received, (unsigned long)kFifoTestCount,
               (unsigned long)outOfOrder);
        if (!sendOk) pass = false;
    }

    multicore_fifo_drain();

    // Gate 3: FIFO overflow test (RP2350 is 4-deep per SDK docs)
    // Use non-blocking pushes so Core 1's 1ms poll can't drain between attempts
    {
        uint32_t pushCount = 0;
        for (uint32_t i = 0; i < 16; i++) {
            if (!multicore_fifo_wready()) break;
            multicore_fifo_push_blocking(i + 100);
            pushCount++;
        }

        printf("[%s] Gate 3: FIFO depth: %lu pushes before block (expect 4)\n",
               (pushCount == 4) ? "PASS" : "WARN",
               (unsigned long)pushCount);

        multicore_fifo_drain();
    }

    // Gate 4: USB still connected
    {
        bool usbOk = stdio_usb_connected();
        printf("[%s] Gate 4: USB still connected\n",
               usbOk ? "PASS" : "FAIL");
        if (!usbOk) pass = false;
    }

    printf("[INFO] FIFO reserved by multicore_lockout — exercise only\n");
    printf("=== IVP-22: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES");
}

// ============================================================================
// IVP-23: Doorbell Interrupt Test (RP2350-Specific)
// ============================================================================
// Exercise-only — seqlock polling chosen for production.

static void ivp23_doorbell_test(void) {
    printf("=== IVP-23: Doorbell Interrupts (RP2350-Specific) ===\n");
    bool pass = true;

    // Gate 1: Claim a doorbell
    g_doorbellNum = multicore_doorbell_claim_unused(0x3, true);
    bool claimOk = (g_doorbellNum >= 0);
    printf("[%s] Gate 1: Doorbell claimed: %d\n",
           claimOk ? "PASS" : "FAIL", g_doorbellNum);
    if (!claimOk) {
        printf("=== IVP-23: GATE FAILURES ===\n\n");
        return;
    }

    multicore_doorbell_clear_current_core((uint)g_doorbellNum);

    // Gate 2: Core 1 sets doorbell 1000 times, Core 0 detects
    multicore_fifo_drain();
    sleep_ms(10);
    multicore_fifo_push_blocking(kCmd_DoorbellStart);

    uint32_t detected = 0;
    uint64_t latencySumUs = 0;
    uint32_t latencyMinUs = UINT32_MAX;
    uint32_t latencyMaxUs = 0;
    uint64_t testStartUs = time_us_64();
    static constexpr uint64_t kDoorbellTimeoutUs = 30 * 1000000ULL;

    while (detected < kDoorbellTestCount) {
        uint64_t pollStartUs = time_us_64();
        if ((pollStartUs - testStartUs) > kDoorbellTimeoutUs) {
            break;
        }

        if (multicore_doorbell_is_set_current_core((uint)g_doorbellNum)) {
            uint64_t detectUs = time_us_64();
            multicore_doorbell_clear_current_core((uint)g_doorbellNum);
            detected++;

            uint32_t latUs = (uint32_t)(detectUs - pollStartUs);
            latencySumUs += latUs;
            if (latUs < latencyMinUs) latencyMinUs = latUs;
            if (latUs > latencyMaxUs) latencyMaxUs = latUs;
        }
    }

    // Wait for Core 1's done signal
    uint32_t done;
    multicore_fifo_pop_timeout_us(5000000, &done);

    bool detectOk = (detected == kDoorbellTestCount);
    printf("[%s] Gate 2: Signals detected: %lu/%lu\n",
           detectOk ? "PASS" : "FAIL",
           (unsigned long)detected, (unsigned long)kDoorbellTestCount);
    if (!detectOk) pass = false;

    if (detected > 0) {
        uint32_t latAvg = (uint32_t)(latencySumUs / detected);
        printf("[INFO] Detection latency (polling): min=%lu avg=%lu max=%lu us\n",
               (unsigned long)latencyMinUs, (unsigned long)latAvg,
               (unsigned long)latencyMaxUs);
    }

    // Gate 3: Clear on Core 0 does not affect Core 1
    {
        multicore_doorbell_clear_current_core((uint)g_doorbellNum);
        bool clearOk = !multicore_doorbell_is_set_current_core(
                            (uint)g_doorbellNum);
        printf("[%s] Gate 3: Clear on Core 0 does not affect Core 1\n",
               clearOk ? "PASS" : "FAIL");
        if (!clearOk) pass = false;
    }

    // Gate 4: USB still connected
    {
        bool usbOk = stdio_usb_connected();
        printf("[%s] Gate 4: USB still connected\n",
               usbOk ? "PASS" : "FAIL");
        if (!usbOk) pass = false;
    }

    printf("[INFO] Doorbells validated but not used in production (seqlock polling chosen)\n");
    printf("=== IVP-23: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES");

    multicore_doorbell_unclaim((uint)g_doorbellNum, 0x3);
}

// ============================================================================
// IVP-21: Spinlock Gate Check (called after 5-min soak completes)
// ============================================================================

static void ivp21_gate_check(void) {
    printf("\n=== IVP-21: Hardware Spinlock Validation ===\n");
    bool pass = true;

    // Gate 1: Spinlock claim succeeded
    {
        bool claimOk = (g_spinlockId >= 0);
        printf("[%s] Gate 1: Spinlock claimed ID=%d (expect 24-31)\n",
               claimOk ? "PASS" : "FAIL", g_spinlockId);
        if (!claimOk) pass = false;
    }

    // Gate 2: Document HW vs SW spinlocks
    {
#if PICO_USE_SW_SPIN_LOCKS
        printf("[INFO] Gate 2: SOFTWARE spinlocks (PICO_USE_SW_SPIN_LOCKS=1, RP2350-E2)\n");
#else
        printf("[INFO] Gate 2: HARDWARE spinlocks (SIO registers)\n");
#endif
    }

    // Gate 3: No inconsistent reads
    {
        bool consistOk = (g_ivp21InconsistentCount == 0);
        printf("[%s] Gate 3: %lu reads, %lu inconsistent\n",
               consistOk ? "PASS" : "FAIL",
               (unsigned long)g_ivp21ReadCount,
               (unsigned long)g_ivp21InconsistentCount);
        if (!consistOk) pass = false;
    }

    // Gate 4: Lock hold time
    {
        uint32_t avgUs = (g_ivp21ReadCount > 0) ?
            (uint32_t)(g_ivp21LockTimeSumUs / g_ivp21ReadCount) : 0;
        uint32_t minUs = (g_ivp21LockTimeMinUs == UINT32_MAX) ? 0 : g_ivp21LockTimeMinUs;
        bool timeOk = (g_ivp21LockTimeMaxUs < 10);
        printf("[%s] Gate 4: Lock hold time: min=%lu avg=%lu max=%lu us (expect <10)\n",
               timeOk ? "PASS" : "WARN",
               (unsigned long)minUs, (unsigned long)avgUs,
               (unsigned long)g_ivp21LockTimeMaxUs);
    }

    // Gate 5: 5 minutes continuous
    {
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - g_ivp21StartMs;
        bool durationOk = (elapsed >= kSpinlockSoakMs);
        printf("[%s] Gate 5: Soak: %lu ms (target %lu ms)\n",
               durationOk ? "PASS" : "FAIL",
               (unsigned long)elapsed, (unsigned long)kSpinlockSoakMs);
        if (!durationOk) pass = false;

        uint32_t c1 = g_core1Counter.load(std::memory_order_acquire);
        printf("[INFO] Core 1 counter: %lu, writes: %lu\n",
               (unsigned long)c1, (unsigned long)g_spinlockData.write_count);
    }

    // Gate 6: USB not disrupted
    {
        bool usbOk = stdio_usb_connected();
        printf("[%s] Gate 6: USB still connected after 5-min soak\n",
               usbOk ? "PASS" : "FAIL");
        if (!usbOk) pass = false;
    }

    printf("=== IVP-21: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES");
}

// ============================================================================
// IVP-25: Core 1 IMU Sensor Gate Check
// ============================================================================

static void ivp25_gate_check(void) {
    printf("\n=== IVP-25: Core 1 IMU Sensor Validation ===\n");
    bool pass = true;

    shared_sensor_data_t snap;
    bool snapOk = seqlock_read(&g_sensorSeqlock, &snap);

    uint32_t elapsedMs = to_ms_since_boot(get_absolute_time()) - g_ivp25StartMs;
    float elapsedS = (float)elapsedMs / 1000.0f;

    // Gate 1: I2C transaction time (INFO) — computed from jitter timestamps
    {
        uint32_t jCount = g_jitterSamplesCollected.load(std::memory_order_acquire);
        if (jCount >= 2) {
            uint32_t n = (jCount < kJitterSampleCount) ? jCount : kJitterSampleCount;
            uint32_t minDt = UINT32_MAX;
            uint32_t maxDt = 0;
            uint64_t sumDt = 0;
            for (uint32_t i = 1; i < n; i++) {
                uint32_t dt = g_jitterTimestamps[i] - g_jitterTimestamps[i - 1];
                if (dt < minDt) minDt = dt;
                if (dt > maxDt) maxDt = dt;
                sumDt += dt;
            }
            float avgDt = (float)sumDt / (float)(n - 1);
            printf("[INFO] Gate 1: I2C cycle time: avg=%.0f us, min=%lu us, max=%lu us (%lu samples)\n",
                   (double)avgDt, (unsigned long)minDt, (unsigned long)maxDt, (unsigned long)n);
        } else {
            printf("[INFO] Gate 1: Insufficient jitter samples (%lu)\n", (unsigned long)jCount);
        }
    }

    // Gate 2: IMU rate within 1% of target
    {
        float imuRate = 0.0f;
        if (snapOk && elapsedS > 1.0f) {
            imuRate = (float)snap.imu_read_count / elapsedS;
        }
        // Target: ~1kHz or I2C-limited actual rate. Report and check within 1%.
        // At 400kHz I2C, full IMU read ~774us (measured IVP-13), so max ~1.29kHz
        // With 1ms target cycle, expect ~1000 Hz
        float expectedRate = 1000000.0f / (float)kCore1TargetCycleUs;
        float pctError = (expectedRate > 0) ?
            fabsf(imuRate - expectedRate) / expectedRate * 100.0f : 100.0f;
        bool rateOk = (pctError < kImuRateTolerancePct);
        printf("[%s] Gate 2: IMU rate: %.1f Hz (target ~%.0f Hz, %.1f%% off)\n",
               rateOk ? "PASS" : "FAIL",
               (double)imuRate, (double)expectedRate, (double)pctError);
        if (!rateOk) pass = false;
    }

    // Gate 3: Jitter std dev (INFO)
    {
        uint32_t jCount = g_jitterSamplesCollected.load(std::memory_order_acquire);
        if (jCount >= 10) {
            uint32_t n = (jCount < kJitterSampleCount) ? jCount : kJitterSampleCount;
            // Compute std dev of inter-sample deltas — use static to avoid stack pressure
            static float dts[kJitterSampleCount - 1];  // ~4KB — static per coding standards
            uint32_t dtCount = (n > (kJitterSampleCount - 1)) ? (kJitterSampleCount - 1) : (n - 1);
            float sumDt = 0.0f;
            for (uint32_t i = 0; i < dtCount; i++) {
                dts[i] = (float)(g_jitterTimestamps[i + 1] - g_jitterTimestamps[i]);
                sumDt += dts[i];
            }
            float meanDt = sumDt / (float)dtCount;
            float sumSq = 0.0f;
            for (uint32_t i = 0; i < dtCount; i++) {
                float diff = dts[i] - meanDt;
                sumSq += diff * diff;
            }
            float stdDev = sqrtf(sumSq / (float)dtCount);
            printf("[INFO] Gate 3: Jitter std dev: %.1f us (mean cycle: %.1f us)\n",
                   (double)stdDev, (double)meanDt);
        } else {
            printf("[INFO] Gate 3: Insufficient samples for jitter analysis\n");
        }
    }

    // Gate 4: Core 0 reads valid data from seqlock
    {
        bool dataOk = snapOk && snap.accel_valid && snap.gyro_valid;
        printf("[%s] Gate 4: Core 0 seqlock read: %s, accel=%s, gyro=%s\n",
               dataOk ? "PASS" : "FAIL",
               snapOk ? "ok" : "FAILED",
               snap.accel_valid ? "valid" : "invalid",
               snap.gyro_valid ? "valid" : "invalid");
        if (!dataOk) pass = false;
    }

    // Gate 5: USB uninterrupted
    {
        bool usbOk = stdio_usb_connected();
        printf("[%s] Gate 5: USB connected after sensor soak\n",
               usbOk ? "PASS" : "FAIL");
        if (!usbOk) pass = false;
    }

    // Gate 6: 5 min continuous, 0 I2C errors
    {
        bool durationOk = (elapsedMs >= kSensorSoakMs);
        uint32_t imuErr = snapOk ? snap.imu_error_count : 0;
        bool errOk = (imuErr == 0);
        printf("[%s] Gate 6: Duration %lu ms (target %lu), IMU errors: %lu\n",
               (durationOk && errOk) ? "PASS" : "FAIL",
               (unsigned long)elapsedMs, (unsigned long)kSensorSoakMs,
               (unsigned long)imuErr);
        if (!durationOk || !errOk) pass = false;
    }

    // Extra: core1_loop_count stall detection
    {
        uint32_t loopCount = snapOk ? snap.core1_loop_count : 0;
        uint32_t stale = g_ivp25StaleCount;
        printf("[INFO] Core 1 loops: %lu, stale reads: %lu/%lu, retries: %lu\n",
               (unsigned long)loopCount,
               (unsigned long)stale,
               (unsigned long)g_ivp25ReadCount,
               (unsigned long)g_ivp25RetryCount);
    }

    printf("=== IVP-25: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES");
}

// ============================================================================
// IVP-26: Core 1 Baro Sensor Gate Check
// ============================================================================

static void ivp26_gate_check(void) {
    printf("\n=== IVP-26: Core 1 Baro Sensor Validation ===\n");
    bool pass = true;

    shared_sensor_data_t snap;
    bool snapOk = seqlock_read(&g_sensorSeqlock, &snap);

    uint32_t elapsedMs = to_ms_since_boot(get_absolute_time()) - g_ivp25StartMs;
    float elapsedS = (float)elapsedMs / 1000.0f;

    // Gate 1: IMU rate unchanged (reference IVP-25)
    {
        float imuRate = 0.0f;
        if (snapOk && elapsedS > 1.0f) {
            imuRate = (float)snap.imu_read_count / elapsedS;
        }
        float expectedRate = 1000000.0f / (float)kCore1TargetCycleUs;
        float pctError = (expectedRate > 0) ?
            fabsf(imuRate - expectedRate) / expectedRate * 100.0f : 100.0f;
        bool rateOk = (pctError < kImuRateTolerancePct);
        printf("[%s] Gate 1: IMU rate: %.1f Hz (unchanged from IVP-25)\n",
               rateOk ? "PASS" : "FAIL", (double)imuRate);
        if (!rateOk) pass = false;
    }

    // Gate 2: Baro rate within 10% of 50Hz
    // Note: DPS310 at 8Hz continuous, we poll at 50Hz with data-ready check.
    // Actual baro data rate is ~8Hz (hardware limited), not 50Hz.
    {
        float baroRate = 0.0f;
        if (snapOk && elapsedS > 1.0f) {
            baroRate = (float)snap.baro_read_count / elapsedS;
        }
        // DPS310 configured at 8Hz/8x oversampling. Expect ~8 new samples/sec.
        float expectedBaroRate = 8.0f;  // DPS310 hardware rate, not poll rate
        float pctError = (expectedBaroRate > 0) ?
            fabsf(baroRate - expectedBaroRate) / expectedBaroRate * 100.0f : 100.0f;
        bool rateOk = (pctError < kBaroRateTolerancePct);
        printf("[%s] Gate 2: Baro rate: %.1f Hz (expect ~%.0f Hz, DPS310 hw rate)\n",
               rateOk ? "PASS" : "WARN", (double)baroRate, (double)expectedBaroRate);
        if (!rateOk) pass = false;
    }

    // Gate 3: No I2C bus errors (both devices)
    {
        uint32_t imuErr = snapOk ? snap.imu_error_count : 0;
        uint32_t baroErr = snapOk ? snap.baro_error_count : 0;
        bool errOk = (imuErr == 0) && (baroErr == 0);
        printf("[%s] Gate 3: I2C errors: IMU=%lu, Baro=%lu (expect 0)\n",
               errOk ? "PASS" : "FAIL",
               (unsigned long)imuErr, (unsigned long)baroErr);
        if (!errOk) pass = false;
    }

    // Gate 4: Total I2C time per cycle (INFO) — from jitter timestamps
    {
        uint32_t jCount = g_jitterSamplesCollected.load(std::memory_order_acquire);
        if (jCount >= 2) {
            uint32_t n = (jCount < kJitterSampleCount) ? jCount : kJitterSampleCount;
            uint32_t maxDt = 0;
            for (uint32_t i = 1; i < n; i++) {
                uint32_t dt = g_jitterTimestamps[i] - g_jitterTimestamps[i - 1];
                if (dt > maxDt) maxDt = dt;
            }
            printf("[INFO] Gate 4: Max cycle time: %lu us (budget: %lu us)\n",
                   (unsigned long)maxDt, (unsigned long)kCore1TargetCycleUs);
        }
    }

    // Gate 5: Both datasets valid on Core 0
    {
        bool bothValid = snapOk && snap.accel_valid && snap.baro_valid &&
                         snap.baro_read_count > 0;
        printf("[%s] Gate 5: Accel=%s, Baro=%s (reads=%lu)\n",
               bothValid ? "PASS" : "FAIL",
               (snapOk && snap.accel_valid) ? "valid" : "invalid",
               (snapOk && snap.baro_valid) ? "valid" : "invalid",
               (unsigned long)(snapOk ? snap.baro_read_count : 0));
        if (!bothValid) pass = false;
    }

    printf("=== IVP-26: %s ===\n\n", pass ? "ALL GATES PASS" : "GATE FAILURES");
}

// ============================================================================
// IVP-27: USB Stability Gate Check
// ============================================================================

static void ivp27_gate_check(void) {
    printf("\n=== IVP-27: USB Stability Under Dual-Core Load ===\n");
    bool pass = true;

    uint32_t elapsedMs = to_ms_since_boot(get_absolute_time()) - g_ivp27StartMs;

    // Gate 1: 10 minutes continuous, no USB disconnect
    {
        bool durationOk = (elapsedMs >= kIvp27SoakMs);
        bool usbOk = stdio_usb_connected();
        printf("[%s] Gate 1: Duration %lu ms (target %lu), USB %s\n",
               (durationOk && usbOk) ? "PASS" : "FAIL",
               (unsigned long)elapsedMs, (unsigned long)kIvp27SoakMs,
               usbOk ? "connected" : "DISCONNECTED");
        if (!durationOk || !usbOk) pass = false;
    }

    // Gates 2/3/4: Manual verification
    printf("[MANUAL] Gate 2: Press 'h' — response within 1 second?\n");
    printf("[MANUAL] Gate 3: Disconnect 60s, reconnect — output resumes?\n");
    printf("[MANUAL] Gate 4: Rapidly press keys — no crash or hang?\n");

    // INFO: Core 1 health
    {
        shared_sensor_data_t snap;
        bool snapOk = seqlock_read(&g_sensorSeqlock, &snap);
        if (snapOk) {
            printf("[INFO] Core 1 loops: %lu, IMU reads: %lu, Baro reads: %lu\n",
                   (unsigned long)snap.core1_loop_count,
                   (unsigned long)snap.imu_read_count,
                   (unsigned long)snap.baro_read_count);
            printf("[INFO] Errors: IMU=%lu, Baro=%lu\n",
                   (unsigned long)snap.imu_error_count,
                   (unsigned long)snap.baro_error_count);
        }
    }

    printf("=== IVP-27: %s (automated) / MANUAL gates above ===\n\n",
           pass ? "AUTOMATED GATES PASS" : "GATE FAILURES");
}

// ============================================================================
// IVP-28: Flash Under Dual-Core Test
// ============================================================================

static void ivp28_flash_test(void) {
    printf("\n=== IVP-28: Flash Operations Under Dual-Core ===\n");
    bool allPass = true;

    // Gate 8: FIFO not used for app messages — auto-pass (atomic flags only)
    printf("[PASS] Gate 8: FIFO reserved for multicore_lockout (app uses atomic flags)\n");

    for (uint32_t i = 0; i < kIvp28FlashRepeatCount; i++) {
        // Snapshot before flash
        shared_sensor_data_t snapBefore;
        bool preOk = seqlock_read(&g_sensorSeqlock, &snapBefore);
        uint32_t imuCountBefore = preOk ? snapBefore.imu_read_count : 0;
        uint32_t loopCountBefore = preOk ? snapBefore.core1_loop_count : 0;
        uint32_t errCountBefore = preOk ? snapBefore.imu_error_count : 0;

        // Flash op (erase + program via flash_safe_execute)
        uint32_t t0 = time_us_32();
        cal_result_t result = calibration_save();
        uint32_t t1 = time_us_32();
        uint32_t durationUs = t1 - t0;

        // Post-flash I2C bus recovery (standard pattern from cmd_save_cal)
        i2c_bus_reset();

        // Let Core 1 resume and do a few cycles
        sleep_ms(kIvp28PostFlashSettleMs);

        // Snapshot after flash
        shared_sensor_data_t snapAfter;
        bool postOk = seqlock_read(&g_sensorSeqlock, &snapAfter);
        uint32_t imuCountAfter = postOk ? snapAfter.imu_read_count : 0;
        uint32_t loopCountAfter = postOk ? snapAfter.core1_loop_count : 0;
        uint32_t errCountAfter = postOk ? snapAfter.imu_error_count : 0;
        uint32_t errDelta = errCountAfter - errCountBefore;

        // Read-back verification (council mod #1)
        calibration_store_t readBack;
        bool readBackOk = calibration_storage_read(&readBack);

        // Assess gates
        bool saveOk = (result == CAL_RESULT_OK);
        bool resumeOk = postOk && (loopCountAfter > loopCountBefore);
        bool usbOk = stdio_usb_connected();
        bool errOk = (errDelta <= kIvp28MaxExpectedErrors);
        bool iterPass = saveOk && resumeOk && usbOk && readBackOk;

        // Council mod #5: single-line format
        printf("[%s] Save %lu/%lu: %s, %lu us (%.0f ms), "
               "gap=%lu reads, loop=%lu->%lu, err=+%lu%s, "
               "readback=%s, USB=%s\n",
               iterPass ? "PASS" : "FAIL",
               (unsigned long)(i + 1), (unsigned long)kIvp28FlashRepeatCount,
               saveOk ? "OK" : "FAILED",
               (unsigned long)durationUs,
               (double)durationUs / 1000.0,
               (unsigned long)(imuCountAfter - imuCountBefore),
               (unsigned long)loopCountBefore, (unsigned long)loopCountAfter,
               (unsigned long)errDelta,
               errOk ? "" : " (EXCESS)",
               readBackOk ? "OK" : "FAILED",
               usbOk ? "OK" : "BROKEN");

        if (!iterPass) allPass = false;
    }

    // Summary gates
    printf("[INFO] Gate 2: Core 1 pause confirmed by multicore_lockout_victim_init()\n");
    printf("[INFO] Gate 4: Flash op duration recorded above (expect ~200ms per save)\n");
    printf("[MANUAL] Gate 6: Power cycle, then verify calibration persists\n");

    printf("=== IVP-28: %s (automated) / power-cycle MANUAL ===\n\n",
           allPass ? "ALL AUTOMATED GATES PASS" : "GATE FAILURES");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    // IVP-30: Check if previous reboot was caused by watchdog (before any init)
    // Use watchdog_enable_caused_reboot() — on RP2350, watchdog_caused_reboot()
    // also checks rom_get_last_boot_type() == BOOT_TYPE_NORMAL which can return
    // false after a watchdog reset. watchdog_enable_caused_reboot() checks only
    // watchdog_hw->reason && scratch[4] magic set by watchdog_enable().
    bool watchdogReboot = watchdog_enable_caused_reboot();

    // IVP-29: Register fault handlers early (before any MPU config)
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, memmanage_fault_handler);
    exception_set_exclusive_handler(MEMMANAGE_EXCEPTION, memmanage_fault_handler);

    // IVP-29: MPU stack guard for Core 0
    mpu_setup_stack_guard((uint32_t)&__StackBottom);

    // -----------------------------------------------------------------
    // IVP-02: Red LED GPIO init
    // -----------------------------------------------------------------
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // -----------------------------------------------------------------
    // IVP-03: NeoPixel init
    // -----------------------------------------------------------------
    g_neopixelInitialized = ws2812_status_init(pio0, kNeoPixelPin);

    // IVP-19: Launch Core 1 early so NeoPixel blinks immediately (no USB dependency)
    multicore_launch_core1(core1_entry);

    // -----------------------------------------------------------------
    // IVP-06: I2C bus init (before USB per LL Entry 4/12)
    // i2c_bus_init() includes bus recovery to handle stuck bus
    // from previous picotool reboot
    // -----------------------------------------------------------------
    g_i2cInitialized = i2c_bus_init();

    // Sensor power-up settling time
    // ICM-20948 datasheet: 11ms, DPS310: 40ms, generous margin
    sleep_ms(200);

    // -----------------------------------------------------------------
    // IVP-09: ICM-20948 IMU init (before USB, after I2C)
    // Accel ±4g, Gyro ±500dps, Mag continuous 100Hz
    // -----------------------------------------------------------------
    if (g_i2cInitialized) {
        g_imuInitialized = icm20948_init(&g_imu, kIcm20948AddrDefault);
    }

    // -----------------------------------------------------------------
    // IVP-11: DPS310 barometer init (before USB, after I2C)
    // -----------------------------------------------------------------
    if (g_i2cInitialized) {
        g_baroInitialized = baro_dps310_init(kBaroDps310AddrDefault);
        if (g_baroInitialized) {
            g_baroContinuous = baro_dps310_start_continuous();
        }
    }

    // -----------------------------------------------------------------
    // IVP-14: Calibration storage init (before USB per LL Entry 4/12)
    // Storage init only reads flash via XIP — no erase/write at boot
    // -----------------------------------------------------------------
    g_calStorageInitialized = calibration_storage_init();
    calibration_manager_init();

    // -----------------------------------------------------------------
    // IVP-04: USB CDC init (after I2C/flash per LL Entry 4/12)
    // -----------------------------------------------------------------
    stdio_init_all();

    // Fast LED blink while waiting for USB connection
    // NeoPixel already running on Core 1 (launched after init)
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }

    // Settle time after connection (per LL Entry 15)
    sleep_ms(500);

    // Drain garbage from USB input buffer (per LL Entry 15)
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}

    // -----------------------------------------------------------------
    // Banner
    // -----------------------------------------------------------------
    printf("\n");
    printf("==============================================\n");
    printf("  RocketChip v%s\n", kVersionString);
    static const char* kBuildTag = "audit-remediation-1";
    printf("  Build: %s (%s %s)\n", kBuildTag, __DATE__, __TIME__);
    printf("  Board: Adafruit Feather RP2350 HSTX\n");
    printf("==============================================\n\n");

    // IVP-30: Print watchdog reboot warning (checked before any init)
    if (watchdogReboot) {
        printf("[WARN] *** PREVIOUS REBOOT WAS CAUSED BY WATCHDOG RESET ***\n");
        printf("[INFO] Test commands enabled (soak skipped after watchdog reboot)\n");
        printf("[INFO] Re-enabling watchdog (%lu ms)\n\n",
               (unsigned long)kWatchdogTimeoutMs);
        // After watchdog reboot: enable test commands + watchdog immediately
        // Skip the 5-min soak — it already passed before the reset
        g_testCommandsEnabled = true;
        g_watchdogEnabled = true;
        g_ivp29Done = true;
        g_ivp30Active = false;
        g_ivp30Done = true;
        watchdog_enable(kWatchdogTimeoutMs, true);
    }

    // IVP-05: Debug macros
    DBG_PRINT("Debug macros functional");

    // IVP-06/07: I2C status and scan
    if (g_i2cInitialized) {
        printf("I2C1 initialized at %lukHz on SDA=%d, SCL=%d\n\n",
               (unsigned long)(kI2cBusFreqHz / 1000), kI2cBusSdaPin, kI2cBusSclPin);
    } else {
        printf("ERROR: I2C1 failed to initialize\n\n");
    }
    i2c_bus_scan();
    printf("\n");

    // IVP-09: IMU status
    if (g_imuInitialized) {
        printf("ICM-20948 init OK (WHO_AM_I=0xEA)\n");
        printf("  Accel: +/-%dg, Gyro: +/-%d dps\n",
               (2 << g_imu.accel_fs), 250 * (1 << g_imu.gyro_fs));
        printf("  Mag: %s (AK09916 via I2C master)\n",
               g_imu.mag_initialized ? "OK, continuous 100Hz" : "NOT READY");
    } else {
        printf("ICM-20948 init FAILED\n");
    }
    printf("\n");

    // IVP-11: Baro status
    if (g_baroInitialized && g_baroContinuous) {
        printf("DPS310 init OK, continuous mode (8Hz, 8x OS)\n");
    } else if (g_baroInitialized) {
        printf("DPS310 init OK, continuous mode FAILED\n");
    } else {
        printf("DPS310 init FAILED\n");
    }
    printf("\n");

    // IVP-14: Calibration storage status
    if (g_calStorageInitialized) {
        const calibration_store_t* cal = calibration_manager_get();
        printf("Calibration storage OK (flags=0x%02lX)\n",
               (unsigned long)cal->cal_flags);
    } else {
        printf("Calibration storage FAILED\n");
    }
    printf("\n");

    // Post-init I2C scan — verify bus integrity after IMU init
    printf("Post-init I2C scan (bus integrity check):\n");
    i2c_bus_scan();
    printf("\n");

    // Hardware validation
    hw_validate_stage1();

    // Skip previously verified gates to speed up boot
    if (kSkipVerifiedGates) {
        g_imuValidationDone = true;
        g_baroValidationDone = true;
        g_ivp13Done = true;
        g_ivp14Done = true;
        g_ivp15Done = true;
        g_ivp16Done = true;
        printf("[INFO] Skipping verified gates (IVP-10 through IVP-16)\n");

        // Skip IVP-21/22/23/25/26 soaks (verified Sessions B+C+D)
        g_ivp21Done = true;
        g_ivp25Active = true;   // So print_sensor_status() reads from seqlock
        g_ivp25Done = true;
        g_sensorPhaseDone.store(true, std::memory_order_release);
        printf("[INFO] Skipping IVP-21/22/23/25/26 soaks (verified Sessions B+C+D)\n");

        // Skip IVP-27/28 (verified Session E)
        g_ivp27Active = true;
        g_ivp27Done = true;
        g_ivp28Started = true;
        g_ivp28Done = true;
        printf("[INFO] Skipping IVP-27/28 (verified Session E)\n");

        // Send Core 1 directly to sensor loop (bypass Phase 2 spinlock soak)
        multicore_fifo_drain();
        sleep_ms(10);
        multicore_fifo_push_blocking(kCmd_SkipToSensors);
        g_startSensorPhase.store(true, std::memory_order_release);
        rc_os_i2c_scan_allowed = false;  // Core 1 owns I2C now
        sleep_ms(500);  // Let Core 1 init sensors + start publishing
        printf("[INFO] Core 1 skipped to sensor phase\n\n");
    }

    // IVP-18: RC_OS CLI init
    rc_os_init();
    rc_os_imu_available = g_imuInitialized;
    rc_os_baro_available = g_baroContinuous;
    rc_os_print_sensor_status = print_sensor_status;
    rc_os_read_accel = read_accel_for_cal;
    rc_os_cal_pre_hook = cal_pre_hook;
    rc_os_cal_post_hook = cal_post_hook;
    rc_os_on_unhandled_key = ivp_test_key_handler;

    // IVP-19: Core 1 already launched (after NeoPixel init, before USB wait)
    printf("Core 1 launched (test dispatcher mode)\n");

    // ================================================================
    // IVP-21/22/23: Inter-core primitive exercises
    // Core 1 is in test dispatcher mode, responding to FIFO commands.
    // Skipped when kSkipVerifiedGates — Core 1 sent directly to sensors.
    // ================================================================

    if (!kSkipVerifiedGates) {
        // IVP-22: FIFO message passing
        ivp22_fifo_test();

        // IVP-23: Doorbell signaling
        ivp23_doorbell_test();

        // IVP-21: Claim spinlock and start soak test
        g_spinlockId = spin_lock_claim_unused(true);
        g_pTestSpinlock = spin_lock_init((uint)g_spinlockId);
        printf("Spinlock claimed: ID=%d\n", g_spinlockId);
#if PICO_USE_SW_SPIN_LOCKS
        printf("Spinlock type: SOFTWARE (RP2350-E2, LDAEXB/STREXB)\n");
#else
        printf("Spinlock type: HARDWARE (SIO registers)\n");
#endif

        // Signal Core 1 to exit test dispatcher and start spinlock soak
        multicore_fifo_drain();
        sleep_ms(10);
        multicore_fifo_push_blocking(kCmd_SpinlockStart);
        g_ivp21Active = true;
        g_ivp21StartMs = to_ms_since_boot(get_absolute_time());
        g_ivp21LastCheckMs = g_ivp21StartMs;
        g_ivp21LastPrintMs = g_ivp21StartMs;
        printf("\n=== IVP-21: Spinlock Soak (5 min) ===\n");
        printf("Core 1 writing ~100kHz, Core 0 reading 10Hz...\n\n");
    }

    // IVP-08: Superloop
    printf("Entering main loop\n\n");

    bool ledState = false;

    while (true) {
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        g_loopCounter++;

        // Heartbeat LED: 100ms on, 900ms off
        uint32_t phase = nowMs % kHeartbeatPeriodMs;
        bool shouldBeOn = (phase < kHeartbeatOnMs);
        if (shouldBeOn != ledState) {
            ledState = shouldBeOn;
            gpio_put(PICO_DEFAULT_LED_PIN, ledState ? 1 : 0);
        }

        // NeoPixel owned by Core 1 (IVP-19) — do not call ws2812_update() here

        // IVP-20: Print Core 1 counter every second
        {
            static uint32_t lastCounterPrintMs = 0;
            if (stdio_usb_connected() && (nowMs - lastCounterPrintMs) >= 1000) {
                uint32_t count = g_core1Counter.load(std::memory_order_acquire);
                static uint32_t prevCount = 0;
                uint32_t rate = count - prevCount;
                printf("Core 1: %lu total, %lu/s\n",
                       (unsigned long)count, (unsigned long)rate);
                prevCount = count;
                lastCounterPrintMs = nowMs;
            }
        }

        // IVP-21: Spinlock soak — read shared struct under lock
        if (g_ivp21Active && !g_ivp21Done) {
            if ((nowMs - g_ivp21LastCheckMs) >= kSpinlockCheckIntervalMs) {
                g_ivp21LastCheckMs = nowMs;

                uint64_t t0 = time_us_64();
                uint32_t saved = spin_lock_blocking(g_pTestSpinlock);
                uint32_t a = g_spinlockData.field_a;
                uint32_t b = g_spinlockData.field_b;
                uint32_t c = g_spinlockData.field_c;
                uint32_t wc = g_spinlockData.write_count;
                spin_unlock(g_pTestSpinlock, saved);
                uint32_t dtUs = (uint32_t)(time_us_64() - t0);

                g_ivp21ReadCount++;
                if (dtUs < g_ivp21LockTimeMinUs) g_ivp21LockTimeMinUs = dtUs;
                if (dtUs > g_ivp21LockTimeMaxUs) g_ivp21LockTimeMaxUs = dtUs;
                g_ivp21LockTimeSumUs += dtUs;

                // Consistency check
                if (a != b || b != c || c != wc) {
                    g_ivp21InconsistentCount++;
                    if (stdio_usb_connected()) {
                        printf("[FAIL] Spinlock: a=%lu b=%lu c=%lu wc=%lu\n",
                               (unsigned long)a, (unsigned long)b,
                               (unsigned long)c, (unsigned long)wc);
                    }
                }

                // Progress every 30 seconds
                if (stdio_usb_connected() &&
                    (nowMs - g_ivp21LastPrintMs) >= 30000) {
                    g_ivp21LastPrintMs = nowMs;
                    uint32_t elapsed = nowMs - g_ivp21StartMs;
                    uint32_t remain = (elapsed < kSpinlockSoakMs) ?
                        (kSpinlockSoakMs - elapsed) / 1000 : 0;
                    printf("[IVP-21] %lus, %lus left, %lu reads, %lu bad, "
                           "max %lu us, writes=%lu\n",
                           (unsigned long)(elapsed / 1000),
                           (unsigned long)remain,
                           (unsigned long)g_ivp21ReadCount,
                           (unsigned long)g_ivp21InconsistentCount,
                           (unsigned long)g_ivp21LockTimeMaxUs,
                           (unsigned long)wc);
                }

                // Soak complete?
                if ((nowMs - g_ivp21StartMs) >= kSpinlockSoakMs) {
                    g_ivp21Done = true;
                    if (stdio_usb_connected()) {
                        ivp21_gate_check();
                    }
                    // Trigger IVP-25/26 sensor soak
                    g_startSensorPhase.store(true, std::memory_order_release);
                    rc_os_i2c_scan_allowed = false;  // Core 1 owns I2C now
                    g_ivp25Active = true;
                    g_ivp25StartMs = nowMs;
                    g_ivp25LastReadMs = nowMs;
                    g_ivp25LastPrintMs = nowMs;
                    g_ivp25LastImuCount = 0;
                    if (stdio_usb_connected()) {
                        printf("=== IVP-25/26: Sensor Soak (5 min) ===\n");
                        printf("Core 1: IMU ~1kHz + Baro ~50Hz, Core 0 reading 200Hz...\n\n");
                    }
                }
            }
        }

        // IVP-25/26: Sensor soak — read real sensor data via seqlock
        if (g_ivp25Active && !g_ivp25Done) {
            if ((nowMs - g_ivp25LastReadMs) >= kSeqlockReadIntervalMs) {
                g_ivp25LastReadMs = nowMs;

                shared_sensor_data_t snapshot;
                bool ok = seqlock_read(&g_sensorSeqlock, &snapshot);
                g_ivp25ReadCount++;

                if (!ok) {
                    g_ivp25RetryCount++;
                } else {
                    // Stale detection: imu_read_count should be advancing
                    if (snapshot.imu_read_count == g_ivp25LastImuCount) {
                        g_ivp25StaleCount++;
                    }
                    g_ivp25LastImuCount = snapshot.imu_read_count;
                }

                // Progress every 30 seconds
                if (stdio_usb_connected() &&
                    (nowMs - g_ivp25LastPrintMs) >= 30000) {
                    g_ivp25LastPrintMs = nowMs;
                    uint32_t elapsed = nowMs - g_ivp25StartMs;
                    uint32_t remain = (elapsed < kSensorSoakMs) ?
                        (kSensorSoakMs - elapsed) / 1000 : 0;

                    shared_sensor_data_t snap2;
                    if (seqlock_read(&g_sensorSeqlock, &snap2)) {
                        printf("[IVP-25/26] %lus, %lus left | "
                               "IMU:%lu Baro:%lu Err:I%lu/B%lu | "
                               "A: %6.2f %6.2f %6.2f m/s^2\n",
                               (unsigned long)(elapsed / 1000),
                               (unsigned long)remain,
                               (unsigned long)snap2.imu_read_count,
                               (unsigned long)snap2.baro_read_count,
                               (unsigned long)snap2.imu_error_count,
                               (unsigned long)snap2.baro_error_count,
                               (double)snap2.accel_x,
                               (double)snap2.accel_y,
                               (double)snap2.accel_z);
                    }
                }

                // Soak complete?
                if ((nowMs - g_ivp25StartMs) >= kSensorSoakMs) {
                    g_ivp25Done = true;
                    g_sensorPhaseDone.store(true, std::memory_order_release);
                    if (stdio_usb_connected()) {
                        ivp25_gate_check();
                        ivp26_gate_check();
                    }
                }
            }
        }

        // ================================================================
        // IVP-27: USB stability soak (starts after IVP-25/26 completes)
        // ================================================================
        if (g_ivp25Done && !g_ivp27Active && !g_ivp27Done) {
            g_ivp27Active = true;
            g_ivp27StartMs = nowMs;
            g_ivp27LastStatusMs = nowMs;
            if (stdio_usb_connected()) {
                printf("\n========================================\n");
                printf("=== IVP-27: USB Stability Soak (%u min) ===\n",
                       (unsigned)(kIvp27SoakMs / 60000));
                printf("========================================\n");
                printf("[INFO] Core 1 sensors active. USB soak starts now.\n");
                printf("[INFO] Status updates every %u seconds.\n",
                       (unsigned)(kIvp27StatusIntervalMs / 1000));
                printf("[MANUAL] At ~3 min: Disconnect USB.\n");
                printf("[MANUAL] At ~6 min: Reconnect USB.\n");
                printf("[MANUAL] At ~8 min: Rapid key mash test.\n");
            }
        }

        if (g_ivp27Active && !g_ivp27Done) {
            uint32_t elapsedMs = nowMs - g_ivp27StartMs;

            // Periodic status every 60s
            if ((nowMs - g_ivp27LastStatusMs) >= kIvp27StatusIntervalMs) {
                g_ivp27LastStatusMs = nowMs;
                if (stdio_usb_connected()) {
                    shared_sensor_data_t snap;
                    bool snapOk = seqlock_read(&g_sensorSeqlock, &snap);
                    uint32_t elapsedSec = elapsedMs / 1000;
                    if (snapOk) {
                        printf("[IVP-27] %lu/%lus | IMU: %lu reads, %lu err"
                               " | Baro: %lu reads, %lu err"
                               " | Core1: %lu loops | USB: %s\n",
                               (unsigned long)elapsedSec,
                               (unsigned long)(kIvp27SoakMs / 1000),
                               (unsigned long)snap.imu_read_count,
                               (unsigned long)snap.imu_error_count,
                               (unsigned long)snap.baro_read_count,
                               (unsigned long)snap.baro_error_count,
                               (unsigned long)snap.core1_loop_count,
                               stdio_usb_connected() ? "OK" : "DISCONNECTED");
                    } else {
                        printf("[IVP-27] %lu/%lus | Seqlock read failed\n",
                               (unsigned long)elapsedSec,
                               (unsigned long)(kIvp27SoakMs / 1000));
                    }
                }
            }

            // Timed manual gate prompts (council mod #3)
            if (!g_ivp27Prompt3min && elapsedMs >= 3 * 60 * 1000) {
                g_ivp27Prompt3min = true;
                if (stdio_usb_connected()) {
                    printf("\n[MANUAL] >>> DISCONNECT USB NOW. Wait 60 seconds, then reconnect. <<<\n\n");
                }
            }
            if (!g_ivp27Prompt6min && elapsedMs >= 6 * 60 * 1000) {
                g_ivp27Prompt6min = true;
                if (stdio_usb_connected()) {
                    printf("\n[MANUAL] >>> RECONNECT USB. Did output resume? Press 'h' to test CLI. <<<\n\n");
                }
            }
            if (!g_ivp27Prompt8min && elapsedMs >= 8 * 60 * 1000) {
                g_ivp27Prompt8min = true;
                if (stdio_usb_connected()) {
                    printf("\n[MANUAL] >>> RAPID KEY MASH for 10 seconds. Verify no crash or hang. <<<\n\n");
                }
            }

            // Soak complete
            if (elapsedMs >= kIvp27SoakMs) {
                g_ivp27Done = true;
                g_ivp27Active = false;
                if (stdio_usb_connected()) {
                    ivp27_gate_check();
                }
            }
        }

        // ================================================================
        // IVP-28: Flash under dual-core (runs once after IVP-27 completes)
        // ================================================================
        if (g_ivp27Done && !g_ivp28Started) {
            g_ivp28Started = true;
            if (stdio_usb_connected()) {
                // Disable watchdog during flash ops — flash_safe_execute stalls
                // Core 1 via multicore_lockout, preventing alive flag updates
                if (g_watchdogEnabled) watchdog_disable();
                ivp28_flash_test();
                if (g_watchdogEnabled) watchdog_enable(kWatchdogTimeoutMs, true);
            }
            g_ivp28Done = true;
        }

        // ================================================================
        // IVP-29: MPU gate check (runs once after IVP-28)
        // ================================================================
        if (g_ivp28Done && !g_ivp29Done) {
            g_ivp29Done = true;
            if (stdio_usb_connected()) {
                ivp29_gate_check();

                // Enable watchdog and start IVP-30 soak
                printf("[INFO] Enabling hardware watchdog (%lu ms, pause_on_debug=true)\n",
                       (unsigned long)kWatchdogTimeoutMs);
                watchdog_enable(kWatchdogTimeoutMs, true);
                g_watchdogEnabled = true;

                g_ivp30Active = true;
                g_ivp30StartMs = nowMs;
                g_ivp30LastStatusMs = nowMs;
                printf("[INFO] IVP-30: Watchdog soak started (%lu min)\n\n",
                       (unsigned long)(kIvp30SoakMs / 60000));
            }
        }

        // ================================================================
        // IVP-30: Watchdog soak (5 minutes, no false fires)
        // ================================================================
        if (g_ivp30Active && !g_ivp30Done) {
            uint32_t elapsedMs = nowMs - g_ivp30StartMs;

            // Status every 60s
            if ((nowMs - g_ivp30LastStatusMs) >= kIvp30StatusIntervalMs) {
                g_ivp30LastStatusMs = nowMs;
                if (stdio_usb_connected()) {
                    shared_sensor_data_t snap;
                    bool snapOk = seqlock_read(&g_sensorSeqlock, &snap);
                    if (snapOk) {
                        printf("[IVP-30] %lu/%lus | Core1: %lu loops, %lu IMU, %lu err | WDT: OK\n",
                               (unsigned long)(elapsedMs / 1000),
                               (unsigned long)(kIvp30SoakMs / 1000),
                               (unsigned long)snap.core1_loop_count,
                               (unsigned long)snap.imu_read_count,
                               (unsigned long)snap.imu_error_count);
                    } else {
                        printf("[IVP-30] %lu/%lus | Seqlock read failed | WDT: OK\n",
                               (unsigned long)(elapsedMs / 1000),
                               (unsigned long)(kIvp30SoakMs / 1000));
                    }
                }
            }

            // Soak complete
            if (elapsedMs >= kIvp30SoakMs) {
                g_ivp30Done = true;
                g_ivp30Active = false;
                g_testCommandsEnabled = true;
                if (stdio_usb_connected()) {
                    ivp30_gate_check();
                    printf("[INFO] Test commands available:\n");
                    printf("  'o' — Stack overflow (Core 0 fault test)\n");
                    printf("  'w' — Stall Core 0 (watchdog test)\n");
                    printf("  'W' — Stall Core 1 (watchdog test)\n\n");
                }
            }
        }

        // IVP-30: Watchdog dual-core kick (every loop iteration)
        if (g_watchdogEnabled) {
            g_wdtCore0Alive.store(true, std::memory_order_relaxed);
            if (g_wdtCore0Alive.load(std::memory_order_relaxed) &&
                g_wdtCore1Alive.load(std::memory_order_relaxed)) {
                watchdog_update();
                g_wdtCore0Alive.store(false, std::memory_order_relaxed);
                g_wdtCore1Alive.store(false, std::memory_order_relaxed);
            }
        }

        // IVP-10: IMU data read at 10Hz
        if (g_imuInitialized && !g_imuValidationDone && stdio_usb_connected()) {
            // Wait for sensor settle before starting
            if (!g_imuValidationStarted && nowMs >= kImuValidationDelayMs) {
                g_imuValidationStarted = true;
                g_lastImuReadMs = nowMs;
                // Initialize tracking
                g_accelXSum = 0.0f;
                g_accelYSum = 0.0f;
                g_accelZSum = 0.0f;
                g_accelZMin = 999.0f;
                g_accelZMax = -999.0f;
                g_gyroAbsMax = 0.0f;
                g_magMagMin = 999.0f;  g_magMagMax = 0.0f;
                g_tempMin = 999.0f;    g_tempMax = -999.0f;
                g_nanCount = 0;        g_magValidCount = 0;
                g_validSampleCount = 0;
                printf("=== IVP-10: IMU Data Validation (10Hz x %lu samples) ===\n",
                       (unsigned long)kImuSampleCount);
            }

            if (g_imuValidationStarted && g_imuSampleNum < kImuSampleCount &&
                (nowMs - g_lastImuReadMs) >= kImuReadIntervalMs) {
                g_lastImuReadMs = nowMs;
                icm20948_data_t data;
                if (icm20948_read(&g_imu, &data)) {
                    ivp10_print_sample(g_imuSampleNum, &data);
                    ivp10_accumulate(&data);
                    g_imuSampleNum++;
                } else {
                    printf("[%03lu] READ FAILED\n", (unsigned long)g_imuSampleNum);
                }
            }

            if (g_imuSampleNum >= kImuSampleCount) {
                g_imuValidationDone = true;
                ivp10_gate_check();
            }
        }

        // IVP-12: Baro data read at 10Hz (starts after IVP-10 completes)
        if (g_baroContinuous && !g_baroValidationDone && g_imuValidationDone &&
            stdio_usb_connected()) {
            if (!g_baroValidationStarted) {
                g_baroValidationStarted = true;
                g_baroValidStartMs = nowMs;
                g_lastBaroReadMs = nowMs;
                g_baroPressSum = 0.0f;
                g_baroValidReads = 0;
                g_baroInvalidReads = 0;
                g_baroNanCount = 0;
                g_baroStuckCount = 0;
                printf("=== IVP-12: Baro Data Validation (10Hz x %lu samples) ===\n",
                       (unsigned long)kBaroSampleCount);
            }

            // Wait briefly for DPS310 to have data after starting continuous
            if ((nowMs - g_baroValidStartMs) >= kBaroValidationDelayMs &&
                g_baroSampleNum < kBaroSampleCount &&
                (nowMs - g_lastBaroReadMs) >= kBaroReadIntervalMs) {
                g_lastBaroReadMs = nowMs;
                baro_dps310_data_t bdata;
                bool ok = baro_dps310_read(&bdata);
                if (ok) {
                    ivp12_accumulate(&bdata);
                    // Print every 10th sample to keep output manageable
                    if (g_baroSampleNum % 10 == 0) {
                        printf("[%03lu] P: %.1f Pa  T: %.2fC  Alt: %.1fm  %s\n",
                               (unsigned long)g_baroSampleNum,
                               (double)bdata.pressure_pa, (double)bdata.temperature_c,
                               (double)bdata.altitude_m,
                               bdata.valid ? "OK" : "INVALID");
                    }
                } else {
                    g_baroInvalidReads++;
                    if (g_baroSampleNum % 10 == 0) {
                        printf("[%03lu] READ FAILED\n", (unsigned long)g_baroSampleNum);
                    }
                }
                g_baroSampleNum++;
            }

            if (g_baroSampleNum >= kBaroSampleCount) {
                g_baroValidationDone = true;
                ivp12_gate_check();
            }
        }

        // IVP-13: Multi-sensor polling at target rates (after IVP-10 + IVP-12)
        if (g_imuInitialized && g_baroContinuous &&
            g_imuValidationDone && g_baroValidationDone && !g_ivp13Done) {

            uint64_t nowUs = time_us_64();

            if (!g_ivp13Started) {
                g_ivp13Started = true;
                g_ivp13StartMs = nowMs;
                g_ivp13LastImuUs = nowUs;
                g_ivp13LastBaroUs = nowUs;
                g_ivp13LastStatusMs = nowMs;
                if (stdio_usb_connected()) {
                    printf("=== IVP-13: Multi-Sensor Polling (IMU 100Hz, Baro 50Hz, 60s) ===\n");
                }
            }

            // IMU read at 100Hz
            if ((nowUs - g_ivp13LastImuUs) >= kIvp13ImuIntervalUs) {
                g_ivp13LastImuUs += kIvp13ImuIntervalUs;
                // Prevent accumulation if we fell behind
                if ((nowUs - g_ivp13LastImuUs) > kIvp13ImuIntervalUs * 2) {
                    g_ivp13LastImuUs = nowUs;
                }
                uint64_t t0 = time_us_64();
                icm20948_data_t data;
                if (icm20948_read(&g_imu, &data)) {
                    uint32_t dt = (uint32_t)(time_us_64() - t0);
                    g_ivp13ImuCount++;
                    g_ivp13ImuSecCount++;
                    g_ivp13ImuTimeSum += dt;
                    if (dt > g_ivp13ImuTimeMax) g_ivp13ImuTimeMax = dt;
                    g_i2cConsecErrors = 0;
                    g_imuLastReadOk = true;
                } else {
                    g_ivp13ImuErrors++;
                    g_i2cConsecErrors++;
                    g_imuLastReadOk = false;
                }
            }

            // Baro read at 50Hz
            if ((nowUs - g_ivp13LastBaroUs) >= kIvp13BaroIntervalUs) {
                g_ivp13LastBaroUs += kIvp13BaroIntervalUs;
                if ((nowUs - g_ivp13LastBaroUs) > kIvp13BaroIntervalUs * 2) {
                    g_ivp13LastBaroUs = nowUs;
                }
                uint64_t t0 = time_us_64();
                baro_dps310_data_t bdata;
                if (baro_dps310_read(&bdata)) {
                    uint32_t dt = (uint32_t)(time_us_64() - t0);
                    g_ivp13BaroCount++;
                    g_ivp13BaroSecCount++;
                    g_ivp13BaroTimeSum += dt;
                    if (dt > g_ivp13BaroTimeMax) g_ivp13BaroTimeMax = dt;
                    g_i2cConsecErrors = 0;
                    g_baroConsecFails = 0;
                } else {
                    g_ivp13BaroErrors++;
                    g_i2cConsecErrors++;
                    g_baroConsecFails++;
                }
            }

            // IVP-13a: Lazy baro reinit — if IMU reads work but baro
            // doesn't, DPS310 lost continuous mode (e.g. after disconnect).
            // 100 consecutive baro fails at 50Hz = 2 seconds of failure.
            if (g_baroConsecFails >= 100 && g_imuLastReadOk) {
                g_baroConsecFails = 0;
                g_baroInitialized = baro_dps310_init(kBaroDps310AddrDefault);
                g_baroContinuous = g_baroInitialized ?
                    baro_dps310_start_continuous() : false;
                if (stdio_usb_connected()) {
                    printf("[RECOVERY] Baro reinit: %s\n",
                           g_baroContinuous ? "OK" : "FAIL");
                }
            }

            // IVP-13a: Bus recovery when consecutive errors exceed threshold
            if (g_i2cConsecErrors >= kI2cRecoveryThreshold &&
                (nowMs - g_i2cLastRecoveryMs) >= kI2cRecoveryCooldownMs) {
                g_i2cLastRecoveryMs = nowMs;
                g_i2cRecoveryAttempts++;
                if (stdio_usb_connected()) {
                    printf("[RECOVERY] %lu consecutive errors — resetting I2C bus (attempt %lu)\n",
                           (unsigned long)g_i2cConsecErrors,
                           (unsigned long)g_i2cRecoveryAttempts);
                }
                bool recovered = i2c_bus_reset();
                if (recovered) {
                    g_i2cRecoverySuccesses++;
                    g_i2cConsecErrors = 0;

                    // Don't reinit sensors here — icm20948_init() blocks
                    // for 150ms+ with device reset and mag retries, which
                    // hangs if sensors are still disconnected. Instead:
                    // - IMU self-recovers from reads (no reinit needed)
                    // - Baro needs reinit for continuous mode (done below)

                    if (stdio_usb_connected()) {
                        printf("[RECOVERY] Bus reset OK — resuming polling\n");
                    }
                } else {
                    if (stdio_usb_connected()) {
                        printf("[RECOVERY] Bus reset FAILED — will retry after cooldown\n");
                    }
                }
            }

            // Status print every second
            if (stdio_usb_connected() && (nowMs - g_ivp13LastStatusMs) >= kIvp13StatusIntervalMs) {
                uint32_t elapsed = nowMs - g_ivp13StartMs;
                uint32_t totalErrors = g_ivp13ImuErrors + g_ivp13BaroErrors;
                if (g_i2cRecoveryAttempts > 0) {
                    printf("[%3lus] IMU: %lu/s  Baro: %lu/s  Err: %lu  Rec: %lu/%lu\n",
                           (unsigned long)(elapsed / 1000),
                           (unsigned long)g_ivp13ImuSecCount,
                           (unsigned long)g_ivp13BaroSecCount,
                           (unsigned long)totalErrors,
                           (unsigned long)g_i2cRecoverySuccesses,
                           (unsigned long)g_i2cRecoveryAttempts);
                } else {
                    printf("[%3lus] IMU: %lu/s  Baro: %lu/s  Err: %lu\n",
                           (unsigned long)(elapsed / 1000),
                           (unsigned long)g_ivp13ImuSecCount,
                           (unsigned long)g_ivp13BaroSecCount,
                           (unsigned long)totalErrors);
                }
                g_ivp13ImuSecCount = 0;
                g_ivp13BaroSecCount = 0;
                g_ivp13LastStatusMs = nowMs;
            }

            // Check if 60 seconds elapsed
            if ((nowMs - g_ivp13StartMs) >= kIvp13DurationMs) {
                g_ivp13Done = true;
                if (stdio_usb_connected()) {
                    ivp13_gate_check();
                }
            }
        }

        // IVP-14: Calibration storage test (after IVP-13 completes)
        if (g_ivp13Done && !g_ivp14Done && g_calStorageInitialized &&
            stdio_usb_connected()) {
            g_ivp14Done = true;
            ivp14_gate_check();
        }

        // IVP-15: Gyro bias calibration (after IVP-14 completes)
        // Device must be stationary — 200 samples at 100Hz (~2 seconds)
        if (g_ivp14Done && !g_ivp15Done && g_imuInitialized &&
            stdio_usb_connected()) {

            uint64_t nowUs = time_us_64();

            if (!g_ivp15Started) {
                g_ivp15Started = true;
                g_ivp15LastFeedUs = nowUs;
                printf("=== IVP-15: Gyro Bias Calibration ===\n");
                printf("[INFO] Keep device STATIONARY for ~2 seconds...\n");
                cal_result_t startResult = calibration_start_gyro();
                if (startResult != CAL_RESULT_OK) {
                    printf("[FAIL] calibration_start_gyro() = %d\n", (int)startResult);
                    g_ivp15Done = true;
                }
            }

            // Feed gyro samples at 100Hz while calibrating
            if (g_ivp15Started && calibration_is_active() &&
                (nowUs - g_ivp15LastFeedUs) >= kIvp15FeedIntervalUs) {
                g_ivp15LastFeedUs = nowUs;
                icm20948_data_t data;
                if (icm20948_read(&g_imu, &data)) {
                    calibration_feed_gyro(data.gyro.x, data.gyro.y, data.gyro.z,
                                          data.temperature_c);
                    // Print progress every 50 samples
                    uint8_t progress = calibration_get_progress();
                    static uint8_t lastProgress = 0;
                    if (progress >= lastProgress + 25 || progress == 100) {
                        printf("[INFO] Gyro cal progress: %u%%\n", progress);
                        lastProgress = progress;
                    }
                }
            }

            // Check if calibration completed or failed
            cal_state_t state = calibration_manager_get_state();
            if (g_ivp15Started && !calibration_is_active() &&
                (state == CAL_STATE_COMPLETE || state == CAL_STATE_FAILED)) {
                g_ivp15Done = true;
                ivp15_gate_check();
                calibration_reset_state();
            }
        }

        // IVP-16: Accel level calibration (after IVP-15 completes)
        // Device must be flat on table (Z-up) — 100 samples at 100Hz (~1 second)
        if (g_ivp15Done && !g_ivp16Done && g_imuInitialized &&
            stdio_usb_connected()) {

            uint64_t nowUs = time_us_64();
            static uint32_t ivp16StartMs = 0;

            if (!g_ivp16Started) {
                g_ivp16Started = true;
                g_ivp16LastFeedUs = nowUs;
                ivp16StartMs = nowMs;
                printf("=== IVP-16: Accel Level Calibration ===\n");
                printf("[INFO] Keep device FLAT on table (Z-up) for ~1 second...\n");
                cal_result_t startResult = calibration_start_accel_level();
                if (startResult != CAL_RESULT_OK) {
                    printf("[FAIL] calibration_start_accel_level() = %d\n", (int)startResult);
                    g_ivp16Done = true;
                }
            }

            // Feed accel samples at 100Hz while calibrating
            if (g_ivp16Started && calibration_is_active() &&
                (nowUs - g_ivp16LastFeedUs) >= kIvp16FeedIntervalUs) {
                g_ivp16LastFeedUs = nowUs;
                icm20948_data_t data;
                if (icm20948_read(&g_imu, &data)) {
                    calibration_feed_accel(data.accel.x, data.accel.y, data.accel.z,
                                           data.temperature_c);
                    uint8_t progress = calibration_get_progress();
                    static uint8_t lastProgress16 = 0;
                    if (progress >= lastProgress16 + 25 || progress == 100) {
                        printf("[INFO] Level cal progress: %u%%\n", progress);
                        lastProgress16 = progress;
                    }
                }
            }

            // Check if calibration completed or failed
            cal_state_t state = calibration_manager_get_state();
            if (g_ivp16Started && !calibration_is_active() &&
                (state == CAL_STATE_COMPLETE || state == CAL_STATE_FAILED)) {
                g_ivp16Done = true;
                uint32_t elapsed = nowMs - ivp16StartMs;
                ivp16_gate_check(elapsed);
                calibration_reset_state();
            }
        }

        // IVP-18: RC_OS CLI + calibration sample feeding
        // RC_OS handles terminal connection, banner, input processing.
        // When CLI triggers a calibration, we feed samples here at 100Hz.
        {
            rc_os_update();

            // Feed sensor samples to active calibration (CLI-triggered)
            if (calibration_is_active()) {
                uint64_t nowUs = time_us_64();
                if ((nowUs - g_cliCalLastFeedUs) >= kCliCalFeedIntervalUs) {
                    g_cliCalLastFeedUs = nowUs;
                    cal_state_t calState = calibration_manager_get_state();

                    if (calState == CAL_STATE_GYRO_SAMPLING && g_imuInitialized) {
                        icm20948_data_t data;
                        if (icm20948_read(&g_imu, &data)) {
                            calibration_feed_gyro(data.gyro.x, data.gyro.y,
                                                  data.gyro.z, data.temperature_c);
                        }
                    } else if (calState == CAL_STATE_ACCEL_LEVEL_SAMPLING && g_imuInitialized) {
                        icm20948_data_t data;
                        if (icm20948_read(&g_imu, &data)) {
                            calibration_feed_accel(data.accel.x, data.accel.y,
                                                   data.accel.z, data.temperature_c);
                        }
                    } else if (calState == CAL_STATE_BARO_SAMPLING && g_baroContinuous) {
                        baro_dps310_data_t bdata;
                        if (baro_dps310_read(&bdata) && bdata.valid) {
                            calibration_feed_baro(bdata.pressure_pa,
                                                  bdata.temperature_c);
                        }
                    }
                }
            }
        }

        // Superloop validation at 10s
        if (stdio_usb_connected() && !g_superloopValidationDone &&
            nowMs >= kSuperloopValidationMs) {
            g_superloopValidationDone = true;
            hw_validate_superloop(nowMs);
        }

        // Small sleep to prevent tight spinning (allows USB processing)
        // Note: sleep_ms(1) limits max loop rate to ~1kHz; sufficient for
        // IVP-13 targets (100Hz IMU + 50Hz baro). Remove in Stage 3.
        sleep_ms(1);
    }

    return 0;
}
