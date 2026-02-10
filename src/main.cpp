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
#include "pico/time.h"
#include "hardware/gpio.h"
#include "drivers/ws2812_status.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include "drivers/baro_dps310.h"
#include "drivers/gps_pa1010d.h"
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
static const char* kBuildTag = "recover-fix-2";

// Heartbeat: 100ms on, 900ms off
static constexpr uint32_t kHeartbeatOnMs = 100;
static constexpr uint32_t kHeartbeatOffMs = 900;
static constexpr uint32_t kHeartbeatPeriodMs = kHeartbeatOnMs + kHeartbeatOffMs;

// Core 1 sensor loop timing
static constexpr uint32_t kCore1TargetCycleUs = 1000;           // ~1kHz target
static constexpr uint32_t kCore1BaroDivider = 20;               // Baro at ~50Hz
static constexpr uint32_t kCore1GpsDivider = 100;               // GPS at ~10Hz
static constexpr uint32_t kGpsMinIntervalUs = 2000;             // MT3333 buffer refill time
static constexpr uint32_t kCore1ConsecFailMax = 10;             // I2C recovery threshold

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

// Baro reinit threshold (consecutive failures)
static constexpr uint32_t kBaroReinitThreshold = 100;

// Seqlock parameters
static constexpr uint32_t kSeqlockMaxRetries = 4;

// NeoPixel timing (Core 1 sensor phase)
static constexpr uint32_t kNeoToggleSoakMs     = 250;           // ~2Hz blink
static constexpr uint32_t kSensorPhaseTimeoutMs = 300000;       // 5 min — switch to magenta

// CLI calibration feed interval
static constexpr uint32_t kCliCalFeedIntervalUs = 10000;        // 100Hz

// ============================================================================
// Global State
// ============================================================================

static bool g_neopixelInitialized = false;
static bool g_i2cInitialized = false;
static bool g_imuInitialized = false;
static bool g_baroInitialized = false;
static bool g_baroContinuous = false;
static bool g_gpsInitialized = false;
static bool g_gpsOnI2C = false;       // True when GPS detected at I2C address 0x10

static uint32_t g_loopCounter = 0;

// Shared sensor data struct (per SEQLOCK_DESIGN.md, council-approved)
// All values calibration-applied, body frame, SI units. 128 bytes in SRAM.
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

    // Health (16 bytes)
    uint32_t imu_error_count;
    uint32_t baro_error_count;
    uint32_t gps_error_count;
    uint32_t core1_loop_count;              // For watchdog/stall detection
};

static_assert(sizeof(shared_sensor_data_t) == 128, "Struct size changed — update SEQLOCK_DESIGN.md");
static_assert(sizeof(shared_sensor_data_t) % 4 == 0, "Struct must be 4-byte aligned for memcpy");

// Seqlock wrapper — single buffer with sequence counter
struct sensor_seqlock_t {
    std::atomic<uint32_t> sequence{0};      // Odd = write in progress
    shared_sensor_data_t data;
};

static sensor_seqlock_t g_sensorSeqlock;

// Cross-core signaling (atomic flags — FIFO reserved by multicore_lockout)
static std::atomic<bool> g_startSensorPhase{false};
static std::atomic<bool> g_sensorPhaseDone{false};
static std::atomic<bool> g_calReloadPending{false};
static std::atomic<bool> g_core1PauseI2C{false};
static std::atomic<bool> g_core1I2CPaused{false};

// Dual-core watchdog kick flags — std::atomic per MULTICORE_RULES.md
// volatile is NOT sufficient for cross-core visibility on ARM (no hardware barrier)
static std::atomic<bool> g_wdtCore0Alive{false};
static std::atomic<bool> g_wdtCore1Alive{false};
static bool g_watchdogEnabled = false;

// Sensor phase active — set by Core 0 when Core 1 enters sensor loop.
// Read only by Core 0 (cal hooks, print_sensor_status). Plain bool is correct —
// single-core write/read, no cross-core visibility needed.
static bool g_sensorPhaseActive = false;

// Calibration storage state
static bool g_calStorageInitialized = false;

// CLI feed interval for calibrations triggered via menu
static uint64_t g_cliCalLastFeedUs = 0;

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
        if (seq1 & 1U) {
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
    io_rw_32 *gpio_out = &sio_hw->gpio_out;
    const uint32_t led_mask = 1U << PICO_DEFAULT_LED_PIN;

    while (true) {
        for (uint8_t i = 0; i < kFaultFastBlinks; i++) {
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
// MPU Stack Guard Setup (per-core, PMSAv8)
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

    // NOLINTBEGIN(readability-magic-numbers) — PMSAv8 MPU register bit fields per ARMv8-M Architecture Reference Manual
    // Region 0: Stack guard (no access, execute-never)
    // PMSAv8 RBAR: [31:5]=BASE, [4:3]=SH(0=non-shareable), [2:1]=AP(0=priv no-access), [0]=XN(1)
    mpu_hw->rnr = 0;
    mpu_hw->rbar = (stack_bottom & ~0x1FU)
                  | (0U << 3)   // SH: Non-shareable
                  | (0U << 1)   // AP: Privileged no-access
                  | (1U << 0);  // XN: Execute-never

    // PMSAv8 RLAR: [31:5]=LIMIT, [3:1]=ATTRINDX(0), [0]=EN(1)
    mpu_hw->rlar = ((stack_bottom + kMpuGuardSizeBytes - 1) & ~0x1FU)
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

static void core1_read_imu(shared_sensor_data_t* localData,
                            calibration_store_t* localCal,
                            uint32_t* imuConsecFail) {
    icm20948_data_t imuData;
    bool imuOk = icm20948_read(&g_imu, &imuData);
    if (imuOk) {
        *imuConsecFail = 0;
        float ax, ay, az, gx, gy, gz;
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
            localData->mag_x = imuData.mag.x;
            localData->mag_y = imuData.mag.y;
            localData->mag_z = imuData.mag.z;
            localData->mag_valid = true;
            localData->mag_read_count++;
        }
    } else {
        (*imuConsecFail)++;
        localData->accel_valid = false;
        localData->gyro_valid = false;
        localData->imu_error_count++;
        if (*imuConsecFail >= kCore1ConsecFailMax) {
            i2c_bus_recover();
            *imuConsecFail = 0;
        }
    }
}

static void core1_read_baro(shared_sensor_data_t* localData) {
    uint8_t measCfg = 0;
    if (i2c_bus_read_reg(kI2cAddrDps310, kDps310MeasCfgReg, &measCfg) == 0 &&
        (measCfg & (kDps310PrsRdy | kDps310TmpRdy)) == (kDps310PrsRdy | kDps310TmpRdy)) {
        baro_dps310_data_t baroData;
        if (baro_dps310_read(&baroData) && baroData.valid) {
            localData->pressure_pa = baroData.pressure_pa;
            localData->baro_temperature_c = baroData.temperature_c;
            localData->baro_timestamp_us = time_us_32();
            localData->baro_read_count++;
            localData->baro_valid = true;
        } else {
            localData->baro_error_count++;
        }
    }
}

static void core1_read_gps(shared_sensor_data_t* localData,
                            uint32_t* lastGpsReadUs) {
    uint32_t gpsNowUs = time_us_32();
    if (gpsNowUs - *lastGpsReadUs < kGpsMinIntervalUs) {
        return;
    }

    *lastGpsReadUs = gpsNowUs;
    bool parsed = gps_pa1010d_update();
    if (g_gpsOnI2C) {
        busy_wait_us(kGpsSdaSettleUs);  // SDA settling delay (LL Entry 24)
    }

    gps_pa1010d_data_t gpsData;
    gps_pa1010d_get_data(&gpsData);

    double lat = gpsData.latitude;
    double lon = gpsData.longitude;
    if (lat > kLatMaxDeg) { lat = kLatMaxDeg; }
    if (lat < kLatMinDeg) { lat = kLatMinDeg; }
    if (lon > kLonMaxDeg) { lon = kLonMaxDeg; }
    if (lon < kLonMinDeg) { lon = kLonMinDeg; }

    localData->gps_lat_1e7 = static_cast<int32_t>(lat * kGpsCoordScale);
    localData->gps_lon_1e7 = static_cast<int32_t>(lon * kGpsCoordScale);
    localData->gps_alt_msl_m = gpsData.altitudeM;
    localData->gps_ground_speed_mps = gpsData.speedMps;
    localData->gps_course_deg = gpsData.courseDeg;
    localData->gps_timestamp_us = gpsNowUs;
    localData->gps_read_count++;
    localData->gps_fix_type = static_cast<uint8_t>(gpsData.fix);
    localData->gps_satellites = gpsData.satellites;
    localData->gps_valid = gpsData.valid;
    localData->gps_gga_fix = gpsData.ggaFix;
    localData->gps_gsa_fix_mode = gpsData.gsaFixMode;
    localData->gps_rmc_valid = gpsData.rmcValid;

    if (!parsed) {
        localData->gps_error_count++;
    }
}

// ============================================================================
// Core 1: NeoPixel State Update
// ============================================================================

static void core1_neopixel_update(shared_sensor_data_t* localData,
                                   uint32_t nowMs, uint32_t sensorPhaseStartMs,
                                   uint32_t* lastNeoMs, bool* neoState) {
    if (nowMs - *lastNeoMs < kNeoToggleSoakMs) {
        return;
    }

    *lastNeoMs = nowMs;
    *neoState = !*neoState;

    if ((nowMs - sensorPhaseStartMs) >= kSensorPhaseTimeoutMs) {
        ws2812_set_mode(WS2812_MODE_SOLID, kColorMagenta);
    } else if (g_gpsInitialized) {
        if (localData->gps_valid) {
            ws2812_set_mode(WS2812_MODE_SOLID, kColorGreen);
        } else if (localData->gps_read_count > 0) {
            ws2812_set_mode(WS2812_MODE_SOLID,
                *neoState ? kColorYellow : kColorOff);
        } else {
            ws2812_set_mode(WS2812_MODE_SOLID,
                *neoState ? kColorBlue : kColorCyan);
        }
    } else if (g_sensorPhaseDone.load(std::memory_order_acquire)) {
        ws2812_set_mode(WS2812_MODE_SOLID, kColorGreen);
    } else {
        ws2812_set_mode(WS2812_MODE_SOLID,
            *neoState ? kColorBlue : kColorCyan);
    }
    ws2812_update();
}

// ============================================================================
// Core 1: Sensor Loop
// ============================================================================
// INVARIANT: Core 0 must NOT call icm20948_*() or baro_dps310_*() unless
// g_core1I2CPaused == true. Core 1 owns I2C during this phase.
// Seqlock write stays here — read helpers write into localData by pointer.

static void core1_sensor_loop() {
    calibration_store_t localCal;
    if (!calibration_load_into(&localCal)) {
        memset(&localCal, 0, sizeof(localCal));
        localCal.accel.scale.x = 1.0F;
        localCal.accel.scale.y = 1.0F;
        localCal.accel.scale.z = 1.0F;
        // NOLINTBEGIN(readability-magic-numbers) — 3x3 identity matrix diagonal indices
        localCal.board_rotation.m[0] = 1.0F;
        localCal.board_rotation.m[4] = 1.0F;
        localCal.board_rotation.m[8] = 1.0F;
        // NOLINTEND(readability-magic-numbers)
    }

    shared_sensor_data_t localData = {};
    uint32_t loopCount = 0;
    uint32_t baroCycle = 0;
    uint32_t imuConsecFail = 0;
    uint32_t gpsCycle = 0;
    uint32_t lastGpsReadUs = 0;
    uint32_t sensorPhaseStartMs = to_ms_since_boot(get_absolute_time());
    uint32_t lastNeoMs = sensorPhaseStartMs;
    bool neoState = false;

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
            ws2812_set_mode(WS2812_MODE_SOLID, kColorOrange);
            ws2812_update();
            while (g_core1PauseI2C.load(std::memory_order_acquire)) {
                sleep_ms(1);
            }
            g_core1I2CPaused.store(false, std::memory_order_release);
            ws2812_set_mode(WS2812_MODE_SOLID, kColorBlue);
            ws2812_update();
            continue;
        }

        // Sensor reads — each writes into localData by pointer
        core1_read_imu(&localData, &localCal, &imuConsecFail);

        baroCycle++;
        if (baroCycle >= kCore1BaroDivider) {
            baroCycle = 0;
            core1_read_baro(&localData);
        }

        gpsCycle++;
        if (gpsCycle >= kCore1GpsDivider && g_gpsInitialized) {
            gpsCycle = 0;
            core1_read_gps(&localData, &lastGpsReadUs);
        }

        // Seqlock publish (always write, even on IMU failure — council mod #4)
        localData.core1_loop_count = loopCount;
        seqlock_write(&g_sensorSeqlock, &localData);

        g_wdtCore1Alive.store(true, std::memory_order_relaxed);

        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        core1_neopixel_update(&localData, nowMs, sensorPhaseStartMs,
                              &lastNeoMs, &neoState);

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
    core1_sensor_loop();
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
    *temp_c = 0.0F;  // Not needed for 6-pos cal
    return true;
}

// ============================================================================
// Calibration Hooks — disable I2C master during rapid accel reads
// ============================================================================
// The ICM-20948's internal I2C master (for mag reads) performs autonomous
// Bank 3 transactions that race with our Bank 0 accel reads, causing
// bank-select corruption after ~150 reads. Disable it during calibration.

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
    if (g_imuInitialized) {
        icm20948_set_i2c_master_enable(&g_imu, false);
    }
}

static void cal_post_hook() {
    if (g_imuInitialized) {
        icm20948_set_i2c_master_enable(&g_imu, true);
    }
    // Tell Core 1 to reload calibration and resume sensors
    if (g_sensorPhaseActive) {
        g_calReloadPending.store(true, std::memory_order_release);
        g_core1PauseI2C.store(false, std::memory_order_release);
    }
}

// ============================================================================
// Sensor Status Callback (for RC_OS CLI 's' command)
// ============================================================================

// Print sensor data from seqlock snapshot (Core 1 driving sensors)
static void print_seqlock_sensors(const shared_sensor_data_t& snap) {
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
    // GPS
    if (snap.gps_read_count > 0) {
        if (snap.gps_valid) {
            printf("GPS: %.7f, %.7f, %.1f m MSL\n",
                   snap.gps_lat_1e7 / 1e7,
                   snap.gps_lon_1e7 / 1e7,
                   (double)snap.gps_alt_msl_m);
            printf("     Fix=%u Sats=%u Speed=%.1f m/s\n",
                   snap.gps_fix_type, snap.gps_satellites,
                   (double)snap.gps_ground_speed_mps);
        } else {
            printf("GPS: no fix (%u sats)\n",
                   snap.gps_satellites);
            printf("     RMC=%c GGA=%u GSA=%u\n",
                   snap.gps_rmc_valid ? 'A' : 'V',
                   snap.gps_gga_fix,
                   snap.gps_gsa_fix_mode);
            if (snap.gps_lat_1e7 != 0 || snap.gps_lon_1e7 != 0) {
                printf("     Last fix: %.7f, %.7f\n",
                       snap.gps_lat_1e7 / 1e7,
                       snap.gps_lon_1e7 / 1e7);
            }
        }
    } else if (g_gpsInitialized) {
        printf("GPS: initialized, no reads yet\n");
    } else {
        printf("GPS: not detected\n");
    }
    printf("Reads: I=%lu B=%lu G=%lu  "
           "Errors: I=%lu B=%lu G=%lu\n",
           (unsigned long)snap.imu_read_count,
           (unsigned long)snap.baro_read_count,
           (unsigned long)snap.gps_read_count,
           (unsigned long)snap.imu_error_count,
           (unsigned long)snap.baro_error_count,
           (unsigned long)snap.gps_error_count);
}

// Print sensor data from direct I2C reads (Core 0 owns bus, pre-sensor-phase)
static void print_direct_sensors() {
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

static void print_sensor_status() {
    printf("\n========================================\n");
    printf("  Sensor Readings (calibrated)\n");
    printf("========================================\n");

    // Once Core 1 owns I2C, read from seqlock to prevent bus contention.
    // Before sensor phase, fall back to direct I2C reads.
    if (g_sensorPhaseActive) {
        shared_sensor_data_t snap;
        if (seqlock_read(&g_sensorSeqlock, &snap)) {
            print_seqlock_sensors(snap);
        } else {
            printf("Seqlock read failed (retries exhausted)\n");
        }
    } else {
        print_direct_sensors();
    }

    printf("========================================\n\n");
}

// ============================================================================
// Hardware Status (boot banner + CLI 'b' command callback)
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

// Print I2C device detection results
static void hw_validate_i2c_devices() {
    // Skip I2C probes when Core 1 owns the bus (LL Entry 23: probe collision)
    if (rc_os_i2c_scan_allowed) {
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
            if (found) {
                foundCount++;
            }
        }
        printf("[INFO] Sensors found: %d/%zu expected\n",
               foundCount, sizeof(expected) / sizeof(expected[0]));
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
        printf("[PASS] PA1010D GPS init at 0x10 (I2C mode, 500us settling delay active)\n");
    } else {
        printf("[----] GPS not detected on I2C (delay disabled)\n");
    }
}

static void print_hw_status() {
    printf("\n=== Hardware Status ===\n");
    printf("  Build: %s (%s %s)\n", kBuildTag, __DATE__, __TIME__);

    printf("[PASS] Build + boot (you're reading this)\n");
    printf("[PASS] Red LED GPIO initialized (pin %d)\n", PICO_DEFAULT_LED_PIN);
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
    bool imuDetected  = i2c_bus_probe(kIcm20948AddrDefault);
    bool baroDetected = i2c_bus_probe(kBaroDps310AddrDefault);
    bool gpsDetected  = i2c_bus_probe(kGpsPa1010dAddr);

    // If GPS is present, drain its buffer before IMU/baro init.
    // The PA1010D streams NMEA autonomously on I2C — undrained data
    // causes bus contention during other device init (LL Entry 20).
    if (gpsDetected) {
        uint8_t gpsDrain[255];
        i2c_bus_read(kGpsPa1010dAddr, gpsDrain, sizeof(gpsDrain));
    }

    // Sensor power-up settling time
    // ICM-20948 datasheet: 11ms, DPS310: 40ms, generous margin
    sleep_ms(200);

    // Only init detected devices
    if (imuDetected) {
        g_imuInitialized = icm20948_init(&g_imu, kIcm20948AddrDefault);
    }

    if (baroDetected) {
        g_baroInitialized = baro_dps310_init(kBaroDps310AddrDefault);
        if (g_baroInitialized) {
            g_baroContinuous = baro_dps310_start_continuous();
        }
    }

    // GPS init is non-fatal (LL Entry 20)
    if (gpsDetected) {
        g_gpsInitialized = gps_pa1010d_init();
        if (g_gpsInitialized) {
            g_gpsOnI2C = true;
        }
    }
}

// Wait for USB CDC connection with LED blink, then drain input buffer.
static void wait_for_usb_connection() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(100);
    }

    // Settle time after connection (per LL Entry 15)
    sleep_ms(500);

    // Drain garbage from USB input buffer (per LL Entry 15)
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
}

static bool init_hardware() {
    // Check if previous reboot was caused by watchdog (before any init)
    bool watchdogReboot = watchdog_enable_caused_reboot();

    // Register fault handlers early (before any MPU config)
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, memmanage_fault_handler);
    exception_set_exclusive_handler(MEMMANAGE_EXCEPTION, memmanage_fault_handler);
    mpu_setup_stack_guard(reinterpret_cast<uint32_t>(&__StackBottom));

    // Red LED GPIO init
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // NeoPixel init
    g_neopixelInitialized = ws2812_status_init(pio0, kNeoPixelPin);

    // Launch Core 1 early so NeoPixel blinks immediately (no USB dependency)
    multicore_launch_core1(core1_entry);

    // I2C bus init (before USB per LL Entry 4/12)
    g_i2cInitialized = i2c_bus_init();

    if (g_i2cInitialized) {
        init_sensors();
    }

    // Calibration storage init (before USB per LL Entry 4/12)
    g_calStorageInitialized = calibration_storage_init();
    calibration_manager_init();

    // USB CDC init (after I2C/flash per LL Entry 4/12)
    wait_for_usb_connection();

    return watchdogReboot;
}

// ============================================================================
// Init: Boot banner and hardware status output
// ============================================================================

static void print_boot_status(bool watchdogReboot) {
    printf("\n");
    printf("==============================================\n");
    printf("  RocketChip v%s\n", kVersionString);
    printf("  Board: Adafruit Feather RP2350 HSTX\n");
    printf("==============================================\n\n");

    if (watchdogReboot) {
        printf("[WARN] *** PREVIOUS REBOOT WAS CAUSED BY WATCHDOG RESET ***\n");
        printf("[INFO] Re-enabling watchdog (%lu ms)\n\n",
               (unsigned long)kWatchdogTimeoutMs);
    }

    print_hw_status();
}

// ============================================================================
// Init: RC_OS setup, Core 1 sensor phase start, watchdog enable
// ============================================================================

static void init_application() {
    // RC_OS CLI init
    rc_os_init();
    rc_os_imu_available = g_imuInitialized;
    rc_os_baro_available = g_baroContinuous;
    rc_os_print_sensor_status = print_sensor_status;
    rc_os_print_boot_summary = print_hw_status;
    rc_os_read_accel = read_accel_for_cal;
    rc_os_cal_pre_hook = cal_pre_hook;
    rc_os_cal_post_hook = cal_post_hook;

    // Signal Core 1 to start sensor phase
    g_sensorPhaseActive = true;
    g_startSensorPhase.store(true, std::memory_order_release);
    rc_os_i2c_scan_allowed = false;  // LL Entry 23: prevent CLI I2C scan from corrupting bus
    sleep_ms(500);  // Let Core 1 start up
    printf("[INFO] Core 1 sensor phase started\n");

    // Enable watchdog (council critical fix: must be unconditional)
    g_watchdogEnabled = true;
    watchdog_enable(kWatchdogTimeoutMs, true);
    printf("[INFO] Watchdog enabled (%lu ms)\n\n",
           (unsigned long)kWatchdogTimeoutMs);
}

// ============================================================================
// Main Loop Tick Functions
// ============================================================================
// Each tick function manages one subsystem. nowMs is computed once per loop
// iteration and passed to all ticks to prevent temporal skew.

// Council recommendation: track which tick was running when watchdog fires.
static const char* g_lastTickFunction = "init";

static void heartbeat_tick(uint32_t nowMs) {
    static bool ledState = false;
    uint32_t phase = nowMs % kHeartbeatPeriodMs;
    bool shouldBeOn = (phase < kHeartbeatOnMs);
    if (shouldBeOn != ledState) {
        ledState = shouldBeOn;
        gpio_put(PICO_DEFAULT_LED_PIN, ledState ? true : false);
    }
}

static void watchdog_kick_tick() {
    if (!g_watchdogEnabled) {
        return;
    }

    g_wdtCore0Alive.store(true, std::memory_order_relaxed);
    if (g_wdtCore0Alive.load(std::memory_order_relaxed) &&
        g_wdtCore1Alive.load(std::memory_order_relaxed)) {
        watchdog_update();
        g_wdtCore0Alive.store(false, std::memory_order_relaxed);
        g_wdtCore1Alive.store(false, std::memory_order_relaxed);
    }
}

static void cli_update_tick() {
    rc_os_update();

    if (!calibration_is_active()) {
        return;
    }

    uint64_t nowUs = time_us_64();
    if ((nowUs - g_cliCalLastFeedUs) < kCliCalFeedIntervalUs) {
        return;
    }

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

// ============================================================================
// Main
// ============================================================================

int main() {
    bool watchdogReboot = init_hardware();
    print_boot_status(watchdogReboot);
    init_application();

    printf("Entering main loop\n\n");

    while (true) {
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        g_loopCounter++;

        g_lastTickFunction = "heartbeat";
        heartbeat_tick(nowMs);

        g_lastTickFunction = "watchdog";
        watchdog_kick_tick();

        g_lastTickFunction = "cli";
        cli_update_tick();

        g_lastTickFunction = "sleep";
        sleep_ms(1);
    }

    return 0;
}
