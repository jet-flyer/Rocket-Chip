// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file sensor_core1.cpp
 * @brief Core 1 sensor loop — high-rate sampling + seqlock publish
 *
 * Extracted from main.cpp (Stage 13 Phase 1). All functions are static
 * except core1_entry() which is the public interface.
 *
 * Core 1 owns I2C during sensor phase. Core 0 must NOT call icm20948_*()
 * or baro_dps310_*() unless g_core1I2CPaused == true.
 */

#include "core1/sensor_core1.h"

#include "rocketchip/config.h"
#include "rocketchip/sensor_seqlock.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/structs/mpu.h"
#include "drivers/ws2812_status.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include "drivers/baro_dps310.h"
#include "drivers/gps_pa1010d.h"
#include "drivers/gps_uart.h"
#include "calibration/calibration_manager.h"
#include "cli/rc_os.h"

#include <math.h>
#include <string.h>

// ============================================================================
// Constants (Core 1 private)
// ============================================================================

// Core 1 sensor loop timing
static constexpr uint32_t kCore1TargetCycleUs = 1000;           // ~1kHz target
static constexpr uint32_t kCore1BaroDivider = 32;               // Baro at ~31Hz (DPS310 continuous = 32 SPS)
static constexpr uint32_t kCore1GpsDivider = 100;               // GPS at ~10Hz
static constexpr uint32_t kGpsMinIntervalUs = 2000;             // MT3333 buffer refill time
static constexpr uint32_t kCore1CalFeedDivider = 10;            // Cal feed at ~100Hz
static constexpr uint32_t kCore1ConsecFailBusRecover = 10;      // I2C bus recovery threshold
static constexpr uint32_t kCore1ConsecFailDevReset = 50;        // ICM-20948 device reset threshold
static constexpr uint32_t kBaroMaxReinitAttempts = 3;           // Max baro re-inits before declaring dead
// Minimum plausible accel magnitude: 9.8 * cos(72 deg) = 3.0 m/s^2.
// Below any valid sensor reading -- all-zeros output means device is in sleep/reset state.
// Source: gravity projection floor when tilted 72 deg off vertical.
static constexpr float kAccelMinHealthyMag = 3.0F;

// PA1010D SDA settling delay (LL Entry 24)
static constexpr uint32_t kGpsSdaSettleUs = 500;

// kCore1PauseAckMaxMs stays in main.cpp — used by Core 0 wait loop, not Core 1.

// GPS coordinate bounds (WGS-84)
static constexpr double kLatMaxDeg  =  90.0;
static constexpr double kLatMinDeg  = -90.0;
static constexpr double kLonMaxDeg  = 180.0;
static constexpr double kLonMinDeg  = -180.0;
static constexpr double kGpsCoordScale = 1e7;                   // Degrees to 1e-7 degree integers

// GPS staleness watchdog
static constexpr uint32_t kGpsStalenessTimeoutUs = 10000000;    // 10s
// Velocity threshold for "probably flying" heuristic (flight state gate).
// Prevents UART reinit mid-flight. Replaced by real state machine in IVP-67.
static constexpr float kGpsFlyingVelocityThreshold = 5.0F;      // m/s

// ============================================================================
// Cross-Core Globals (written by Core 1, read by Core 0)
// ============================================================================

best_gps_fix_t g_bestGpsFix = {};
std::atomic<bool> g_bestGpsValid{false};

// ============================================================================
// MPU Stack Guard (per-core, PMSAv8) — duplicated from main.cpp
// Each core has its own MPU. This is called from Core 1's entry point.
// ============================================================================

static constexpr uint32_t kMpuGuardSizeBytes = 64;

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" uint32_t __StackOneBottom;  // Core 1 stack bottom (SCRATCH_X, linker-defined)

static void mpu_setup_stack_guard(uint32_t stackBottom) {
    mpu_hw->ctrl = 0;
    __dsb();
    __isb();

    // NOLINTBEGIN(readability-magic-numbers) -- PMSAv8 MPU register bit fields per ARMv8-M Architecture Reference Manual
    mpu_hw->rnr = 0;
    mpu_hw->rbar = (stackBottom & ~0x1FU)
                  | (0U << 3)   // SH: Non-shareable
                  | (0U << 1)   // AP: Privileged no-access
                  | (1U << 0);  // XN: Execute-never

    mpu_hw->rlar = ((stackBottom + kMpuGuardSizeBytes - 1) & ~0x1FU)
                  | (0U << 1)   // ATTRINDX: 0 (uses MAIR0 attr 0)
                  | (1U << 0);  // EN: Enable region

    mpu_hw->mair[0] = 0;

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
// call stays in the sensor loop -- NOT inside these helpers (council rule).

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
    // measures at least 3 m/s^2 (gravity floor at 72 deg tilt: 9.8*cos(72 deg)=3.0).
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

    // Feed calibration with RAW data (before cal apply) -- Core 1 owns I2C,
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

    // Read directly via ruuvi driver -- no MEAS_CFG pre-check.
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

    // I2C failure or driver error -- escalate (mirrors IMU pattern)
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
        g_gpsInitialized = false;  // GPS dead -- stop polling
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

    // gps_read_count always increments -- counts driver polls, not valid fixes.
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
// Core 1: Sensor Loop
// ============================================================================
// INVARIANT: Core 0 must NOT call icm20948_*() or baro_dps310_*() unless
// g_core1I2CPaused == true. Core 1 owns I2C during this phase.
// Seqlock write stays here -- read helpers write into localData by pointer.

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
        // NOLINTBEGIN(readability-magic-numbers) -- 3x3 identity matrix diagonal indices
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

    while (true) {
        uint32_t cycleStartUs = time_us_32();
        loopCount++;

        if (core1_check_pause_and_reload(&localCal)) {
            continue;
        }

        // Sensor reads -- each writes into localData by pointer
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
            && !rc_os_mag_cal_active.load(std::memory_order_acquire)) {
            gpsCycle = 0;
            core1_read_gps(&localData, &lastGpsReadUs);
        }

        // Seqlock publish (always write, even on IMU failure -- council mod #4)
        localData.core1_loop_count = loopCount;
        seqlock_write(&g_sensorSeqlock, &localData);

        // IVP-90: Core 1 heartbeat removed (was g_wdtCore1Alive).
        // PIO heartbeat watchdog monitors both cores via FIFO feed from Core 0.

        // NeoPixel update migrated to AO_LedEngine (Phase 5). Core 1 only
        // does sensor reads + seqlock publish. LED state evaluated on Core 0.

        uint32_t elapsed = time_us_32() - cycleStartUs;
        if (elapsed < kCore1TargetCycleUs) {
            busy_wait_us(kCore1TargetCycleUs - elapsed);
        }
    }
}

// ============================================================================
// Core 1 Entry Point
// ============================================================================

void core1_entry() {
    mpu_setup_stack_guard(reinterpret_cast<uint32_t>(&__StackOneBottom));

    // Always register as lockout victim -- flash_safe_execute() needs this
    // even in station/relay mode (calibration storage init uses flash).
    multicore_lockout_victim_init();
    g_core1LockoutReady.store(true, std::memory_order_release);

    // Wait for Core 0 to signal sensor phase start (Vehicle only).
    // Station/Relay never set this flag -- Core 1 idles here.
    while (!g_startSensorPhase.load(std::memory_order_acquire)) {
        sleep_ms(10);
    }

    core1_sensor_loop();
}
