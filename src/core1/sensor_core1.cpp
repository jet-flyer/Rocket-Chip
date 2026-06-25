// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file sensor_core1.cpp
 * @brief Core 1 sensor loop — high-rate sampling + seqlock publish
 *
 * All functions are static except core1_entry() which is the public
 * interface.
 *
 * Core 1 owns I2C during sensor phase. Core 0 must NOT call icm20948_*()
 * or baro_dps310_*() unless g_core1I2CPaused == true.
 */

#include "core1/sensor_core1.h"

#include "rocketchip/config.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/job.h"            // R-1: distinguish vehicle vs station/relay path for boot-wait bound
#include "safety/fault_protection.h"   // OPT-IVP-01: shared MPU stack guard (removes duplication)
#include "safety/crash_record.h"       // R-1: capture-state-then-reset on boot-wait timeout
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
#include "drivers/mcu_temp.h"
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
static constexpr uint32_t kCore1McuTempDivider = 1000;          // MCU temp at ~1Hz (Stage 16C IVP-142a)
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
// Prevents UART reinit mid-flight.
static constexpr float kGpsFlyingVelocityThreshold = 5.0F;      // m/s

// ============================================================================
// Cross-Core Globals (written by Core 1, read by Core 0)
// ============================================================================

best_gps_fix_t g_bestGpsFix = {};
std::atomic<bool> g_bestGpsValid{false};

// Fault protection and MPU stack guard now provided by safety/fault_protection.h
// (OPT-IVP-01). core1_entry() calls the shared mpu_setup_stack_guard().

// ============================================================================
// Core 1: Sensor Read Helpers
// ============================================================================
// Each helper writes into localData (passed by pointer). The seqlock_write()
// call stays in the sensor loop -- NOT inside these helpers (council rule).

// IMU error path: increment fail counter, invalidate accel/gyro, attempt recovery.
static void core1_imu_error_recovery(uint32_t* imu_consec_fail,
                                      shared_sensor_data_t* local_data) {
    (*imu_consec_fail)++;
    local_data->accel_valid = false;
    local_data->gyro_valid = false;
    local_data->imu_error_count++;
    if (*imu_consec_fail >= kCore1ConsecFailDevReset) {
        icm20948_init(&g_imu, kIcm20948AddrDefault);
        *imu_consec_fail = 0;
    } else if (*imu_consec_fail >= kCore1ConsecFailBusRecover
               && *imu_consec_fail % kCore1ConsecFailBusRecover == 0) {
        i2c_bus_recover();
    }
}

// Apply calibration to raw IMU data and write calibrated values into localData.
static void core1_apply_imu_cal(const icm20948_data_t& imu_data,
                                 shared_sensor_data_t* local_data,
                                 calibration_store_t* local_cal) {
    cal_vec3_t accel_out;
    cal_vec3_t gyro_out;
    calibration_apply_accel_with(local_cal,
        {imu_data.accel.x, imu_data.accel.y, imu_data.accel.z}, accel_out);
    calibration_apply_gyro_with(local_cal,
        {imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z}, gyro_out);

    local_data->accel_x = accel_out.x;
    local_data->accel_y = accel_out.y;
    local_data->accel_z = accel_out.z;
    local_data->gyro_x = gyro_out.x;
    local_data->gyro_y = gyro_out.y;
    local_data->gyro_z = gyro_out.z;
    local_data->imu_timestamp_us = time_us_32();
    local_data->imu_read_count++;
    local_data->accel_valid = true;
    local_data->gyro_valid = true;

    if (imu_data.mag_valid) {
        cal_vec3_t mag_out;
        calibration_apply_mag_with(local_cal,
            {imu_data.mag.x, imu_data.mag.y, imu_data.mag.z}, mag_out);
        local_data->mag_x = mag_out.x;
        local_data->mag_y = mag_out.y;
        local_data->mag_z = mag_out.z;
        local_data->mag_raw_x = imu_data.mag.x;
        local_data->mag_raw_y = imu_data.mag.y;
        local_data->mag_raw_z = imu_data.mag.z;
        local_data->mag_valid = true;
        local_data->mag_read_count++;
    }
}

static void core1_read_imu(shared_sensor_data_t* local_data,
                            calibration_store_t* local_cal,
                            uint32_t* imu_consec_fail,
                            bool feed_cal) {
    icm20948_data_t imu_data;
    bool imu_ok = icm20948_read(&g_imu, &imu_data);
    if (!imu_ok) {
        core1_imu_error_recovery(imu_consec_fail, local_data);
        return;
    }

    // Sanity-check raw accel magnitude. A working sensor in ANY orientation always
    // measures at least 3 m/s^2 (gravity floor at 72 deg tilt: 9.8*cos(72 deg)=3.0).
    // All-zeros = ICM-20948 silent reset to sleep state (LL Entry 29).
    const float raw_accel_mag = sqrtf(
        imu_data.accel.x * imu_data.accel.x +
        imu_data.accel.y * imu_data.accel.y +
        imu_data.accel.z * imu_data.accel.z);
    if (raw_accel_mag < kAccelMinHealthyMag) {
        core1_imu_error_recovery(imu_consec_fail, local_data);
        return;
    }

    *imu_consec_fail = 0;

    // Feed calibration with RAW data (before cal apply) -- Core 1 owns I2C,
    // so no bus contention. Core 0 must NOT do concurrent icm20948_read().
    if (feed_cal) {
        cal_state_t cal_state = calibration_manager_get_state();
        if (cal_state == CAL_STATE_GYRO_SAMPLING) {
            calibration_feed_gyro(imu_data.gyro.x, imu_data.gyro.y,
                                  imu_data.gyro.z, imu_data.temperature_c);
        } else if (cal_state == CAL_STATE_ACCEL_LEVEL_SAMPLING ||
                   cal_state == CAL_STATE_ACCEL_6POS_SAMPLING) {
            calibration_feed_accel(imu_data.accel.x, imu_data.accel.y,
                                   imu_data.accel.z, imu_data.temperature_c);
        }
    }

    core1_apply_imu_cal(imu_data, local_data, local_cal);
}

static void core1_read_baro(shared_sensor_data_t* local_data) {
    static uint32_t baro_consec_fail = 0;
    static uint32_t baro_reinit_attempts = 0;

    // Read directly via ruuvi driver -- no MEAS_CFG pre-check.
    // The DPS310 in continuous mode alternates PRS_RDY and TMP_RDY;
    // requiring both simultaneously is too strict and produces false errors.
    // The ruuvi driver reads raw registers; I2C failure returns !valid.
    baro_dps310_data_t baro_data;
    if (baro_dps310_read(&baro_data) && baro_data.valid) {
        local_data->pressure_pa = baro_data.pressure_pa;
        local_data->baro_temperature_c = baro_data.temperature_c;
        local_data->baro_timestamp_us = time_us_32();
        local_data->baro_read_count++;
        local_data->baro_valid = true;
        baro_consec_fail = 0;
        baro_reinit_attempts = 0;  // Successful read proves sensor is alive

        // Feed baro calibration with raw data from Core 1
        if (calibration_manager_get_state() == CAL_STATE_BARO_SAMPLING) {
            calibration_feed_baro(baro_data.pressure_pa,
                                  baro_data.temperature_c);
        }
        return;
    }

    // I2C failure or driver error -- escalate (mirrors IMU pattern)
    baro_consec_fail++;
    local_data->baro_error_count++;
    if (baro_consec_fail >= kCore1ConsecFailDevReset) {
        if (baro_reinit_attempts < kBaroMaxReinitAttempts) {
            baro_dps310_start_continuous();
            baro_reinit_attempts++;
        } else {
            g_baroInitialized = false;  // Declare baro dead
        }
        baro_consec_fail = 0;
    } else if (baro_consec_fail >= kCore1ConsecFailBusRecover
               && baro_consec_fail % kCore1ConsecFailBusRecover == 0) {
        i2c_bus_recover();
    }
}

// Update best-fix diagnostic when satellite count or HDOP improves.
// Public (sensor_core1.h) — shared with station idle-bridge tick (IVP-141).
void core1_update_best_gps_fix(const shared_sensor_data_t* local_data) {
    if (local_data->gps_valid && local_data->gps_fix_type >= 2) {
        bool better = !g_bestGpsValid.load(std::memory_order_relaxed)
                      || (local_data->gps_satellites > g_bestGpsFix.satellites)
                      || (local_data->gps_satellites == g_bestGpsFix.satellites &&
                          local_data->gps_hdop > 0.0F &&
                          local_data->gps_hdop < g_bestGpsFix.hdop);
        if (better) {
            g_bestGpsFix.lat_1e7     = local_data->gps_lat_1e7;
            g_bestGpsFix.lon_1e7     = local_data->gps_lon_1e7;
            g_bestGpsFix.alt_msl_m   = local_data->gps_alt_msl_m;
            g_bestGpsFix.hdop        = local_data->gps_hdop;
            g_bestGpsFix.satellites  = local_data->gps_satellites;
            g_bestGpsFix.fix_type    = local_data->gps_fix_type;
            g_bestGpsValid.store(true, std::memory_order_release);
        }
    }
}

// GPS UART staleness watchdog. Reinits UART if no valid parse for 10s,
// gated on flight state (don't reinit mid-flight). Blocks up to 2s.
static void core1_gps_staleness_check(bool parsed, uint32_t now_us) {
    static uint32_t last_valid_gps_us = 0;

    if (parsed) {
        last_valid_gps_us = now_us;
        return;
    }

    if (g_gpsTransport != GPS_TRANSPORT_UART || last_valid_gps_us == 0) {
        return;
    }
    if (now_us - last_valid_gps_us <= kGpsStalenessTimeoutUs) {
        return;
    }

    // Flight state gate: proxy heuristic.
    bool probably_flying = g_eskfInitialized &&
                          g_eskf.v.norm() > kGpsFlyingVelocityThreshold;
    if (probably_flying) {
        return;
    }

    if (!gps_uart_reinit()) {
        g_gpsInitialized = false;  // GPS dead -- stop polling
    }
    last_valid_gps_us = time_us_32();  // Reset timer after attempt
}

// Public (sensor_core1.h) — shared with station idle-bridge tick (IVP-141).
void core1_read_gps(shared_sensor_data_t* local_data,
                    uint32_t* last_gps_read_us) {
    uint32_t gps_now_us = time_us_32();
    if (gps_now_us - *last_gps_read_us < kGpsMinIntervalUs) {
        return;
    }

    *last_gps_read_us = gps_now_us;

    // Transport-neutral poll via function pointers (set once in init_sensors)
    bool parsed = g_gpsFnUpdate();
    if (g_gpsTransport == GPS_TRANSPORT_I2C) {
        busy_wait_us(kGpsSdaSettleUs);  // SDA settling delay (LL Entry 24)
    }

    gps_data_t d;
    g_gpsFnGetData(&d);

    // gps_read_count always increments -- counts driver polls, not valid fixes.
    local_data->gps_read_count++;
    local_data->gps_timestamp_us = gps_now_us;

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

        local_data->gps_lat_1e7 = static_cast<int32_t>(lat * kGpsCoordScale);
        local_data->gps_lon_1e7 = static_cast<int32_t>(lon * kGpsCoordScale);
        local_data->gps_alt_msl_m = d.altitudeM;
        local_data->gps_ground_speed_mps = d.speedMps;
        local_data->gps_course_deg = d.courseDeg;
        local_data->gps_valid = true;
        local_data->gps_hdop = d.hdop;
        local_data->gps_vdop = d.vdop;
    }

    // Always update diagnostic fields (satellites, fix type, raw sentence flags)
    local_data->gps_fix_type = static_cast<uint8_t>(d.fix);
    local_data->gps_satellites = d.satellites;
    local_data->gps_gga_fix = d.ggaFix;
    local_data->gps_gsa_fix_mode = d.gsaFixMode;
    local_data->gps_rmc_valid = d.rmcValid;

    if (!parsed) {
        local_data->gps_error_count++;
    }

    core1_gps_staleness_check(parsed, gps_now_us);
    core1_update_best_gps_fix(local_data);
}

// ============================================================================
// Core 1: Sensor Loop
// ============================================================================
// INVARIANT: Core 0 must NOT call icm20948_*() or baro_dps310_*() unless
// g_core1I2CPaused == true. Core 1 owns I2C during this phase.
// Seqlock write stays here -- read helpers write into localData by pointer.

// Check calibration reload request and I2C pause from Core 0. Returns true
// if the caller should skip the rest of the loop iteration (continue).
static bool core1_check_pause_and_reload(calibration_store_t* local_cal) {
    if (g_calReloadPending.load(std::memory_order_acquire)) {
        calibration_load_into(local_cal);
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
static void core1_load_cal_or_defaults(calibration_store_t* local_cal) {
    if (!calibration_load_into(local_cal)) {
        memset(local_cal, 0, sizeof(*local_cal));
        local_cal->accel.scale.x = 1.0F;
        local_cal->accel.scale.y = 1.0F;
        local_cal->accel.scale.z = 1.0F;
        // NOLINTBEGIN(readability-magic-numbers) -- 3x3 identity matrix diagonal indices
        local_cal->board_rotation.m[0] = 1.0F;
        local_cal->board_rotation.m[4] = 1.0F;
        local_cal->board_rotation.m[8] = 1.0F;
        // NOLINTEND(readability-magic-numbers)
    }
}

// Per-sensor rate dividers and consecutive-failure counter shared across
// every iteration of the Core 1 sensor loop. Extracted from
// core1_sensor_loop() for JSF AV rule 1 compliance; pure state container.
struct Core1SensorCycle {
    uint32_t baroCycle     = 0;
    uint32_t calFeedCycle  = 0;
    uint32_t imuConsecFail = 0;
    uint32_t gpsCycle      = 0;
    uint32_t lastGpsReadUs = 0;
    uint32_t mcuTempCycle  = 0;
};

// Run one pass of each sensor (rate-divided) into localData. Mirrors the
// dispatch that used to live inline in core1_sensor_loop().
static void core1_sensor_pass(shared_sensor_data_t* local_data,
                               calibration_store_t* local_cal,
                               Core1SensorCycle* cyc) {
    cyc->calFeedCycle++;
    bool feed_cal = (cyc->calFeedCycle >= kCore1CalFeedDivider);
    if (feed_cal) {
        cyc->calFeedCycle = 0;
    }
    if (g_imuInitialized) {
        core1_read_imu(local_data, local_cal, &cyc->imuConsecFail, feed_cal);
    }

    cyc->baroCycle++;
    if (cyc->baroCycle >= kCore1BaroDivider && g_baroInitialized) {
        cyc->baroCycle = 0;
        core1_read_baro(local_data);
    }

    cyc->gpsCycle++;
    if (cyc->gpsCycle >= kCore1GpsDivider && g_gpsInitialized
        && !rc_os_mag_cal_active.load(std::memory_order_acquire)) {
        cyc->gpsCycle = 0;
        core1_read_gps(local_data, &cyc->lastGpsReadUs);
    }

    cyc->mcuTempCycle++;
    if (cyc->mcuTempCycle >= kCore1McuTempDivider && rc::mcu_temp_available()) {
        cyc->mcuTempCycle = 0;
        local_data->mcu_die_temp_c = rc::mcu_temp_read_c();
        local_data->mcu_temp_read_count++;
    }
}

static void core1_sensor_loop() {
    calibration_store_t local_cal;
    core1_load_cal_or_defaults(&local_cal);

    shared_sensor_data_t local_data = {};
    // Initial MCU temp sentinel so seqlock readers don't see an
    // all-zeros 0.0°C before the first capture.
    local_data.mcu_die_temp_c = -999.0F;

    Core1SensorCycle cyc{};
    uint32_t loop_count = 0;

    while (true) {
        uint32_t cycle_start_us = time_us_32();
        loop_count++;

        if (!core1_check_pause_and_reload(&local_cal)) {
            core1_sensor_pass(&local_data, &local_cal, &cyc);

            // Seqlock publish (always write, even on IMU failure — council mod #4)
            local_data.core1_loop_count = loop_count;
            seqlock_write(&g_sensorSeqlock, &local_data);

            // PIO heartbeat watchdog monitors both cores via FIFO feed from Core 0.
            // LED state evaluated on Core 0 via AO_LedEngine.

            uint32_t elapsed = time_us_32() - cycle_start_us;
            if (elapsed < kCore1TargetCycleUs) {
                busy_wait_us(kCore1TargetCycleUs - elapsed);
            }
        }
    }
}

// ============================================================================
// Core 1 Entry Point
// ============================================================================

void core1_entry() {
    mpu_setup_stack_guard(reinterpret_cast<uintptr_t>(&__StackOneBottom));

    // Always register as lockout victim -- flash_safe_execute() needs this
    // even in station/relay mode (calibration storage init uses flash).
    multicore_lockout_victim_init();
    g_core1LockoutReady.store(true, std::memory_order_release);

    // Wait for Core 0 to signal sensor phase start.
    //
    // R-1 (audit 2026-05-07):
    //   - Vehicle path: bounded — Core 0's init sequence sets the flag in
    //     well under a second. A 10s timeout is far above the worst-case
    //     observed boot path. Exceeding it indicates Core 0 is wedged, so
    //     capture a CrashRecord (reason=Core1BootWait) and reset; the
    //     post-reset health_monitor_init() latches kHealthCriticalPriorHardfault
    //     and the existing safe-mode pivot owns recovery. Static analyzer
    //     can prove the loop terminates (either flag is set, or the
    //     timeout branch fires and crash_record_capture() is [[noreturn]]).
    //   - Station/Relay path: intentionally non-terminating per Holzmann
    //     P10 Rule 2 inverted-rule scheduler exemption (Core 1 has no
    //     work in these roles). The static_assert pattern below documents
    //     that the unbounded branch is statically reachable only on these
    //     roles.
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        // Vehicle: bounded wait. Loop bound expressed as constants so the
        // compiler folds the static-bound check at -O2.
        constexpr uint32_t k_core1_boot_wait_tick_ms = 10U;
        constexpr uint32_t k_core1_boot_wait_max_iters = 1000U;  // 10 s ceiling
        for (uint32_t iter = 0U; iter < k_core1_boot_wait_max_iters; ++iter) {
            if (g_startSensorPhase.load(std::memory_order_acquire)) {
                break;
            }
            sleep_ms(k_core1_boot_wait_tick_ms);
        }
        if (!g_startSensorPhase.load(std::memory_order_acquire)) {
            // Timeout — Core 0 didn't signal sensor phase start in 10 s.
            // Capture state and reset; safe-mode pivot owns recovery.
            uint32_t self_pc = 0U;
            __asm volatile ("mov %0, pc" : "=r"(self_pc));
            rc::crash_record_capture(rc::kCrashReasonCore1BootWait, self_pc, 0U);
            // crash_record_capture is [[noreturn]] — fires NVIC_SystemReset().
        }
    } else {
        // Station/Relay: Holzmann scheduler exemption. Statically provable
        // non-terminating: the flag is never set on these roles, by
        // construction in their init paths.
        while (!g_startSensorPhase.load(std::memory_order_acquire)) {
            sleep_ms(10);
        }
    }

    core1_sensor_loop();
}
