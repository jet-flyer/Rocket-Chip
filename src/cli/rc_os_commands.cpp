// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file cli_commands.cpp
 * @brief CLI command handlers and display functions
 *
 * Extracted from main.cpp (Phase 7 AO architecture migration).
 * Pure command/display code — reads state from AO public APIs and
 * sensor seqlock, owns no state.
 */

#include "cli/rc_os_commands.h"

#include "rocketchip/config.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/pcm_frame.h"
#include "rocketchip/telemetry_state.h"
#include "rocketchip/station_output_mode.h"
#include "rocketchip/telemetry_encoder.h"
#include "core1/sensor_core1.h"
#include "fusion/eskf.h"
#include "fusion/eskf_runner.h"
#include "fusion/confidence_gate.h"
#include "fusion/mahony_ahrs.h"
#include "active_objects/ao_logger.h"
#include "active_objects/ao_flight_director.h"
#include "active_objects/ao_radio.h"
#include "active_objects/ao_telemetry.h"
#include "active_objects/ao_rcos.h"
#include "active_objects/ao_led_engine.h"
#include "calibration/calibration_manager.h"
#include "calibration/calibration_storage.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include "drivers/baro_dps310.h"
#include "drivers/gps_uart.h"
#include "logging/psram_init.h"
#include "logging/ring_buffer.h"
#include "logging/flash_flush.h"
#include "logging/flight_table.h"
#include "logging/crc32.h"
#include "safety/pio_watchdog.h"
#include "watchdog/watchdog_recovery.h"
#include "flight_director/flight_state.h"
#include "flight_director/mission_profile_data.h"
#include "cli/rc_os.h"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include <stdio.h>
#include <math.h>

// ============================================================================
// Extern globals from main.cpp (init-time state, not owned here)
// ============================================================================

// NOLINTBEGIN(readability-redundant-declaration) — cross-TU externs
extern bool g_neopixelInitialized;
extern bool g_radioInitialized;
extern bool g_sensorPhaseActive;
extern sensor_seqlock_t g_sensorSeqlock;
extern rc::WatchdogRecovery g_recovery;
// NOLINTEND(readability-redundant-declaration)

// main.cpp statics made non-static for CLI access (Phase 7)
extern size_t g_psramSize;
extern bool   g_psramSelfTestPassed;
extern bool   g_psramFlashSafePassed;
extern bool   g_i2cInitialized;
extern bool   g_spiInitialized;
extern bool   g_watchdogReboot;
extern bool   g_calStorageInitialized;

// ============================================================================
// Constants (moved from main.cpp — used only by CLI display/commands)
// ============================================================================

static constexpr float kRadToDeg = 180.0F / 3.14159265F;
static constexpr float kFullCircleDeg = 360.0F;
static constexpr double kGpsCoordScale = 1e7;
static constexpr float kGps1e7ToDegreesF = 1e-7F;  // NOLINT(readability-identifier-naming)
static constexpr double kGps1e7ToDegrees = 1e-7;    // NOLINT(readability-identifier-naming)
static constexpr int32_t kEraseInputMaxChars = 7;
static constexpr uint32_t kEraseInputTimeoutUs = 10000000;
static constexpr uint32_t kFlightNumTimeoutUs = 3000000;
static constexpr uint32_t kCrc32InitXor = 0xFFFFFFFFU;
static constexpr uint8_t kDlpfCfgMask = 0x07U;

// ESKF state buffer size — must match main.cpp kEskfBufferSamples
static constexpr uint32_t kEskfBufferSamples = 1000;

// I2C alternate addresses for device identification
static constexpr uint8_t kI2cAddrIcm20948Alt = 0x68;
static constexpr uint8_t kI2cAddrDps310Alt   = 0x76;

// ============================================================================
// Display Helpers
// ============================================================================

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
    printf("      conf=%c div=%.1f%c unc=%lums\n",
           conf->confident ? 'Y' : 'N',
           (double)conf->ahrs_divergence_deg,
           '\xb0',  // degree symbol — avoid raw char in printf format
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

static void print_seqlock_sensors(const shared_sensor_data_t& snap) {
    print_imu_status(snap);
    print_eskf_status();
    print_gps_status(snap);
    print_sensor_counts(snap);
}

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

// ============================================================================
// Sensor Status (vehicle mode 's' callback)
// ============================================================================

void cli_print_sensor_status() {
    printf("\n========================================\n");
    printf("  Sensor Readings (calibrated)\n");
    printf("========================================\n");

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
// Hardware Status ('b' key and boot banner)
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

static void hw_validate_i2c_devices() {
    if (rc_os_i2c_scan_allowed) {
        static constexpr uint8_t kExpected[] = {
            kI2cAddrAk09916,
            kI2cAddrIcm20948,
            kI2cAddrDps310,
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

static void hw_validate_sensors() {
    if (g_imuInitialized) {
        printf("[PASS] ICM-20948 init (WHO_AM_I=0xEA)\n");
        printf("[%s] AK09916 magnetometer %s\n",
               g_imu.mag_initialized ? "PASS" : "WARN",
               g_imu.mag_initialized ? "ready" : "not ready");

        uint8_t accelCfg = 0;
        uint8_t gyroCfg1 = 0;
        uint8_t gyroDiv = 0;
        if (icm20948_read_config_registers(&g_imu, &accelCfg, &gyroCfg1, &gyroDiv)) {
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

static void print_gps_status_boot() {
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

// Build tag constant — must match main.cpp
static constexpr const char* kBuildTag = "ivp74-profile-1";

void cli_print_hw_status() {
    printf("\n=== Hardware Status ===\n");
    printf("  Build: %s (%s %s)\n", kBuildTag, __DATE__, __TIME__);

    printf("[PASS] Build + boot (you're reading this)\n");
    printf("[PASS] Red LED GPIO initialized (pin %d, %s)\n",
           board::kLedPin, board::kLedActiveHigh ? "active-high" : "active-low");
    printf("[%s] NeoPixel PIO initialized (pin %d)\n",
           g_neopixelInitialized ? "PASS" : "FAIL", board::kNeoPixelPin);
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
    print_gps_status_boot();

    printf("=== Status Complete ===\n\n");
}

// ============================================================================
// Boot Banner
// ============================================================================

// Watchdog timeout constant — must match main.cpp
static constexpr uint32_t kWatchdogTimeoutMs = 5000;

void cli_print_boot_status() {
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

    cli_print_hw_status();
}

// ============================================================================
// ESKF Live Output (1Hz, 'e' key)
// ============================================================================

void cli_print_eskf_live() {
    if (!g_eskfInitialized) {
        printf("ESKF: waiting for init...\n");
        return;
    }
    if (!g_eskf.healthy()) {
        printf("ESKF: UNHEALTHY\n");
        return;
    }
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

// ============================================================================
// Station RX Telemetry Display (IVP-99)
// ============================================================================

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

void cli_print_station_status() {
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

// ============================================================================
// Logging / Flight Commands
// ============================================================================

// Watchdog kick callback for flash operations.
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
    uint8_t rate = isPsram ? 50U : 25U;

    printf("Flushing %lu frames to flash...\n", (unsigned long)stored);

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
        putchar(ch);
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

    if (!rc::flight_table_erase_flash()) {
        printf("Table erase error.\n");
        return;
    }

    rc::flight_table_erase_all(ft);
    ft->loaded = true;

    rc::flight_table_save(ft);

    printf("All flights erased.\n");
}

// ============================================================================
// Flight Download
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

static int read_flight_number() {
    int num = 0;
    bool gotDigit = false;
    for (int i = 0; i < 3; ++i) {
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

static uint32_t stream_flight_binary(const rc::FlightLogEntry& entry) {
    uint32_t frame_size = entry.frame_size;
    uint32_t frames_per_sector = rc::kFlashSectorSize / frame_size;
    uint32_t flash_base = entry.start_sector * rc::kFlashSectorSize;
    const uint8_t* xip_base = reinterpret_cast<const uint8_t*>(
        XIP_BASE + flash_base);

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

    printf("RCBIN:%d:%lu:%u:%lu\n",
           num,
           (unsigned long)entry.frame_count,
           (unsigned)entry.log_rate_hz,
           (unsigned long)entry.frame_size);

    uint32_t crc = stream_flight_binary(entry);
    printf("RCEND:%08lX\n", (unsigned long)crc);
}

// ============================================================================
// Radio Status
// ============================================================================

static void cmd_radio_status() {
    const auto* rs = AO_Radio_get_state();
    if (!rs->initialized) {
        printf("Radio not initialized.\n");
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if constexpr (kRadioModeRx) {
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
        printf("TX: %lu sent  %u fail  phase=%d\n",
               (unsigned long)rs->tx_count,
               static_cast<unsigned>(rs->tx_consec_fail),
               static_cast<int>(rs->scheduler.phase));
    }
}

// ============================================================================
// Station GPS Commands (IVP-97)
// ============================================================================

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
    printf("Station: %.7f, %.7f, %.1fm MSL\n",
           static_cast<double>(g_bestGpsFix.lat_1e7) / 1e7,
           static_cast<double>(g_bestGpsFix.lon_1e7) / 1e7,
           static_cast<double>(g_bestGpsFix.alt_msl_m));
    printf("Vehicle distance: needs received position (IVP-97c)\n");
}

// ============================================================================
// Unhandled Key Dispatcher
// ============================================================================

void cli_handle_unhandled_key(int key) {
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
