// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file cli_commands.cpp
 * @brief CLI command handlers and display functions
 *
 * Pure command/display code — reads state from AO public APIs and
 * sensor seqlock, owns no state.
 */

#include "cli/rc_os_commands.h"

#include "ao_rcos.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/config.h"
#include "rocketchip/shared_state.h"
#include "safety/health_monitor.h"       // IVP-107: 2-bit health decode
#include "safety/core1_i2c_pause.h"       // R-17 audit 2026-05-07: cooperative pause around flash ops
#include "flight_director/go_nogo_checks.h"  // IVP-T14: RF Link pre-arm station
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/pcm_frame.h"
#include "rocketchip/telemetry_state.h"
#include "rocketchip/station_output_mode.h"
#include "rocketchip/telemetry_encoder.h"
#include "core1/sensor_core1.h"
#include "fusion/eskf.h"
#include "fusion/eskf_runner.h"
#include "fusion/wmm_tables.h"
#include "fusion/confidence_gate.h"
#include "fusion/mahony_ahrs.h"
#include "active_objects/ao_logger.h"
#include "active_objects/ao_flight_director.h"
#include "active_objects/ao_radio.h"
#include "active_objects/ao_telemetry.h"
#include "rocketchip/radio_config_table.h"  // T5.5 sub 2c: SET cycle

// MAVLink command IDs for station command menu (IVP-62c)
// Values from common/common.h — avoids pulling full mavlink.h with packed struct warnings
static constexpr uint16_t kMavCmdArmDisarm = 400;        // MAV_CMD_COMPONENT_ARM_DISARM
static constexpr uint16_t kMavCmdFlightTermination = 185; // MAV_CMD_DO_FLIGHTTERMINATION
static constexpr uint16_t kMavCmdSetHome = 179;           // MAV_CMD_DO_SET_HOME
static constexpr uint16_t kMavCmdBeacon = 31010;          // MAV_CMD_USER_1 — Stage L manual beacon
static constexpr uint16_t kMavCmdSetRadioConfig = 31011;  // MAV_CMD_USER_2 — T5.5 SET

// ============================================================================
// Stage T2 cheat-mode — throwaway code, revertible in one commit.
// Gated behind -DROCKETCHIP_STAGE_T2_CHEAT=ON (station build only).
// Queues X presses and fires tracked_command when a vehicle nav packet
// is received (station is now guaranteed to be just after vehicle TX,
// which means vehicle is entering kRxWindow).
// ============================================================================
#ifdef ROCKETCHIP_STAGE_T2_CHEAT
static volatile bool g_t2_pending = false;
static volatile uint16_t g_t2_cmd = 0;
static volatile float g_t2_p1 = 0.0f;

void stage_t2_queue_command(uint16_t cmd, float p1) {
    g_t2_cmd = cmd;
    g_t2_p1 = p1;
    g_t2_pending = true;
    rc::rc_log("[STAGE_T2] queued cmd=%u pending fire on next vehicle RX\n", cmd);
}

// Called from handle_rx_packet when a CCSDS nav packet decodes successfully.
// Fires any pending command immediately — we know the vehicle just completed
// TX and is entering kRxWindow within a few ms.
void stage_t2_fire_pending_if_any() {
    if (g_t2_pending) {
        g_t2_pending = false;
        uint16_t cmd = g_t2_cmd;
        float p1 = g_t2_p1;
        AO_Telemetry_send_tracked_command(cmd, p1);
        rc::rc_log("[STAGE_T2] fired cmd=%u on vehicle RX\n", cmd);
    }
}
#endif  // ROCKETCHIP_STAGE_T2_CHEAT

// ============================================================================
// Stage T IVP-T5.5 sub 2c — station SET_RADIO_CONFIG cycle
// ============================================================================
// Operator presses `r` to step to the NEXT entry in kRadioConfigTable. Station
// issues MAV_CMD_USER_2 with the target config's params. Vehicle validates +
// ACKs on OLD, applies on TxDone. Station's own radio switch (so it can hear
// the vehicle on NEW) lands in a follow-up (sub 2c.2 — LOS watchdog +
// own-radio reconfigure on ACK receipt). For this first step we just send
// the command and log it — operator verifies on dashboard that the NEW
// config shows up in the echo.
static void cmd_radio_config_cycle() {
    if constexpr (!kRadioModeRx) { return; }  // station-only

    // Find the current station-side target by matching against the whitelist.
    // On first press we don't know what the vehicle is on — start at index 0
    // (the default config) and advance from there.
    static size_t s_cycle_idx = 0;
    s_cycle_idx = (s_cycle_idx + 1) % rc::kRadioConfigTableSize;
    const auto& target = rc::kRadioConfigTable[s_cycle_idx];

    rc::rc_log("[CMD] SET_RADIO_CONFIG BW=%u nav=%u SF=%u CR=%u pwr=%u (idx %u/%u)\n",
               static_cast<unsigned>(target.bw_khz),
               static_cast<unsigned>(target.nav_rate_hz),
               static_cast<unsigned>(target.sf),
               static_cast<unsigned>(target.cr),
               static_cast<unsigned>(target.power_dbm),
               static_cast<unsigned>(s_cycle_idx),
               static_cast<unsigned>(rc::kRadioConfigTableSize - 1));
    // Params packed as floats: lroundf() on vehicle side for robust cast.
    AO_Telemetry_send_tracked_command(
        kMavCmdSetRadioConfig,
        static_cast<float>(target.bw_khz),
        static_cast<float>(target.nav_rate_hz),
        static_cast<float>(target.sf),
        static_cast<float>(target.cr),
        static_cast<float>(target.power_dbm));
    rc::rc_log("[CMD] SET_RADIO_CONFIG sent, waiting for ACK...\n");
    // Station's own radio switch happens in ao_telemetry.cpp's ACK-match
    // path when ACK-accepted arrives for this command. No LOS watchdog yet
    // (sub 2d adds symmetric revert on vehicle side; a station-side LOS
    // watchdog with auto-revert is wired in sub 2e/2f alongside the
    // dashboard status banner).
}
#include "active_objects/ao_rcos.h"
#include "active_objects/ao_led_engine.h"
#include "calibration/calibration_manager.h"
#include "calibration/calibration_storage.h"
#include "drivers/i2c_bus.h"
#include "drivers/icm20948.h"
#include "drivers/baro_dps310.h"
#include "drivers/gps_uart.h"
#include "drivers/gps_pa1010d.h"
#include "logging/psram_init.h"
#include "logging/ring_buffer.h"
#include "logging/flash_flush.h"
#include "logging/flight_table.h"
#include "logging/crc32.h"
#include "safety/pio_watchdog.h"
#include "flight_director/flight_director.h"  // flight_director_launch_abort()
#include "flight_director/flight_state.h"
#include "flight_director/mission_profile_data.h"
#include "cli/rc_os.h"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "tusb.h"
#include "rocketchip/rc_log.h"
#include <math.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr float kRadToDeg = 180.0F / 3.14159265F;
static constexpr float kFullCircleDeg = 360.0F;
static constexpr double kGpsCoordScale = 1e7;
static constexpr float kGps1e7ToDegreesF = 1e-7F;  // NOLINT(readability-identifier-naming)
static constexpr double kGps1e7ToDegrees = 1e-7;    // NOLINT(readability-identifier-naming)
// Erase/download input constants removed — blocking input now in AO_RCOS.
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
        rc::rc_log("Accel (m/s^2): X=%7.3f Y=%7.3f Z=%7.3f |A|=%.3f\n",
               (double)snap.accel_x, (double)snap.accel_y, (double)snap.accel_z,
               (double)aMag);
    } else {
        rc::rc_log("Accel: invalid\n");
    }
    if (snap.gyro_valid) {
        rc::rc_log("Gyro  (rad/s): X=%7.4f Y=%7.4f Z=%7.4f\n",
               (double)snap.gyro_x, (double)snap.gyro_y, (double)snap.gyro_z);
    } else {
        rc::rc_log("Gyro: invalid\n");
    }
    if (snap.mag_valid) {
        float magMag = sqrtf(snap.mag_x*snap.mag_x + snap.mag_y*snap.mag_y + snap.mag_z*snap.mag_z);
        rc::rc_log("Mag     (uT):  X=%6.1f  Y=%6.1f  Z=%6.1f  |M|=%.1f\n",
               (double)snap.mag_x, (double)snap.mag_y, (double)snap.mag_z, (double)magMag);
        float heading = atan2f(-snap.mag_y, snap.mag_x) * kRadToDeg;
        if (heading < 0.0F) { heading += kFullCircleDeg; }
        rc::rc_log("Heading: %.1f deg (level only)\n", (double)heading);
    } else {
        rc::rc_log("Mag: not ready\n");
    }
    if (snap.baro_valid) {
        float alt = calibration_get_altitude_agl(snap.pressure_pa);
        rc::rc_log("Baro: %.1f Pa, %.2f C, AGL=%.2f m\n",
               (double)snap.pressure_pa, (double)snap.baro_temperature_c,
               (double)alt);
    } else {
        rc::rc_log("Baro: no data yet\n");
    }
}

// NOLINTBEGIN(readability-magic-numbers) — ESKF P indices are state layout
static void print_eskf_gates_and_diags() {
    rc::rc_log("      gate: bA=%lu/%lu mA=%lu/%lu mR=%lu gA=%lu/%lu zA=%lu/%lu\n",
           (unsigned long)g_eskf.baro_total_accepts_,
           (unsigned long)(g_eskf.baro_total_accepts_ + g_eskf.baro_total_rejects_),
           (unsigned long)g_eskf.mag_total_accepts_,
           (unsigned long)(g_eskf.mag_total_accepts_ + g_eskf.mag_total_rejects_),
           (unsigned long)g_eskf.mag_resets_,
           (unsigned long)g_eskf.gps_pos_total_accepts_,
           (unsigned long)(g_eskf.gps_pos_total_accepts_ + g_eskf.gps_pos_total_rejects_),
           (unsigned long)g_eskf.zupt_total_accepts_,
           (unsigned long)(g_eskf.zupt_total_accepts_ + g_eskf.zupt_total_rejects_));
    rc::rc_log("      Pvel=%.4f,%.4f,%.4f  Pab=%.6f  Pgb=%.6f\n",
           (double)g_eskf.P(6, 6), (double)g_eskf.P(7, 7), (double)g_eskf.P(8, 8),
           (double)g_eskf.P(9, 9), (double)g_eskf.P(12, 12));
    // 24-state inhibit flags + conditional extended state display
    rc::rc_log("      inhib: mag=%c wind=%c bbias=%c\n",
           g_eskf.inhibit_mag_states_ ? 'Y' : 'N',
           g_eskf.inhibit_wind_states_ ? 'Y' : 'N',
           g_eskf.inhibit_baro_bias_ ? 'Y' : 'N');
    if (!g_eskf.inhibit_mag_states_) {
        rc::rc_log("      eMag=%.1f,%.1f,%.1f bMag=%.1f,%.1f,%.1f\n",
               (double)g_eskf.earth_mag.x, (double)g_eskf.earth_mag.y,
               (double)g_eskf.earth_mag.z,
               (double)g_eskf.body_mag_bias.x, (double)g_eskf.body_mag_bias.y,
               (double)g_eskf.body_mag_bias.z);
    }
    if (!g_eskf.inhibit_wind_states_) {
        rc::rc_log("      wind=%.2f,%.2f m/s\n",
               (double)g_eskf.wind_n_, (double)g_eskf.wind_e_);
    }
    if (!g_eskf.inhibit_baro_bias_) {
        rc::rc_log("      bBias=%.3f m\n", (double)g_eskf.baro_bias_);
    }
    // Phase Q/R + confidence gate status
    if (g_eskf.phase_qr_) {
        rc::rc_log("      phQ: att=%.1f vel=%.1f  innov: b=%.2f m=%.2f gp=%.2f gv=%.2f\n",
               (double)g_eskf.q_active_.attitude,
               (double)g_eskf.q_active_.velocity,
               (double)g_eskf.innov_baro_.alpha,
               (double)g_eskf.innov_mag_.alpha,
               (double)g_eskf.innov_gps_pos_.alpha,
               (double)g_eskf.innov_gps_vel_.alpha);
    }
    const rc::ConfidenceState* conf = eskf_runner_get_confidence();
    rc::rc_log("      conf=%c div=%.1f%c unc=%lums\n",
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
        rc::rc_log("ESKF: R=%6.2f P=%6.2f Y=%6.2f deg  Patt=%.4f  qnorm=%.6f\n",
               (double)rollDeg, (double)pitchDeg, (double)yawDeg,
               (double)patt, (double)g_eskf.q.norm());
        rc::rc_log("      vel=%.3f,%.3f,%.3f m/s  bNIS=%.2f mNIS=%.2f\n",
               (double)g_eskf.v.x, (double)g_eskf.v.y, (double)g_eskf.v.z,
               (double)g_eskf.last_baro_nis_,
               (double)g_eskf.last_mag_nis_);
        print_eskf_gates_and_diags();
        const rc::MahonyAHRS* mahony = eskf_runner_get_mahony();
        if (eskf_runner_is_mahony_initialized() && mahony->healthy()) {
            rc::Vec3 meuler = mahony->q.to_euler();
            float mdivDeg = rc::MahonyAHRS::divergence_rad(g_eskf.q, mahony->q) * kRadToDeg;
            rc::rc_log("Mahony: R=%6.2f P=%6.2f Y=%6.2f deg  Mdiv=%.1f deg\n",
                   (double)(meuler.x * kRadToDeg),
                   (double)(meuler.y * kRadToDeg),
                   (double)(meuler.z * kRadToDeg),
                   (double)mdivDeg);
        }
        // R-25-exec step 8 (2026-05-13): ifdef stripped. ESKF profiling
        // counters always run; CLI print is unconditional.
        {
            uint32_t benchAvg = 0, benchMin = 0, benchMax = 0, benchCount = 0;
            eskf_runner_get_bench(&benchAvg, &benchMin, &benchMax, &benchCount);
            if (benchCount > 0) {
                rc::rc_log("      predict: %luus avg, %luus min, %luus max (%lu calls)\n",
                       (unsigned long)benchAvg, (unsigned long)benchMin,
                       (unsigned long)benchMax, (unsigned long)benchCount);
            }
            uint32_t fAvg = 0, fMin = 0, fMax = 0, fCount = 0;
            eskf_runner_get_bench_full_tick(&fAvg, &fMin, &fMax, &fCount);
            if (fCount > 0) {
                rc::rc_log("      full-tick: %luus avg, %luus min, %luus max (%lu calls)\n",
                       (unsigned long)fAvg, (unsigned long)fMin,
                       (unsigned long)fMax, (unsigned long)fCount);
            }
        }
        rc::rc_log("      buf: %lu/%lu samples\n",
               (unsigned long)eskf_runner_get_buffer_count(), (unsigned long)kEskfBufferSamples);
    } else if (g_eskfInitialized) {
        rc::rc_log("ESKF: UNHEALTHY (stopped, awaiting re-init)\n");
    } else {
        rc::rc_log("ESKF: waiting for stationary init...\n");
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
            rc::rc_log("GPS (%s): %.7f, %.7f, %.1f m MSL\n",
                   gpsLabel,
                   snap.gps_lat_1e7 / kGpsCoordScale,
                   snap.gps_lon_1e7 / kGpsCoordScale,
                   (double)snap.gps_alt_msl_m);
            rc::rc_log("     Fix=%u Sats=%u Speed=%.1f m/s HDOP=%.2f VDOP=%.2f\n",
                   snap.gps_fix_type, snap.gps_satellites,
                   (double)snap.gps_ground_speed_mps,
                   (double)snap.gps_hdop, (double)snap.gps_vdop);
        } else {
            rc::rc_log("GPS (%s): no fix (%u sats)\n",
                   gpsLabel, snap.gps_satellites);
            rc::rc_log("     RMC=%c GGA=%u GSA=%u",
                   snap.gps_rmc_valid ? 'A' : 'V',
                   snap.gps_gga_fix,
                   snap.gps_gsa_fix_mode);
            if (g_gpsTransport == GPS_TRANSPORT_UART) {
                rc::rc_log("  rxOvf=%lu",
                       (unsigned long)gps_uart_get_overflow_count());
            }
            rc::rc_log("\n");
            if (g_bestGpsValid.load(std::memory_order_relaxed)) {
                rc::rc_log("     Best: %.7f, %.7f  Sats=%u HDOP=%.2f\n",
                       g_bestGpsFix.lat_1e7 / kGpsCoordScale,
                       g_bestGpsFix.lon_1e7 / kGpsCoordScale,
                       g_bestGpsFix.satellites,
                       (double)g_bestGpsFix.hdop);
            } else if (snap.gps_lat_1e7 != 0 || snap.gps_lon_1e7 != 0) {
                rc::rc_log("     Last fix: %.7f, %.7f\n",
                       snap.gps_lat_1e7 / kGpsCoordScale,
                       snap.gps_lon_1e7 / kGpsCoordScale);
            }
        }
    } else if (g_gpsInitialized) {
        rc::rc_log("GPS (%s): initialized, no reads yet\n", gpsLabel);
    } else {
        rc::rc_log("GPS: not detected\n");
    }
}

static void print_sensor_counts(const shared_sensor_data_t& snap) {
    rc::rc_log("Reads: I=%lu M=%lu B=%lu G=%lu  "
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
        rc::rc_log("  Log: %lu frames stored, %lu capacity\n",
               (unsigned long)rc::ring_stored_count(ring),
               (unsigned long)rc::ring_capacity_frames(ring));
    }
    {
        const rc::FlightTableState* ft = AO_Logger_get_flight_table();
        if (ft->loaded) {
            rc::rc_log("  Flash: %lu flights, %.1f%% used\n",
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
            rc::rc_log("Accel (m/s^2): X=%7.3f Y=%7.3f Z=%7.3f\n",
                   (double)ax, (double)ay, (double)az);
            rc::rc_log("Gyro  (rad/s): X=%7.4f Y=%7.4f Z=%7.4f\n",
                   (double)gx, (double)gy, (double)gz);
            if (data.mag_valid) {
                float mxCal = 0.0F;
                float myCal = 0.0F;
                float mzCal = 0.0F;
                calibration_apply_mag(data.mag.x, data.mag.y, data.mag.z,
                                      &mxCal, &myCal, &mzCal);
                float magMag = sqrtf(mxCal*mxCal + myCal*myCal + mzCal*mzCal);
                rc::rc_log("Mag     (uT):  X=%6.1f  Y=%6.1f  Z=%6.1f  |M|=%.1f\n",
                       (double)mxCal, (double)myCal, (double)mzCal, (double)magMag);
                float heading = atan2f(-myCal, mxCal) * kRadToDeg;
                if (heading < 0.0F) { heading += kFullCircleDeg; }
                rc::rc_log("Heading: %.1f deg (level only)\n", (double)heading);
            } else {
                rc::rc_log("Mag: not ready\n");
            }
            rc::rc_log("Temp: %.1f C\n", (double)data.temperature_c);
        } else {
            rc::rc_log("IMU: read failed\n");
        }
    } else {
        rc::rc_log("IMU: not initialized\n");
    }

    if (g_baroContinuous) {
        baro_dps310_data_t bdata;
        if (baro_dps310_read(&bdata) && bdata.valid) {
            float alt = calibration_get_altitude_agl(bdata.pressure_pa);
            rc::rc_log("Baro: %.1f Pa, %.2f C, AGL=%.2f m\n",
                   (double)bdata.pressure_pa, (double)bdata.temperature_c,
                   (double)alt);
        } else {
            rc::rc_log("Baro: read failed\n");
        }
    } else {
        rc::rc_log("Baro: not initialized\n");
    }
}

static void print_cal_params() {
    const calibration_store_t* cal = calibration_manager_get();
    uint16_t flags = cal->cal_flags;
    rc::rc_log("  Cal Flags: 0x%04X [%s%s%s%s%s]\n", flags,
           ((flags & CAL_STATUS_LEVEL) != 0) ? "Lv " : "",
           ((flags & CAL_STATUS_ACCEL_6POS) != 0) ? "6P " : "",
           ((flags & CAL_STATUS_GYRO) != 0) ? "Gy " : "",
           ((flags & CAL_STATUS_MAG) != 0) ? "Mg " : "",
           ((flags & CAL_STATUS_BARO) != 0) ? "Ba " : "");
    rc::rc_log("  Accel: off=[%.4f, %.4f, %.4f]\n",
           (double)cal->accel.offset.x, (double)cal->accel.offset.y,
           (double)cal->accel.offset.z);
    rc::rc_log("         scl=[%.4f, %.4f, %.4f]\n",
           (double)cal->accel.scale.x, (double)cal->accel.scale.y,
           (double)cal->accel.scale.z);
    rc::rc_log("         odg=[%.4f, %.4f, %.4f]\n",
           (double)cal->accel.offdiag.x, (double)cal->accel.offdiag.y,
           (double)cal->accel.offdiag.z);
    rc::rc_log("  Gyro:  bias=[%.6f, %.6f, %.6f]\n",
           (double)cal->gyro.bias.x, (double)cal->gyro.bias.y,
           (double)cal->gyro.bias.z);
    if ((flags & CAL_STATUS_MAG) != 0) {
        rc::rc_log("  Mag:   off=[%.1f, %.1f, %.1f]\n",
               (double)cal->mag.offset.x, (double)cal->mag.offset.y,
               (double)cal->mag.offset.z);
        rc::rc_log("         scl=[%.4f, %.4f, %.4f]\n",
               (double)cal->mag.scale.x, (double)cal->mag.scale.y,
               (double)cal->mag.scale.z);
    }
    const float* rotMat = cal->board_rotation.m;
    // NOLINTBEGIN(readability-magic-numbers) — row-major DCM indices [0..8]
    rc::rc_log("  Rot:   [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]\n",
           (double)rotMat[0], (double)rotMat[1], (double)rotMat[2],
           (double)rotMat[3], (double)rotMat[4], (double)rotMat[5],
           (double)rotMat[6], (double)rotMat[7], (double)rotMat[8]);
    // NOLINTEND(readability-magic-numbers)
}

// ============================================================================
// Sensor Status (vehicle mode 's' callback)
// ============================================================================

void cli_print_sensor_status() {
    rc::rc_log("\n========================================\n");
    rc::rc_log("  Sensor Readings (calibrated)\n");
    rc::rc_log("========================================\n");

    if (g_sensorPhaseActive) {
        shared_sensor_data_t snap = {};
        if (seqlock_read(&g_sensorSeqlock, &snap)) {
            print_seqlock_sensors(snap);
        } else {
            rc::rc_log("Seqlock read failed (retries exhausted)\n");
        }
    } else {
        print_direct_sensors();
    }

    print_cal_params();

    // WMM geomagnetic field info
    float wmmLat = 0.0F;
    float wmmLon = 0.0F;
    bool wmmValid = eskf_runner_get_wmm_position(&wmmLat, &wmmLon);
    if (wmmValid) {
        static constexpr const char* kWmmSrc[] = {"?", "default", "stored", "GPS"};
        uint8_t src = eskf_runner_get_wmm_source();
        rc::WmmField field = rc::wmm_get_field(wmmLat, wmmLon);
        rc::rc_log("WMM(%s): %.1f%c %.1f%c  D=%.1f%c I=%.1f%c F=%.1fuT\n",
               kWmmSrc[src < 4 ? src : 0],
               fabsf(wmmLat), wmmLat >= 0 ? 'N' : 'S',
               fabsf(wmmLon), wmmLon >= 0 ? 'E' : 'W',
               fabsf(field.declination_rad * 180.0F / 3.14159265F),
               field.declination_rad >= 0 ? 'E' : 'W',
               fabsf(field.inclination_rad * 180.0F / 3.14159265F),
               field.inclination_rad >= 0 ? 'D' : 'U',
               field.intensity_ut);
        rc::rc_log("Mag3D: %s", eskf_runner_mag_3d_active() ? "ON" : "OFF");
        if (eskf_runner_mag_3d_active()) {
            const rc::ESKF* eskf = eskf_runner_get_eskf();
            rc::rc_log("  eM=[%.1f %.1f %.1f] bB=[%.1f %.1f %.1f]",
                   static_cast<double>(eskf->earth_mag.x),
                   static_cast<double>(eskf->earth_mag.y),
                   static_cast<double>(eskf->earth_mag.z),
                   static_cast<double>(eskf->body_mag_bias.x),
                   static_cast<double>(eskf->body_mag_bias.y),
                   static_cast<double>(eskf->body_mag_bias.z));
        }
        rc::rc_log("\n");
    } else {
        rc::rc_log("WMM: no position (waiting for GPS or default)\n");
    }

    rc::rc_log("========================================\n\n");
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
            rc::rc_log("[----] I2C 0x%02X (%s): %s\n",
                   addr, get_device_name(addr),
                   found ? "FOUND" : "NOT FOUND");
            if (found) {
                foundCount++;
            }
        }
        rc::rc_log("[INFO] Sensors found: %d/%zu expected\n",
               foundCount, sizeof(kExpected) / sizeof(kExpected[0]));
    } else {
        rc::rc_log("[INFO] I2C probe skipped (Core 1 owns bus)\n");
    }
}

static void print_imu_status() {
    if (g_imuInitialized) {
        rc::rc_log("[PASS] ICM-20948 init (WHO_AM_I=0xEA)\n");
        rc::rc_log("[%s] AK09916 magnetometer %s\n",
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
            rc::rc_log("  IMU config: accelDLPF=%u(%s) gyroDLPF=%u(%s) gyroDiv=%u\n",
                   accelDlpf, accelDlpfEn ? "on" : "off",
                   gyroDlpf, gyroDlpfEn ? "on" : "off",
                   gyroDiv);
        }
        return;
    }
    rc::rc_log(g_imuInitAttempted ? "[FAIL] ICM-20948 init failed\n"
                              : "[N/A ] ICM-20948 not installed\n");
}

static void print_baro_status() {
    if (g_baroInitialized && g_baroContinuous) {
        rc::rc_log("[PASS] DPS310 init OK, continuous mode active\n");
        return;
    }
    if (g_baroInitialized) {
        rc::rc_log("[WARN] DPS310 init OK, continuous mode failed\n");
        return;
    }
    rc::rc_log(g_baroInitAttempted ? "[FAIL] DPS310 init failed\n"
                               : "[N/A ] DPS310 not installed\n");
}

static void print_gps_status() {
    if (g_gpsInitialized) {
        rc::rc_log(g_gpsTransport == GPS_TRANSPORT_UART
                   ? "[PASS] GPS init (UART on GPIO0/1, 57600 baud)\n"
                   : "[PASS] GPS init (I2C at 0x10, 500us settling delay)\n");
    } else {
        rc::rc_log(g_gpsInitAttempted ? "[FAIL] GPS init failed\n"
                                  : "[N/A ] GPS not installed\n");
    }

    // Grok-triage debug: show PMTK write return codes + window-hit flag
    // captured by the ultra-early gps_pa1010d_init() call in init_early_hw()
    // (printf from there is dropped because USB CDC is not up yet).
    char gpsDbg[96] = { 0 };
    gps_pa1010d_get_debug_status(gpsDbg, sizeof(gpsDbg));
    rc::rc_log("[DBG ] GPS early-init: %s\n", gpsDbg);
}

static void hw_validate_sensors() {
    print_imu_status();
    print_baro_status();
    print_gps_status();
}

static void print_psram_status() {
    if (g_psramSize > 0) {
        rc::rc_log("[%s] PSRAM: %luMB at 0x%08lX",
               g_psramSelfTestPassed ? "PASS" : "FAIL",
               (unsigned long)(g_psramSize / (1024 * 1024)),
               (unsigned long)rc::kPsramCachedBase);
        if (g_psramSelfTestPassed) {
            rc::rc_log(" self-test OK");
        } else {
            rc::rc_log(" self-test FAIL");
        }
        if (g_psramFlashSafePassed) {
            rc::rc_log(", flash-safe OK\n");
        } else {
            rc::rc_log(", flash-safe %s\n",
                   g_psramSelfTestPassed ? "FAIL" : "skipped");
        }
    } else {
        rc::rc_log("[----] PSRAM: not detected (GPIO %d)\n",
               rocketchip::pins::kPsramCs);
    }
}

static void print_logging_status() {
    if (AO_Logger_is_initialized()) {
        bool isPsram = (g_psramSize > 0 && g_psramSelfTestPassed);
        uint32_t rate = isPsram ? 50U : 25U;
        const rc::RingBuffer* ring = AO_Logger_get_ring();
        rc::rc_log("[PASS] Logging: %s ring, %luHz, %lu frames capacity, %lu stored\n",
               isPsram ? "PSRAM" : "SRAM",
               (unsigned long)rate,
               (unsigned long)rc::ring_capacity_frames(ring),
               (unsigned long)rc::ring_stored_count(ring));
    } else {
        rc::rc_log("[----] Logging: not initialized\n");
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
        rc::rc_log("[PASS] Flash: %.1f%% used (%lu flights, %luMB free)\n",
               static_cast<double>(usedPct),
               (unsigned long)nFlights,
               (unsigned long)freeMB);
    } else {
        rc::rc_log("[INFO] Flash: flight table empty (fresh)\n");
    }
}

static void print_gps_status_boot() {
    if (g_bestGpsValid.load(std::memory_order_acquire)) {
        rc::rc_log("  Best GPS: Fix=%u Sats=%u HDOP=%.2f\n",
               g_bestGpsFix.fix_type, g_bestGpsFix.satellites,
               (double)g_bestGpsFix.hdop);
        rc::rc_log("            %.7f, %.7f, %.1f m MSL\n",
               g_bestGpsFix.lat_1e7 / kGpsCoordScale,
               g_bestGpsFix.lon_1e7 / kGpsCoordScale,
               (double)g_bestGpsFix.alt_msl_m);
    } else if (g_gpsInitialized) {
        rc::rc_log("  Best GPS: no fix acquired yet\n");
    }

    const gps_session_stats_t* gpsSess = eskf_runner_get_gps_session();
    if (gpsSess->gps_updates > 0) {
        rc::rc_log("  GPS session: %lu updates, max_dist=%.1fm, last_dist=%.1fm\n",
               (unsigned long)gpsSess->gps_updates,
               (double)gpsSess->max_dist_from_origin_m,
               (double)gpsSess->last_dist_from_origin_m);
        rc::rc_log("              last_pos N=%.1f E=%.1f m  gNIS=[%.2f, %.2f]\n",
               (double)gpsSess->last_pos_n_m,
               (double)gpsSess->last_pos_e_m,
               (double)gpsSess->min_gps_nis,
               (double)gpsSess->max_gps_nis);
    }
}

// Build tag constant — must match main.cpp
// Build tag from version.h (single source of truth)

void cli_print_hw_status() {
    rc::rc_log("\n=== Hardware Status ===\n");
    rc::rc_log("  Build: %s-%s-%s (%s %s)\n",
           kFirmwareVersion, kBuildConfig, kGitHash, __DATE__, __TIME__);

    rc::rc_log("[PASS] Build + boot (you're reading this)\n");
    rc::rc_log("[PASS] Red LED GPIO initialized (pin %d, %s)\n",
           board::kLedPin, board::kLedActiveHigh ? "active-high" : "active-low");
    rc::rc_log("[%s] NeoPixel PIO initialized (pin %d)\n",
           g_neopixelInitialized ? "PASS" : "FAIL", board::kNeoPixelPin);
    rc::rc_log("[PASS] USB CDC connected\n");

    uint32_t ts = time_us_32();
    rc::rc_log("[%s] Debug macros functional (timestamp=%lu us)\n",
           ts > 0 ? "PASS" : "FAIL", (unsigned long)ts);

    if (g_i2cInitialized) {
        rc::rc_log("[PASS] I2C bus initialized at %lukHz (SDA=%d, SCL=%d)\n",
               (unsigned long)(kI2cBusFreqHz / 1000), kI2cBusSdaPin, kI2cBusSclPin);
    } else {
        rc::rc_log("[FAIL] I2C bus failed to initialize\n");
    }

    hw_validate_i2c_devices();
    hw_validate_sensors();

    if (AO_Radio_get_state()->initialized) {
        rc::rc_log("[PASS] Radio: RFM95W LoRa 915 MHz SF7 20dBm (CS=%d RST=%d IRQ=%d)\n",
               rocketchip::pins::kRadioCs, rocketchip::pins::kRadioRst,
               rocketchip::pins::kRadioIrq);
        if constexpr (kRadioModeRx) {
            auto mode = AO_RCOS_get_output_mode();
            const char* mode_name = (mode == StationOutputMode::kAnsi) ? "ANSI dashboard" :
                                    (mode == StationOutputMode::kCsv)  ? "CSV" : "MAVLink";
            rc::rc_log("[MODE] RX — %s output ('m' to cycle)\n", mode_name);
        } else {
            rc::rc_log("[MODE] TX CCSDS %dB%s\n",
                   static_cast<int>(rc::ccsds::kNavPacketLen),
                   AO_Telemetry_get_mavlink_output() ? " + USB MAVLink" : "");
        }
    } else if (g_spiInitialized) {
        rc::rc_log("[----] Radio: not detected (module unpopulated?)\n");
    }

    print_psram_status();
    print_logging_status();
    print_flash_status();
    print_gps_status_boot();

    rc::rc_log("=== Status Complete ===\n\n");
}

// ============================================================================
// Boot Banner
// ============================================================================

// Watchdog timeout constant — must match main.cpp
static constexpr uint32_t kWatchdogTimeoutMs = 5000;

// Count HW init pass/fail for boot summary
static void count_hw_checks(uint8_t& pass, uint8_t& fail) {
    pass = 0;
    fail = 0;
    auto check = [&](bool ok) { if (ok) { ++pass; } else { ++fail; } };
    check(true);                                    // Build + boot
    check(true);                                    // Red LED GPIO
    check(g_neopixelInitialized);                   // NeoPixel
    check(true);                                    // USB CDC
    check(time_us_32() > 0);                        // Debug macros
    check(g_i2cInitialized);                        // I2C bus
    // Sensors: "OK" == initialized, or "not counted at all" == not
    // installed (IVP-142c A2). A sensor the role doesn't use (station
    // IMU/baro) shouldn't flag as FAIL in the boot summary.
    auto check_sensor = [&](bool attempted, bool initialized) {
        if (!attempted) { return; }             // not counted
        if (initialized) { ++pass; } else { ++fail; }
    };
    check_sensor(g_imuInitAttempted, g_imuInitialized);   // ICM-20948
    check_sensor(g_imuInitAttempted, g_imuInitialized);   // AK09916 (same init)
    check_sensor(g_baroInitAttempted, g_baroInitialized); // DPS310
    check_sensor(g_gpsInitAttempted, g_gpsInitialized);   // GPS
    check(AO_Radio_get_state()->initialized);       // Radio
    check(true);                                    // PSRAM
    check(true);                                    // Logging
    check(true);                                    // Flash
}

// Print specific FAIL items. Sensors only flag as FAIL when attempted
// AND init returned false (IVP-142c A2) — uninstalled sensors on this
// role are silent.
static void print_hw_failures() {
    if (!g_neopixelInitialized) { rc::rc_log("  [FAIL] NeoPixel\n"); }
    if (!g_i2cInitialized)      { rc::rc_log("  [FAIL] I2C bus\n"); }
    if (g_imuInitAttempted  && !g_imuInitialized)  { rc::rc_log("  [FAIL] ICM-20948 IMU\n"); }
    if (g_baroInitAttempted && !g_baroInitialized) { rc::rc_log("  [FAIL] DPS310 barometer\n"); }
    if (g_gpsInitAttempted  && !g_gpsInitialized)  { rc::rc_log("  [FAIL] GPS\n"); }
    if (!AO_Radio_get_state()->initialized && g_spiInitialized) {
        rc::rc_log("  [FAIL] Radio\n");
    }
}

// Compact boot summary — auto-printed on terminal connect.
void cli_print_boot_summary() {
    uint8_t pass = 0;
    uint8_t fail = 0;
    count_hw_checks(pass, fail);
    uint8_t total = pass + fail;

    rc::rc_log("\n");
    rc::rc_log("==============================================\n");
    rc::rc_log("  RocketChip v%s  RCOS v%s  %s-%s\n",
           kFirmwareVersion, kRcOsVersion, kBuildConfig, kGitHash);
    rc::rc_log("  Board: %s\n", board::kBoardName);
    rc::rc_log("  Profile: %s  Uptime: %lus\n",
           rc::kDefaultRocketProfile.name,
           (unsigned long)(to_ms_since_boot(get_absolute_time()) / 1000));
    rc::rc_log("==============================================\n");

    if (rc::flight_director_launch_abort()) {
        rc::rc_log("[WARN] LAUNCH ABORT — power cycle required to clear\n");
    }

    if (fail == 0) {
        rc::rc_log("Hardware: %u/%u OK\n", pass, total);
    } else {
        rc::rc_log("Hardware: %u/%u OK (%u FAIL)\n", pass, total, fail);
        print_hw_failures();
    }

    // Flash summary
    rc::FlightTableState* ft = AO_Logger_get_flight_table_mut();
    if (ft != nullptr && ft->loaded) {
        uint32_t count = rc::flight_table_count(ft);
        uint32_t used_sectors = ft->table.next_free_sector - rc::kFlightLogStart / rc::kFlashSectorSize;
        uint32_t pct = (used_sectors * 100) / rc::kFlightLogSectors;
        rc::rc_log("Flash: %lu flights, %lu%% used\n",
               (unsigned long)count, (unsigned long)pct);
    }
}

// Full boot status — called by cli_print_boot_status() and 'b' key
void cli_print_boot_status() {
    cli_print_boot_summary();
    rc::rc_log("\n");
    cli_print_hw_status();
}

// ============================================================================
// ESKF Live Output (1Hz, 'e' key)
// ============================================================================

void cli_print_eskf_live() {
    if (!g_eskfInitialized) {
        rc::rc_log("ESKF: waiting for init...\n");
        return;
    }
    if (!g_eskf.healthy()) {
        rc::rc_log("ESKF: UNHEALTHY\n");
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
    rc::rc_log("alt=%.2f vz=%.2f vh=%.2f Y=%.1f Patt=%.4f Pp=%.4f bNIS=%.2f mNIS=%.2f mA=%lu/%lu Z=%c zNIS=%.2f G=%c gNIS=%.2f Mdiv=%.1f B=%lu\n",
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
// Station RX Telemetry Display
// ============================================================================

static void print_station_rx_fields(const rc::TelemetryState& t,
                                     const RadioAoState* rs,
                                     uint32_t met_ms, uint16_t seq) {
    static constexpr float kMmToM  = 0.001F;
    static constexpr float kMmToFt = 0.00328084F;
    static constexpr float kCmsToMs = 0.01F;

    float alt_m  = static_cast<float>(t.baro_alt_mm) * kMmToM;
    float alt_ft = static_cast<float>(t.baro_alt_mm) * kMmToFt;
    float vvel   = static_cast<float>(t.baro_vvel_cms) * kCmsToMs;
    uint8_t fix  = (t.gps_fix_sats >> 4) & 0x0F;
    uint8_t sats = t.gps_fix_sats & 0x0F;
    bool eskf_ok = (rc::health_eskf(t.health) >= rc::kHealthDegraded);

    uint32_t age_ms = to_ms_since_boot(get_absolute_time()) - rs->last_rx_ms;
    uint32_t lost = 0;
    if (rs->rx_count > 1) {
        uint32_t expected = static_cast<uint32_t>(rs->last_rx_seq) + 1;
        if (expected > rs->rx_count) { lost = expected - rs->rx_count; }
    }

    const char* phase = rc::flight_phase_name(
        static_cast<rc::FlightPhase>(t.flight_state));

    rc::rc_log("State: %-8s  Pkts: %lu (%lu lost)\n", phase,
           (unsigned long)rs->rx_count, (unsigned long)lost);
    rc::rc_log("Alt:   %.1f m (%.0f ft)  Vvel: %.1f m/s\n",
           static_cast<double>(alt_m), static_cast<double>(alt_ft),
           static_cast<double>(vvel));
    rc::rc_log("RSSI:  %d dBm  SNR: %d dB\n",
           static_cast<int>(rs->last_rx_rssi),
           static_cast<int>(rs->last_rx_snr));
    rc::rc_log("GPS:   fix=%u sats=%u  Batt: %.2f V\n",
           static_cast<unsigned>(fix), static_cast<unsigned>(sats),
           static_cast<double>(t.battery_mv) * 0.001);
    rc::rc_log("ESKF:  %s  Temp: %d C\n",
           eskf_ok ? "HEALTHY" : "UNHEALTHY",
           static_cast<int>(t.temperature_c));
    rc::rc_log("Last:  %lu.%lus ago  MET: %lu.%lus  seq=%u\n",
           (unsigned long)(age_ms / 1000), (unsigned long)((age_ms % 1000) / 100),
           (unsigned long)(met_ms / 1000), (unsigned long)((met_ms % 1000) / 100),
           static_cast<unsigned>(seq));
    if (rs->rx_crc_errors > 0) {
        rc::rc_log("CRC errors: %lu\n", (unsigned long)rs->rx_crc_errors);
    }
}

void cli_print_station_status() {
    const auto* rs = AO_Radio_get_state();
    const auto* rx = AO_Telemetry_get_rx_state();

    rc::rc_log("\n=== RocketChip Station ===\n");
    if (!rs->initialized) {
        rc::rc_log("Radio: not initialized\n");
        return;
    }
    if (!rx->valid) {
        rc::rc_log("Waiting for vehicle packets...\n");
        rc::rc_log("RX: %lu pkts  %lu CRC err\n",
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
        rc::rc_log("Logging not initialized.\n");
        return;
    }
    rc::FlightTableState* ft = AO_Logger_get_flight_table_mut();
    if (!ft->loaded) {
        rc::rc_log("Flight table not loaded.\n");
        return;
    }

    rc::RingBuffer* ring = AO_Logger_get_ring_mut();
    uint32_t stored = rc::ring_stored_count(ring);
    if (stored == 0) {
        rc::rc_log("No frames in ring buffer.\n");
        return;
    }

    bool isPsram = (g_psramSize > 0 && g_psramSelfTestPassed);
    uint8_t rate = isPsram ? 50U : 25U;

    rc::rc_log("Flushing %lu frames to flash...\n", (unsigned long)stored);

    rc::FlightMetadata meta = {};
    rc::FlightSummary summ = {};
    summ.frame_count = stored;

    // R-17 (2026-05-07 audit): pause Core 1 I2C BEFORE the flush op chain
    // to prevent the LL-31 race. flush_ring_to_flash() invokes
    // flash_safe_execute() many times; each invocation is a separate
    // multicore_lockout window. Pausing Core 1 once around the whole flush
    // is far cheaper than pausing per-invocation.
    rc::core1_i2c_pause();

    rc::FlushResult result = rc::flush_ring_to_flash(
        ring, ft, &meta, &summ, rate, flush_kick_watchdog);

    // R-15 (2026-05-07 audit): flush_ring_to_flash() invokes flash_safe_execute()
    // many times; per LL Entry 31, runtime flash_safe_execute() leaves the I2C
    // peripheral in a corrupted state on RP2350 until a peripheral reset clears
    // it. Apply the same reset pattern that ao_rcos.cpp cal_save_to_flash()
    // uses (line 338). Run regardless of FlushResult — even a partial flush
    // means flash_safe_execute() ran. R-17 added the pause BEFORE this point;
    // the reset is still required as belt-and-suspenders against any in-flight
    // transaction that didn't drain in time.
    if (!i2c_bus_reset()) {
        rc::rc_log("[WARN] I2C bus reset failed after flush\n");
    }
    rc::core1_i2c_resume();

    switch (result) {
    case rc::FlushResult::kOk:
        rc::rc_log("Flush OK. Flight #%lu saved (%lu frames, %lu sectors).\n",
               (unsigned long)rc::flight_table_count(ft),
               (unsigned long)stored,
               (unsigned long)(stored * rc::kPcmFrameStandardSize + rc::kFlashSectorSize - 1)
                   / rc::kFlashSectorSize);
        break;
    case rc::FlushResult::kFlashFull:
        rc::rc_log("Flash full. Erase flights with 'x' command.\n");
        break;
    case rc::FlushResult::kTableFull:
        rc::rc_log("Flight table full (32 flights). Erase with 'x'.\n");
        break;
    case rc::FlushResult::kEraseError:
        rc::rc_log("Flash erase error.\n");
        break;
    case rc::FlushResult::kWriteError:
        rc::rc_log("Flash write error.\n");
        break;
    case rc::FlushResult::kTableSaveError:
        rc::rc_log("Flight table save error.\n");
        break;
    default:
        rc::rc_log("Flush error: %d\n", static_cast<int>(result));
        break;
    }
}

// Erase confirmation prompt + input now in AO_RCOS (non-blocking).
// This function is the completion callback — does the actual erase.
void cli_do_erase_flights() {
    rc::FlightTableState* ft = AO_Logger_get_flight_table_mut();
    rc::rc_log("Erasing flight log sectors...\n");
    // R-17 (2026-05-07 audit): pause Core 1 I2C BEFORE the flash op chain
    // to prevent the LL-31 race (in-flight I2C transactions corrupted by
    // multicore_lockout). See R-11 SPIN model + ao_rcos.cpp cal_save_to_flash
    // for the same pattern.
    rc::core1_i2c_pause();
    bool ok = true;
    if (!rc::flight_log_erase_all(ft, flush_kick_watchdog)) {
        rc::rc_log("Flash erase error.\n");
        ok = false;
    } else if (!rc::flight_table_erase_flash()) {
        rc::rc_log("Table erase error.\n");
        ok = false;
    } else {
        rc::flight_table_erase_all(ft);
        ft->loaded = true;
        rc::flight_table_save(ft);
        rc::rc_log("All flights erased.\n");
    }

    // R-15 (2026-05-07 audit): per LL Entry 31, every runtime flash_safe_execute()
    // must be followed by i2c_bus_reset() — matches ao_rcos.cpp:338 protocol on
    // cal_save_to_flash. Even on partial failure above, flash_safe_execute() ran
    // for some sectors before bailing, so the reset is required regardless of ok.
    (void)ok;
    if (!i2c_bus_reset()) {
        rc::rc_log("[WARN] I2C bus reset failed after erase\n");
    }
    // R-17: resume Core 1 sensor reads.
    rc::core1_i2c_resume();
}

// ============================================================================
// Flight Download
// ============================================================================

static void cmd_list_flights() {
    const rc::FlightTableState* ft = AO_Logger_get_flight_table();
    if (!ft->loaded) {
        rc::rc_log("Flight table not loaded.\n");
        return;
    }
    uint32_t count = rc::flight_table_count(ft);
    if (count == 0) {
        rc::rc_log("No flights stored.\n");
        return;
    }
    rc::rc_log("\n  #  Frames  Rate  Sectors  Size(KB)\n");
    rc::rc_log("  -- ------  ----  -------  --------\n");
    for (uint32_t i = 0; i < count; ++i) {
        rc::FlightLogEntry entry = {};
        if (rc::flight_table_get_entry(ft, i, &entry)) {
            uint32_t sizeKb = (entry.sector_count * rc::kFlashSectorSize) / 1024;
            rc::rc_log("  %2lu %6lu  %3uHz  %5lu  %6lu\n",
                   (unsigned long)(i + 1),
                   (unsigned long)entry.frame_count,
                   (unsigned)entry.log_rate_hz,
                   (unsigned long)entry.sector_count,
                   (unsigned long)sizeKb);
        }
    }
    rc::rc_log("\n  Flash: %.1f%% used (%lu flights)\n\n",
           static_cast<double>(rc::flight_table_used_pct(ft)),
           (unsigned long)count);
}

// read_flight_number() removed — blocking input now in AO_RCOS (non-blocking).
// Flight number input via AO_RCOS_start_download_flight() → cal_ui_handle_flight_num().

// Block-with-watchdog-feed write — binary download path waits for CDC
// backpressure rather than dropping. Operator wants complete data.
static void cdc_write_blocking(const uint8_t* buf, uint32_t len) {
    if (!tud_cdc_connected()) { return; }
    uint32_t written = 0;
    while (written < len) {
        uint32_t avail = tud_cdc_write_available();
        if (avail == 0) {
            rc::pio_watchdog_feed();
        } else {
            uint32_t chunk = (avail < (len - written)) ? avail : (len - written);
            tud_cdc_write(buf + written, chunk);
            written += chunk;
        }
    }
}

static uint32_t stream_flight_binary(const rc::FlightLogEntry& entry) {
    uint32_t frame_size = entry.frame_size;
    uint32_t frames_per_sector = rc::kFlashSectorSize / frame_size;
    uint32_t flash_base = entry.start_sector * rc::kFlashSectorSize;
    const uint8_t* xip_base = reinterpret_cast<const uint8_t*>(
        XIP_BASE + flash_base);

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

        cdc_write_blocking(xip_base + offset, batch_bytes);

        running_crc = rc::crc32_update(running_crc,
                                       xip_base + offset, batch_bytes);
        frames_sent += batch;
        rc::pio_watchdog_feed();
    }
    return running_crc ^ kCrc32InitXor;
}

// Download prompt + flight number input now in AO_RCOS (non-blocking).
// This function is the completion callback — does the actual download.
void cli_do_download_flight(int num) {
    const rc::FlightTableState* ft = AO_Logger_get_flight_table();
    uint32_t count = rc::flight_table_count(ft);
    if (num < 1 || static_cast<uint32_t>(num) > count) {
        rc::rc_log("Invalid flight number.\n");
        return;
    }

    rc::FlightLogEntry entry = {};
    if (!rc::flight_table_get_entry(ft, static_cast<uint32_t>(num - 1),
                                     &entry)) {
        rc::rc_log("Failed to read flight entry.\n");
        return;
    }

    rc::rc_log("RCBIN:%d:%lu:%u:%lu\n",
           num,
           (unsigned long)entry.frame_count,
           (unsigned)entry.log_rate_hz,
           (unsigned long)entry.frame_size);

    uint32_t crc = stream_flight_binary(entry);
    rc::rc_log("RCEND:%08lX\n", (unsigned long)crc);
}

// ============================================================================
// Radio Status
// ============================================================================

static void cmd_radio_status() {
    const auto* rs = AO_Radio_get_state();
    if (!rs->initialized) {
        rc::rc_log("Radio not initialized.\n");
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if constexpr (kRadioModeRx) {
        uint32_t gap = (rs->rx_count > 0) ? (now - rs->last_rx_ms) : 0;
        rc::rc_log("RX: %lu pkts  seq=%u  %ddBm  %ddB SNR  %lu CRC err\n",
               (unsigned long)rs->rx_count,
               static_cast<unsigned>(rs->last_rx_seq),
               static_cast<int>(rs->last_rx_rssi),
               static_cast<int>(rs->last_rx_snr),
               (unsigned long)rs->rx_crc_errors);
        rc::rc_log("    last=%lu.%lus ago  phase=%d\n",
               (unsigned long)(gap / 1000),
               (unsigned long)((gap % 1000) / 100),
               static_cast<int>(rs->scheduler.phase));
        if constexpr (job::kRole == job::DeviceRole::kRelay) {
            rc::rc_log("    relayed=%lu\n", (unsigned long)rs->relay_count);
        }
    } else {
        rc::rc_log("TX: %lu sent  %u fail  phase=%d\n",
               (unsigned long)rs->tx_count,
               static_cast<unsigned>(rs->tx_consec_fail),
               static_cast<int>(rs->scheduler.phase));
    }

    // IVP-T11 boot-register audit. Printed on every `t` so the Batch A
    // gate ("RegInvertIQ=0x27, CRC on, LNA=0x23, CFG3=0x04") is always
    // verifiable post-boot.
    if (rs->boot_audit_valid) {
        const rfm95w_audit_t& a = rs->boot_audit;
        bool iq_ok   = (a.invert_iq == kAuditInvertIqExpected);
        bool crc_ok  = (a.modem_config2 & kAuditModemCfg2CrcBitMask) != 0;
        bool lna_ok  = (a.lna == kAuditLnaExpected);
        bool cfg3_ok = (a.modem_config3 == kAuditModemCfg3Expected);
        rc::rc_log("Audit: IQ=0x%02x(%s) CFG2=0x%02x CRC=%s LNA=0x%02x(%s) CFG3=0x%02x(%s)\n",
               a.invert_iq,     iq_ok   ? "OK" : "MISMATCH",
               a.modem_config2, crc_ok  ? "on" : "OFF",
               a.lna,           lna_ok  ? "OK" : "MISMATCH",
               a.modem_config3, cfg3_ok ? "OK" : "MISMATCH");
    }
}

// ============================================================================
// Station GPS Commands
// ============================================================================

[[maybe_unused]]
static float haversine_m(int32_t lat1_e7, int32_t lon1_e7,
                          int32_t lat2_e7, int32_t lon2_e7) {
    static constexpr float kDegToRad = 3.14159265F / 180.0F;
    static constexpr float kEarthR   = 6371000.0F;
    static constexpr float kScale    = 1e-7F;

    float lat1 = static_cast<float>(lat1_e7) * kScale * kDegToRad;
    float lat2 = static_cast<float>(lat2_e7) * kScale * kDegToRad;
    float dlat = lat2 - lat1;
    float dlon = (static_cast<float>(lon2_e7 - lon1_e7)) * kScale * kDegToRad;

    float a = sinf(dlat * 0.5F) * sinf(dlat * 0.5F)
            + cosf(lat1) * cosf(lat2) * sinf(dlon * 0.5F) * sinf(dlon * 0.5F);
    float c = 2.0F * atan2f(sqrtf(a), sqrtf(1.0F - a));
    return kEarthR * c;
}

static void cmd_station_gps() {
    if constexpr (!kRadioModeRx) { return; }
    if (!g_gpsInitialized) {
        rc::rc_log("Station GPS: not connected\n");
        return;
    }
    rc::rc_log("Station GPS: fix=%u sats=%u hdop=%.1f\n",
           g_bestGpsFix.fix_type, g_bestGpsFix.satellites,
           static_cast<double>(g_bestGpsFix.hdop));
    rc::rc_log("  Lat=%.7f Lon=%.7f Alt=%.1fm\n",
           static_cast<double>(g_bestGpsFix.lat_1e7) / 1e7,
           static_cast<double>(g_bestGpsFix.lon_1e7) / 1e7,
           static_cast<double>(g_bestGpsFix.alt_msl_m));
}

// IVP-123: bearing from station to vehicle (degrees, 0-360)
[[maybe_unused]]
static float bearing_deg(int32_t lat1_e7, int32_t lon1_e7,
                          int32_t lat2_e7, int32_t lon2_e7) {
    static constexpr float kDegToRad = 3.14159265F / 180.0F;
    // kRadToDeg: use the file-scope constant (no shadowing local) — see top of file.
    static constexpr float kScale    = 1e-7F;

    float lat1 = static_cast<float>(lat1_e7) * kScale * kDegToRad;
    float lat2 = static_cast<float>(lat2_e7) * kScale * kDegToRad;
    float dlon = static_cast<float>(lon2_e7 - lon1_e7) * kScale * kDegToRad;

    float y = sinf(dlon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);
    float bearing = atan2f(y, x) * kRadToDeg;
    return fmodf(bearing + 360.0F, 360.0F);
}

static void cmd_station_distance() {
    if constexpr (!kRadioModeRx) { return; }
    if (!g_gpsInitialized || g_bestGpsFix.fix_type < 2) {
        rc::rc_log("Distance: station GPS has no fix\n");
        return;
    }
    const RxTelemSnapshot* rx = AO_Telemetry_get_rx_state();
    if (!rx || !rx->valid) {
        rc::rc_log("Distance: no vehicle telemetry received\n");
        return;
    }
#ifndef ROCKETCHIP_HOST_TEST
    uint32_t age_ms = to_ms_since_boot(get_absolute_time()) - rx->met_ms;
    if (age_ms > 5000) {
        rc::rc_log("Distance: telemetry stale (%lu ms)\n", static_cast<unsigned long>(age_ms));
        return;
    }
#endif
    if (rx->telem.lat_1e7 == 0 && rx->telem.lon_1e7 == 0) {
        rc::rc_log("Distance: no vehicle GPS\n");
        return;
    }

    float dist = haversine_m(g_bestGpsFix.lat_1e7, g_bestGpsFix.lon_1e7,
                             rx->telem.lat_1e7, rx->telem.lon_1e7);
    float brg = bearing_deg(g_bestGpsFix.lat_1e7, g_bestGpsFix.lon_1e7,
                            rx->telem.lat_1e7, rx->telem.lon_1e7);
    rc::rc_log("Distance: %.0f m  Bearing: %.0f deg\n",
           static_cast<double>(dist), static_cast<double>(brg));
}

// ============================================================================
// Unhandled Key Dispatcher
// ============================================================================

// Station GPS position push to vehicle over LoRa
static void cmd_station_gps_push() {
    shared_sensor_data_t snap = {};
    if (seqlock_read(&g_sensorSeqlock, &snap) &&
        snap.gps_valid && snap.gps_fix_type >= 3) {
        float lat = static_cast<float>(snap.gps_lat_1e7) * 1e-7F;
        float lon = static_cast<float>(snap.gps_lon_1e7) * 1e-7F;
        AO_Telemetry_send_command(kMavCmdSetHome, {.p5 = lat, .p6 = lon});
        rc::rc_log("GPS push: %.5f, %.5f\n",
               static_cast<double>(lat), static_cast<double>(lon));
    } else {
        rc::rc_log("No GPS 3D fix on station\n");
    }
}

// ============================================================================
// Preflight Go/No-Go Poll (IVP-110)
// ============================================================================

static const char* health_level_str(rc::HealthLevel level) {
    switch (level) {
        case rc::kHealthOk:       return "GO";
        case rc::kHealthDegraded: return "DEGRADED";
        case rc::kHealthFault:    return "FAULT";
        case rc::kHealthAbsent:   return "ABSENT";
        default:                  return "?";
    }
}

static bool is_go(rc::HealthLevel level) {
    return level >= rc::kHealthDegraded;  // OK or degraded = GO
}

static void preflight_print_primary(const rc::HealthState* hs) {
    rc::HealthLevel imu  = rc::health_imu(hs->primary);
    rc::HealthLevel baro = rc::health_baro(hs->primary);
    rc::HealthLevel eskf = rc::health_eskf(hs->primary);
    rc::HealthLevel gps  = rc::health_gps(hs->primary);

    rc::rc_log("IMU:      %s\n", is_go(imu)  ? "GO" : health_level_str(imu));
    rc::rc_log("Baro:     %s\n", is_go(baro) ? "GO" : health_level_str(baro));
    rc::rc_log("ESKF:     %s\n", is_go(eskf) ? "GO" : health_level_str(eskf));

    // GPS: show fix detail on non-GO
    if (is_go(gps)) {
        rc::rc_log("GPS:      GO\n");
    } else {
        shared_sensor_data_t snap{};
        seqlock_read(&g_sensorSeqlock, &snap);
        rc::rc_log("GPS:      %s  fix=%u sats=%u\n",
               health_level_str(gps),
               static_cast<unsigned>(snap.gps_fix_type),
               static_cast<unsigned>(snap.gps_satellites));
    }
}

static void preflight_print_secondary(const rc::HealthState* hs) {
    rc::rc_log("Radio HW: %s\n", (hs->secondary & rc::kHealthRadioOk)    ? "GO" : "ABSENT");
    rc::rc_log("Flash:    %s\n", (hs->secondary & rc::kHealthFlashOk)    ? "GO" : "FAULT");
    rc::rc_log("Watchdog: %s\n", (hs->secondary & rc::kHealthWatchdogOk) ? "GO" : "FAULT");
    rc::rc_log("PIO WDT:  %s\n", (hs->secondary & rc::kHealthPioOk)      ? "GO" : "FAULT");
}

static void preflight_print_mcu_and_critical(const rc::HealthState* hs) {
    // MCU die temp (Stage 16C IVP-142b-1) — separate field, not in primary.
    shared_sensor_data_t snap{};
    seqlock_read(&g_sensorSeqlock, &snap);
    if (hs->mcu == rc::kHealthAbsent || snap.mcu_die_temp_c < -100.0F) {
        rc::rc_log("MCU temp: --- (sensor not ready)\n");
    } else {
        rc::rc_log("MCU temp: %s  %.1fC\n",
               health_level_str(hs->mcu),
               static_cast<double>(snap.mcu_die_temp_c));
    }

    // Critical conditions (IVP-142b-2) — threshold-bound invariants that
    // warrant loud operator attention. Does NOT auto-abort; operator
    // must manually command abort if they decide to act on the flag.
    if (hs->critical != 0) {
        rc::rc_log("CRITICAL: ");
        if (hs->critical & rc::kHealthCriticalMcu) {
            rc::rc_log("MCU>=%.0fC ", static_cast<double>(rc::kMcuTempSafeModeC));
        }
        rc::rc_log(" (manual abort recommended)\n");
    }
}

void cli_print_preflight() {
    const rc::HealthState* hs = rc::health_monitor_get_state();

    rc::rc_log("\n=== PREFLIGHT ===\n");
    preflight_print_primary(hs);
    preflight_print_secondary(hs);
    preflight_print_mcu_and_critical(hs);

    // Stage T Batch B IVP-T14: show "RF Link" station (learned-link state
    // from AO_RfManager) alongside the hardware-level Radio health above.
    // Uses the same go_nogo_evaluate path the ARM-time check uses, so the
    // operator sees what ARM will see.
    rc::GoNoGoInput gng{};
    rc::health_monitor_fill_go_nogo(&gng);
    rc::GoNoGoResult gngResult = rc::go_nogo_evaluate(gng);
    for (uint8_t i = 0; i < gngResult.num_checks; ++i) {
        const rc::GoNoGoCheck& c = gngResult.checks[i];
        // Only surface the RF-specific stations — the others duplicate the
        // primary/secondary block above.
        if (strcmp(c.name, "RF Link") == 0) {
            rc::rc_log("RF Link:  %s\n", c.reason);
        }
    }

    rc::rc_log("----------------\n");
    rc::rc_log("VERDICT:  %s\n", hs->go_nogo_ready ? "GO" : "NO-GO");
}

void cli_handle_unhandled_key(int key) {
    switch (key) {
    case 'l': case 'L': cmd_flush_log(); break;
    case 'x': AO_RCOS_start_erase_flights(); break;
    case 'd': case 'D':
        if constexpr (kRadioModeRx) { cmd_station_distance(); }
        else { AO_RCOS_start_download_flight(); }
        break;
    case 'g': case 'G':
        if constexpr (kRadioModeRx) { cmd_station_gps(); }
        else { cmd_list_flights(); }
        break;
    case 't': case 'T': cmd_radio_status(); break;
    case 'r':
        if constexpr (kRadioModeRx) {
            // Stage T IVP-T5.5 sub 2c: station SET_RADIO_CONFIG.
            // Cycle through kRadioConfigTable entries, sending the next
            // entry after the current. Operator can press repeatedly to
            // advance. Station's own radio switches after ACK (wired via
            // SET's own ACK-handling path — station mirrors the vehicle).
            cmd_radio_config_cycle();
        } else if (AO_Radio_get_state()->initialized) {
            uint8_t newRate = AO_Telemetry_cycle_rate();
            rc::rc_log("[TX] Rate changed to %dHz\n", static_cast<int>(newRate));
        }
        break;
    case 'm': case 'M':
        AO_RCOS_cycle_output_mode();
        {
            auto mode = AO_RCOS_get_output_mode();
            const char* name = (mode == StationOutputMode::kAnsi) ? "ANSI" :
                               (mode == StationOutputMode::kCsv)  ? "CSV" : "MAVLink";
            if (mode == StationOutputMode::kAnsi) {
                rc::rc_log("\033[2J\033[H");
            } else {
                rc::rc_log("\n[RX] Output: %s\n", name);
            }
        }
        break;
    case 'a':
        if constexpr (kRadioModeRx) {
            // IVP-122: ARM confirm flow — enter multi-char confirm state
            rc_os_start_arm_confirm();
        }
        break;
    case 'X':
        if constexpr (kRadioModeRx) {
            // IVP-122: Station DISARM — single-key, no confirm, ACK-tracked.
            // Capital X only (lowercase x is erase-flights in TX mode).
#ifdef ROCKETCHIP_STAGE_T2_CHEAT
            stage_t2_queue_command(kMavCmdArmDisarm, 0.0f);
            rc::rc_log("[CMD] DISARM sent, waiting for ACK...\n");
#else
            AO_Telemetry_send_tracked_command(kMavCmdArmDisarm, 0.0F);
            rc::rc_log("[CMD] DISARM sent, waiting for ACK...\n");
#endif
        }
        break;
    case 'p': case 'P':
        if constexpr (kRadioModeRx) {
            cmd_station_gps_push();
        }
        break;
    default: break;
    }
}

// ============================================================================
// Stage L — manual beacon ('b' / find-me)
// ============================================================================
// Publishes SIG_BEACON_MANUAL so AO_Notify sets NotifyState.beacon_manual,
// which the resolver overlay turns into pure-white 2Hz (kFdBeacon).
// Valid in any phase — non-destructive. Clears automatically on the next
// SIG_PHASE_CHANGE out of {LANDED, ABORT}.
//
// Static event per LL Entry 35: QP stores the pointer, not a copy.
//
// Stage L IVP-L5: role-gated. On vehicle, publish SIG_BEACON_MANUAL locally
// (AO_Notify flips beacon_manual, resolver returns pure white). On station,
// send MAV_CMD_USER_1 over the radio — the vehicle's ao_telemetry.cpp command
// handler turns that into the same SIG_BEACON_MANUAL publish, so the visual
// behavior is identical from either side.
void cmd_findme_beacon() {
    if constexpr (kRadioModeRx) {
        // Station — send beacon command to vehicle via tracked command (IVP-122
        // ACK protocol). cmd.param1 unused by MAV_CMD_USER_1 receiver.
        AO_Telemetry_send_tracked_command(kMavCmdBeacon, 0.0F);
        rc::rc_log("[CMD] find-me beacon command sent, waiting for ACK...\n");
    } else {
        // Vehicle — publish directly to local AO_Notify.
        static QEvt s_findme_evt;
        s_findme_evt.sig = rc::SIG_BEACON_MANUAL;
        QActive_publish_(&s_findme_evt, AO_RCOS, AO_RCOS->prio);
        rc::rc_log("[CMD] find-me beacon ON (manual) — white 2Hz until state change\n");
    }
}
