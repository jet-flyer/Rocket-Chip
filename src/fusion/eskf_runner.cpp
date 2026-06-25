// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// ESKF Runner — Fusion Tick Module (Implementation)
//
// All ESKF-related state and tick functions live here. The public API
// (eskf_runner.h) exposes read-only accessors per Council A6.
//============================================================================

#include "fusion/eskf_runner.h"
#include "rocketchip/config.h"
#include "rocketchip/shared_state.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/ao_signals.h"
#include "fusion/wmm_tables.h"
#include "calibration/calibration_manager.h"
#include "flight_director/flight_director.h"
#include "flight_director/mission_profile.h"
#include "flight_director/mission_profile_data.h"
#include "active_objects/ao_flight_director.h"

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
extern "C" {
#include "qp_port.h"
}
#else
// Host test stubs
static inline uint32_t time_us_32() { return 0; }
#endif

#include <math.h>

// ============================================================================
// Constants (moved from main.cpp)
// ============================================================================

// ESKF propagation rate (Hz) — derived from IMU rate / divider
static constexpr uint32_t kEskfRateHz_local = 200;

// ESKF propagation — every 5th IMU sample = 200Hz
static constexpr uint32_t kEskfImuDivider = 5;

// Rad/deg conversion for CLI display and geodetic conversion
static constexpr float kRadToDeg = 180.0F / 3.14159265F;
static constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// Microsecond-to-second conversion
static constexpr float kUsToSec = 1e-6F;
static constexpr float kGps1e7ToDegreesF = 1e-7F;
static constexpr double kGps1e7ToDegrees = 1e-7;

// ESKF dt sanity bounds (reject if too fast or too slow)
static constexpr uint32_t kEskfMinDtUs = 1000;     // 1ms
static constexpr uint32_t kEskfMaxDtUs = 100000;    // 100ms

// ESKF state buffer: 200Hz x 5s = 1000 samples x 68B = 68KB (SRAM)
static constexpr uint32_t kEskfBufferSamples = 1000;

// GPS session NIS sentinel (larger than any valid NIS)
static constexpr float kGpsNisSentinel = 1e9F;

// Mission profile pointer for phase Q/R wiring
static const rc::MissionProfile* g_profile = nullptr;

// ============================================================================
// ESKF Module State (moved from main.cpp)
// ============================================================================

// ESKF error-state Kalman filter (Core 0 at 200Hz)
// Non-static: Core 1 reads g_eskf.v for GPS staleness heuristic (sensor_core1.cpp).
// Per LL Entry 1: ESKF struct is ~970 bytes — file-scope, not stack.
rc::ESKF g_eskf;
bool g_eskfInitialized = false;
static uint32_t g_lastEskfImuCount = 0;
static uint32_t g_lastEskfTimestampUs = 0;
static uint32_t g_eskfEpoch = 0;        // Incremented on each ESKF propagation

// Mahony AHRS cross-check (~33 bytes, no stack overflow risk)
static rc::MahonyAHRS g_mahony;
static bool g_mahonyInitialized = false;
static uint32_t g_lastMahonyTimestampUs = 0;

// Confidence gate (feeds into SafetyLockout)
static rc::ConfidenceState g_confidence;

// GPS outdoor session stats
static gps_session_stats_t g_gpsSess = {
    0.0F, 0.0F, 0.0F, 0.0F, 0, kGpsNisSentinel, 0.0F
};

// Compact ESKF state circular buffer
static eskf_state_snap_t g_eskfBuffer[kEskfBufferSamples];
static uint32_t g_eskfBufferIndex = 0;
static uint32_t g_eskfBufferCount = 0;

// R-25-exec step 8 (2026-05-13): ifdef stripped per Approach A.
// Benchmark counters always accumulate; ~30 cycles per predict is
// negligible at 200 Hz. Pattern matches ArduPilot/PX4 — flight ops
// debugging needs these timings in production.
static uint32_t g_eskfBenchMin = UINT32_MAX;
static uint32_t g_eskfBenchMax = 0;
static uint32_t g_eskfBenchSum = 0;
static uint32_t g_eskfBenchCount = 0;
static uint32_t g_eskfBenchFullMin = UINT32_MAX;
static uint32_t g_eskfBenchFullMax = 0;
static uint32_t g_eskfBenchFullSum = 0;
static uint32_t g_eskfBenchFullCount = 0;

// Event logging callback (injected by main.cpp)
static EskfEventLogFn g_logEventFn = nullptr;

// ============================================================================
// NED Coordinate Helpers
// ============================================================================

// INTERIM: Adafruit ICM-20948 breakout has sensor Z-up convention.
// ESKF expects NED (Z-down). Negate Z for accel, gyro, and mag at
// the ESKF feed boundary. Proper fix: set board_rotation matrix with
// Z-negate (needs level cal redo and council review for axis mapping).
// X->X, Y->Y, Z->-Z preserves right-handedness for rotation about Z.
static rc::Vec3 sensor_to_ned_accel(const shared_sensor_data_t& snap) {
    return rc::Vec3(snap.accel_x, snap.accel_y, -snap.accel_z);
}
static rc::Vec3 sensor_to_ned_gyro(const shared_sensor_data_t& snap) {
    return rc::Vec3(snap.gyro_x, snap.gyro_y, -snap.gyro_z);
}
static rc::Vec3 sensor_to_ned_mag(const shared_sensor_data_t& snap) {
    return rc::Vec3(snap.mag_x, snap.mag_y, -snap.mag_z);
}

// ============================================================================
// ESKF Init and Predict
// ============================================================================

// ESKF init from first stable accel/gyro reading.
// If mag is available, set initial yaw from tilt-compensated heading.
// Returns true if init succeeded.
static bool eskf_try_init(const shared_sensor_data_t& snap) {
    rc::Vec3 accel = sensor_to_ned_accel(snap);
    rc::Vec3 gyro = sensor_to_ned_gyro(snap);

    if (!g_eskf.init(accel, gyro)) {
        return false;  // Stationarity check failed — try again next tick
    }

    // Set initial yaw from magnetometer heading if available.
    // Without this, yaw starts at 0 deg and the mag gate rejects updates
    // if actual heading is far from 0 deg (innovation >> 3 sigma).
    // Same approach as ArduPilot EKF3 InitialiseFilterBootstrap().
    if (snap.mag_valid) {
        rc::Vec3 mag = sensor_to_ned_mag(snap);
        // Tilt-compensate using roll/pitch from current quaternion (yaw=0)
        rc::Vec3 euler = g_eskf.q.to_euler();
        rc::Quat q_zero_yaw = rc::Quat::from_euler(euler.x, euler.y, 0.0F);
        rc::Vec3 mag_level = q_zero_yaw.rotate(mag);
        float initial_yaw = -atan2f(mag_level.y, mag_level.x);
        // Rebuild quaternion with mag-derived yaw
        g_eskf.q = rc::Quat::from_euler(euler.x, euler.y, initial_yaw);
        g_eskf.q.normalize();
    }

    g_eskfInitialized = true;
    g_eskf.reset_p_growth_baseline();
    g_lastEskfTimestampUs = snap.imu_timestamp_us;  // CR-2: set for first predict dt

    // Wire phase Q/R from active mission profile
    if (g_profile != nullptr) {
        g_eskf.set_phase_qr(&g_profile->phase_qr);
    }

    // Initialize confidence gate
    rc::confidence_gate_init(&g_confidence);

    return true;
}

// ESKF predict step with benchmark + state buffer write.
static void eskf_run_predict(const shared_sensor_data_t& snap) {
    // Compute dt from IMU timestamps (unsigned subtraction handles 32-bit wrap)
    uint32_t dt_us = snap.imu_timestamp_us - g_lastEskfTimestampUs;
    g_lastEskfTimestampUs = snap.imu_timestamp_us;  // CR-3: always update timestamp

    // Sanity-check dt — reject if too fast or too slow
    if (dt_us < kEskfMinDtUs || dt_us > kEskfMaxDtUs) {
        return;
    }

    float dt = static_cast<float>(dt_us) * kUsToSec;

    rc::Vec3 accel = sensor_to_ned_accel(snap);
    rc::Vec3 gyro = sensor_to_ned_gyro(snap);

    // R-25-exec step 8 (2026-05-13): ifdef stripped. Always profile.
    uint32_t t0 = time_us_32();
    g_eskf.predict(accel, gyro, dt);

    // CR-1: stop propagation if filter diverges
    if (!g_eskf.healthy()) {
        g_eskfInitialized = false;
        eskf_note_divergence();  // backoff (runaway-restart brake)
        return;
    }

    uint32_t elapsed = time_us_32() - t0;
    if (elapsed < g_eskfBenchMin) { g_eskfBenchMin = elapsed; }
    if (elapsed > g_eskfBenchMax) { g_eskfBenchMax = elapsed; }
    g_eskfBenchSum += elapsed;
    g_eskfBenchCount++;

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

// ============================================================================
// Measurement Updates
// ============================================================================

// Baro altitude measurement update (~32Hz DPS310 rate, on new data)
static void eskf_tick_baro(const shared_sensor_data_t& snap) {
    if (snap.baro_valid && g_baroContinuous) {
        static uint32_t s_last_eskf_baro_count = 0;
        if (snap.baro_read_count != s_last_eskf_baro_count) {
            s_last_eskf_baro_count = snap.baro_read_count;
            float alt = calibration_get_altitude_agl(snap.pressure_pa);
            g_eskf.update_baro(alt);
        }
    }
}

// WMM field source — tracks how the position was obtained
enum class WmmSource : uint8_t {
    kNone    = 0,  // Not yet initialized
    kDefault = 1,  // Mission Profile DEFAULT_LAT/LON
    kStored  = 2,  // Loaded from cal storage (previous GPS fix)
    kGps     = 3,  // Live GPS 3D fix (most accurate)
};

// WMM field state
static WmmSource g_wmmSource = WmmSource::kNone;
static rc::Vec3 g_wmmFieldNed;   // Earth field NED from WMM (µT)
static float g_wmmLatDeg = 0.0F;
static float g_wmmLonDeg = 0.0F;

// Auto-enable 3-axis mag: requires cal + WMM field
static bool g_mag3dEnabled = false;

// Initialize WMM field from a position (GPS, stored, or default)
static void init_wmm_field(float lat_deg, float lon_deg, WmmSource source) {
    g_wmmFieldNed = rc::wmm_get_earth_field_ned(lat_deg, lon_deg);
    g_wmmLatDeg = lat_deg;
    g_wmmLonDeg = lon_deg;
    g_wmmSource = source;
}

// Persist WMM position to cal storage (survives reboot)
static void save_wmm_position(float lat_deg, float lon_deg) {
    calibration_store_t* cal =
        const_cast<calibration_store_t*>(calibration_manager_get());
    cal->wmm_lat_deg = lat_deg;
    cal->wmm_lon_deg = lon_deg;
    cal->cal_flags |= CAL_STATUS_WMM_SET;
    calibration_save();
}

// Try to auto-enable 3-axis mag states
static void try_enable_mag_3axis(const shared_sensor_data_t& snap) {
    if (g_mag3dEnabled) { return; }

    const calibration_store_t* cal = calibration_manager_get();
    if ((cal->cal_flags & CAL_STATUS_MAG) == 0) { return; }

    // WMM position priority: GPS 3D fix → stored cal → profile default
    if (g_wmmSource == WmmSource::kNone) {
        if (snap.gps_valid && snap.gps_fix_type >= 3) {
            float lat = static_cast<float>(snap.gps_lat_1e7) * kGps1e7ToDegreesF;
            float lon = static_cast<float>(snap.gps_lon_1e7) * kGps1e7ToDegreesF;
            init_wmm_field(lat, lon, WmmSource::kGps);
            save_wmm_position(lat, lon);
        } else if ((cal->cal_flags & CAL_STATUS_WMM_SET) != 0) {
            init_wmm_field(cal->wmm_lat_deg, cal->wmm_lon_deg, WmmSource::kStored);
        } else if (g_profile->has_default_location) {
            init_wmm_field(g_profile->default_lat_deg, g_profile->default_lon_deg,
                           WmmSource::kDefault);
        } else {
            return;
        }
    }

    // Enable mag states and initialize earth_mag from WMM
    g_eskf.earth_mag = g_wmmFieldNed;
    g_eskf.set_inhibit_mag(false);
    g_mag3dEnabled = true;
}

// Mag measurement update (~10Hz from AK09916 via seqlock)
static void eskf_tick_mag(const shared_sensor_data_t& snap) {
    if (!snap.mag_valid) { return; }

    static uint32_t s_last_eskf_mag_count = 0;
    if (snap.mag_read_count == s_last_eskf_mag_count) { return; }
    s_last_eskf_mag_count = snap.mag_read_count;

    rc::Vec3 mag_body = sensor_to_ned_mag(snap);

    // Upgrade WMM source to GPS on first 3D fix (even if already using stored/default)
    if (snap.gps_valid && snap.gps_fix_type >= 3 && g_wmmSource != WmmSource::kGps) {
        float lat = static_cast<float>(snap.gps_lat_1e7) * kGps1e7ToDegreesF;
        float lon = static_cast<float>(snap.gps_lon_1e7) * kGps1e7ToDegreesF;
        init_wmm_field(lat, lon, WmmSource::kGps);
        save_wmm_position(lat, lon);
    }

    // Try to enable 3-axis mag if not already
    try_enable_mag_3axis(snap);

    if (g_mag3dEnabled) {
        // 3-axis fusion — uses earth_mag + body_mag_bias states
        g_eskf.update_mag_3axis(mag_body, g_wmmFieldNed);
    } else {
        // Heading-only fallback — uses yaw state only
        const calibration_store_t* cal = calibration_manager_get();
        float expected_mag = ((cal->cal_flags & CAL_STATUS_MAG) != 0)
                            ? cal->mag.expected_radius : 0.0F;
        float lat_deg = g_profile->default_lat_deg;
        float lon_deg = g_profile->default_lon_deg;
        if (snap.gps_valid && snap.gps_fix_type >= 2) {
            lat_deg = static_cast<float>(snap.gps_lat_1e7) * kGps1e7ToDegreesF;
            lon_deg = static_cast<float>(snap.gps_lon_1e7) * kGps1e7ToDegreesF;
        }
        float declination_rad = rc::wmm_get_declination(lat_deg, lon_deg);
        g_eskf.update_mag_heading(mag_body, expected_mag, declination_rad);
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

// GPS position + velocity measurement update.
// Gated on 3D fix with new data. On first quality fix, sets NED origin
// and resets p/v to zero. Subsequent fixes convert geodetic to NED and
// inject position + velocity. Velocity gated on speed >= 0.5 m/s to
// suppress noisy updates at rest.
static void eskf_tick_gps(const shared_sensor_data_t& snap) {
    if (snap.gps_valid && snap.gps_fix_type >= 3) {
        static uint32_t s_last_gps_count = 0;
        if (snap.gps_read_count != s_last_gps_count) {
            s_last_gps_count = snap.gps_read_count;

            double lat_rad = static_cast<double>(snap.gps_lat_1e7) * kGps1e7ToDegrees * kDeg2Rad;
            double lon_rad = static_cast<double>(snap.gps_lon_1e7) * kGps1e7ToDegrees * kDeg2Rad;
            float alt_m = snap.gps_alt_msl_m;
            float hdop = snap.gps_hdop;

            // First 3D fix: set origin + reset p/v
            if (!g_eskf.has_origin_) {
                if (g_eskf.set_origin(lat_rad, lon_rad, alt_m, hdop)) {
                    g_eskf.p = rc::Vec3();
                    g_eskf.v = rc::Vec3();
                }
            } else {
                // Re-center origin if position drifts > 10km (HAB flights)
                float dist_xy = sqrtf(g_eskf.p.x * g_eskf.p.x +
                                     g_eskf.p.y * g_eskf.p.y);
                if (dist_xy > rc::ESKF::kOriginResetDistance) {
                    g_eskf.reset_origin(lat_rad, lon_rad, alt_m);
                }

                // GPS position update (3 sequential scalar updates N/E/D)
                rc::Vec3 gps_ned = g_eskf.geodetic_to_ned(lat_rad, lon_rad, alt_m);
                g_eskf.update_gps_position(gps_ned, hdop);

                // GPS velocity update — only when moving (>= 0.5 m/s)
                if (snap.gps_ground_speed_mps >= rc::ESKF::kGpsMinSpeedForVel) {
                    float course_rad = snap.gps_course_deg * static_cast<float>(kDeg2Rad);
                    float v_north = snap.gps_ground_speed_mps * cosf(course_rad);
                    float v_east  = snap.gps_ground_speed_mps * sinf(course_rad);
                    g_eskf.update_gps_velocity(v_north, v_east);
                }

                eskf_tick_gps_stats();
            }
        }
    }
}

// ESKF tick — ZUPT zero-velocity pseudo-measurement.
// When on pad (IDLE/ARMED), the flight state machine guarantees
// stationarity — skip IMU check, use tight R. ArduPilot EKF3 onGround,
// PX4 vehicle_at_rest.
static void eskf_tick_zupt(const shared_sensor_data_t& snap) {
    rc::Vec3 accel = sensor_to_ned_accel(snap);
    rc::Vec3 gyro = sensor_to_ned_gyro(snap);
    const rc::FlightDirector* fd = AO_FlightDirector_get_director();
    bool on_pad = AO_FlightDirector_is_initialized() &&
        (fd->state.current_phase == rc::FlightPhase::kIdle ||
         fd->state.current_phase == rc::FlightPhase::kArmed);
    g_eskf.update_zupt(accel, gyro, on_pad);
}

// Mahony AHRS cross-check — independent attitude estimator.
// Runs at same 200Hz tick as ESKF. Uses its own dt tracking so it
// remains independent of the ESKF timestamp variable.
static void eskf_tick_mahony(const shared_sensor_data_t& snap) {
    rc::Vec3 accel = sensor_to_ned_accel(snap);
    rc::Vec3 gyro  = sensor_to_ned_gyro(snap);

    if (!g_mahonyInitialized) {
        rc::Vec3 mag_body = snap.mag_valid ? sensor_to_ned_mag(snap) : rc::Vec3();
        if (g_mahony.init(accel, mag_body)) {
            g_mahonyInitialized = true;
            g_lastMahonyTimestampUs = snap.imu_timestamp_us;
        }
    } else {
        uint32_t dt_us = snap.imu_timestamp_us - g_lastMahonyTimestampUs;
        g_lastMahonyTimestampUs = snap.imu_timestamp_us;
        if (dt_us >= kEskfMinDtUs && dt_us <= kEskfMaxDtUs) {
            float dt = static_cast<float>(dt_us) * kUsToSec;
            rc::Vec3 mag_body = snap.mag_valid ? sensor_to_ned_mag(snap) : rc::Vec3();
            const calibration_store_t* cal = calibration_manager_get();
            float expected_mag = ((cal->cal_flags & CAL_STATUS_MAG) != 0)
                                ? cal->mag.expected_radius : 0.0F;
            bool mag_cal_valid = (cal->cal_flags & CAL_STATUS_MAG) != 0;
            g_mahony.update(accel, gyro, mag_body, expected_mag, mag_cal_valid, dt);
            if (!g_mahony.healthy()) {
                g_mahonyInitialized = false;
            }
        }
    }
}

// Phase notification + confidence gate evaluation
static void eskf_tick_phase_and_confidence() {
    // Notify ESKF of flight phase changes for Q/R scheduling
    if (AO_FlightDirector_is_initialized()) {
        static uint8_t s_last_phase = 0;
        uint8_t phase = static_cast<uint8_t>(
            rc::flight_director_phase(AO_FlightDirector_get_director()));
        if (phase != s_last_phase) {
            g_eskf.notify_phase_change(phase);
            s_last_phase = phase;
        }
    }

    // Evaluate confidence gate
    rc::ConfidenceInput ci{};
    ci.eskf_healthy = g_eskf.healthy();
    ci.mahony_div_deg = (g_mahonyInitialized && g_mahony.healthy())
        ? rc::MahonyAHRS::divergence_rad(g_eskf.q, g_mahony.q) * kRadToDeg
        : 0.0F;
    // Max innovation ratio across all channels
    float max_alpha = g_eskf.innov_baro_.alpha;
    if (g_eskf.innov_mag_.alpha > max_alpha) max_alpha = g_eskf.innov_mag_.alpha;
    if (g_eskf.innov_gps_pos_.alpha > max_alpha) max_alpha = g_eskf.innov_gps_pos_.alpha;
    if (g_eskf.innov_gps_vel_.alpha > max_alpha) max_alpha = g_eskf.innov_gps_vel_.alpha;
    ci.max_innov_ratio = max_alpha;
    // Max P diagonal for attitude and velocity
    ci.p_att_max = g_eskf.P(0, 0);
    for (int32_t i = 1; i < 3; ++i) {
        if (g_eskf.P(i, i) > ci.p_att_max) ci.p_att_max = g_eskf.P(i, i);
    }
    ci.p_vel_max = g_eskf.P(6, 6);
    for (int32_t i = 7; i < 9; ++i) {
        if (g_eskf.P(i, i) > ci.p_vel_max) ci.p_vel_max = g_eskf.P(i, i);
    }
#ifndef ROCKETCHIP_HOST_TEST
    ci.now_ms = to_ms_since_boot(get_absolute_time());
#else
    ci.now_ms = 0;
#endif

    // Track previous confidence state for transition logging
    bool was_confident = g_confidence.confident;
    rc::confidence_gate_evaluate(&g_confidence, ci);

    // Log confidence transitions via injected callback
    if (g_logEventFn != nullptr) {
        // LogEventId::kConfidenceLost = 5, kConfidenceRecovered = 6
        // Use raw uint8_t to avoid including pcm_frame.h
        if (was_confident && !g_confidence.confident) {
            g_logEventFn(5, 0, 0, 0, 0);  // kConfidenceLost
        } else if (!was_confident && g_confidence.confident) {
            g_logEventFn(6, 0, 0, 0, 0);  // kConfidenceRecovered
        }
    }
}

// One fused cycle (predict → updates → phase/conf → bench → publish).
// Split out so `eskf_runner_tick` stays within pre-commit size limits.
static void eskf_runner_fusion_cycle(const shared_sensor_data_t& snap) {
#ifndef ROCKETCHIP_HOST_TEST
    const uint32_t t_fusion = time_us_32();
#endif

    eskf_run_predict(snap);

    // P-growth check: catch slow divergence before velocity hits 500 m/s
    if (!g_eskf.check_p_growth(time_us_32())) {
        g_eskfInitialized = false;  // CR-1 reset
        eskf_note_divergence();  // backoff (runaway-restart brake)
        return;
    }

    eskf_tick_baro(snap);
    eskf_tick_mag(snap);
    eskf_tick_zupt(snap);
    eskf_tick_gps(snap);
    eskf_tick_mahony(snap);

    eskf_tick_phase_and_confidence();

#ifndef ROCKETCHIP_HOST_TEST
    {
        const uint32_t el = time_us_32() - t_fusion;
        if (el < g_eskfBenchFullMin) { g_eskfBenchFullMin = el; }
        if (el > g_eskfBenchFullMax) { g_eskfBenchFullMax = el; }
        g_eskfBenchFullSum += el;
        g_eskfBenchFullCount++;
    }
#endif

    g_eskfEpoch++;

    // Publish SIG_SENSOR_DATA so downstream AOs react to new fusion state
#ifndef ROCKETCHIP_HOST_TEST
    rc::SensorDataEvt evt;
    evt.super = QEVT_INITIALIZER(rc::SIG_SENSOR_DATA);
    evt.eskf_epoch = g_eskfEpoch;
    QActive_publish_(&evt.super, nullptr, 0U);
#endif
}

void eskf_runner_tick() {
    if (!g_sensorPhaseActive) {
        return;
    }

    // ESKF failure backoff — skip if disabled after too many failures
    if (eskf_is_disabled()) {
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
    uint32_t new_samples = snap.imu_read_count - g_lastEskfImuCount;
    if (new_samples < kEskfImuDivider) {
        return;
    }
    g_lastEskfImuCount = snap.imu_read_count;

    if (!g_eskfInitialized) {
        eskf_try_init(snap);
        return;
    }

    eskf_runner_fusion_cycle(snap);
}

// ============================================================================
// Public API
// ============================================================================

void eskf_runner_init(const rc::MissionProfile* profile,
                      EskfEventLogFn log_fn) {
    g_profile = profile;
    g_logEventFn = log_fn;
}

const rc::ESKF* eskf_runner_get_eskf() {
    return &g_eskf;
}

const rc::MahonyAHRS* eskf_runner_get_mahony() {
    return &g_mahony;
}

const rc::ConfidenceState* eskf_runner_get_confidence() {
    return &g_confidence;
}

uint32_t eskf_runner_get_epoch() {
    return g_eskfEpoch;
}

bool eskf_runner_is_initialized() {
    return g_eskfInitialized;
}

bool eskf_runner_is_mahony_initialized() {
    return g_mahonyInitialized;
}

void eskf_runner_request_reinit() {
    g_eskfInitialized = false;
    g_mahonyInitialized = false;
    eskf_reenable();
}

uint32_t eskf_runner_get_buffer_count() {
    return g_eskfBufferCount;
}

const gps_session_stats_t* eskf_runner_get_gps_session() {
    return &g_gpsSess;
}

void eskf_runner_end_mahony_startup() {
    g_mahony.force_end_startup();
}

bool eskf_runner_mag_3d_active() {
    return g_mag3dEnabled;
}

bool eskf_runner_get_wmm_position(float* lat_deg, float* lon_deg) {
    if (g_wmmSource == WmmSource::kNone) { return false; }
    if (lat_deg != nullptr) { *lat_deg = g_wmmLatDeg; }
    if (lon_deg != nullptr) { *lon_deg = g_wmmLonDeg; }
    return true;
}

uint8_t eskf_runner_get_wmm_source() {
    return static_cast<uint8_t>(g_wmmSource);
}

// Read accessors are always callable; production builds use them via
// diag_stats_dump() / debug submenu.
void eskf_runner_get_bench(uint32_t* avg, uint32_t* min_us,
                           uint32_t* max_us, uint32_t* count) {
    if (g_eskfBenchCount > 0 && avg != nullptr) {
        *avg = g_eskfBenchSum / g_eskfBenchCount;
    } else if (avg != nullptr) {
        *avg = 0;
    }
    if (min_us != nullptr) { *min_us = g_eskfBenchMin; }
    if (max_us != nullptr) { *max_us = g_eskfBenchMax; }
    if (count != nullptr) { *count = g_eskfBenchCount; }
}

void eskf_runner_get_bench_full_tick(uint32_t* avg, uint32_t* min_us,
                                    uint32_t* max_us, uint32_t* count) {
    if (g_eskfBenchFullCount > 0 && avg != nullptr) {
        *avg = g_eskfBenchFullSum / g_eskfBenchFullCount;
    } else if (avg != nullptr) {
        *avg = 0;
    }
    if (min_us != nullptr) { *min_us = g_eskfBenchFullMin; }
    if (max_us != nullptr) { *max_us = g_eskfBenchFullMax; }
    if (count != nullptr) { *count = g_eskfBenchFullCount; }
}

// Runaway-restart brake implementation lives in eskf_brake.cpp so host
// tests can link it without pulling eskf_runner's SDK dependencies.
