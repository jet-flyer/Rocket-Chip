// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// ESKF Runner — Fusion Tick Module
//
// Owns the ESKF instance, Mahony AHRS cross-check, confidence gate,
// GPS session stats, state circular buffer, and benchmark timing.
// Called from qv_idle_bridge() at ~200Hz via seqlock sensor data.
//
// After each successful predict, publishes SIG_SENSOR_DATA via QP/C
// pub-sub so downstream AOs (Logger, Telemetry, LED) can react.
//
// Public API uses read-only accessors (Council A6 pattern) — callers
// get const pointers/copies, never modify ESKF state directly.
//============================================================================
#ifndef ROCKETCHIP_FUSION_ESKF_RUNNER_H
#define ROCKETCHIP_FUSION_ESKF_RUNNER_H

#include <stdint.h>
#include "fusion/eskf.h"
#include "fusion/mahony_ahrs.h"
#include "fusion/confidence_gate.h"

// Forward declarations
namespace rc {
struct MissionProfile;
}

// ============================================================================
// Compact ESKF state for circular buffer (no P matrix — R-6 council requirement)
// 68 bytes per sample. At 200Hz for 5s = 1000 samples = 68KB static in SRAM.
// ============================================================================
struct eskf_state_snap_t {
    uint32_t timestamp_us;          // 4B
    float qw, qx, qy, qz;         // 16B — quaternion
    float px, py, pz;              // 12B — position NED (m)
    float vx, vy, vz;              // 12B — velocity NED (m/s)
    float abx, aby, abz;           // 12B — accel bias (m/s²)
    float gbx, gby, gbz;           // 12B — gyro bias (rad/s)
};
static_assert(sizeof(eskf_state_snap_t) == 68, "ESKF snap size changed"); // NOLINT(readability-magic-numbers)

// ============================================================================
// GPS outdoor session stats — accumulated while GPS is active.
// Printed on reconnect via 's'. Lets user verify movement gates.
// ============================================================================
struct gps_session_stats_t {
    float max_dist_from_origin_m;    // Furthest ESKF position from origin (10m gate)
    float last_pos_n_m;              // ESKF N position at last GPS fix (square gate)
    float last_pos_e_m;              // ESKF E position at last GPS fix (square gate)
    float last_dist_from_origin_m;   // Distance from origin at last GPS fix (closure)
    uint32_t gps_updates;            // Total GPS position updates applied
    float min_gps_nis;               // Min GPS NIS seen (innovation health)
    float max_gps_nis;               // Max GPS NIS seen
};

// ============================================================================
// Event logging callback — injected by main.cpp at init time.
// Avoids circular dependency on ring buffer / PCM encoding.
// ============================================================================
using EskfEventLogFn = void (*)(uint8_t event_id, uint8_t d0,
                                uint8_t d1, uint8_t d2, uint8_t d3);

// ============================================================================
// Public API (read-only accessors per Council A6)
// ============================================================================

/// Initialize ESKF runner with mission profile and event logging callback.
/// Must be called once before eskf_runner_tick().
void eskf_runner_init(const rc::MissionProfile* profile,
                      EskfEventLogFn log_fn);

/// Main entry point — called from qv_idle_bridge() every idle iteration.
/// Reads seqlock, runs predict + measurement updates at 200Hz.
void eskf_runner_tick();

/// Read-only access to the ESKF instance.
const rc::ESKF* eskf_runner_get_eskf();

/// Read-only access to the Mahony AHRS cross-check.
const rc::MahonyAHRS* eskf_runner_get_mahony();

/// Read-only access to the confidence gate state.
const rc::ConfidenceState* eskf_runner_get_confidence();

/// Current ESKF propagation epoch (monotonic counter).
uint32_t eskf_runner_get_epoch();

/// Whether ESKF has been initialized (stationary init passed).
bool eskf_runner_is_initialized();

/// Whether Mahony AHRS has been initialized.
bool eskf_runner_is_mahony_initialized();

/// Number of samples in the ESKF state circular buffer.
uint32_t eskf_runner_get_buffer_count();

/// Read-only access to GPS session stats.
const gps_session_stats_t* eskf_runner_get_gps_session();

// R-25-exec step 8 (2026-05-13): ifdef gate stripped. Always available.
/// ESKF predict-step profiling, µs (always-on).
void eskf_runner_get_bench(uint32_t* avg, uint32_t* min_us,
                           uint32_t* max_us, uint32_t* count);
/// Full fusion cycle (predict + all measurements + phase/conf), µs.
void eskf_runner_get_bench_full_tick(uint32_t* avg, uint32_t* min_us,
                                    uint32_t* max_us, uint32_t* count);

/// End Mahony startup Kp boost (called on ARM transition).
void eskf_runner_end_mahony_startup();

/// Whether 3-axis mag fusion is active (mag states un-inhibited).
bool eskf_runner_mag_3d_active();

/// WMM field position used for mag initialization (lat, lon in degrees).
/// Returns false if WMM field not yet initialized.
bool eskf_runner_get_wmm_position(float* lat_deg, float* lon_deg);

/// WMM position source: 0=none, 1=default, 2=stored, 3=GPS
uint8_t eskf_runner_get_wmm_source();

// ============================================================================
// Runaway-restart brake (ex-watchdog_recovery machinery, now internal)
//
// ESKF can diverge via !g_eskf.healthy() or !check_p_growth() and get
// reinitialized. If the underlying condition persists (LL Entry 29 IMU
// silent-zero, LL Entry 34 baro turbulence), reinit re-diverges in a
// loop. kEskfMaxFailCycles=5 caps consecutive divergence events in one
// session; after that, the filter stays disabled until eskf_reenable()
// is called from CLI. Runtime-only — no persistence across power cycle.
// ============================================================================

/// True when the runaway-restart brake has disabled ESKF.
bool eskf_is_disabled();

/// Re-enable ESKF after the brake has tripped. CLI-callable.
/// Resets the consecutive-fail counter and clears the disabled flag.
void eskf_reenable();

/// Record one ESKF divergence event. Increments the consecutive-fail
/// counter; sets disabled=true when the counter reaches the threshold.
/// Called internally from eskf_runner.cpp on CR-1 reset paths.
void eskf_note_divergence();

#endif // ROCKETCHIP_FUSION_ESKF_RUNNER_H
