// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Mission Profile — Flight Configuration Data
//
// IVP-68: QEP Integration + Phase Skeleton (Stage 8: Flight Director)
// IVP-74: Mission Configuration (HAB, freeform profiles + flash persistence)
//
// MissionProfile is the configuration data that feeds the Flight Director.
// It defines thresholds, timeouts, guard parameters, and abort behavior
// for a specific vehicle type. The Flight Director reads from a
// const MissionProfile* — it never modifies the profile.
//
// Distinct from job.h (device role: vehicle vs station). A vehicle always
// uses the same job (vehicle), but may fly different MissionProfiles
// (rocket vs HAB vs freeform).
//
// The struct is final and serialization-ready (no pointers, no vtable).
// Profile values marked with "VALIDATE" are starting points to be tuned
// with flight data.
//============================================================================
#ifndef ROCKETCHIP_MISSION_PROFILE_H
#define ROCKETCHIP_MISSION_PROFILE_H

#include <stdint.h>

namespace rc {

// ============================================================================
// Mission Profile ID — stored in flash, selects active profile at boot
// ============================================================================
enum class ProfileId : uint8_t {
    kRocket     = 0,
    kHab        = 1,
    kFreeform   = 2,
    kCount      = 3
};

// ============================================================================
// Mission Profile — flight configuration data
//
// All thresholds use SI units (m, m/s, m/s^2, ms).
// Profile is boot-locked: read from flash at boot, immutable for session.
// IVP-74 adds HAB + freeform profiles and CLI selection.
// ============================================================================
struct MissionProfile {
    ProfileId id;
    char name[16];                      // Human-readable name for CLI/logs

    // --- Timing ---
    uint32_t armed_timeout_ms;          // Auto-disarm if no launch (Amendment #5)
    uint32_t abort_timeout_ms;          // ABORT → LANDED auto-transition
    uint32_t coast_timeout_ms;          // Missed apogee fallback (Amendment #7)

    // --- Guard thresholds ---
    // ⚠️  ALL values below are PRELIMINARY — sourced from ArduPilot patterns
    // and engineering estimates. Must be validated with real flight data
    // before any flight that depends on automatic detection.
    // See STANDARDS_DEVIATIONS.md if deploying unvalidated.
    float launch_accel_threshold;       // Body-Z accel for launch detect (m/s^2)
    uint32_t launch_sustain_ms;         // Sustain time for launch accel

    float burnout_accel_threshold;      // Accel magnitude for burnout (m/s^2)
    uint32_t burnout_sustain_ms;

    float apogee_velocity_threshold;    // Vertical velocity for zero-cross (m/s)
    uint32_t apogee_sustain_ms;

    uint32_t baro_peak_sustain_ms;      // Baro derivative window for backup apogee

    float main_deploy_altitude_m;       // AGL altitude for main chute (m)
    uint32_t main_deploy_sustain_ms;

    float landing_velocity_threshold;   // Velocity norm for stationary (m/s)
    uint32_t landing_sustain_ms;

    // --- Abort behavior ---
    bool abort_fires_drogue_from_boost; // ABORT-from-BOOST fires drogue
    bool abort_fires_drogue_from_coast; // ABORT-from-COAST fires drogue

    // --- Pre-arm checks ---
    bool require_gps_lock;              // Tier 2: GPS lock required for ARM
    bool require_mag_cal;               // Tier 2: Magnetometer calibration required
    bool require_radio;                 // Tier 2: Radio link required

    // --- Pyro ---
    bool has_pyro;                      // Profile includes pyro channels
};

// ============================================================================
// Default Rocket Profile (Council Amendment #6)
//
// Guard sustain values sourced from ArduPilot (Council Amendment #4):
//   launch:     50ms (5 samples @ 100Hz) — AP_InertialNav accel spike filter
//   burnout:    100ms (10 samples) — motor burnout is gradual
//   apogee:     30ms (3 samples) — velocity sign change is clean
//   baro peak:  200ms (20 samples) — baro noise needs longer window
//   main deploy: 50ms (5 samples) — altitude is smooth
//   landing:    2000ms (200 samples) — AP LAND_SPEED timeout pattern
//
// All thresholds marked VALIDATE — starting points for flight-data tuning.
// ============================================================================
inline constexpr MissionProfile kDefaultRocketProfile = {
    .id                         = ProfileId::kRocket,
    .name                       = "Rocket",

    .armed_timeout_ms           = 5 * 60 * 1000,   // 5 minutes (Amendment #5)
    .abort_timeout_ms           = 5 * 60 * 1000,   // 5 minutes
    .coast_timeout_ms           = 15 * 1000,        // 15 seconds (Amendment #7)

    // VALIDATE — launch detection: body-Z accel > 20 m/s^2 for 50ms
    .launch_accel_threshold     = 20.0f,
    .launch_sustain_ms          = 50,

    // VALIDATE — burnout: accel magnitude < 5 m/s^2 for 100ms
    .burnout_accel_threshold    = 5.0f,
    .burnout_sustain_ms         = 100,

    // VALIDATE — apogee: vertical velocity < 0.5 m/s for 30ms
    .apogee_velocity_threshold  = 0.5f,
    .apogee_sustain_ms          = 30,

    // VALIDATE — baro peak: altitude derivative for backup apogee
    .baro_peak_sustain_ms       = 200,

    // VALIDATE — main deploy: AGL altitude < 150m for 50ms
    .main_deploy_altitude_m     = 150.0f,
    .main_deploy_sustain_ms     = 50,

    // VALIDATE — landing: velocity norm < 0.5 m/s for 2000ms
    .landing_velocity_threshold = 0.5f,
    .landing_sustain_ms         = 2000,

    .abort_fires_drogue_from_boost = true,   // Amendment #1
    .abort_fires_drogue_from_coast = true,   // Amendment #1

    .require_gps_lock           = false,    // Tier 2, optional for rocket
    .require_mag_cal            = false,    // Tier 2, optional
    .require_radio              = false,    // Tier 2, optional

    .has_pyro                   = true,
};

} // namespace rc

#endif // ROCKETCHIP_MISSION_PROFILE_H
