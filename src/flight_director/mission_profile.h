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
#include "../fusion/phase_qr.h"

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

    // --- Safety lockouts (IVP-71, Council A1) ---
    // Protects parachute from high dynamic pressure deployment.
    // Phase gating (COAST/DROGUE only) is primary powered-flight protection.
    // ⚠️  PRELIMINARY — tune per parachute rated deployment speed.
    float deploy_lockout_mps;           // Velocity lockout for deployment (m/s)
    uint32_t apogee_lockout_ms;         // Min time after launch before apogee

    // --- Timer backups (IVP-71, Council A6) ---
    // Tuned for HPR (H+ motors). HAB/low-power profiles need different values.
    // ⚠️  PRELIMINARY
    uint32_t burnout_backup_ms;         // Max time in BOOST before forced COAST
    uint32_t main_backup_ms;            // Max time in DROGUE before forced MAIN

    // --- Combinator config (IVP-71) ---
    bool apogee_require_both;           // true=AND (vel+baro), false=OR

    // --- Emergency override (IVP-71) ---
    // HAB: emergency chute always available (skip lockouts for ABORT pyro)
    // Rocket: ABORT respects lockout gates
    bool emergency_deploy_anytime;

    // --- Abort behavior ---
    bool abort_fires_drogue_from_boost; // ABORT-from-BOOST fires drogue
    bool abort_fires_drogue_from_coast; // ABORT-from-COAST fires drogue

    // --- Pre-arm checks ---
    bool require_gps_lock;              // Tier 2: GPS lock required for ARM
    bool require_mag_cal;               // Tier 2: Magnetometer calibration required
    bool require_radio;                 // Tier 2: Radio link required

    // --- Pyro ---
    bool has_pyro;                      // Profile includes pyro channels

    // --- Phase Q/R (IVP-83) ---
    // Per-phase noise model for ESKF adaptive estimation.
    // Q scales are multipliers on baseline sigma^2 values (>= 1.0).
    // R values are absolute measurement noise (> 0.0).
    // Generated from Q_*/R_* fields in .cfg by generate_profile.py.
    PhaseQRTable phase_qr;
};

} // namespace rc

// Active profile — generated from profiles/*.cfg by scripts/generate_profile.py
// To change: edit the .cfg file, run the generator, rebuild.
#include "mission_profile_data.h"

#endif // ROCKETCHIP_MISSION_PROFILE_H
