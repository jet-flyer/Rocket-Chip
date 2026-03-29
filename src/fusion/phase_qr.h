// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_PHASE_QR_H
#define ROCKETCHIP_FUSION_PHASE_QR_H

// Phase-scheduled Q/R types for ESKF adaptive estimation (IVP-83).
// Pure C++ — no Pico SDK dependencies.
//
// Q scale multipliers are applied as additive diagonal deltas AFTER codegen_fpft()
// runs with baked-in baseline Q_d. The delta formula:
//   P[i][i] += baseline_sigma^2 * (phase_scale - 1.0) * dt
// Only adds positive values to diagonal — cannot break positive-definiteness.
//
// R values are absolute (not multipliers) and replace the baseline kR* constants
// in ESKF measurement updates when phase Q/R is configured.
//
// Council review 2026-03-29: unanimous approval, 7 amendments (A1-A7).

#include <cstdint>

namespace rc {

// Per-phase Q noise scaling factors (multipliers on baseline sigma^2).
// All values must be >= 1.0 (enforced by generate_profile.py).
// A value of 1.0 means "use baseline" (no additional noise).
struct PhaseQScale {
    float attitude;     // multiplier on kSigmaGyro^2
    float velocity;     // multiplier on kSigmaAccel^2
    float accel_bias;   // multiplier on kSigmaAccelBiasWalk^2
    float gyro_bias;    // multiplier on kSigmaGyroBiasWalk^2
};

// Per-phase measurement noise (absolute R values, not multipliers).
// All values must be > 0.0 (enforced by generate_profile.py).
struct PhaseR {
    float r_baro;       // m^2 (baseline: ~0.001 from DPS310 @ 8x OS)
    float r_mag;        // rad^2 (baseline: ~0.008 from AK09916)
    float r_gps_pos;    // m^2, before HDOP scaling (baseline: 12.25)
    float r_gps_vel;    // m^2/s^2 (baseline: 0.25)
};

// Combined Q/R entry for a single flight phase.
struct PhaseQREntry {
    PhaseQScale q_scale;
    PhaseR r;
};

// Number of flight phases (matches FlightPhase::kCount in flight_state.h).
// Index order: IDLE=0, ARMED=1, BOOST=2, COAST=3, DROGUE_DESCENT=4,
//              MAIN_DESCENT=5, LANDED=6, ABORT=7.
static constexpr uint8_t kPhaseCount = 8;

// Full phase Q/R table — one entry per flight phase plus transition config.
struct PhaseQRTable {
    PhaseQREntry phases[kPhaseCount];
    uint8_t ramp_steps;  // transition smoothing: lerp over this many predict steps
};

// Default phase Q/R table.
// All values are VALIDATE defaults — actual tuning deferred to Stage 13.
// Q scales: 1.0 = baseline (no additional noise).
// R values: match eskf.h baseline constants.
//
// Phase index:         IDLE  ARMED BOOST COAST DROG  MAIN  LAND  ABORT
inline constexpr PhaseQRTable kDefaultPhaseQR = {.phases = {
    // IDLE (0)
    {.q_scale = {1.0f, 1.0f, 1.0f, 1.0f},
     .r = {0.001f, 0.008f, 12.25f, 0.25f}},
    // ARMED (1)
    {.q_scale = {1.0f, 1.0f, 1.0f, 1.0f},
     .r = {0.001f, 0.008f, 12.25f, 0.25f}},
    // BOOST (2) — high vibration, rapid acceleration
    {.q_scale = {10.0f, 50.0f, 5.0f, 5.0f},          // VALIDATE
     .r = {0.004f, 0.016f, 12.25f, 0.25f}},           // VALIDATE
    // COAST (3) — drag deceleration, moderate dynamics
    {.q_scale = {5.0f, 10.0f, 2.0f, 2.0f},            // VALIDATE
     .r = {0.001f, 0.008f, 12.25f, 0.25f}},
    // DROGUE_DESCENT (4)
    {.q_scale = {2.0f, 5.0f, 1.0f, 1.0f},             // VALIDATE
     .r = {0.001f, 0.008f, 12.25f, 0.25f}},
    // MAIN_DESCENT (5)
    {.q_scale = {2.0f, 5.0f, 1.0f, 1.0f},             // VALIDATE
     .r = {0.001f, 0.008f, 12.25f, 0.25f}},
    // LANDED (6) — stationary
    {.q_scale = {1.0f, 1.0f, 1.0f, 1.0f},
     .r = {0.001f, 0.008f, 12.25f, 0.25f}},
    // ABORT (7) — same as COAST (conservative)
    {.q_scale = {5.0f, 10.0f, 2.0f, 2.0f},            // VALIDATE
     .r = {0.001f, 0.008f, 12.25f, 0.25f}},
}, .ramp_steps = 20};  // VALIDATE: 100ms at 200Hz predict rate

}  // namespace rc

#endif  // ROCKETCHIP_FUSION_PHASE_QR_H
