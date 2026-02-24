// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_MAHONY_AHRS_H
#define ROCKETCHIP_FUSION_MAHONY_AHRS_H

// Mahony AHRS — complementary filter attitude estimator.
// Pure C++ — no Pico SDK dependencies.
//
// Runs alongside the 15-state ESKF as an independent cross-check.
// NOT a navigation solution — lightweight "second opinion" for the
// confidence gate (IVP-50). When ESKF diverges but Mahony doesn't
// (or vice versa), the gate can flag the discrepancy.
//
// Algorithm: Mahony et al. (2008) "Nonlinear complementary filters
// on the special orthogonal group." IEEE Trans. Automatic Control.
// Council-approved parameters — see PHASE5_MAHONY_PLAN.md.
//
// Reference implementations consulted:
//   ArduPilot AP_AHRS_DCM (Kp, Ki values)
//   PX4/BetaFlight/INAV (accel/spin gate consensus)
//   BetaFlight (startup Kp × 10 for 20s convergence)

#include "math/quat.h"
#include "math/vec3.h"

namespace rc {

struct MahonyAHRS {
    // Nominal attitude (body-to-NED quaternion)
    Quat q;

    // PI integral term (rad/s accumulated error)
    Vec3 integral_error;

    // Time elapsed since init (for startup gain decay)
    float elapsed_s{};

    bool initialized_{};

    // =================================================================
    // Council-approved parameters (arXiv:0811.4303 + 3-stack consensus)
    // =================================================================

    // Proportional gain — ArduPilot/PX4/INAV 3-stack consensus.
    static constexpr float kKp = 0.2f;

    // Integral gain — ArduPilot AP_AHRS_DCM hardcoded value.
    static constexpr float kKi = 0.0087f;

    // Startup: 10× Kp for first 20s, then decay to normal.
    // BetaFlight pattern — fast convergence on power-on.
    // TODO(IVP-50): also terminate on ARM state transition.
    static constexpr float kStartupKpMultiplier = 10.0f;
    static constexpr float kStartupDurationS    = 20.0f;

    // Accel gate: reject correction outside 0.9g–1.1g.
    // PX4/BetaFlight consensus (gravitational field check).
    static constexpr float kAccelGateLow  = 0.9f * 9.80665f;   // ~8.83 m/s²
    static constexpr float kAccelGateHigh = 1.1f * 9.80665f;   // ~10.79 m/s²

    // Mag gate: reject if magnitude deviates >±15% from expected.
    // Council tightened from ±25%. Set expected_mag=0 to skip check.
    static constexpr float kMagGateFraction = 0.15f;

    // Ki spin cutoff: freeze integral above 20°/s.
    // AP/BetaFlight/INAV consensus — prevents integral windup in spin.
    static constexpr float kKiSpinCutoffRadS = 20.0f * (3.14159265f / 180.0f);

    // =================================================================
    // Methods
    // =================================================================

    // Initialize from gravity vector + tilt-compensated mag yaw.
    // accel: body-frame accelerometer reading (m/s²).
    // mag_body: body-frame magnetometer reading (µT). If zero vector,
    //   attitude initializes with yaw=0 (North assumed).
    // Returns false if accel fails gate (not stationary).
    bool init(const Vec3& accel, const Vec3& magBody);

    // Update attitude estimate.
    // accel, gyro: body-frame IMU readings (m/s², rad/s).
    // mag_body: body-frame calibrated mag reading (µT).
    // expected_mag: calibrated field magnitude (µT). 0 = skip mag gate.
    // mag_cal_valid: if false, skip mag correction entirely (council addition).
    // dt: time step (s).
    void update(const Vec3& accel, const Vec3& gyro,
                const Vec3& magBody, float expectedMag,
                bool magCalValid, float dt);

    // Angular divergence between two quaternions (rad).
    // Returns the minimum rotation angle to align a to b.
    static float divergence_rad(const Quat& a, const Quat& b);

    // Health check: NaN/Inf detection in quaternion components.
    bool healthy() const;

private:
    // Accel gravity-reference error: cross product of measured vs predicted down.
    // Returns zero vector if accel magnitude fails gate.
    Vec3 compute_accel_error(const Vec3& accel) const;

    // Mag heading-reference error: cross product of measured vs predicted field.
    // Returns zero vector if mag is invalid, too small, or fails gate.
    Vec3 compute_mag_error(const Vec3& magBody, float expectedMag,
                           bool magCalValid) const;
};

} // namespace rc

#endif // ROCKETCHIP_FUSION_MAHONY_AHRS_H
