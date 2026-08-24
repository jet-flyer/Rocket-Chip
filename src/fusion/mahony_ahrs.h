// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_MAHONY_AHRS_H
#define ROCKETCHIP_FUSION_MAHONY_AHRS_H

// Mahony AHRS — complementary filter attitude estimator.
// Pure C++ — no Pico SDK dependencies.
//
// Runs alongside the 24-state ESKF as an independent cross-check.
// NOT a navigation solution — lightweight "second opinion" for the
// confidence gate. When ESKF diverges but Mahony doesn't
// (or vice versa), the gate can flag the discrepancy.
//
// Mahony et al. 2008. Gains from AP_AHRS_DCM; accel/spin gates from
// PX4/Betaflight consensus.

#include "math/quat.h"
#include "math/vec3.h"

namespace rc {

struct MahonyAHRS {
    // Nominal attitude (body-to-NED quaternion)
    Quat q;

    // PI integral term (rad/s accumulated error)
    Vec3 integral_error;

    // Time elapsed since init (10× Kp while elapsed_s < kStartupDurationS)
    float elapsed_s{};

    bool initialized_{};
    bool startup_ended_{};  // force_end_startup() only; elapsed timeout does not set this

    // =================================================================
    // Council-approved parameters (arXiv:0811.4303 + 3-stack consensus)
    // =================================================================

    // Proportional gain — ArduPilot/PX4/INAV 3-stack consensus.
    static constexpr float kKp = 0.2F;

    // Integral gain — ArduPilot AP_AHRS_DCM hardcoded value.
    static constexpr float kKi = 0.0087F;

    // 10× Kp while elapsed_s < kStartupDurationS and !startup_ended_; then step to kKp.
    static constexpr float kStartupKpMultiplier = 10.0F;
    static constexpr float kStartupDurationS    = 20.0F;

    // Accel gate: reject correction outside 0.9g–1.1g.
    // PX4/BetaFlight consensus (gravitational field check).
    static constexpr float kAccelGateLow  = 0.9F * 9.80665F;   // ~8.83 m/s²
    static constexpr float kAccelGateHigh = 1.1F * 9.80665F;   // ~10.79 m/s²

    // Mag gate: reject if magnitude deviates >±15% from expected.
    // Council tightened from ±25%. Set expected_mag=0 to skip check.
    static constexpr float kMagGateFraction = 0.15F;
    // Init/update skip mag heading when |mag| <= this (µT). Not “zero vector”.
    static constexpr float kMagMinNormUt = 1.0F;

    // Ki spin cutoff: freeze integral above 20°/s.
    // AP/BetaFlight/INAV consensus — prevents integral windup in spin.
    static constexpr float kKiSpinCutoffRadS = 20.0F * (3.14159265F / 180.0F);

    // =================================================================
    // Methods
    // =================================================================

    // Initialize from gravity + optional mag yaw (|mag| > 1 µT, else tilt-only q).
    // Returns false if accel fails gate (not stationary).
    bool init(const Vec3& accel, const Vec3& magBody);

    // Update attitude estimate.
    // accel, gyro: body-frame IMU readings (m/s², rad/s).
    // mag_body: body-frame calibrated mag reading (µT).
    // expected_mag: calibrated field magnitude (µT). 0 = skip mag gate.
    // mag_cal_valid: if false, skip mag correction entirely (council addition).
    // dt: seconds; non-finite or <=0 is a no-op.
    void update(const Vec3& accel, const Vec3& gyro,
                const Vec3& magBody, float expectedMag,
                bool magCalValid, float dt);

    // Angular divergence between two quaternions (rad).
    // Returns the minimum rotation angle to align a to b.
    static float divergence_rad(const Quat& a, const Quat& b);

    // False if not initialized. Else NaN/Inf check on quaternion components.
    bool healthy() const;

    // End startup boost immediately (called on ARM transition).
    // Snaps Kp from 10× to nominal to avoid aggressive corrections in flight.
    void force_end_startup() { startup_ended_ = true; }

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
