// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "fusion/mahony_ahrs.h"

#include <cmath>

namespace rc {

// ============================================================================
// init: Initialize from gravity vector + tilt-compensated mag yaw
// ============================================================================
bool MahonyAHRS::init(const Vec3& accel, const Vec3& mag_body) {
    // Require approximately stationary (within accel gate)
    const float a_mag = accel.norm();
    if (a_mag < kAccelGateLow || a_mag > kAccelGateHigh) {
        return false;
    }

    // Roll/pitch from gravity: body-down = -accel.normalized(), map to NED-down
    const Vec3 body_down = (accel * -1.0F).normalized();
    const Vec3 ned_down(0.0F, 0.0F, 1.0F);
    q = Quat::from_two_vectors(body_down, ned_down);
    q.normalize();

    // Yaw from tilt-compensated magnetometer (council: match ESKF init).
    // If magBody is zero or too small, skip — yaw = 0 (North assumed).
    if (mag_body.norm() > 1.0F) {
        // Project mag into level plane using current roll/pitch.
        // Rotate mag to NED frame, then extract yaw from horizontal components.
        const Vec3 mag_ned = q.rotate(mag_body);
        const float yaw = atan2f(-mag_ned.y, mag_ned.x);  // East-positive heading → negate y
        // Rebuild quaternion with this yaw
        const Vec3 euler = q.to_euler();
        q = Quat::from_euler(euler.x, euler.y, yaw);
        q.normalize();
    }

    integral_error = Vec3();
    elapsed_s = 0.0F;
    initialized_ = true;
    return true;
}

// ============================================================================
// compute_accel_error: gravity-reference correction term
// ============================================================================
Vec3 MahonyAHRS::compute_accel_error(const Vec3& accel) const {
    const float a_mag = accel.norm();
    if (a_mag < kAccelGateLow || a_mag > kAccelGateHigh) {
        return Vec3();
    }
    // Estimated gravity direction in body frame: R^T * [0,0,1]_NED
    // q.conjugate().rotate([0,0,1]) = predicted body-frame down
    const Vec3 v_hat = q.conjugate().rotate(Vec3(0.0F, 0.0F, 1.0F));
    // Measured body-frame down: -accel.normalized()
    const Vec3 a_norm = (accel * -1.0F).normalized();
    // Error: cross product (measurement x prediction)
    return a_norm.cross(v_hat);
}

// ============================================================================
// compute_mag_error: heading-reference correction term
// ============================================================================
Vec3 MahonyAHRS::compute_mag_error(const Vec3& mag_body, float expected_mag,
                                    bool mag_cal_valid) const {
    if (!mag_cal_valid || mag_body.norm() <= 1.0F) {
        return Vec3();
    }
    // Mag gate: skip if magnitude deviates beyond +/-15%
    if (expected_mag > 0.0F) {
        const float ratio = mag_body.norm() / expected_mag;
        if (ratio < (1.0F - kMagGateFraction) ||
            ratio > (1.0F + kMagGateFraction)) {
            return Vec3();
        }
    }
    // Project mag to NED, extract the horizontal reference vector.
    // Mahony reference: b = [sqrt(hx^2+hy^2), 0, hz] -- removes declination
    // dependence while preserving dip angle (Mahony et al. section IV).
    const Vec3 m_norm = mag_body.normalized();
    const Vec3 mag_ned = q.rotate(mag_body);
    const float h_xy = sqrtf(mag_ned.x * mag_ned.x + mag_ned.y * mag_ned.y);
    // Reference in NED: [hXy, 0, hz] -- points North, ignores declination
    const Vec3 b_ned(h_xy, 0.0F, mag_ned.z);
    // Rotate reference back to body frame
    const Vec3 b_body = q.conjugate().rotate(b_ned).normalized();
    // Error: cross product (measurement x prediction)
    return m_norm.cross(b_body);
}

// ============================================================================
// update: Mahony complementary filter step
// ============================================================================
void MahonyAHRS::update(const Vec3& accel, const Vec3& gyro,
                        const Vec3& mag_body, float expected_mag,
                        bool mag_cal_valid, float dt) {
    if (!initialized_) {
        return;
    }

    // Effective Kp: startup 10x boost until time expires or ARM forces end
    const bool in_startup = !startup_ended_ && elapsed_s < kStartupDurationS;
    const float kp_eff = in_startup ? kKp * kStartupKpMultiplier : kKp;

    const Vec3 e_accel = compute_accel_error(accel);
    const Vec3 e_mag = compute_mag_error(mag_body, expected_mag, mag_cal_valid);

    // ----------------------------------------------------------------
    // PI controller
    // Integral: freeze above kKiSpinCutoffRadS to prevent windup
    // ----------------------------------------------------------------
    const Vec3 e_total = e_accel + e_mag;

    if (gyro.norm() < kKiSpinCutoffRadS) {
        integral_error = integral_error + e_total * (kKi * dt);
    }

    // Corrected angular rate
    const Vec3 omega = gyro + e_total * kp_eff + integral_error;

    // ----------------------------------------------------------------
    // Integrate: q += 0.5 * q ⊗ [0, omega] * dt
    // Use from_small_angle for numerical consistency with ESKF
    // ----------------------------------------------------------------
    const Vec3 delta_theta = omega * dt;
    q = q * Quat::from_small_angle(delta_theta);
    q.normalize();

    elapsed_s += dt;
}

// ============================================================================
// divergence_rad: minimum rotation angle between two quaternions
// ============================================================================
float MahonyAHRS::divergence_rad(const Quat& a, const Quat& b) {
    // q_rel = a^-1 * b
    // angle = 2 * acos(|q_rel.w|)  — clamp to [-1,1] for numeric safety
    const Quat q_rel = a.conjugate() * b;
    float w_clamped = q_rel.w;
    if (w_clamped > 1.0F) {
        w_clamped = 1.0F;
    } else if (w_clamped < -1.0F) {
        w_clamped = -1.0F;
    }
    return 2.0F * acosf(fabsf(w_clamped));
}

// ============================================================================
// healthy: NaN/Inf check on quaternion components
// ============================================================================
bool MahonyAHRS::healthy() const {
    if (!initialized_) {
        return false;
    }
    return std::isfinite(q.w) && std::isfinite(q.x) &&
           std::isfinite(q.y) && std::isfinite(q.z);
}

} // namespace rc
