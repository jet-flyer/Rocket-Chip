// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "fusion/mahony_ahrs.h"

#include <cmath>

namespace rc {

// ============================================================================
// init: Initialize from gravity vector + tilt-compensated mag yaw
// ============================================================================
bool MahonyAHRS::init(const Vec3& accel, const Vec3& magBody) {
    // Require approximately stationary (within accel gate)
    const float aMag = accel.norm();
    if (aMag < kAccelGateLow || aMag > kAccelGateHigh) {
        return false;
    }

    // Roll/pitch from gravity: body-down = -accel.normalized(), map to NED-down
    const Vec3 bodyDown = (accel * -1.0F).normalized();
    const Vec3 nedDown(0.0F, 0.0F, 1.0F);
    q = Quat::from_two_vectors(bodyDown, nedDown);
    q.normalize();

    // Yaw from tilt-compensated magnetometer (council: match ESKF init).
    // If magBody is zero or too small, skip — yaw = 0 (North assumed).
    if (magBody.norm() > 1.0F) {
        // Project mag into level plane using current roll/pitch.
        // Rotate mag to NED frame, then extract yaw from horizontal components.
        const Vec3 magNed = q.rotate(magBody);
        const float yaw = atan2f(-magNed.y, magNed.x);  // East-positive heading → negate y
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
    const float aMag = accel.norm();
    if (aMag < kAccelGateLow || aMag > kAccelGateHigh) {
        return Vec3();
    }
    // Estimated gravity direction in body frame: R^T * [0,0,1]_NED
    // q.conjugate().rotate([0,0,1]) = predicted body-frame down
    const Vec3 vHat = q.conjugate().rotate(Vec3(0.0F, 0.0F, 1.0F));
    // Measured body-frame down: -accel.normalized()
    const Vec3 aNorm = (accel * -1.0F).normalized();
    // Error: cross product (measurement x prediction)
    return aNorm.cross(vHat);
}

// ============================================================================
// compute_mag_error: heading-reference correction term
// ============================================================================
Vec3 MahonyAHRS::compute_mag_error(const Vec3& magBody, float expectedMag,
                                    bool magCalValid) const {
    if (!magCalValid || magBody.norm() <= 1.0F) {
        return Vec3();
    }
    // Mag gate: skip if magnitude deviates beyond +/-15%
    if (expectedMag > 0.0F) {
        const float ratio = magBody.norm() / expectedMag;
        if (ratio < (1.0F - kMagGateFraction) ||
            ratio > (1.0F + kMagGateFraction)) {
            return Vec3();
        }
    }
    // Project mag to NED, extract the horizontal reference vector.
    // Mahony reference: b = [sqrt(hx^2+hy^2), 0, hz] -- removes declination
    // dependence while preserving dip angle (Mahony et al. section IV).
    const Vec3 mNorm = magBody.normalized();
    const Vec3 magNed = q.rotate(magBody);
    const float hXy = sqrtf(magNed.x * magNed.x + magNed.y * magNed.y);
    // Reference in NED: [hXy, 0, hz] -- points North, ignores declination
    const Vec3 bNed(hXy, 0.0F, magNed.z);
    // Rotate reference back to body frame
    const Vec3 bBody = q.conjugate().rotate(bNed).normalized();
    // Error: cross product (measurement x prediction)
    return mNorm.cross(bBody);
}

// ============================================================================
// update: Mahony complementary filter step
// ============================================================================
void MahonyAHRS::update(const Vec3& accel, const Vec3& gyro,
                        const Vec3& magBody, float expectedMag,
                        bool magCalValid, float dt) {
    if (!initialized_) {
        return;
    }

    // -- Effective Kp: startup 10x for first kStartupDurationS --
    const float kpEff = (elapsed_s < kStartupDurationS)
                        ? kKp * kStartupKpMultiplier
                        : kKp;

    const Vec3 eAccel = compute_accel_error(accel);
    const Vec3 eMag = compute_mag_error(magBody, expectedMag, magCalValid);

    // ----------------------------------------------------------------
    // PI controller
    // Integral: freeze above kKiSpinCutoffRadS to prevent windup
    // ----------------------------------------------------------------
    const Vec3 eTotal = eAccel + eMag;

    if (gyro.norm() < kKiSpinCutoffRadS) {
        integral_error = integral_error + eTotal * (kKi * dt);
    }

    // Corrected angular rate
    const Vec3 omega = gyro + eTotal * kpEff + integral_error;

    // ----------------------------------------------------------------
    // Integrate: q += 0.5 * q ⊗ [0, omega] * dt
    // Use from_small_angle for numerical consistency with ESKF
    // ----------------------------------------------------------------
    const Vec3 deltaTheta = omega * dt;
    q = q * Quat::from_small_angle(deltaTheta);
    q.normalize();

    elapsed_s += dt;
}

// ============================================================================
// divergence_rad: minimum rotation angle between two quaternions
// ============================================================================
float MahonyAHRS::divergence_rad(const Quat& a, const Quat& b) {
    // q_rel = a^-1 * b
    // angle = 2 * acos(|q_rel.w|)  — clamp to [-1,1] for numeric safety
    const Quat qRel = a.conjugate() * b;
    float wClamped = qRel.w;
    if (wClamped > 1.0F) {
        wClamped = 1.0F;
    } else if (wClamped < -1.0F) {
        wClamped = -1.0F;
    }
    return 2.0F * acosf(fabsf(wClamped));
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
