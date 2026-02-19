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
    const Vec3 body_down = (accel * -1.0f).normalized();
    const Vec3 ned_down(0.0f, 0.0f, 1.0f);
    q = Quat::from_two_vectors(body_down, ned_down);
    q.normalize();

    // Yaw from tilt-compensated magnetometer (council: match ESKF init).
    // If mag_body is zero or too small, skip — yaw = 0 (North assumed).
    if (mag_body.norm() > 1.0f) {
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
    elapsed_s = 0.0f;
    initialized_ = true;
    return true;
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

    // -- Effective Kp: startup 10× for first kStartupDurationS --
    const float kp_eff = (elapsed_s < kStartupDurationS)
                         ? kKp * kStartupKpMultiplier
                         : kKp;

    // ----------------------------------------------------------------
    // Accel correction (gravity reference)
    // Reject if accel magnitude outside gate (free-fall or vibration)
    // ----------------------------------------------------------------
    Vec3 e_accel;
    const float a_mag = accel.norm();
    if (a_mag >= kAccelGateLow && a_mag <= kAccelGateHigh) {
        // Estimated gravity direction in body frame: R^T * [0,0,1]_NED
        // q.conjugate().rotate([0,0,1]) = predicted body-frame down
        const Vec3 v_hat = q.conjugate().rotate(Vec3(0.0f, 0.0f, 1.0f));
        // Measured body-frame down: -accel.normalized()
        const Vec3 a_norm = (accel * -1.0f).normalized();
        // Error: cross product (measurement × prediction)
        e_accel = a_norm.cross(v_hat);
    }

    // ----------------------------------------------------------------
    // Mag correction
    // Skip if cal_valid=false (council), if expected_mag gate fails,
    // or if mag_body is too small (likely sensor fault)
    // ----------------------------------------------------------------
    Vec3 e_mag;
    if (mag_cal_valid && mag_body.norm() > 1.0f) {
        // Mag gate: skip if magnitude deviates beyond ±15%
        bool gate_ok = true;
        if (expected_mag > 0.0f) {
            const float ratio = mag_body.norm() / expected_mag;
            gate_ok = (ratio >= (1.0f - kMagGateFraction)) &&
                      (ratio <= (1.0f + kMagGateFraction));
        }

        if (gate_ok) {
            // Project mag to NED, extract the horizontal reference vector.
            // Mahony reference: b = [sqrt(hx²+hy²), 0, hz] — removes declination
            // dependence while preserving dip angle (Mahony et al. §IV).
            const Vec3 m_norm = mag_body.normalized();
            const Vec3 mag_ned = q.rotate(mag_body);
            const float h_xy = sqrtf(mag_ned.x * mag_ned.x + mag_ned.y * mag_ned.y);
            // Reference in NED: [h_xy, 0, hz] — points North, ignores declination
            const Vec3 b_ned(h_xy, 0.0f, mag_ned.z);
            // Rotate reference back to body frame
            const Vec3 b_body = q.conjugate().rotate(b_ned).normalized();
            // Error: cross product (measurement × prediction)
            e_mag = m_norm.cross(b_body);
        }
    }

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
    const float w_clamped = (q_rel.w > 1.0f) ? 1.0f
                          : (q_rel.w < -1.0f) ? -1.0f
                          : q_rel.w;
    return 2.0f * acosf(fabsf(w_clamped));
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
