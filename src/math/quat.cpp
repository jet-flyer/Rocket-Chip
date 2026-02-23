// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "math/quat.h"

#include <cmath>

namespace rc {

// Near-zero threshold for safe division (norm and norm_sq)
constexpr float kNormEpsilon   = 1e-12F;
// Dot product threshold for near-parallel/antiparallel vectors
constexpr float kDotParallel   = 0.9999F;
// Cross product threshold for fallback perpendicular axis
constexpr float kCrossEpsilon  = 1e-6F;
// Pi/2 for gimbal lock pitch clamp
constexpr float kHalfPi        = 1.5707963F;

Quat Quat::operator*(const Quat& rhs) const {
    // Hamilton product
    return {
        w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
        w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
        w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
        w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
    };
}

Quat Quat::inverse() const {
    const float n2 = norm_sq();
    if (n2 < kNormEpsilon) {
        return {1.0F, 0.0F, 0.0F, 0.0F};
    }
    const float invN2 = 1.0F / n2;
    return {w * invN2, -x * invN2, -y * invN2, -z * invN2};
}

float Quat::norm() const {
    return sqrtf(norm_sq());
}

Quat& Quat::normalize() {
    const float n = norm();
    if (n < kNormEpsilon) {
        w = 1.0F;
        x = 0.0F;
        y = 0.0F;
        z = 0.0F;
    } else {
        const float invN = 1.0F / n;
        w *= invN;
        x *= invN;
        y *= invN;
        z *= invN;
    }
    return *this;
}

Quat Quat::normalized() const {
    Quat result = *this;
    result.normalize();
    return result;
}

Vec3 Quat::rotate(const Vec3& v) const {
    // q * [0,v] * q*  — expanded for efficiency (avoids two full quaternion multiplies)
    // Reference: Sola (2017) Eq. 27-28
    const float tx = 2.0F * (y * v.z - z * v.y);
    const float ty = 2.0F * (z * v.x - x * v.z);
    const float tz = 2.0F * (x * v.y - y * v.x);
    return {
        v.x + w * tx + y * tz - z * ty,
        v.y + w * ty + z * tx - x * tz,
        v.z + w * tz + x * ty - y * tx
    };
}

void Quat::to_rotation_matrix(float m[9]) const { // NOLINT(readability-magic-numbers)
    // Row-major DCM. Reference: Sola (2017) Eq. 22
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    m[0] = 1.0F - 2.0F * (yy + zz);
    m[1] = 2.0F * (xy - wz);
    m[2] = 2.0F * (xz + wy);

    m[3] = 2.0F * (xy + wz);
    m[4] = 1.0F - 2.0F * (xx + zz);
    m[5] = 2.0F * (yz - wx);  // NOLINT(readability-magic-numbers)

    m[6] = 2.0F * (xz - wy);  // NOLINT(readability-magic-numbers)
    m[7] = 2.0F * (yz + wx);  // NOLINT(readability-magic-numbers)
    m[8] = 1.0F - 2.0F * (xx + yy);
}

Vec3 Quat::to_euler() const {
    // ZYX convention: returns Vec3(roll, pitch, yaw)
    // Reference: Sola (2017) Eq. 290

    // Roll (x-axis rotation)
    const float sinrCosp = 2.0F * (w * x + y * z);
    const float cosrCosp = 1.0F - 2.0F * (x * x + y * y);
    const float roll = atan2f(sinrCosp, cosrCosp);

    // Pitch (y-axis rotation) — clamp to avoid NaN at gimbal lock
    const float sinp = 2.0F * (w * y - z * x);
    float pitch = 0.0F;
    if (sinp >= 1.0F) {
        pitch = kHalfPi;   // +pi/2
    } else if (sinp <= -1.0F) {
        pitch = -kHalfPi;  // -pi/2
    } else {
        pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    const float sinyCosp = 2.0F * (w * z + x * y);
    const float cosyCosp = 1.0F - 2.0F * (y * y + z * z);
    const float yaw = atan2f(sinyCosp, cosyCosp);

    return {roll, pitch, yaw};
}

Quat Quat::from_euler(float roll, float pitch, float yaw) {
    // ZYX convention
    const float cr = cosf(roll * 0.5F);
    const float sr = sinf(roll * 0.5F);
    const float cp = cosf(pitch * 0.5F);
    const float sp = sinf(pitch * 0.5F);
    const float cy = cosf(yaw * 0.5F);
    const float sy = sinf(yaw * 0.5F);

    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

Quat Quat::from_axis_angle(const Vec3& axis, float angle) {
    const float halfAngle = angle * 0.5F;
    const float s = sinf(halfAngle);
    const Vec3 n = axis.normalized();
    return Quat(cosf(halfAngle), n.x * s, n.y * s, n.z * s).normalized();
}

Quat Quat::from_two_vectors(const Vec3& from, const Vec3& to) {
    // Quaternion that rotates 'from' to 'to'
    const Vec3 fn = from.normalized();
    const Vec3 tn = to.normalized();
    const float d = fn.dot(tn);

    if (d > kDotParallel) {
        // Vectors nearly parallel — identity rotation
        return {1.0F, 0.0F, 0.0F, 0.0F};
    }

    if (d < -kDotParallel) {
        // Vectors nearly antiparallel — 180 deg rotation about any perpendicular axis
        Vec3 perp = Vec3(1.0F, 0.0F, 0.0F).cross(fn);
        if (perp.norm_sq() < kCrossEpsilon) {
            perp = Vec3(0.0F, 1.0F, 0.0F).cross(fn);
        }
        perp = perp.normalized();
        return {0.0F, perp.x, perp.y, perp.z};
    }

    const Vec3 c = fn.cross(tn);
    // q = [1 + dot, cross] then normalize
    // Reference: "half-way quaternion" method
    return Quat(1.0F + d, c.x, c.y, c.z).normalized();
}

Quat Quat::from_small_angle(const Vec3& deltaTheta) {
    // First-order approximation: q ~= [1, deltaTheta/2] normalized
    // Sola (2017) Eq. 186: for small rotation vector deltaTheta,
    // the corresponding quaternion is approximately [1, deltaTheta/2].
    const float hx = deltaTheta.x * 0.5F;
    const float hy = deltaTheta.y * 0.5F;
    const float hz = deltaTheta.z * 0.5F;
    return Quat(1.0F, hx, hy, hz).normalized();
}

} // namespace rc
