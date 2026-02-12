#include "math/quat.h"

#include <cmath>

namespace rc {

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
    if (n2 < 1e-12f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float inv_n2 = 1.0f / n2;
    return {w * inv_n2, -x * inv_n2, -y * inv_n2, -z * inv_n2};
}

float Quat::norm() const {
    return sqrtf(norm_sq());
}

Quat& Quat::normalize() {
    const float n = norm();
    if (n < 1e-12f) {
        w = 1.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    } else {
        const float inv_n = 1.0f / n;
        w *= inv_n;
        x *= inv_n;
        y *= inv_n;
        z *= inv_n;
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
    const float tx = 2.0f * (y * v.z - z * v.y);
    const float ty = 2.0f * (z * v.x - x * v.z);
    const float tz = 2.0f * (x * v.y - y * v.x);
    return {
        v.x + w * tx + y * tz - z * ty,
        v.y + w * ty + z * tx - x * tz,
        v.z + w * tz + x * ty - y * tx
    };
}

void Quat::to_rotation_matrix(float m[9]) const {
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

    m[0] = 1.0f - 2.0f * (yy + zz);
    m[1] = 2.0f * (xy - wz);
    m[2] = 2.0f * (xz + wy);

    m[3] = 2.0f * (xy + wz);
    m[4] = 1.0f - 2.0f * (xx + zz);
    m[5] = 2.0f * (yz - wx);

    m[6] = 2.0f * (xz - wy);
    m[7] = 2.0f * (yz + wx);
    m[8] = 1.0f - 2.0f * (xx + yy);
}

Vec3 Quat::to_euler() const {
    // ZYX convention: returns Vec3(roll, pitch, yaw)
    // Reference: Sola (2017) Eq. 290

    // Roll (x-axis rotation)
    const float sinr_cosp = 2.0f * (w * x + y * z);
    const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    const float roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation) — clamp to avoid NaN at gimbal lock
    const float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (sinp >= 1.0f) {
        pitch = 1.5707963f;   // +pi/2
    } else if (sinp <= -1.0f) {
        pitch = -1.5707963f;  // -pi/2
    } else {
        pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    const float yaw = atan2f(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

Quat Quat::from_euler(float roll, float pitch, float yaw) {
    // ZYX convention
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

Quat Quat::from_axis_angle(const Vec3& axis, float angle) {
    const float half_angle = angle * 0.5f;
    const float s = sinf(half_angle);
    const Vec3 n = axis.normalized();
    return Quat(cosf(half_angle), n.x * s, n.y * s, n.z * s).normalized();
}

Quat Quat::from_two_vectors(const Vec3& from, const Vec3& to) {
    // Quaternion that rotates 'from' to 'to'
    const Vec3 fn = from.normalized();
    const Vec3 tn = to.normalized();
    const float d = fn.dot(tn);

    if (d > 0.9999f) {
        // Vectors nearly parallel — identity rotation
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    if (d < -0.9999f) {
        // Vectors nearly antiparallel — 180 deg rotation about any perpendicular axis
        Vec3 perp = Vec3(1.0f, 0.0f, 0.0f).cross(fn);
        if (perp.norm_sq() < 1e-6f) {
            perp = Vec3(0.0f, 1.0f, 0.0f).cross(fn);
        }
        perp = perp.normalized();
        return {0.0f, perp.x, perp.y, perp.z};
    }

    const Vec3 c = fn.cross(tn);
    // q = [1 + dot, cross] then normalize
    // Reference: "half-way quaternion" method
    return Quat(1.0f + d, c.x, c.y, c.z).normalized();
}

Quat Quat::from_small_angle(const Vec3& delta_theta) {
    // First-order approximation: q ~= [1, delta_theta/2] normalized
    // Sola (2017) Eq. 186: for small rotation vector delta_theta,
    // the corresponding quaternion is approximately [1, delta_theta/2].
    const float hx = delta_theta.x * 0.5f;
    const float hy = delta_theta.y * 0.5f;
    const float hz = delta_theta.z * 0.5f;
    return Quat(1.0f, hx, hy, hz).normalized();
}

} // namespace rc
