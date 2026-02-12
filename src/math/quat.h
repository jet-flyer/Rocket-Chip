#ifndef ROCKETCHIP_MATH_QUAT_H
#define ROCKETCHIP_MATH_QUAT_H

// Quat: Hamilton convention quaternion for sensor fusion.
// Scalar-first [w, x, y, z]. Pure C++ — no Pico SDK dependencies.
//
// Convention: body-to-NED rotation. Hamilton product order.
// Reference: Sola (2017) "Quaternion kinematics for the error-state Kalman filter"

#include "math/vec3.h"

namespace rc {

struct Quat {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Quat() = default;
    constexpr Quat(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    // Hamilton product: q * rhs
    Quat operator*(const Quat& rhs) const;

    // Conjugate: q* = [w, -x, -y, -z]
    Quat conjugate() const { return {w, -x, -y, -z}; }

    // Inverse: q^-1 = q* / |q|^2  (for unit quaternions, same as conjugate)
    Quat inverse() const;

    // Norm
    float norm_sq() const { return w * w + x * x + y * y + z * z; }
    float norm() const;

    // Normalize in place, returns *this for chaining
    Quat& normalize();

    // Return normalized copy
    Quat normalized() const;

    // Rotate a vector: v' = q * [0,v] * q*
    Vec3 rotate(const Vec3& v) const;

    // Convert to 3x3 rotation matrix (row-major, 9 floats)
    // Output: m[0..8] = row-major DCM (body-to-NED)
    void to_rotation_matrix(float m[9]) const;

    // Convert to Euler angles (ZYX convention: yaw, pitch, roll)
    // Returns Vec3(roll, pitch, yaw) in radians
    Vec3 to_euler() const;

    // ---- Static constructors ----

    // From Euler angles (ZYX convention)
    // roll = rotation about X, pitch = about Y, yaw = about Z
    static Quat from_euler(float roll, float pitch, float yaw);

    // From axis-angle: rotation of 'angle' radians about unit vector 'axis'
    static Quat from_axis_angle(const Vec3& axis, float angle);

    // Quaternion that rotates vector 'from' to vector 'to'
    static Quat from_two_vectors(const Vec3& from, const Vec3& to);

    // First-order quaternion from small rotation vector delta_theta:
    // q ~= [1, delta_theta/2] normalized
    // Core ESKF operation for error state injection. Sola (2017) Eq. 186.
    static Quat from_small_angle(const Vec3& delta_theta);
};

} // namespace rc

#endif // ROCKETCHIP_MATH_QUAT_H
