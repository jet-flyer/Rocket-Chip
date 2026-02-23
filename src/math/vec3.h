// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_MATH_VEC3_H
#define ROCKETCHIP_MATH_VEC3_H

// Vec3: 3D vector for sensor fusion math.
// Pure C++ — no Pico SDK dependencies. Must compile on any host.

namespace rc {

struct Vec3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    Vec3& operator-=(const Vec3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float dot(const Vec3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }
    Vec3 cross(const Vec3& rhs) const;
    float norm_sq() const { return dot(*this); }
    float norm() const;
    Vec3 normalized() const;
};

} // namespace rc

#endif // ROCKETCHIP_MATH_VEC3_H
