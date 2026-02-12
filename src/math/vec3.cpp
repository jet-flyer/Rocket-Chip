#include "math/vec3.h"

#include <cmath>

namespace rc {

Vec3 Vec3::cross(const Vec3& rhs) const {
    return {
        y * rhs.z - z * rhs.y,
        z * rhs.x - x * rhs.z,
        x * rhs.y - y * rhs.x
    };
}

float Vec3::norm() const {
    return sqrtf(norm_sq());
}

Vec3 Vec3::normalized() const {
    const float n = norm();
    if (n < 1e-12f) {
        return {0.0f, 0.0f, 0.0f};
    }
    return *this * (1.0f / n);
}

} // namespace rc
