#include "math/vec3.h"

#include <cmath>

namespace rc {

constexpr float kNormEpsilon = 1e-12F;  // Minimum norm for safe division

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
    if (n < kNormEpsilon) {
        return {0.0F, 0.0F, 0.0F};
    }
    return *this * (1.0F / n);
}

} // namespace rc
