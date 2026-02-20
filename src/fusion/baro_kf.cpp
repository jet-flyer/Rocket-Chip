#include "fusion/baro_kf.h"

#include <cmath>

namespace rc {

// Process noise coefficient multiplier for dt^4/4 term
constexpr float kDt4Coeff = 0.25F;
// Minimum innovation covariance to avoid division by zero
constexpr float kMinInnovCov = 1e-30F;

void BaroKF::init(float altitudeM) {
    x[0] = altitudeM;
    x[1] = 0.0F;

    // Initial covariance: moderate uncertainty in altitude,
    // higher uncertainty in velocity (unknown at start)
    P[0][0] = 1.0F;    // 1 m^2 altitude uncertainty
    P[0][1] = 0.0F;
    P[1][0] = 0.0F;
    P[1][1] = 1.0F;    // 1 (m/s)^2 velocity uncertainty

    last_nis_ = 0.0F;
}

void BaroKF::predict(float dt) {
    // State prediction: constant velocity model
    // x_new = F * x  where F = [[1, dt], [0, 1]]
    x[0] += x[1] * dt;
    // x[1] unchanged (constant velocity assumption)

    // Covariance prediction: P_new = F*P*F^T + Q
    //
    // F = [[1, dt], [0, 1]]
    //
    // Q = [[dt^4/4, dt^3/2], [dt^3/2, dt^2]] * kQAccel
    // This is the standard discrete-time process noise for a
    // constant-velocity model with acceleration as white noise.
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;

    // F*P*F^T (expand manually for 2x2 — cheaper than matrix multiply)
    const float p00 = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt2 * P[1][1];
    const float p01 = P[0][1] + dt * P[1][1];
    const float p10 = P[1][0] + dt * P[1][1];
    const float p11 = P[1][1];

    // Add process noise Q
    P[0][0] = p00 + dt4 * kDt4Coeff * kQAccel;
    P[0][1] = p01 + dt3 * 0.5F * kQAccel;
    P[1][0] = p10 + dt3 * 0.5F * kQAccel;
    P[1][1] = p11 + dt2 * kQAccel;

    // Enforce symmetry (compensate for float rounding)
    const float avg01 = (P[0][1] + P[1][0]) * 0.5F;
    P[0][1] = avg01;
    P[1][0] = avg01;
}

bool BaroKF::update(float baroAltM) {
    // Measurement model: H = [1, 0], z = baroAltM, R = kRBaro
    //
    // Innovation: y = z - H*x = z - x[0]
    const float y = baroAltM - x[0];

    // Innovation covariance: S = H*P*H^T + R = P[0][0] + R
    const float s = P[0][0] + kRBaro;

    // NIS (Normalized Innovation Squared) for diagnostics
    // For chi^2(1): 95% should be < 3.84
    last_nis_ = (s > kMinInnovCov) ? (y * y / s) : 0.0F;

    // Kalman gain: K = P*H^T / S = [P[0][0], P[1][0]]^T / S
    const float invS = (s > kMinInnovCov) ? (1.0F / s) : 0.0F;
    const float k0 = P[0][0] * invS;
    const float k1 = P[1][0] * invS;

    // State update: x = x + K*y
    x[0] += k0 * y;
    x[1] += k1 * y;

    // Covariance update: Joseph form
    joseph_update(k0, k1);

    return true;
}

void BaroKF::joseph_update(float k0, float k1) {
    // P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    //
    // For H = [1, 0]:
    //   (I - K*H) = [[1-K0, 0], [-K1, 1]]
    //
    // Expand manually for 2x2 efficiency:
    const float ikh00 = 1.0F - k0;
    const float ikh10 = -k1;
    // ikh01 = 0, ikh11 = 1

    // term1 = (I-KH) * P * (I-KH)^T
    // First: tmp = (I-KH) * P
    const float tmp00 = ikh00 * P[0][0];                    // +0*P[1][0]
    const float tmp01 = ikh00 * P[0][1];                    // +0*P[1][1]
    const float tmp10 = ikh10 * P[0][0] + P[1][0];          // +1*P[1][0]
    const float tmp11 = ikh10 * P[0][1] + P[1][1];          // +1*P[1][1]

    // Then: term1 = tmp * (I-KH)^T   where (I-KH)^T = [[1-K0, -K1], [0, 1]]
    const float t100 = tmp00 * ikh00 + tmp01 * 0.0F;       // *ikh00 col0
    const float t101 = tmp00 * ikh10 + tmp01 * 1.0F;       // *IKH^T col1
    const float t110 = tmp10 * ikh00 + tmp11 * 0.0F;
    const float t111 = tmp10 * ikh10 + tmp11 * 1.0F;

    // term2 = K*R*K^T  (outer product scaled by R)
    const float t200 = k0 * kRBaro * k0;
    const float t201 = k0 * kRBaro * k1;
    const float t210 = k1 * kRBaro * k0;
    const float t211 = k1 * kRBaro * k1;

    P[0][0] = t100 + t200;
    P[0][1] = t101 + t201;
    P[1][0] = t110 + t210;
    P[1][1] = t111 + t211;

    // Enforce symmetry
    const float avg = (P[0][1] + P[1][0]) * 0.5F;
    P[0][1] = avg;
    P[1][0] = avg;
}

bool BaroKF::healthy() const {
    // Check diagonal positive
    if (P[0][0] <= 0.0F || P[1][1] <= 0.0F) {
        return false;
    }
    // Check all elements finite
    for (const auto& row : P) {
        for (const auto& val : row) {
            if (!std::isfinite(val)) {
                return false;
            }
        }
    }
    return std::isfinite(x[0]) && std::isfinite(x[1]);
}

} // namespace rc
