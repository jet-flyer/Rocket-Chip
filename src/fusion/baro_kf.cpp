#include "fusion/baro_kf.h"

#include <cmath>

namespace rc {

void BaroKF::init(float altitude_m) {
    x[0] = altitude_m;
    x[1] = 0.0f;

    // Initial covariance: moderate uncertainty in altitude,
    // higher uncertainty in velocity (unknown at start)
    P[0][0] = 1.0f;    // 1 m^2 altitude uncertainty
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 1.0f;    // 1 (m/s)^2 velocity uncertainty

    last_nis_ = 0.0f;
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
    const float P00 = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt2 * P[1][1];
    const float P01 = P[0][1] + dt * P[1][1];
    const float P10 = P[1][0] + dt * P[1][1];
    const float P11 = P[1][1];

    // Add process noise Q
    P[0][0] = P00 + dt4 * 0.25f * kQAccel;
    P[0][1] = P01 + dt3 * 0.5f * kQAccel;
    P[1][0] = P10 + dt3 * 0.5f * kQAccel;
    P[1][1] = P11 + dt2 * kQAccel;

    // Enforce symmetry (compensate for float rounding)
    const float avg01 = (P[0][1] + P[1][0]) * 0.5f;
    P[0][1] = avg01;
    P[1][0] = avg01;
}

bool BaroKF::update(float baro_alt_m) {
    // Measurement model: H = [1, 0], z = baro_alt_m, R = kRBaro
    //
    // Innovation: y = z - H*x = z - x[0]
    const float y = baro_alt_m - x[0];

    // Innovation covariance: S = H*P*H^T + R = P[0][0] + R
    const float S = P[0][0] + kRBaro;

    // NIS (Normalized Innovation Squared) for diagnostics
    // For chi^2(1): 95% should be < 3.84
    last_nis_ = (S > 1e-30f) ? (y * y / S) : 0.0f;

    // Kalman gain: K = P*H^T / S = [P[0][0], P[1][0]]^T / S
    const float inv_S = (S > 1e-30f) ? (1.0f / S) : 0.0f;
    const float K0 = P[0][0] * inv_S;
    const float K1 = P[1][0] * inv_S;

    // State update: x = x + K*y
    x[0] += K0 * y;
    x[1] += K1 * y;

    // Covariance update: Joseph form
    // P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    //
    // For H = [1, 0]:
    //   (I - K*H) = [[1-K0, 0], [-K1, 1]]
    //
    // Expand manually for 2x2 efficiency:
    const float IKH00 = 1.0f - K0;
    const float IKH10 = -K1;
    // IKH01 = 0, IKH11 = 1

    // term1 = (I-KH) * P * (I-KH)^T
    // First: tmp = (I-KH) * P
    const float tmp00 = IKH00 * P[0][0];                    // +0*P[1][0]
    const float tmp01 = IKH00 * P[0][1];                    // +0*P[1][1]
    const float tmp10 = IKH10 * P[0][0] + P[1][0];          // +1*P[1][0]
    const float tmp11 = IKH10 * P[0][1] + P[1][1];          // +1*P[1][1]

    // Then: term1 = tmp * (I-KH)^T   where (I-KH)^T = [[1-K0, -K1], [0, 1]]
    const float t1_00 = tmp00 * IKH00 + tmp01 * 0.0f;       // *IKH00 col0
    const float t1_01 = tmp00 * IKH10 + tmp01 * 1.0f;       // *IKH^T col1
    const float t1_10 = tmp10 * IKH00 + tmp11 * 0.0f;
    const float t1_11 = tmp10 * IKH10 + tmp11 * 1.0f;

    // term2 = K*R*K^T  (outer product scaled by R)
    const float t2_00 = K0 * kRBaro * K0;
    const float t2_01 = K0 * kRBaro * K1;
    const float t2_10 = K1 * kRBaro * K0;
    const float t2_11 = K1 * kRBaro * K1;

    P[0][0] = t1_00 + t2_00;
    P[0][1] = t1_01 + t2_01;
    P[1][0] = t1_10 + t2_10;
    P[1][1] = t1_11 + t2_11;

    // Enforce symmetry
    const float avg = (P[0][1] + P[1][0]) * 0.5f;
    P[0][1] = avg;
    P[1][0] = avg;

    return true;
}

bool BaroKF::healthy() const {
    // Check diagonal positive
    if (P[0][0] <= 0.0f || P[1][1] <= 0.0f) {
        return false;
    }
    // Check all elements finite
    for (int32_t r = 0; r < 2; ++r) {
        for (int32_t c = 0; c < 2; ++c) {
            if (!std::isfinite(P[r][c])) {
                return false;
            }
        }
    }
    if (!std::isfinite(x[0]) || !std::isfinite(x[1])) {
        return false;
    }
    return true;
}

} // namespace rc
