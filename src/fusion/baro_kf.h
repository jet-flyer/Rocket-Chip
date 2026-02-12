#ifndef ROCKETCHIP_FUSION_BARO_KF_H
#define ROCKETCHIP_FUSION_BARO_KF_H

// BaroKF: 2-state barometric altitude Kalman filter.
// Pure C++ — no Pico SDK dependencies.
//
// State: [altitude_m, vertical_velocity_m_s]
// Process model: constant velocity with acceleration noise.
// Measurement: barometric altitude (scalar).
// Uses Joseph form for covariance update (numerically stable).
//
// Reference: standard linear KF, practice for full ESKF.
// DPS310 noise @ 16x oversampling: 0.35 Pa RMS = ~0.029m altitude noise.
// (See baro_dps310.h for full oversampling tradeoff table.)

#include <cstdint>

namespace rc {

struct BaroKF {
    // State vector: [altitude, vertical_velocity]
    float x[2]{};

    // Covariance matrix (2x2, symmetric, stored as 4 elements)
    float P[2][2]{};

    // Tuning parameters (all with k prefix, justified by source)

    // DPS310 datasheet Table 16: 0.35 Pa RMS @ 16x oversampling
    // Converted: 0.35 Pa * 0.083 m/Pa = 0.029 m altitude noise
    static constexpr float kSigmaBaro = 0.029f;
    static constexpr float kRBaro = kSigmaBaro * kSigmaBaro;  // ~0.000841 m^2

    // Process noise PSD for vertical acceleration.
    // No datasheet value — empirical starting point. Represents expected
    // unmodeled vertical acceleration (vibration, wind, movement).
    // Tune via NIS: if NIS consistently > 3.84 (chi^2(1) 95%), increase.
    static constexpr float kQAccel = 0.1f;  // m/s^2

    // Track NIS for diagnostics
    float last_nis_{};

    // Initialize filter state
    void init(float altitude_m);

    // Predict step: propagate state and covariance forward by dt seconds.
    // Uses constant-velocity model with acceleration process noise.
    void predict(float dt);

    // Update step: incorporate barometric altitude measurement.
    // Uses Joseph form for numerical stability.
    // Returns false if measurement is rejected (not implemented yet — always true).
    bool update(float baro_alt_m);

    // Accessors
    float altitude() const { return x[0]; }
    float velocity() const { return x[1]; }
    float last_nis() const { return last_nis_; }

    // Health check: P diagonal positive and finite
    bool healthy() const;
};

} // namespace rc

#endif // ROCKETCHIP_FUSION_BARO_KF_H
