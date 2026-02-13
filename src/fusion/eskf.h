#ifndef ROCKETCHIP_FUSION_ESKF_H
#define ROCKETCHIP_FUSION_ESKF_H

// ESKF: 15-state Error-State Kalman Filter.
// Pure C++ — no Pico SDK dependencies.
//
// Nominal state: quaternion (body-to-NED), position, velocity, biases.
// Error state: 15-dimensional per eskf_state.h.
// Reference: Solà (2017) "Quaternion kinematics for the error-state KF"
//   arXiv:1711.02508 — 560+ citations (Semantic Scholar, 2026).
//   Industry standard reference used by PX4 ECL-EKF2 and ArduPilot EKF2/3.
//   Tutorial/consolidation paper, not novel research — standardizes the
//   quaternion error-state formulation used across robotics and aerospace.
//
// All noise parameters from ICM-20948 DS-000189 v1.3 Tables 1-2
// unless noted as empirical. Every constant has a source citation.

#include "fusion/eskf_state.h"
#include "math/mat.h"
#include "math/quat.h"
#include "math/vec3.h"

namespace rc {

struct ESKF {
    // =================================================================
    // Nominal state (true state estimate)
    // =================================================================
    Quat q;             // Body-to-NED rotation quaternion
    Vec3 p;             // NED position (m)
    Vec3 v;             // NED velocity (m/s)
    Vec3 accel_bias;    // Accelerometer bias estimate (m/s²)
    Vec3 gyro_bias;     // Gyroscope bias estimate (rad/s)

    // =================================================================
    // Error-state covariance (15×15, symmetric)
    // =================================================================
    Mat15 P;

    // =================================================================
    // Physical constants
    // =================================================================

    // WGS84 standard gravity
    static constexpr float kGravity = 9.80665f;  // m/s²

    // =================================================================
    // IMU noise parameters — ICM-20948 DS-000189 v1.3
    // =================================================================

    // Gyro noise spectral density: 0.015 dps/√Hz (Table 1, BW=10Hz, FS_SEL=0)
    // Conversion: 0.015 × π/180 = 2.618e-4 rad/s/√Hz
    static constexpr float kSigmaGyro = 2.618e-4f;  // rad/s/√Hz

    // Accel noise spectral density: 230 µg/√Hz (Table 2, BW=10Hz)
    // Conversion: 230e-6 × 9.80665 = 2.256e-3 m/s²/√Hz
    static constexpr float kSigmaAccel = 2.256e-3f;  // m/s²/√Hz

    // Gyro bias random walk — no datasheet spec (ICM-20948 does not publish
    // bias instability / Allan deviation). Empirical starting point.
    // ArduPilot EKF3 GBIAS_P_NSE = 1e-3 rad/s (inflated for multirotor).
    // Tune via NEES: if bias states don't converge, increase.
    static constexpr float kSigmaGyroBiasWalk = 1.0e-5f;  // rad/s²/√Hz

    // Accel bias random walk — no datasheet spec. Empirical starting point.
    // ArduPilot EKF3 ABIAS_P_NSE = 2e-2 m/s² (inflated for multirotor).
    // Tune via NEES: if bias states don't converge, increase.
    static constexpr float kSigmaAccelBiasWalk = 1.0e-4f;  // m/s³/√Hz

    // =================================================================
    // P initialization — Solà (2017) typical values + ICM-20948 ZRO specs
    // =================================================================

    // Attitude: ~18° initial uncertainty (Solà typical for gravity-only init)
    static constexpr float kInitPAttitude = 0.1f;  // rad²

    // Position: ~10m uncertainty (no GPS fix at init)
    static constexpr float kInitPPosition = 100.0f;  // m²

    // Velocity: ~1 m/s uncertainty (stationary start assumed)
    static constexpr float kInitPVelocity = 1.0f;  // m²/s²

    // Accel bias: ~0.1 m/s² = ~10mg (within ±50mg board-level ZRO, Table 2)
    static constexpr float kInitPAccelBias = 0.01f;  // (m/s²)²

    // Gyro bias: ~0.017 rad/s = ~1°/s (within ±5°/s ZRO, Table 1)
    static constexpr float kInitPGyroBias = 3.0e-4f;  // (rad/s)²

    // =================================================================
    // P diagonal clamping — council review RF-2
    // =================================================================

    static constexpr float kClampPAttitude = 1.0f;      // rad² (~57° max)
    static constexpr float kClampPPosition = 10000.0f;   // m² (~100m max)
    static constexpr float kClampPVelocity = 100.0f;     // m²/s² (~10 m/s max)

    // =================================================================
    // Stationarity check — council review RF-5
    // =================================================================

    // Accel magnitude tolerance around gravity: ±0.1g
    static constexpr float kStationaryAccelTol = 0.981f;  // m/s² (0.1 × g)

    // Max gyro rate during "stationary" condition
    static constexpr float kStationaryGyroMax = 0.02f;  // rad/s (~1.1°/s)

    // =================================================================
    // Barometric altitude measurement — IVP-43
    // DPS310 @ 16x oversampling noise (same as BaroKF)
    // =================================================================

    static constexpr float kSigmaBaro = 0.029f;                        // m
    static constexpr float kRBaro = kSigmaBaro * kSigmaBaro;           // ~0.000841 m²
    static constexpr float kBaroInnovationGate = 3.0f;                 // 3σ gate

    // =================================================================
    // Methods
    // =================================================================

    // Initialize from stationary accelerometer reading.
    // Derives initial attitude from gravity vector direction.
    // Returns false if stationarity check fails (RF-5).
    bool init(const Vec3& accel_meas, const Vec3& gyro_meas);

    // Propagate (predict) step: integrate IMU measurements forward by dt.
    // Uses sparse FPFT exploiting F_x block structure (R-1).
    // Solà (2017) §5.3.
    void predict(const Vec3& accel_meas, const Vec3& gyro_meas, float dt);

    // Propagate using dense FPFT (verification path only).
    // Same result as predict() but slower — used to validate sparse path.
    void predict_dense(const Vec3& accel_meas, const Vec3& gyro_meas, float dt);

    // Barometric altitude measurement update (IVP-43).
    // z = altitude_agl_m (positive up, from calibration_get_altitude_agl).
    // h(x) = -p.z (NED down negated). H = [0 0 0 | 0 0 -1 | 0...].
    // Returns false if input is non-finite or innovation is gated out.
    // Solà (2017) §7.2, Joseph form for P update.
    bool update_baro(float altitude_agl_m);

    // Error state reset with Jacobian G after measurement update.
    // Absorbs error state into nominal, resets error to zero.
    // Solà (2017) §7.2, council RF-1.
    void reset(const Vec3& delta_theta);

    // Clamp P diagonal to prevent runaway (council RF-2).
    void clamp_covariance();

    // Health check: NaN detection, P bounds, quaternion norm (council RF-3).
    bool healthy() const;

    // =================================================================
    // Diagnostics
    // =================================================================
    float last_propagation_dt_{};
    float last_baro_nis_{};        // IVP-43: Normalized Innovation Squared
    bool initialized_{};

private:
    // Build the error-state transition matrix F_x (15×15).
    // F_x = I + dt * F_delta, where F_delta encodes the linearized dynamics.
    // Output parameter to avoid 900B return-by-value on stack (LL Entry 1).
    static void build_F(Mat15& out, const Quat& q, const Vec3& accel_body,
                         const Vec3& gyro_body, float dt);

    // Build continuous-time process noise Q_c (15×15 diagonal).
    // Output parameter to avoid 900B return-by-value on stack (LL Entry 1).
    static void build_Qc(Mat15& out);

    // Common propagation logic shared by predict() and predict_dense().
    void propagate_nominal(const Vec3& accel_meas, const Vec3& gyro_meas,
                           float dt);
};

} // namespace rc

#endif // ROCKETCHIP_FUSION_ESKF_H
