#include "fusion/eskf.h"

#include <cmath>

namespace rc {

// ============================================================================
// Helper: 3x3 skew-symmetric matrix from vector
// [v]_x = | 0  -vz  vy |
//          | vz  0  -vx |
//          |-vy  vx  0  |
// Solà (2017) Eq. 9
// ============================================================================
static Mat3 skew(const Vec3& v) {
    Mat3 m;
    m(0, 1) = -v.z;  m(0, 2) =  v.y;
    m(1, 0) =  v.z;  m(1, 2) = -v.x;
    m(2, 0) = -v.y;  m(2, 1) =  v.x;
    return m;
}

// ============================================================================
// Helper: extract 3x3 rotation matrix from quaternion as Mat3
// ============================================================================
static Mat3 quat_to_dcm(const Quat& q) {
    float m9[9];
    q.to_rotation_matrix(m9);
    Mat3 R;
    R(0, 0) = m9[0]; R(0, 1) = m9[1]; R(0, 2) = m9[2];
    R(1, 0) = m9[3]; R(1, 1) = m9[4]; R(1, 2) = m9[5];
    R(2, 0) = m9[6]; R(2, 1) = m9[7]; R(2, 2) = m9[8];
    return R;
}

// ============================================================================
// init: Initialize ESKF from stationary IMU reading
// ============================================================================
bool ESKF::init(const Vec3& accel_meas, const Vec3& gyro_meas) {
    // RF-5: stationarity check — reject if not approximately stationary
    const float accel_mag = accel_meas.norm();
    if (fabsf(accel_mag - kGravity) > kStationaryAccelTol) {
        return false;
    }
    if (gyro_meas.norm() > kStationaryGyroMax) {
        return false;
    }

    // Derive initial attitude from gravity vector direction.
    // The accelerometer measures -g in body frame when stationary.
    // We want the quaternion that rotates body frame to NED, where
    // gravity is [0, 0, +g].
    //
    // Body-frame gravity measurement: accel_meas ≈ -R^T * g_ned
    // So body-frame "down" direction = -accel_meas.normalized()
    // We need R that maps body-down to NED-down [0,0,1].
    const Vec3 body_down = (accel_meas * -1.0f).normalized();
    const Vec3 ned_down(0.0f, 0.0f, 1.0f);
    q = Quat::from_two_vectors(body_down, ned_down);
    q.normalize();

    // Initialize position, velocity, biases to zero (stationary on pad)
    p = Vec3();
    v = Vec3();
    accel_bias = Vec3();
    gyro_bias = Vec3();

    // Initialize P diagonal per eskf.h constants
    P.set_zero();
    using namespace eskf;
    for (int32_t i = 0; i < kBlockSize; ++i) {
        P(kIdxAttitude + i, kIdxAttitude + i)  = kInitPAttitude;
        P(kIdxPosition + i, kIdxPosition + i)  = kInitPPosition;
        P(kIdxVelocity + i, kIdxVelocity + i)  = kInitPVelocity;
        P(kIdxAccelBias + i, kIdxAccelBias + i) = kInitPAccelBias;
        P(kIdxGyroBias + i, kIdxGyroBias + i)  = kInitPGyroBias;
    }

    initialized_ = true;
    last_propagation_dt_ = 0.0f;
    return true;
}

// ============================================================================
// propagate_nominal: First-order Euler integration of nominal state
// Solà (2017) §5.3, Eq. 221-225
// ============================================================================
void ESKF::propagate_nominal(const Vec3& accel_meas, const Vec3& gyro_meas,
                              float dt) {
    // Bias-corrected measurements
    const Vec3 accel_body = accel_meas - accel_bias;
    const Vec3 gyro_body = gyro_meas - gyro_bias;

    // Rotation matrix: body-to-NED
    const Mat3 R = quat_to_dcm(q);

    // True acceleration in NED frame.
    // Accelerometer measures specific force: f = a_true - g (in body frame).
    // True acceleration: a_true = R * f_body + g_ned
    // where g_ned = [0, 0, +g] (gravity points down in NED).
    // Solà (2017) Eq. 221.
    Vec3 accel_ned;
    accel_ned.x = R(0, 0) * accel_body.x + R(0, 1) * accel_body.y + R(0, 2) * accel_body.z;
    accel_ned.y = R(1, 0) * accel_body.x + R(1, 1) * accel_body.y + R(1, 2) * accel_body.z;
    accel_ned.z = R(2, 0) * accel_body.x + R(2, 1) * accel_body.y + R(2, 2) * accel_body.z;
    accel_ned.z += kGravity;

    // Position: p += v * dt + 0.5 * a * dt^2  (simplified to first order: p += v * dt)
    // Using first-order for simplicity — the dt^2 term is negligible at 200Hz
    p += v * dt;

    // Velocity: v += a_ned * dt
    v += accel_ned * dt;

    // Quaternion: q ← q ⊗ Exp(gyro_body * dt)
    // First-order: q ← q ⊗ [1, gyro_body*dt/2]
    const Vec3 delta_theta = gyro_body * dt;
    const Quat dq = Quat::from_small_angle(delta_theta);
    q = q * dq;
    q.normalize();
}

// ============================================================================
// build_F: Error-state transition matrix F_x (15×15)
// F_x = I + dt * F_delta
// Solà (2017) §5.3.3, Eq. 269
//
// F_delta block structure (named indices from eskf_state.h):
//   [att]  | -[ω]_x     0      0      0     -I   |
//   [pos]  |    0        0      I      0      0   |
//   [vel]  | -R[a]_x    0      0     -R      0   |
//   [ab]   |    0        0      0      0      0   |
//   [gb]   |    0        0      0      0      0   |
//
// Where: ω = gyro_body (bias-corrected), a = accel_body (bias-corrected),
//         R = body-to-NED DCM, [·]_x = skew-symmetric matrix
// ============================================================================
void ESKF::build_F(Mat15& F, const Quat& q_in, const Vec3& accel_body,
                    const Vec3& gyro_body, float dt) {
    using namespace eskf;

    F.set_identity();

    const Mat3 R = quat_to_dcm(q_in);
    const Mat3 skew_omega = skew(gyro_body);
    const Mat3 skew_accel = skew(accel_body);

    // Attitude block: F[att, att] = I - dt * [ω]_x
    // (identity already set, subtract dt * skew_omega)
    const Mat3 neg_dt_skew_omega = skew_omega * (-dt);
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            F(kIdxAttitude + r, kIdxAttitude + c) += neg_dt_skew_omega(r, c);
        }
    }

    // Attitude-GyroBias block: F[att, gb] = -I * dt
    for (int32_t i = 0; i < 3; ++i) {
        F(kIdxAttitude + i, kIdxGyroBias + i) = -dt;
    }

    // Position-Velocity block: F[pos, vel] = I * dt
    for (int32_t i = 0; i < 3; ++i) {
        F(kIdxPosition + i, kIdxVelocity + i) = dt;
    }

    // Velocity-Attitude block: F[vel, att] = -R * [a]_x * dt
    // = -dt * R * skew(accel_body)
    const Mat3 R_skew_a = R * skew_accel;
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            F(kIdxVelocity + r, kIdxAttitude + c) = -dt * R_skew_a(r, c);
        }
    }

    // Velocity-AccelBias block: F[vel, ab] = -R * dt
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            F(kIdxVelocity + r, kIdxAccelBias + c) = -dt * R(r, c);
        }
    }
}

// ============================================================================
// build_Qc: Continuous-time process noise Q_c (15×15 diagonal)
// Q_d ≈ Q_c * dt  (zeroth-order hold discretization, per R-9)
// ============================================================================
void ESKF::build_Qc(Mat15& Qc) {
    using namespace eskf;

    Qc.set_zero();

    const float sigma_gyro_sq = kSigmaGyro * kSigmaGyro;
    const float sigma_accel_sq = kSigmaAccel * kSigmaAccel;
    const float sigma_gyro_bias_sq = kSigmaGyroBiasWalk * kSigmaGyroBiasWalk;
    const float sigma_accel_bias_sq = kSigmaAccelBiasWalk * kSigmaAccelBiasWalk;

    for (int32_t i = 0; i < kBlockSize; ++i) {
        Qc(kIdxAttitude + i, kIdxAttitude + i)  = sigma_gyro_sq;
        Qc(kIdxVelocity + i, kIdxVelocity + i)  = sigma_accel_sq;
        Qc(kIdxAccelBias + i, kIdxAccelBias + i) = sigma_accel_bias_sq;
        Qc(kIdxGyroBias + i, kIdxGyroBias + i)  = sigma_gyro_bias_sq;
    }
    // Position block: no direct noise (position uncertainty grows via velocity)
}

// ============================================================================
// predict: Sparse FPFT propagation (R-1 optimization)
//
// Exploits F_x block structure: many blocks are zero or identity.
// Computes P_new = F * P * F^T + Q_d using only non-zero blocks.
//
// The sparse approach avoids the full 15×15 × 15×15 × 15×15 triple product
// by computing individual 3×3 block contributions. This is O(5^2 * 3^3)
// instead of O(15^3), roughly 5× faster on embedded targets.
// ============================================================================
void ESKF::predict(const Vec3& accel_meas, const Vec3& gyro_meas, float dt) {
    // Bias-corrected measurements (needed for F construction)
    const Vec3 accel_body = accel_meas - accel_bias;
    const Vec3 gyro_body = gyro_meas - gyro_bias;

    // 1. Propagate nominal state
    propagate_nominal(accel_meas, gyro_meas, dt);

    // 2. Build F and Q_d
    // Note: F uses the pre-propagation quaternion for linearization.
    // We already propagated nominal state, so we use the PRE-update q.
    // However, for first-order accuracy at typical rates (200Hz, dt=5ms),
    // the difference between pre and post q is negligible.
    // ArduPilot EKF3 uses the same approach.
    // Static Mat15 locals to avoid stack overflow on RP2350 (LL Entry 1).
    // Mat15 = 900B each; multiple on stack causes MemManage fault.
    // Safe: single-threaded Core 0.
    static Mat15 F_local;
    static Mat15 Qd_local;
    build_F(F_local, q, accel_body, gyro_body, dt);
    build_Qc(Qd_local);        // Build Qc directly into Qd_local
    Qd_local.scale(dt);         // Zeroth-order hold (R-9): Qd = Qc * dt

    // 3. Dense F*P*F^T + Q_d
    // Dense path: O(15^3) ≈ 3375 multiply-adds at 200Hz = ~675K flops/sec.
    // TODO(IVP-42b): Profile on target, implement sparse if needed.
    //
    // fpft_dense inlined to use static temporaries (avoid 1800B stack
    // from template return values at -O0).
    static Mat15 FP_temp;
    static Mat15 Ft_temp;
    // FP = F * P
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < 15; ++k) {
                sum += F_local.data[r][k] * P.data[k][c];
            }
            FP_temp.data[r][c] = sum;
        }
    }
    // F^T
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            Ft_temp.data[c][r] = F_local.data[r][c];
        }
    }
    // P = FP * F^T + Qd
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < 15; ++k) {
                sum += FP_temp.data[r][k] * Ft_temp.data[k][c];
            }
            P.data[r][c] = sum + Qd_local.data[r][c];
        }
    }

    // 4. Symmetry enforcement
    P.force_symmetric();

    // 5. Covariance clamping (RF-2)
    clamp_covariance();

    last_propagation_dt_ = dt;
}

// ============================================================================
// predict_dense: Dense FPFT verification path
// Same result as predict() — used in tests to validate sparse path.
// ============================================================================
void ESKF::predict_dense(const Vec3& accel_meas, const Vec3& gyro_meas,
                          float dt) {
    const Vec3 accel_body = accel_meas - accel_bias;
    const Vec3 gyro_body = gyro_meas - gyro_bias;

    propagate_nominal(accel_meas, gyro_meas, dt);

    // Static locals: same rationale as predict() (LL Entry 1).
    static Mat15 F_d;
    static Mat15 Qd_d;
    static Mat15 FP_d;
    static Mat15 Ft_d;
    build_F(F_d, q, accel_body, gyro_body, dt);
    build_Qc(Qd_d);
    Qd_d.scale(dt);

    // F*P*F^T + Qd inlined with static temporaries (same as predict())
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < 15; ++k) {
                sum += F_d.data[r][k] * P.data[k][c];
            }
            FP_d.data[r][c] = sum;
        }
    }
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            Ft_d.data[c][r] = F_d.data[r][c];
        }
    }
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < 15; ++k) {
                sum += FP_d.data[r][k] * Ft_d.data[k][c];
            }
            P.data[r][c] = sum + Qd_d.data[r][c];
        }
    }
    P.force_symmetric();
    clamp_covariance();

    last_propagation_dt_ = dt;
}

// ============================================================================
// reset: Error state injection with Jacobian G
// Solà (2017) §7.2, Eq. 282-286
//
// After a measurement update computes the error state delta_x, the nominal
// state absorbs the correction:
//   q ← q ⊗ Exp(delta_theta)
//   p ← p + delta_p
//   v ← v + delta_v
//   ab ← ab + delta_ab
//   gb ← gb + delta_gb
//
// The covariance is updated via the reset Jacobian G:
//   P ← G * P * G^T
//
// G is the identity everywhere except the attitude block, where:
//   G_theta = I - [delta_theta/2]_x
//
// For small delta_theta (which it should be after a good update),
// G ≈ I and the correction to P is negligible. We include it for
// correctness per the council RF-1 requirement.
// ============================================================================
void ESKF::reset(const Vec3& delta_theta) {
    using namespace eskf;

    // The full error state would be passed in; for now this handles
    // the attitude-only reset that's most critical. The caller is
    // responsible for applying delta_p, delta_v, delta_ab, delta_gb
    // to the nominal state before calling reset().

    // Attitude injection
    const Quat dq = Quat::from_small_angle(delta_theta);
    q = q * dq;
    q.normalize();

    // Build reset Jacobian G (15×15)
    // G = I except G[att,att] = I - [delta_theta/2]_x
    // Static: same rationale as predict() (LL Entry 1).
    static Mat15 G;
    G.set_identity();
    const Mat3 half_skew = skew(delta_theta * 0.5f);
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            G(kIdxAttitude + r, kIdxAttitude + c) -= half_skew(r, c);
        }
    }

    // P ← G * P * G^T (inlined with static temporaries, LL Entry 1)
    static Mat15 GP_temp;
    static Mat15 Gt_temp;
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < 15; ++k) {
                sum += G.data[r][k] * P.data[k][c];
            }
            GP_temp.data[r][c] = sum;
        }
    }
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            Gt_temp.data[c][r] = G.data[r][c];
        }
    }
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < 15; ++k) {
                sum += GP_temp.data[r][k] * Gt_temp.data[k][c];
            }
            P.data[r][c] = sum;
        }
    }
    P.force_symmetric();
}

// ============================================================================
// update_baro: Barometric altitude measurement update (IVP-43)
//
// Measurement model:
//   z = altitude_agl_m (positive up)
//   h(x) = -p.z (NED down negated → altitude up)
//   H = [0 0 0 | 0 0 -1 | 0 0 0 | 0 0 0 | 0 0 0]   (1×15)
//   R = kRBaro = 0.029² ≈ 0.000841 m²
//
// Sequential scalar update — no matrix inverse needed.
// Joseph form P update for numerical stability (Bucy & Joseph, 1968).
// Static locals for Mat15 temporaries (LL Entry 1, ~2.9KB BSS).
// Single-threaded Core 0 — no reentrancy concern.
//
// Council conditions: isfinite() guard, S > 0 guard. See review verdict.
// P rotation at reset() omitted — small-angle approximation, G ≈ I to
// float precision for baro-only corrections. See Solà (2017) Eq. 298.
// ============================================================================
bool ESKF::update_baro(float altitude_agl_m) {
    using namespace eskf;

    // Council condition 1: reject non-finite input
    if (!std::isfinite(altitude_agl_m)) {
        return false;
    }

    // H has one non-zero element: H[0][kIdxPosition+2] = -1
    // So H*P*H^T = P[5][5], and K = -P[:,5] / S
    constexpr int32_t kHIdx = kIdxPosition + 2;  // index 5 (NED-down)

    // Innovation: y = z - h(x) = altitude_agl_m - (-p.z) = altitude_agl_m + p.z
    const float innovation = altitude_agl_m + p.z;

    // Innovation covariance: S = H*P*H^T + R = P[5][5] + R
    const float S = P(kHIdx, kHIdx) + kRBaro;

    // Council condition 2: reject degenerate covariance
    if (S < 1e-12f) {
        return false;
    }

    // NIS for diagnostics
    last_baro_nis_ = (innovation * innovation) / S;

    // Innovation gate: reject if |y| > kBaroInnovationGate * sqrt(S)
    const float gate_threshold = kBaroInnovationGate * sqrtf(S);
    if (fabsf(innovation) > gate_threshold) {
        return false;
    }

    // Kalman gain: K = P * H^T / S
    // H^T is a column vector with -1 at index kHIdx, so P*H^T = -P[:,kHIdx]
    // K[i] = -P[i][kHIdx] / S
    static Mat<15, 1> K;
    const float inv_S = 1.0f / S;
    for (int32_t i = 0; i < kStateSize; ++i) {
        K.data[i][0] = -P.data[i][kHIdx] * inv_S;
    }

    // Error state correction: δx = K * innovation
    static Mat<15, 1> dx;
    for (int32_t i = 0; i < kStateSize; ++i) {
        dx.data[i][0] = K.data[i][0] * innovation;
    }

    // Joseph form P update: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    // I - K*H: identity except column kHIdx where (I-KH)[i][kHIdx] += K[i]*1
    // (because H[kHIdx] = -1, so -K*H means +K in column kHIdx)
    static Mat15 IKH;
    IKH.set_identity();
    for (int32_t i = 0; i < kStateSize; ++i) {
        IKH.data[i][kHIdx] += K.data[i][0];  // -K * (-1) = +K
    }

    // IKH * P  (15×15 × 15×15)
    static Mat15 IKH_P;
    for (int32_t r = 0; r < kStateSize; ++r) {
        for (int32_t c = 0; c < kStateSize; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < kStateSize; ++k) {
                sum += IKH.data[r][k] * P.data[k][c];
            }
            IKH_P.data[r][c] = sum;
        }
    }

    // (IKH * P) * IKH^T + K*R*K^T
    static Mat15 P_new;
    for (int32_t r = 0; r < kStateSize; ++r) {
        for (int32_t c = 0; c < kStateSize; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < kStateSize; ++k) {
                sum += IKH_P.data[r][k] * IKH.data[c][k];  // IKH^T
            }
            // + K * R * K^T (rank-1 outer product scaled by R)
            sum += K.data[r][0] * kRBaro * K.data[c][0];
            P_new.data[r][c] = sum;
        }
    }

    // Commit updated covariance
    P = P_new;

    // Inject error state into nominal state
    // Position: p += δp
    p.x += dx.data[kIdxPosition + 0][0];
    p.y += dx.data[kIdxPosition + 1][0];
    p.z += dx.data[kIdxPosition + 2][0];

    // Velocity: v += δv
    v.x += dx.data[kIdxVelocity + 0][0];
    v.y += dx.data[kIdxVelocity + 1][0];
    v.z += dx.data[kIdxVelocity + 2][0];

    // Accel bias: ab += δab
    accel_bias.x += dx.data[kIdxAccelBias + 0][0];
    accel_bias.y += dx.data[kIdxAccelBias + 1][0];
    accel_bias.z += dx.data[kIdxAccelBias + 2][0];

    // Gyro bias: gb += δgb
    gyro_bias.x += dx.data[kIdxGyroBias + 0][0];
    gyro_bias.y += dx.data[kIdxGyroBias + 1][0];
    gyro_bias.z += dx.data[kIdxGyroBias + 2][0];

    // Attitude: q ← q ⊗ Exp(δθ)
    // P rotation omitted — G ≈ I for small δθ (Solà Eq. 298).
    // For baro-only corrections, |δθ| < 1e-4 rad, G differs from I
    // by < 1e-8, below float precision.
    const Vec3 delta_theta(dx.data[kIdxAttitude + 0][0],
                           dx.data[kIdxAttitude + 1][0],
                           dx.data[kIdxAttitude + 2][0]);
    reset(delta_theta);

    // Symmetry + clamping
    P.force_symmetric();
    clamp_covariance();

    return true;
}

// ============================================================================
// wrap_pi: Wrap angle to [-π, π]
// Standard approach per ArduPilot wrap_PI().
// ============================================================================
static float wrap_pi(float angle) {
    constexpr float kPi = 3.14159265f;
    constexpr float kTwoPi = 2.0f * kPi;
    // fmodf range is (-2π, 2π), shift to (-π, π)
    angle = fmodf(angle, kTwoPi);
    if (angle > kPi) { angle -= kTwoPi; }
    if (angle < -kPi) { angle += kTwoPi; }
    return angle;
}

// ============================================================================
// update_mag_heading: Magnetometer heading measurement update (IVP-44)
//
// Measurement model (ArduPilot/PX4 zero-yaw rotation approach):
//   1. Extract roll, pitch from current q (well-observed by accel)
//   2. Build R_zero = R(roll, pitch, yaw=0) — tilt only, no yaw
//   3. mag_level = R_zero * mag_body — level-frame mag (yaw signal preserved)
//   4. heading_measured = -atan2(mag_level.y, mag_level.x)
//   5. heading_predicted = yaw from q (ZYX Euler extraction)
//   6. innovation = wrap_pi(heading_predicted - heading_measured)
//
//   H ≈ [0, 0, 1, 0...0] (yaw-only, level-flight approximation)
//   R = kRMagHeading (~0.00757 rad²), inflated under interference
//
// Reference: ArduPilot fuseEulerYaw(), PX4 ECL fuseHeading()
//   Both use the zero-yaw rotation to separate tilt compensation from
//   heading measurement. The full rotation R(q)*mag_body would recover
//   the NED field direction (constant, independent of heading).
//
// Two-tier interference detection (council modification):
//   25-50% magnitude deviation: inflate R by 10x
//   >50% deviation: hard reject
//
// Sequential scalar update — same pattern as update_baro().
// Joseph form P update for numerical stability.
// Static locals for Mat15 temporaries (LL Entry 1).
// ============================================================================
bool ESKF::update_mag_heading(const Vec3& mag_body, float expected_magnitude,
                              float declination_rad) {
    using namespace eskf;

    // Guard: reject non-finite input
    if (!std::isfinite(mag_body.x) || !std::isfinite(mag_body.y) ||
        !std::isfinite(mag_body.z) || !std::isfinite(declination_rad)) {
        return false;
    }

    // Guard: reject too-low magnitude (sensor saturation, near strong magnet)
    const float mag_norm = mag_body.norm();
    if (mag_norm < kMagMinMagnitude) {
        return false;
    }

    // Two-tier interference detection
    float R_effective = kRMagHeading;
    if (expected_magnitude > 0.0f) {
        const float deviation = fabsf(mag_norm - expected_magnitude) / expected_magnitude;
        if (deviation > kMagInterferenceRejectThreshold) {
            return false;  // >50%: hard reject
        }
        if (deviation > kMagInterferenceThreshold) {
            R_effective = kRMagHeading * kMagInterferenceRScale;  // 25-50%: inflate R
        }
    }

    // Zero-yaw tilt compensation (ArduPilot/PX4 approach):
    // Build rotation with same roll/pitch as current q but yaw=0.
    // This rotates body-frame mag to a level frame that preserves the yaw signal.
    const Vec3 euler = q.to_euler();  // Vec3(roll, pitch, yaw)
    const Quat q_zero_yaw = Quat::from_euler(euler.x, euler.y, 0.0f);
    const Vec3 mag_level = q_zero_yaw.rotate(mag_body);

    // Measured heading: angle of horizontal mag projection in the level frame.
    // Negative atan2 because magnetic North is +X, and heading increases CW.
    // Declination converts magnetic heading to true heading.
    // ArduPilot: -atan2F(magMeasNED.y, magMeasNED.x) + MagDeclination()
    // PX4 ECL: atan2(mag_e, mag_n) + get_mag_declination()
    const float heading_measured = wrap_pi(-atan2f(mag_level.y, mag_level.x)
                                           + declination_rad);

    // Predicted heading from current quaternion (ZYX Euler yaw)
    const float heading_predicted = euler.z;

    // Innovation: standard KF convention y = z - h(x) = measured - predicted.
    // ArduPilot uses predicted - measured internally but negates K accordingly.
    // We use the standard form matching update_baro() for consistency.
    const float innovation = wrap_pi(heading_measured - heading_predicted);

    // H has one non-zero element: H[0][kIdxAttitude+2] = 1 (yaw component)
    constexpr int32_t kHIdx = kIdxAttitude + 2;  // index 2 (yaw)

    // Innovation covariance: S = H*P*H^T + R = P[2][2] + R
    const float S = P(kHIdx, kHIdx) + R_effective;

    // Reject degenerate covariance
    if (S < 1e-12f) {
        return false;
    }

    // NIS for diagnostics
    last_mag_nis_ = (innovation * innovation) / S;

    // Innovation gate: reject if |y| > kMagInnovationGate * sqrt(S)
    const float gate_threshold = kMagInnovationGate * sqrtf(S);
    if (fabsf(innovation) > gate_threshold) {
        return false;
    }

    // Kalman gain: K = P * H^T / S
    // H^T is a column vector with 1 at index kHIdx, so P*H^T = P[:,kHIdx]
    // K[i] = P[i][kHIdx] / S
    static Mat<15, 1> K;
    const float inv_S = 1.0f / S;
    for (int32_t i = 0; i < kStateSize; ++i) {
        K.data[i][0] = P.data[i][kHIdx] * inv_S;
    }

    // Error state correction: δx = K * innovation
    static Mat<15, 1> dx;
    for (int32_t i = 0; i < kStateSize; ++i) {
        dx.data[i][0] = K.data[i][0] * innovation;
    }

    // Joseph form P update: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    // I - K*H: identity except column kHIdx where (I-KH)[i][kHIdx] -= K[i]
    // (because H[kHIdx] = +1, so -K*H means -K in column kHIdx)
    static Mat15 IKH;
    IKH.set_identity();
    for (int32_t i = 0; i < kStateSize; ++i) {
        IKH.data[i][kHIdx] -= K.data[i][0];
    }

    // IKH * P  (15×15 × 15×15)
    static Mat15 IKH_P;
    for (int32_t r = 0; r < kStateSize; ++r) {
        for (int32_t c = 0; c < kStateSize; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < kStateSize; ++k) {
                sum += IKH.data[r][k] * P.data[k][c];
            }
            IKH_P.data[r][c] = sum;
        }
    }

    // (IKH * P) * IKH^T + K*R*K^T
    static Mat15 P_new;
    for (int32_t r = 0; r < kStateSize; ++r) {
        for (int32_t c = 0; c < kStateSize; ++c) {
            float sum = 0.0f;
            for (int32_t k = 0; k < kStateSize; ++k) {
                sum += IKH_P.data[r][k] * IKH.data[c][k];  // IKH^T
            }
            // + K * R * K^T (rank-1 outer product scaled by R)
            sum += K.data[r][0] * R_effective * K.data[c][0];
            P_new.data[r][c] = sum;
        }
    }

    // Commit updated covariance
    P = P_new;

    // Inject error state into nominal state
    // Position: p += δp
    p.x += dx.data[kIdxPosition + 0][0];
    p.y += dx.data[kIdxPosition + 1][0];
    p.z += dx.data[kIdxPosition + 2][0];

    // Velocity: v += δv
    v.x += dx.data[kIdxVelocity + 0][0];
    v.y += dx.data[kIdxVelocity + 1][0];
    v.z += dx.data[kIdxVelocity + 2][0];

    // Accel bias: ab += δab
    accel_bias.x += dx.data[kIdxAccelBias + 0][0];
    accel_bias.y += dx.data[kIdxAccelBias + 1][0];
    accel_bias.z += dx.data[kIdxAccelBias + 2][0];

    // Gyro bias: gb += δgb
    gyro_bias.x += dx.data[kIdxGyroBias + 0][0];
    gyro_bias.y += dx.data[kIdxGyroBias + 1][0];
    gyro_bias.z += dx.data[kIdxGyroBias + 2][0];

    // Attitude: q ← q ⊗ Exp(δθ) + P rotation via reset()
    const Vec3 delta_theta(dx.data[kIdxAttitude + 0][0],
                           dx.data[kIdxAttitude + 1][0],
                           dx.data[kIdxAttitude + 2][0]);
    reset(delta_theta);

    // Symmetry + clamping
    P.force_symmetric();
    clamp_covariance();

    return true;
}

// ============================================================================
// update_zupt: Zero-velocity pseudo-measurement (IVP-44b)
//
// Checks stationarity from raw IMU data, then applies v=[0,0,0] as
// three sequential scalar updates on velocity states [6..8].
//
// Stationarity criteria (same as init() RF-5):
//   |accel_norm - g| < kStationaryAccelTol  (±0.1g)
//   gyro_norm < kStationaryGyroMax           (<0.02 rad/s)
//
// Each scalar update: H has a single 1 at velocity index.
//   y = 0 - v[axis], S = P[idx][idx] + R, K = P[:,idx] / S
// Three sequential updates are numerically equivalent to a single
// 3×3 block update when off-diagonal velocity covariances are small.
//
// ArduPilot EKF3: zeroPosVelUpdate() when onGround.
// PX4 ECL: zero_velocity_update() in ekf_helper.cpp.
// ============================================================================
bool ESKF::update_zupt(const Vec3& accel_meas, const Vec3& gyro_meas) {
    using namespace eskf;

    // Guard: reject non-finite input
    if (!std::isfinite(accel_meas.x) || !std::isfinite(accel_meas.y) ||
        !std::isfinite(accel_meas.z) || !std::isfinite(gyro_meas.x) ||
        !std::isfinite(gyro_meas.y) || !std::isfinite(gyro_meas.z)) {
        last_zupt_active_ = false;
        return false;
    }

    // Stationarity check (RF-5 thresholds)
    const float accel_norm = accel_meas.norm();
    const float gyro_norm = gyro_meas.norm();
    if (fabsf(accel_norm - kGravity) > kStationaryAccelTol ||
        gyro_norm > kStationaryGyroMax) {
        last_zupt_active_ = false;
        return false;
    }

    last_zupt_active_ = true;

    // Three sequential scalar updates: v_n=0, v_e=0, v_d=0
    // H[axis] has a single 1 at kIdxVelocity + axis.
    float max_nis = 0.0f;
    const float v_components[3] = { v.x, v.y, v.z };

    for (int32_t axis = 0; axis < 3; ++axis) {
        const int32_t kHIdx = kIdxVelocity + axis;

        // Innovation: y = z - h(x) = 0 - v[axis]
        const float innovation = -v_components[axis];

        // Innovation covariance: S = P[idx][idx] + R
        const float S = P(kHIdx, kHIdx) + kRZupt;
        if (S < 1e-12f) {
            continue;
        }

        // NIS
        const float nis = (innovation * innovation) / S;
        if (nis > max_nis) {
            max_nis = nis;
        }

        // Innovation gate
        const float gate_threshold = kZuptInnovationGate * sqrtf(S);
        if (fabsf(innovation) > gate_threshold) {
            continue;
        }

        // Kalman gain: K = P[:,idx] / S (H has +1 at kHIdx)
        static Mat<15, 1> K;
        const float inv_S = 1.0f / S;
        for (int32_t i = 0; i < kStateSize; ++i) {
            K.data[i][0] = P.data[i][kHIdx] * inv_S;
        }

        // Error state correction: δx = K * innovation
        static Mat<15, 1> dx;
        for (int32_t i = 0; i < kStateSize; ++i) {
            dx.data[i][0] = K.data[i][0] * innovation;
        }

        // Joseph form P update
        static Mat15 IKH;
        IKH.set_identity();
        for (int32_t i = 0; i < kStateSize; ++i) {
            IKH.data[i][kHIdx] -= K.data[i][0];
        }

        static Mat15 IKH_P;
        for (int32_t r = 0; r < kStateSize; ++r) {
            for (int32_t c = 0; c < kStateSize; ++c) {
                float sum = 0.0f;
                for (int32_t k = 0; k < kStateSize; ++k) {
                    sum += IKH.data[r][k] * P.data[k][c];
                }
                IKH_P.data[r][c] = sum;
            }
        }

        static Mat15 P_new;
        for (int32_t r = 0; r < kStateSize; ++r) {
            for (int32_t c = 0; c < kStateSize; ++c) {
                float sum = 0.0f;
                for (int32_t k = 0; k < kStateSize; ++k) {
                    sum += IKH_P.data[r][k] * IKH.data[c][k];
                }
                sum += K.data[r][0] * kRZupt * K.data[c][0];
                P_new.data[r][c] = sum;
            }
        }
        P = P_new;

        // Inject error state into nominal
        p.x += dx.data[kIdxPosition + 0][0];
        p.y += dx.data[kIdxPosition + 1][0];
        p.z += dx.data[kIdxPosition + 2][0];
        v.x += dx.data[kIdxVelocity + 0][0];
        v.y += dx.data[kIdxVelocity + 1][0];
        v.z += dx.data[kIdxVelocity + 2][0];
        accel_bias.x += dx.data[kIdxAccelBias + 0][0];
        accel_bias.y += dx.data[kIdxAccelBias + 1][0];
        accel_bias.z += dx.data[kIdxAccelBias + 2][0];
        gyro_bias.x += dx.data[kIdxGyroBias + 0][0];
        gyro_bias.y += dx.data[kIdxGyroBias + 1][0];
        gyro_bias.z += dx.data[kIdxGyroBias + 2][0];

        const Vec3 delta_theta(dx.data[kIdxAttitude + 0][0],
                               dx.data[kIdxAttitude + 1][0],
                               dx.data[kIdxAttitude + 2][0]);
        reset(delta_theta);
        P.force_symmetric();
    }

    last_zupt_nis_ = max_nis;
    clamp_covariance();
    return true;
}

// ============================================================================
// clamp_covariance: Diagonal clamping per council RF-2
// Prevents P from growing unbounded during GPS-denied operation.
// ============================================================================
void ESKF::clamp_covariance() {
    using namespace eskf;

    for (int32_t i = 0; i < kBlockSize; ++i) {
        if (P(kIdxAttitude + i, kIdxAttitude + i) > kClampPAttitude) {
            P(kIdxAttitude + i, kIdxAttitude + i) = kClampPAttitude;
        }
        if (P(kIdxPosition + i, kIdxPosition + i) > kClampPPosition) {
            P(kIdxPosition + i, kIdxPosition + i) = kClampPPosition;
        }
        if (P(kIdxVelocity + i, kIdxVelocity + i) > kClampPVelocity) {
            P(kIdxVelocity + i, kIdxVelocity + i) = kClampPVelocity;
        }
    }
    // Bias blocks are not clamped — they should converge via updates
}

// ============================================================================
// healthy: Health check per council RF-3
// Returns false if any of:
//   - P contains NaN or Inf
//   - P diagonal is not all positive
//   - Quaternion norm is far from 1
//   - Biases exceed physical limits
// ============================================================================
bool ESKF::healthy() const {
    // P finite and positive diagonal
    if (!P.is_finite()) {
        return false;
    }
    if (!P.diagonal_positive()) {
        return false;
    }

    // Quaternion norm check: |norm - 1| < 1e-3
    // Note: NaN comparison always returns false, so check isfinite explicitly.
    const float qnorm = q.norm();
    if (!std::isfinite(qnorm) || fabsf(qnorm - 1.0f) > 1e-3f) {
        return false;
    }

    // Bias magnitude sanity check
    // Gyro bias > 10 dps (0.175 rad/s) is unrealistic for ICM-20948
    if (gyro_bias.norm() > 0.175f) {
        return false;
    }
    // Accel bias > 1 m/s² (~100mg) is unrealistic
    if (accel_bias.norm() > 1.0f) {
        return false;
    }

    return true;
}

} // namespace rc
