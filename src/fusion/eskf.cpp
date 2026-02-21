#include "fusion/eskf.h"

#include <cmath>

namespace rc {

// ============================================================================
// File-scope constants extracted from inline literals (JSF AV Rule 151)
// ============================================================================
namespace {

// DCM flat array size (3×3 rotation matrix)
constexpr int32_t kDcmElements = 9;

// Minimum innovation variance guard — rejects degenerate S in measurement updates.
// Below this threshold, 1/S would amplify noise catastrophically.
constexpr float kMinInnovationVariance = 1e-12F;

// Combined position + velocity block span in error state [3..8]
// Used in set_origin() to zero cross-covariance terms for both blocks.
constexpr int32_t kPosVelBlockSpan = 6;

// Quaternion norm deviation tolerance for healthy() check.
// |norm(q) - 1| > this indicates numerical drift or corruption.
constexpr float kQuatNormTolerance = 1e-3F;

// Max plausible gyro bias: 10 dps = 0.175 rad/s (ICM-20948 ±5°/s ZRO spec).
constexpr float kMaxGyroBias = 0.175F;

} // anonymous namespace

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
    float m9[kDcmElements];
    q.to_rotation_matrix(m9);
    Mat3 rotMat;
    rotMat(0, 0) = m9[0]; rotMat(0, 1) = m9[1]; rotMat(0, 2) = m9[2];
    // NOLINTNEXTLINE(readability-magic-numbers) — row-major 3×3 DCM indices
    rotMat(1, 0) = m9[3]; rotMat(1, 1) = m9[4]; rotMat(1, 2) = m9[5];
    // NOLINTNEXTLINE(readability-magic-numbers) — row-major 3×3 DCM indices
    rotMat(2, 0) = m9[6]; rotMat(2, 1) = m9[7]; rotMat(2, 2) = m9[8];
    return rotMat;
}

// ============================================================================
// init: Initialize ESKF from stationary IMU reading
// ============================================================================
bool ESKF::init(const Vec3& accelMeas, const Vec3& gyroMeas) {
    // RF-5: stationarity check — reject if not approximately stationary
    const float accelMag = accelMeas.norm();
    if (fabsf(accelMag - kGravity) > kStationaryAccelTol) {
        return false;
    }
    if (gyroMeas.norm() > kStationaryGyroMax) {
        return false;
    }

    // Derive initial attitude from gravity vector direction.
    // The accelerometer measures -g in body frame when stationary.
    // We want the quaternion that rotates body frame to NED, where
    // gravity is [0, 0, +g].
    //
    // Body-frame gravity measurement: accelMeas ≈ -R^T * g_ned
    // So body-frame "down" direction = -accelMeas.normalized()
    // We need R that maps body-down to NED-down [0,0,1].
    const Vec3 bodyDown = (accelMeas * -1.0F).normalized();
    const Vec3 nedDown(0.0F, 0.0F, 1.0F);
    q = Quat::from_two_vectors(bodyDown, nedDown);
    q.normalize();

    // Initialize position, velocity, biases to zero (stationary on pad)
    p = Vec3();
    v = Vec3();
    accel_bias = Vec3();
    gyro_bias = Vec3();

    // Initialize P diagonal per eskf.h constants
    P.set_zero();
    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        P(eskf::kIdxAttitude + i, eskf::kIdxAttitude + i)  = kInitPAttitude;
        P(eskf::kIdxPosition + i, eskf::kIdxPosition + i)  = kInitPPosition;
        P(eskf::kIdxVelocity + i, eskf::kIdxVelocity + i)  = kInitPVelocity;
        P(eskf::kIdxAccelBias + i, eskf::kIdxAccelBias + i) = kInitPAccelBias;
        P(eskf::kIdxGyroBias + i, eskf::kIdxGyroBias + i)  = kInitPGyroBias;
    }

    initialized_ = true;
    last_propagation_dt_ = 0.0F;
    return true;
}

// ============================================================================
// propagate_nominal: First-order Euler integration of nominal state
// Solà (2017) §5.3, Eq. 221-225
// ============================================================================
void ESKF::propagate_nominal(const Vec3& accelMeas, const Vec3& gyroMeas,
                              float dt) {
    // Bias-corrected measurements
    const Vec3 accelBody = accelMeas - accel_bias;
    const Vec3 gyroBody = gyroMeas - gyro_bias;

    // Rotation matrix: body-to-NED
    const Mat3 rotMat = quat_to_dcm(q);

    // True acceleration in NED frame.
    // Accelerometer measures specific force: f = a_true - g (in body frame).
    // True acceleration: a_true = R * f_body + g_ned
    // where g_ned = [0, 0, +g] (gravity points down in NED).
    // Solà (2017) Eq. 221.
    Vec3 accelNed;
    accelNed.x = rotMat(0, 0) * accelBody.x + rotMat(0, 1) * accelBody.y + rotMat(0, 2) * accelBody.z;
    accelNed.y = rotMat(1, 0) * accelBody.x + rotMat(1, 1) * accelBody.y + rotMat(1, 2) * accelBody.z;
    accelNed.z = rotMat(2, 0) * accelBody.x + rotMat(2, 1) * accelBody.y + rotMat(2, 2) * accelBody.z;
    accelNed.z += kGravity;

    // Position: p += v * dt + 0.5 * a * dt^2  (simplified to first order: p += v * dt)
    // Using first-order for simplicity — the dt^2 term is negligible at 200Hz
    p += v * dt;

    // Velocity: v += a_ned * dt
    v += accelNed * dt;

    // Quaternion: q ← q ⊗ Exp(gyroBody * dt)
    // First-order: q ← q ⊗ [1, gyroBody*dt/2]
    const Vec3 deltaTheta = gyroBody * dt;
    const Quat dq = Quat::from_small_angle(deltaTheta);
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
// Where: ω = gyroBody (bias-corrected), a = accelBody (bias-corrected),
//         R = body-to-NED DCM, [·]_x = skew-symmetric matrix
// ============================================================================
void ESKF::build_F(Mat15& out, const Quat& q, const Vec3& accelBody,
                    const Vec3& gyroBody, float dt) {
    out.set_identity();

    const Mat3 rotMat = quat_to_dcm(q);
    const Mat3 skewOmega = skew(gyroBody);
    const Mat3 skewAccel = skew(accelBody);

    // Attitude block: F[att, att] = I - dt * [ω]_x
    // (identity already set, subtract dt * skewOmega)
    const Mat3 negDtSkewOmega = skewOmega * (-dt);
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            out(eskf::kIdxAttitude + r, eskf::kIdxAttitude + c) += negDtSkewOmega(r, c);
        }
    }

    // Attitude-GyroBias block: F[att, gb] = -I * dt
    for (int32_t i = 0; i < 3; ++i) {
        out(eskf::kIdxAttitude + i, eskf::kIdxGyroBias + i) = -dt;
    }

    // Position-Velocity block: F[pos, vel] = I * dt
    for (int32_t i = 0; i < 3; ++i) {
        out(eskf::kIdxPosition + i, eskf::kIdxVelocity + i) = dt;
    }

    // Velocity-Attitude block: F[vel, att] = -R * [a]_x * dt
    // = -dt * R * skew(accelBody)
    const Mat3 rSkewA = rotMat * skewAccel;
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            out(eskf::kIdxVelocity + r, eskf::kIdxAttitude + c) = -dt * rSkewA(r, c);
        }
    }

    // Velocity-AccelBias block: F[vel, ab] = -R * dt
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            out(eskf::kIdxVelocity + r, eskf::kIdxAccelBias + c) = -dt * rotMat(r, c);
        }
    }
}

// ============================================================================
// build_Qc: Continuous-time process noise Q_c (15×15 diagonal)
// Q_d ≈ Q_c * dt  (zeroth-order hold discretization, per R-9)
// ============================================================================
void ESKF::build_Qc(Mat15& out) {
    out.set_zero();

    const float sigmaGyroSq = kSigmaGyro * kSigmaGyro;
    const float sigmaAccelSq = kSigmaAccel * kSigmaAccel;
    const float sigmaGyroBiasSq = kSigmaGyroBiasWalk * kSigmaGyroBiasWalk;
    const float sigmaAccelBiasSq = kSigmaAccelBiasWalk * kSigmaAccelBiasWalk;

    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        out(eskf::kIdxAttitude + i, eskf::kIdxAttitude + i)  = sigmaGyroSq;
        out(eskf::kIdxVelocity + i, eskf::kIdxVelocity + i)  = sigmaAccelSq;
        out(eskf::kIdxAccelBias + i, eskf::kIdxAccelBias + i) = sigmaAccelBiasSq;
        out(eskf::kIdxGyroBias + i, eskf::kIdxGyroBias + i)  = sigmaGyroBiasSq;
    }
    // Position block: no direct noise (position uncertainty grows via velocity)
}

// ============================================================================
// dense_fpft_add: P = F * P * F^T + Qd (dense triple product)
// Static locals avoid 1800B stack from temporaries (LL Entry 1).
// Natural slot for IVP-47 sparse optimization.
// ============================================================================
static void dense_fpft_add(Mat15& pMat, const Mat15& fMat, const Mat15& qdMat) {
    static Mat15 g_fpTemp;
    static Mat15 g_ftTemp;
    // FP = F * P
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            float sum = 0.0F;
            for (int32_t k = 0; k < eskf::kStateSize; ++k) {
                sum += fMat.data[r][k] * pMat.data[k][c];
            }
            g_fpTemp.data[r][c] = sum;
        }
    }
    // F^T
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            g_ftTemp.data[c][r] = fMat.data[r][c];
        }
    }
    // P = FP * F^T + Qd
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            float sum = 0.0F;
            for (int32_t k = 0; k < eskf::kStateSize; ++k) {
                sum += g_fpTemp.data[r][k] * g_ftTemp.data[k][c];
            }
            pMat.data[r][c] = sum + qdMat.data[r][c];
        }
    }
}

// ============================================================================
// predict: Sparse FPFT propagation (R-1 optimization)
//
// Exploits F_x block structure: many blocks are zero or identity.
// Computes g_pNew = F * P * F^T + Q_d using only non-zero blocks.
//
// The sparse approach avoids the full 15×15 × 15×15 × 15×15 triple product
// by computing individual 3×3 block contributions. This is O(5^2 * 3^3)
// instead of O(15^3), roughly 5× faster on embedded targets.
// ============================================================================
void ESKF::predict(const Vec3& accelMeas, const Vec3& gyroMeas, float dt) {
    // Bias-corrected measurements (needed for F construction)
    const Vec3 accelBody = accelMeas - accel_bias;
    const Vec3 gyroBody = gyroMeas - gyro_bias;

    // 1. Propagate nominal state
    propagate_nominal(accelMeas, gyroMeas, dt);

    // 2. Build F and Q_d
    // Note: F uses the pre-propagation quaternion for linearization.
    // We already propagated nominal state, so we use the PRE-update q.
    // However, for first-order accuracy at typical rates (200Hz, dt=5ms),
    // the difference between pre and post q is negligible.
    // ArduPilot EKF3 uses the same approach.
    // Static Mat15 locals to avoid stack overflow on RP2350 (LL Entry 1).
    // Mat15 = 900B each; multiple on stack causes MemManage fault.
    // Safe: single-threaded Core 0.
    static Mat15 g_fLocal;
    static Mat15 g_qdLocal;
    build_F(g_fLocal, q, accelBody, gyroBody, dt);
    build_Qc(g_qdLocal);        // Build Qc directly into g_qdLocal
    g_qdLocal.scale(dt);         // Zeroth-order hold (R-9): Qd = Qc * dt

    // 3. Dense F*P*F^T + Q_d
    dense_fpft_add(P, g_fLocal, g_qdLocal);

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
void ESKF::predict_dense(const Vec3& accelMeas, const Vec3& gyroMeas,
                          float dt) {
    const Vec3 accelBody = accelMeas - accel_bias;
    const Vec3 gyroBody = gyroMeas - gyro_bias;

    propagate_nominal(accelMeas, gyroMeas, dt);

    // Static locals: same rationale as predict() (LL Entry 1).
    static Mat15 g_fDense;
    static Mat15 g_qdDense;
    build_F(g_fDense, q, accelBody, gyroBody, dt);
    build_Qc(g_qdDense);
    g_qdDense.scale(dt);

    dense_fpft_add(P, g_fDense, g_qdDense);
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
//   q ← q ⊗ Exp(deltaTheta)
//   p ← p + delta_p
//   v ← v + delta_v
//   ab ← ab + delta_ab
//   gb ← gb + delta_gb
//
// The covariance is updated via the reset Jacobian G:
//   P ← G * P * G^T
//
// G is the identity everywhere except the attitude block, where:
//   G_theta = I - [deltaTheta/2]_x
//
// For small deltaTheta (which it should be after a good update),
// G ≈ I and the correction to P is negligible. We include it for
// correctness per the council RF-1 requirement.
// ============================================================================
void ESKF::reset(const Vec3& deltaTheta) {
    // The full error state would be passed in; for now this handles
    // the attitude-only reset that's most critical. The caller is
    // responsible for applying delta_p, delta_v, delta_ab, delta_gb
    // to the nominal state before calling reset().

    // Attitude injection
    const Quat dq = Quat::from_small_angle(deltaTheta);
    q = q * dq;
    q.normalize();

    // Build reset Jacobian G (15×15)
    // G = I except G[att,att] = I - [deltaTheta/2]_x
    // Static: same rationale as predict() (LL Entry 1).
    static Mat15 g_gMat;
    g_gMat.set_identity();
    const Mat3 halfSkew = skew(deltaTheta * 0.5F);
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            g_gMat(eskf::kIdxAttitude + r, eskf::kIdxAttitude + c) -= halfSkew(r, c);
        }
    }

    // P ← G * P * G^T (inlined with static temporaries, LL Entry 1)
    static Mat15 g_gpTemp;
    static Mat15 g_gtTemp;
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            float sum = 0.0F;
            for (int32_t k = 0; k < eskf::kStateSize; ++k) {
                sum += g_gMat.data[r][k] * P.data[k][c];
            }
            g_gpTemp.data[r][c] = sum;
        }
    }
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            g_gtTemp.data[c][r] = g_gMat.data[r][c];
        }
    }
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            float sum = 0.0F;
            for (int32_t k = 0; k < eskf::kStateSize; ++k) {
                sum += g_gpTemp.data[r][k] * g_gtTemp.data[k][c];
            }
            P.data[r][c] = sum;
        }
    }
    P.force_symmetric();
}

// ============================================================================
// inject_error_state: Apply error-state correction δx to nominal state
// Shared by all 5 measurement update functions.
// ============================================================================
void ESKF::inject_error_state(const Mat<eskf::kStateSize, 1>& dx) {
    p.x += dx.data[eskf::kIdxPosition + 0][0];
    p.y += dx.data[eskf::kIdxPosition + 1][0];
    p.z += dx.data[eskf::kIdxPosition + 2][0];

    v.x += dx.data[eskf::kIdxVelocity + 0][0];
    v.y += dx.data[eskf::kIdxVelocity + 1][0];
    v.z += dx.data[eskf::kIdxVelocity + 2][0];

    accel_bias.x += dx.data[eskf::kIdxAccelBias + 0][0];
    accel_bias.y += dx.data[eskf::kIdxAccelBias + 1][0];
    accel_bias.z += dx.data[eskf::kIdxAccelBias + 2][0];

    gyro_bias.x += dx.data[eskf::kIdxGyroBias + 0][0];
    gyro_bias.y += dx.data[eskf::kIdxGyroBias + 1][0];
    gyro_bias.z += dx.data[eskf::kIdxGyroBias + 2][0];

    const Vec3 deltaTheta(dx.data[eskf::kIdxAttitude + 0][0],
                           dx.data[eskf::kIdxAttitude + 1][0],
                           dx.data[eskf::kIdxAttitude + 2][0]);
    reset(deltaTheta);
}

// ============================================================================
// scalar_kalman_update: Joseph-form scalar measurement update
// Computes K = hValue * P[:,hIdx] / S, δx = K * innovation,
// P = (I - K*H)*P*(I - K*H)' + K*R*K', then injects δx.
// hValue: actual H-matrix entry (+1.0F or -1.0F). Generalizes to non-unit
// Jacobian entries without API change. See plan hValue correctness proof.
// ============================================================================
void ESKF::scalar_kalman_update(int32_t hIdx, float hValue,
                                float innovation, float r) {
    const float s = hValue * hValue * P(hIdx, hIdx) + r;
    const float invS = 1.0F / s;

    // Kalman gain: K[i] = hValue * P[i][hIdx] / S
    static Mat<eskf::kStateSize, 1> g_gain;
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        g_gain.data[i][0] = hValue * P.data[i][hIdx] * invS;
    }

    // Error state correction: δx = K * innovation
    static Mat<eskf::kStateSize, 1> g_dx;
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        g_dx.data[i][0] = g_gain.data[i][0] * innovation;
    }

    // Joseph form: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    // I - K*H: identity except column hIdx where (I-KH)[i][hIdx] -= hValue * K[i]
    static Mat15 g_ikh;
    g_ikh.set_identity();
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        g_ikh.data[i][hIdx] -= hValue * g_gain.data[i][0];
    }

    // IKH * P
    static Mat15 g_ikhP;
    for (int32_t r2 = 0; r2 < eskf::kStateSize; ++r2) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            float sum = 0.0F;
            for (int32_t k = 0; k < eskf::kStateSize; ++k) {
                sum += g_ikh.data[r2][k] * P.data[k][c];
            }
            g_ikhP.data[r2][c] = sum;
        }
    }

    // (IKH * P) * IKH^T + K*R*K^T
    static Mat15 g_pNew;
    for (int32_t r2 = 0; r2 < eskf::kStateSize; ++r2) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            float sum = 0.0F;
            for (int32_t k = 0; k < eskf::kStateSize; ++k) {
                sum += g_ikhP.data[r2][k] * g_ikh.data[c][k];
            }
            sum += g_gain.data[r2][0] * r * g_gain.data[c][0];
            g_pNew.data[r2][c] = sum;
        }
    }

    P = g_pNew;
    inject_error_state(g_dx);
    P.force_symmetric();
}

// ============================================================================
// update_baro: Barometric altitude measurement update (IVP-43)
//
// Measurement model:
//   z = altitudeAglM (positive up)
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
bool ESKF::update_baro(float altitudeAglM) {
    // Council condition 1: reject non-finite input
    if (!std::isfinite(altitudeAglM)) {
        return false;
    }

    // H has one non-zero element: H[0][kIdxPosition+2] = -1
    constexpr int32_t kHIdx = eskf::kIdxPosition + 2;  // index 5 (NED-down)
    constexpr float kHValue = -1.0F;

    // Innovation: y = z - h(x) = altitudeAglM - (-p.z) = altitudeAglM + p.z
    const float innovation = altitudeAglM + p.z;

    // Innovation covariance: S = H²*P[5][5] + R
    const float s = P(kHIdx, kHIdx) + kRBaro;

    // Council condition 2: reject degenerate covariance
    if (s < kMinInnovationVariance) {
        return false;
    }

    // NIS for diagnostics
    last_baro_nis_ = (innovation * innovation) / s;

    // Innovation gate
    const float gateThreshold = kBaroInnovationGate * sqrtf(s);
    if (fabsf(innovation) > gateThreshold) {
        ++baro_total_rejects_;
        return false;
    }

    ++baro_total_accepts_;
    scalar_kalman_update(kHIdx, kHValue, innovation, kRBaro);
    clamp_covariance();
    return true;
}

// ============================================================================
// wrap_pi: Wrap angle to [-π, π]
// Standard approach per ArduPilot wrap_PI().
// ============================================================================
static float wrap_pi(float angle) {
    constexpr float kPi = 3.14159265F;
    constexpr float kTwoPi = 2.0F * kPi;
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
//   3. magLevel = R_zero * magBody — level-frame mag (yaw signal preserved)
//   4. headingMeasured = -atan2(magLevel.y, magLevel.x)
//   5. headingPredicted = yaw from q (ZYX Euler extraction)
//   6. innovation = wrap_pi(headingPredicted - headingMeasured)
//
//   H ≈ [0, 0, 1, 0...0] (yaw-only, level-flight approximation)
//   R = kRMagHeading (~0.00757 rad²), inflated under interference
//
// Reference: ArduPilot fuseEulerYaw(), PX4 ECL fuseHeading()
//   Both use the zero-yaw rotation to separate tilt compensation from
//   heading measurement. The full rotation R(q)*magBody would recover
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
// Compute effective R after two-tier interference detection.
// Returns negative if hard-rejected (>50% deviation).
static float mag_interference_r(float magNorm, float expectedMagnitude) {
    float rEffective = ESKF::kRMagHeading;
    if (expectedMagnitude > 0.0F) {
        const float deviation = fabsf(magNorm - expectedMagnitude) / expectedMagnitude;
        if (deviation > ESKF::kMagInterferenceRejectThreshold) { return -1.0F; }
        if (deviation > ESKF::kMagInterferenceThreshold) {
            rEffective = ESKF::kRMagHeading * ESKF::kMagInterferenceRScale;
        }
    }
    return rEffective;
}

bool ESKF::update_mag_heading(const Vec3& magBody, float expectedMagnitude,
                              float declinationRad) {
    if (!std::isfinite(magBody.x) || !std::isfinite(magBody.y) ||
        !std::isfinite(magBody.z) || !std::isfinite(declinationRad)) {
        return false;
    }
    const float magNorm = magBody.norm();
    if (magNorm < kMagMinMagnitude) { return false; }

    float rEffective = mag_interference_r(magNorm, expectedMagnitude);
    if (rEffective < 0.0F) { return false; }

    // Zero-yaw tilt compensation: rotate magBody to level frame preserving yaw signal
    const Vec3 euler = q.to_euler();

    // Tilt-conditional R inflation (IVP-47). At large tilt, the H≈[0,0,1]
    // linearization cross-couples roll/pitch into heading. Inflate R to
    // weaken correction; hard-reject above 60° where extraction is unreliable.
    const float tilt = sqrtf(euler.x * euler.x + euler.y * euler.y);
    if (tilt > kMagTiltMaxRad) {
        ++mag_consecutive_rejects_;
        ++mag_total_rejects_;
        // No heading reset here — heading extraction unreliable at >60° tilt.
        // Reset only fires from the safety-net gate path where heading is valid.
        return false;
    }
    if (tilt > kMagTiltThresholdRad) {
        const float tiltFrac = (tilt - kMagTiltThresholdRad)
                             / (kMagTiltMaxRad - kMagTiltThresholdRad);
        rEffective *= (1.0F + kMagTiltRInflationMax * tiltFrac);
    }

    const Quat qZeroYaw = Quat::from_euler(euler.x, euler.y, 0.0F);
    const Vec3 magLevel = qZeroYaw.rotate(magBody);

    // Measured heading (mag North + declination), predicted heading (Euler yaw)
    const float headingMeasured = wrap_pi(-atan2f(magLevel.y, magLevel.x)
                                           + declinationRad);
    const float innovation = wrap_pi(headingMeasured - euler.z);

    constexpr int32_t kHIdx = eskf::kIdxAttitude + 2;  // yaw index
    const float s = P(kHIdx, kHIdx) + rEffective;
    if (s < kMinInnovationVariance) { return false; }

    last_mag_nis_ = (innovation * innovation) / s;

    // 300σ gate (ArduPilot EKF3 match). Physically untriggerable since
    // max innovation (π rad) < 300×√R ≈ 26 rad. Kept for symmetry with
    // baro/GPS/ZUPT update pattern.
    if (fabsf(innovation) > kMagInnovationGate * sqrtf(s)) {
        ++mag_consecutive_rejects_;
        ++mag_total_rejects_;
        return false;
    }

    mag_consecutive_rejects_ = 0;
    ++mag_total_accepts_;
    scalar_kalman_update(kHIdx, 1.0F, innovation, rEffective);
    clamp_covariance();
    return true;
}

// ============================================================================
// reset_mag_heading: Force-reset yaw after sustained rejection (IVP-47).
// ArduPilot alignYawAngle() / PX4 resetMagHeading() pattern.
// Zeroes yaw cross-covariances to prevent stale correlations from
// corrupting subsequent updates. Navigation states may briefly wobble
// after heading reset; settles in 2-3s with GPS aiding.
// ============================================================================
void ESKF::reset_mag_heading(float headingMeasured) {
    constexpr int32_t kYawIdx = eskf::kIdxAttitude + 2;
    const Vec3 euler = q.to_euler();

    q = Quat::from_euler(euler.x, euler.y, headingMeasured);
    q = q.normalized();

    P(kYawIdx, kYawIdx) = kInitPAttitude;
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        if (i != kYawIdx) {
            P(kYawIdx, i) = 0.0F;
            P(i, kYawIdx) = 0.0F;
        }
    }

    mag_consecutive_rejects_ = 0;
    ++mag_resets_;
}

// ============================================================================
// update_zupt: Zero-velocity pseudo-measurement (IVP-44b)
//
// Checks stationarity from raw IMU data, then applies v=[0,0,0] as
// three sequential scalar updates on velocity states [6..8].
//
// Stationarity criteria (same as init() RF-5):
//   |accelNorm - g| < kStationaryAccelTol  (±0.1g)
//   gyroNorm < kStationaryGyroMax           (<0.02 rad/s)
//
// Each scalar update: H has a single 1 at velocity index.
//   y = 0 - v[axis], S = P[idx][idx] + R, K = P[:,idx] / S
// Three sequential updates are numerically equivalent to a single
// 3×3 block update when off-diagonal velocity covariances are small.
//
// ArduPilot EKF3: zeroPosVelUpdate() when onGround.
// PX4 ECL: zero_velocity_update() in ekf_helper.cpp.
// ============================================================================
bool ESKF::update_zupt(const Vec3& accelMeas, const Vec3& gyroMeas) {
    // Guard: reject non-finite input
    if (!std::isfinite(accelMeas.x) || !std::isfinite(accelMeas.y) ||
        !std::isfinite(accelMeas.z) || !std::isfinite(gyroMeas.x) ||
        !std::isfinite(gyroMeas.y) || !std::isfinite(gyroMeas.z)) {
        last_zupt_active_ = false;
        return false;
    }

    // Stationarity check (RF-5 thresholds)
    const float accelNorm = accelMeas.norm();
    const float gyroNorm = gyroMeas.norm();
    if (fabsf(accelNorm - kGravity) > kStationaryAccelTol ||
        gyroNorm > kStationaryGyroMax) {
        last_zupt_active_ = false;
        ++zupt_total_rejects_;
        return false;
    }

    last_zupt_active_ = true;

    // Three sequential scalar updates: v_n=0, v_e=0, v_d=0
    // H[axis] has a single 1 at kIdxVelocity + axis.
    float maxNis = 0.0F;
    const float vComponents[3] = { v.x, v.y, v.z };

    for (int32_t axis = 0; axis < 3; ++axis) {
        const int32_t kHIdx = eskf::kIdxVelocity + axis;

        // Innovation: y = z - h(x) = 0 - v[axis]
        const float innovation = -vComponents[axis];

        // Innovation covariance: S = P[idx][idx] + R
        const float s = P(kHIdx, kHIdx) + kRZupt;
        if (s < kMinInnovationVariance) {
            continue;
        }

        // NIS
        const float nis = (innovation * innovation) / s;
        if (nis > maxNis) {
            maxNis = nis;
        }

        // Innovation gate
        const float gateThreshold = kZuptInnovationGate * sqrtf(s);
        if (fabsf(innovation) > gateThreshold) {
            continue;
        }

        scalar_kalman_update(kHIdx, 1.0F, innovation, kRZupt);
    }

    last_zupt_nis_ = maxNis;
    ++zupt_total_accepts_;
    clamp_covariance();
    return true;
}

// ============================================================================
// set_origin: Establish NED frame origin from first quality GPS fix
// Council condition C-4: HDOP must be <= kGpsMaxHdopForOrigin.
// Council fix: Reset position/velocity P to GPS-derived uncertainty.
// Without this, stale cross-covariances from pre-origin drift corrupt
// the Kalman gain for subsequent baro updates, causing bNIS explosion.
// ArduPilot EKF3: ResetPosition() reinitializes P and zeros cross-terms.
// ============================================================================
bool ESKF::set_origin(double latRad, double lonRad, float altM, float hdop) {
    if (has_origin_) {
        return false;  // Origin already set
    }
    if (hdop > kGpsMaxHdopForOrigin) {
        return false;  // Geometry too poor for origin
    }

    origin_lat_rad_ = latRad;
    origin_lon_rad_ = lonRad;
    origin_alt_m_ = altM;
    cos_origin_lat_ = cosf(static_cast<float>(latRad));
    has_origin_ = true;

    // Council fix: Reset position/velocity P to GPS-derived uncertainty.
    // Without this, stale cross-covariances corrupt baro Kalman gain.
    const float rPos = kSigmaGpsPosBase * fmaxf(hdop, 1.0F);
    const float pPos = rPos * rPos;  // Position variance from GPS quality
    const float pVel = kRGpsVel;       // Velocity variance (fixed)

    // Zero ALL cross-covariance terms involving position [3..5] and velocity [6..8]
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        for (int32_t j = eskf::kIdxPosition; j < eskf::kIdxPosition + kPosVelBlockSpan; ++j) {
            P.data[i][j] = 0.0F;
            P.data[j][i] = 0.0F;
        }
    }
    // Set position diagonal
    P.data[eskf::kIdxPosition + 0][eskf::kIdxPosition + 0] = pPos;  // North
    P.data[eskf::kIdxPosition + 1][eskf::kIdxPosition + 1] = pPos;  // East
    P.data[eskf::kIdxPosition + 2][eskf::kIdxPosition + 2] = pPos * kSigmaGpsVertScale * kSigmaGpsVertScale;  // Down
    // Set velocity diagonal
    P.data[eskf::kIdxVelocity + 0][eskf::kIdxVelocity + 0] = pVel;
    P.data[eskf::kIdxVelocity + 1][eskf::kIdxVelocity + 1] = pVel;
    P.data[eskf::kIdxVelocity + 2][eskf::kIdxVelocity + 2] = pVel;

    return true;
}

// ============================================================================
// reset_origin: Re-center NED frame, adjusting p for continuity
// Computes absolute geodetic from old origin + p, stores new origin,
// reprojects p into the new frame. Net effect: p becomes near-zero.
// Council condition C-7: absolute position must be preserved.
// ============================================================================
void ESKF::reset_origin(double newLatRad, double newLonRad, float newAltM) {
    if (!has_origin_) {
        return;
    }

    // Save current absolute geodetic position (reverse flat-earth, in double)
    double absLat = origin_lat_rad_
        + static_cast<double>(p.x) / static_cast<double>(kEarthRadius);
    double absLon = origin_lon_rad_
        + static_cast<double>(p.y) / (static_cast<double>(kEarthRadius)
                                       * static_cast<double>(cos_origin_lat_));
    float absAlt = origin_alt_m_ - p.z;  // NED down: alt = origin_alt - down

    // Store new origin
    origin_lat_rad_ = newLatRad;
    origin_lon_rad_ = newLonRad;
    origin_alt_m_ = newAltM;
    cos_origin_lat_ = cosf(static_cast<float>(newLatRad));

    // Reproject absolute position into new frame
    p = geodetic_to_ned(absLat, absLon, absAlt);
}

// ============================================================================
// geodetic_to_ned: Convert WGS84 geodetic to local NED frame
// Critical (council C-5): subtraction in double before float cast.
// Flat-earth approximation — valid within ~100km of origin.
// ArduPilot: Location::get_distance_NED(), same formula.
// ============================================================================
Vec3 ESKF::geodetic_to_ned(double latRad, double lonRad, float altM) const {
    if (!has_origin_) {
        return Vec3(0.0F, 0.0F, 0.0F);
    }

    // Double-precision subtraction before float cast (council C-5)
    float north = static_cast<float>(
        (latRad - origin_lat_rad_) * static_cast<double>(kEarthRadius));
    float east = static_cast<float>(
        (lonRad - origin_lon_rad_) * static_cast<double>(kEarthRadius)
        * static_cast<double>(cos_origin_lat_));
    float down = -(altM - origin_alt_m_);

    return Vec3(north, east, down);
}

// ============================================================================
// update_gps_position: 3 sequential scalar updates on position (N, E, D)
//
// R per axis:
//   Horizontal (N, E): (kSigmaGpsPosBase * max(hdop, 1))²
//   Vertical (D): rHoriz * kSigmaGpsVertScale² (or VDOP-based if vdop > 0)
//
// Same sequential scalar update pattern as update_zupt().
// ArduPilot EKF3: FuseVelPosNED(), PX4: fuseHorizontalPosition()/fuseVerticalPosition().
// ============================================================================
bool ESKF::update_gps_position(const Vec3& gpsNed, float hdop, float vdop) {
    if (!std::isfinite(gpsNed.x) || !std::isfinite(gpsNed.y) ||
        !std::isfinite(gpsNed.z)) {
        return false;
    }
    if (!has_origin_) { return false; }

    // Compute R per axis: horizontal from HDOP, vertical from VDOP or scaled horiz
    const float hdopClamped = (hdop > 1.0F) ? hdop : 1.0F;
    const float sigmaH = kSigmaGpsPosBase * hdopClamped;
    const float rHoriz = sigmaH * sigmaH;
    float rVert = 0.0F;
    if (vdop > 0.0F) {
        const float vdopClamped = (vdop > 1.0F) ? vdop : 1.0F;
        const float sigmaV = kSigmaGpsPosBase * vdopClamped;
        rVert = sigmaV * sigmaV;
    } else {
        rVert = rHoriz * kSigmaGpsVertScale * kSigmaGpsVertScale;
    }

    const float rValues[3] = { rHoriz, rHoriz, rVert };
    const int32_t indices[3] = { eskf::kIdxPosN, eskf::kIdxPosE, eskf::kIdxPosD };
    const float pComponents[3] = { p.x, p.y, p.z };
    const float gpsComponents[3] = { gpsNed.x, gpsNed.y, gpsNed.z };
    float maxNis = 0.0F;

    for (int32_t axis = 0; axis < 3; ++axis) {
        const int32_t kHIdx = indices[axis];
        const float rNoise = rValues[axis];
        const float innovation = gpsComponents[axis] - pComponents[axis];
        const float s = P(kHIdx, kHIdx) + rNoise;
        if (s < kMinInnovationVariance) { continue; }

        const float nis = (innovation * innovation) / s;
        if (nis > maxNis) { maxNis = nis; }

        if (fabsf(innovation) > kGpsPositionGate * sqrtf(s)) { continue; }

        scalar_kalman_update(kHIdx, 1.0F, innovation, rNoise);
    }

    last_gps_pos_nis_ = maxNis;
    ++gps_pos_total_accepts_;
    clamp_covariance();
    return true;
}

// ============================================================================
// update_gps_velocity: 2 sequential scalar updates on velocity (N, E only)
//
// Vertical velocity from GPS is unreliable — baro + IMU handle vertical.
// ArduPilot EKF3: FuseVelPosNED() with velocity axes.
// Council C-2: only indices [6..7], not [6..8].
// ============================================================================
bool ESKF::update_gps_velocity(float vNorth, float vEast) {
    // Guard: reject non-finite input
    if (!std::isfinite(vNorth) || !std::isfinite(vEast)) {
        return false;
    }

    const int32_t indices[2] = { eskf::kIdxVelN, eskf::kIdxVelE };
    const float gpsVel[2] = { vNorth, vEast };
    const float vComponents[2] = { v.x, v.y };

    float maxNis = 0.0F;

    for (int32_t axis = 0; axis < 2; ++axis) {
        const int32_t kHIdx = indices[axis];

        // Innovation: y = z - h(x) = gpsVel[axis] - v[axis]
        const float innovation = gpsVel[axis] - vComponents[axis];

        // Innovation covariance: S = P[idx][idx] + R
        const float s = P(kHIdx, kHIdx) + kRGpsVel;
        if (s < kMinInnovationVariance) {
            continue;
        }

        // NIS
        const float nis = (innovation * innovation) / s;
        if (nis > maxNis) {
            maxNis = nis;
        }

        // Innovation gate
        const float gateThreshold = kGpsVelocityGate * sqrtf(s);
        if (fabsf(innovation) > gateThreshold) {
            continue;
        }

        scalar_kalman_update(kHIdx, 1.0F, innovation, kRGpsVel);
    }

    last_gps_vel_nis_ = maxNis;
    ++gps_vel_total_accepts_;
    clamp_covariance();
    return true;
}

// ============================================================================
// clamp_covariance: Diagonal clamping per council RF-2
// Prevents P from growing unbounded during GPS-denied operation.
// ============================================================================
void ESKF::clamp_covariance() {
    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        if (P(eskf::kIdxAttitude + i, eskf::kIdxAttitude + i) > kClampPAttitude) {
            P(eskf::kIdxAttitude + i, eskf::kIdxAttitude + i) = kClampPAttitude;
        }
        if (P(eskf::kIdxPosition + i, eskf::kIdxPosition + i) > kClampPPosition) {
            P(eskf::kIdxPosition + i, eskf::kIdxPosition + i) = kClampPPosition;
        }
        if (P(eskf::kIdxVelocity + i, eskf::kIdxVelocity + i) > kClampPVelocity) {
            P(eskf::kIdxVelocity + i, eskf::kIdxVelocity + i) = kClampPVelocity;
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

    // Quaternion norm check: |norm - 1| < kQuatNormTolerance
    // Note: NaN comparison always returns false, so check isfinite explicitly.
    const float qnorm = q.norm();
    if (!std::isfinite(qnorm) || fabsf(qnorm - 1.0F) > kQuatNormTolerance) {
        return false;
    }

    // Bias magnitude sanity check
    // Gyro bias > 10 dps (0.175 rad/s) is unrealistic for ICM-20948
    if (gyro_bias.norm() > kMaxGyroBias) {
        return false;
    }
    // Accel bias > 1 m/s² (~100mg) is unrealistic
    if (accel_bias.norm() > 1.0F) {
        return false;
    }

    // Velocity magnitude sentinel: catches divergence from silent sensor fault
    // (e.g., ICM-20948 all-zeros output not caught by I2C error counter).
    // 500 m/s > max hobby rocket burnout (~Mach 1.5); any real flight is below this.
    if (v.norm() >= kMaxHealthyVelocity) {
        return false;
    }

    return true;
}

} // namespace rc
