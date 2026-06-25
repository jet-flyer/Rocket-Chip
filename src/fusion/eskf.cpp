// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "fusion/eskf.h"
#include "fusion/eskf_codegen.h"

#include <cassert>
#include <cmath>

// R3: Compile-time sync — fail build if codegen constants drift from eskf.h.
// These compare two named constexpr constants for exact identity (the whole
// point of the drift guard). -Wfloat-equal targets the runtime-rounding hazard,
// which cannot arise comparing two compile-time literals — so it's suppressed
// for this block only (re-enabled by the pop). NOT a runtime float compare.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
static_assert(codegen::kSigmaGyro == rc::ESKF::kSigmaGyro,
              "codegen sigma mismatch -- re-run scripts/generate_fpft.py");
static_assert(codegen::kSigmaAccel == rc::ESKF::kSigmaAccel,
              "codegen sigma mismatch -- re-run scripts/generate_fpft.py");
static_assert(codegen::kSigmaGyroBiasWalk == rc::ESKF::kSigmaGyroBiasWalk,
              "codegen sigma mismatch -- re-run scripts/generate_fpft.py");
static_assert(codegen::kSigmaAccelBiasWalk == rc::ESKF::kSigmaAccelBiasWalk,
              "codegen sigma mismatch -- re-run scripts/generate_fpft.py");
static_assert(codegen::kSigmaEarthMagWalk == rc::ESKF::kSigmaEarthMagWalk,
              "codegen sigma mismatch -- re-run scripts/generate_fpft.py");
static_assert(codegen::kSigmaBodyMagBiasWalk == rc::ESKF::kSigmaBodyMagBiasWalk,
              "codegen sigma mismatch -- re-run scripts/generate_fpft.py");
static_assert(codegen::kSigmaWindWalk == rc::ESKF::kSigmaWindWalk,
              "codegen sigma mismatch -- re-run scripts/generate_fpft.py");
static_assert(codegen::kSigmaBaroBiasWalk == rc::ESKF::kSigmaBaroBiasWalk,
              "codegen sigma mismatch -- re-run scripts/generate_fpft.py");
#pragma GCC diagnostic pop

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

// Mag state block span: earth_mag(3) + body_mag_bias(3) = 6
constexpr int32_t kMagBlockSpan = 6;

// Minimum baseline P diagonal for growth ratio check.
// Below this, baseline is too small for meaningful ratio comparison.
constexpr float kPGrowthBaselineEpsilon = 1e-6F;

// Quaternion norm deviation tolerance for healthy() check.
// |norm(q) - 1| > this indicates numerical drift or corruption.
constexpr float kQuatNormTolerance = 1e-3F;

// Max plausible gyro bias: 10 dps = 0.175 rad/s (ICM-20948 ±5°/s ZRO spec).
constexpr float kMaxGyroBias = 0.175F;

} // anonymous namespace

// ============================================================================
// Bierman measurement update — UD-factored covariance (ESKF_USE_BIERMAN)
// ============================================================================
#ifdef ESKF_USE_BIERMAN

// P representation state machine.
// DENSE: P is a valid dense 24×24 covariance (codegen FPFT output).
// UD:    P is stale; g_bierman_ud holds the valid UD factorization.
enum class PRepr { DENSE, UD };

static PRepr g_pRepr = PRepr::DENSE;

// File-scope UD24 — 2,400 bytes BSS (LL Entry 1: too large for struct member).
static UD24 g_biermanUd;

#endif // ESKF_USE_BIERMAN

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
    Mat3 rot_mat;
    rot_mat(0, 0) = m9[0]; rot_mat(0, 1) = m9[1]; rot_mat(0, 2) = m9[2];
    // NOLINTNEXTLINE(readability-magic-numbers) — row-major 3×3 DCM indices
    rot_mat(1, 0) = m9[3]; rot_mat(1, 1) = m9[4]; rot_mat(1, 2) = m9[5];
    // NOLINTNEXTLINE(readability-magic-numbers) — row-major 3×3 DCM indices
    rot_mat(2, 0) = m9[6]; rot_mat(2, 1) = m9[7]; rot_mat(2, 2) = m9[8];
    return rot_mat;
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
    // Body-frame gravity measurement: accelMeas ≈ -R^T * g_ned
    // So body-frame "down" direction = -accelMeas.normalized()
    // We need R that maps body-down to NED-down [0,0,1].
    const Vec3 body_down = (accel_meas * -1.0F).normalized();
    const Vec3 ned_down(0.0F, 0.0F, 1.0F);
    q = Quat::from_two_vectors(body_down, ned_down);
    q.normalize();

    // Initialize position, velocity, biases to zero (stationary on pad)
    p = Vec3();
    v = Vec3();
    accel_bias = Vec3();
    gyro_bias = Vec3();

    // Zero extended states
    earth_mag = Vec3();
    body_mag_bias = Vec3();
    wind_n_ = 0.0F;
    wind_e_ = 0.0F;
    baro_bias_ = 0.0F;

    // All extended states start inhibited
    inhibit_mag_states_ = true;
    inhibit_wind_states_ = true;
    inhibit_baro_bias_ = true;

    // Initialize P diagonal per eskf.h constants
    // Extended states: P = 0 (inhibited by default)
    P.set_zero();
    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        P(eskf::kIdxAttitude + i, eskf::kIdxAttitude + i)  = kInitPAttitude;
        P(eskf::kIdxPosition + i, eskf::kIdxPosition + i)  = kInitPPosition;
        P(eskf::kIdxVelocity + i, eskf::kIdxVelocity + i)  = kInitPVelocity;
        P(eskf::kIdxAccelBias + i, eskf::kIdxAccelBias + i) = kInitPAccelBias;
        P(eskf::kIdxGyroBias + i, eskf::kIdxGyroBias + i)  = kInitPGyroBias;
    }
    // P[15..23] stays zero — inhibited states have no covariance

#ifdef ESKF_USE_BIERMAN
    g_pRepr = PRepr::DENSE;  // Fresh P from init — always dense
#endif

    initialized_ = true;
    last_propagation_dt_ = 0.0F;
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
    const Mat3 rot_mat = quat_to_dcm(q);

    // True acceleration in NED frame.
    // Accelerometer measures specific force: f = a_true - g (in body frame).
    // True acceleration: a_true = R * f_body + g_ned
    // where g_ned = [0, 0, +g] (gravity points down in NED).
    // Solà (2017) Eq. 221.
    Vec3 accel_ned;
    accel_ned.x = rot_mat(0, 0) * accel_body.x + rot_mat(0, 1) * accel_body.y + rot_mat(0, 2) * accel_body.z;
    accel_ned.y = rot_mat(1, 0) * accel_body.x + rot_mat(1, 1) * accel_body.y + rot_mat(1, 2) * accel_body.z;
    accel_ned.z = rot_mat(2, 0) * accel_body.x + rot_mat(2, 1) * accel_body.y + rot_mat(2, 2) * accel_body.z;
    accel_ned.z += kGravity;

    // Position: p += v * dt + 0.5 * a * dt^2  (simplified to first order: p += v * dt)
    // Using first-order for simplicity — the dt^2 term is negligible at 200Hz
    p += v * dt;

    // Velocity: v += a_ned * dt
    v += accel_ned * dt;

    // Quaternion: q ← q ⊗ Exp(gyroBody * dt)
    // First-order: q ← q ⊗ [1, gyroBody*dt/2]
    const Vec3 delta_theta = gyro_body * dt;
    const Quat dq = Quat::from_small_angle(delta_theta);
    q = q * dq;
    q.normalize();
}

// ============================================================================
// build_F: Error-state transition matrix F_x (24×24)
// F_x = I + dt * F_delta
// Solà (2017) §5.3.3, Eq. 269
//
// F_delta block structure (named indices from eskf_state.h):
//   [att]  | -[ω]_x     0      0      0     -I    0   0   0   0  |
//   [pos]  |    0        0      I      0      0    0   0   0   0  |
//   [vel]  | -R[a]_x    0      0     -R      0    0   0   0   0  |
//   [ab]   |    0        0      0      0      0    0   0   0   0  |
//   [gb]   |    0        0      0      0      0    0   0   0   0  |
//   [emag] |    0        0      0      0      0    0   0   0   0  |
//   [bmag] |    0        0      0      0      0    0   0   0   0  |
//   [wind] |    0        0      0      0      0    0   0   0   0  |
//   [bbias]|    0        0      0      0      0    0   0   0   0  |
//
// States 15-23: identity F propagation (no coupling to core states).
// Where: ω = gyroBody (bias-corrected), a = accelBody (bias-corrected),
//         R = body-to-NED DCM, [·]_x = skew-symmetric matrix
// ============================================================================
void ESKF::build_F(Mat24& out, const Quat& q, const Vec3& accel_body,
                    const Vec3& gyro_body, float dt) {
    out.set_identity();

    const Mat3 rot_mat = quat_to_dcm(q);
    const Mat3 skew_omega = skew(gyro_body);
    const Mat3 skew_accel = skew(accel_body);

    // Attitude block: F[att, att] = I - dt * [ω]_x
    // (identity already set, subtract dt * skewOmega)
    const Mat3 neg_dt_skew_omega = skew_omega * (-dt);
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            out(eskf::kIdxAttitude + r, eskf::kIdxAttitude + c) += neg_dt_skew_omega(r, c);
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
    const Mat3 r_skew_a = rot_mat * skew_accel;
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            out(eskf::kIdxVelocity + r, eskf::kIdxAttitude + c) = -dt * r_skew_a(r, c);
        }
    }

    // Velocity-AccelBias block: F[vel, ab] = -R * dt
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            out(eskf::kIdxVelocity + r, eskf::kIdxAccelBias + c) = -dt * rot_mat(r, c);
        }
    }
}

// ============================================================================
// build_Qc: Continuous-time process noise Q_c (24×24 diagonal)
// Q_d ≈ Q_c * dt  (zeroth-order hold discretization, per R-9)
// Extended states 15-23 have random walk noise even when inhibited.
// clamp_covariance() zeroes inhibited blocks after propagation (R-8).
// ============================================================================
void ESKF::build_Qc(Mat24& out) {
    out.set_zero();

    const float sigma_gyro_sq = kSigmaGyro * kSigmaGyro;
    const float sigma_accel_sq = kSigmaAccel * kSigmaAccel;
    const float sigma_gyro_bias_sq = kSigmaGyroBiasWalk * kSigmaGyroBiasWalk;
    const float sigma_accel_bias_sq = kSigmaAccelBiasWalk * kSigmaAccelBiasWalk;

    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        out(eskf::kIdxAttitude + i, eskf::kIdxAttitude + i)  = sigma_gyro_sq;
        out(eskf::kIdxVelocity + i, eskf::kIdxVelocity + i)  = sigma_accel_sq;
        out(eskf::kIdxAccelBias + i, eskf::kIdxAccelBias + i) = sigma_accel_bias_sq;
        out(eskf::kIdxGyroBias + i, eskf::kIdxGyroBias + i)  = sigma_gyro_bias_sq;
    }
    // Position block: no direct noise (position uncertainty grows via velocity)

    // Extended states — random walk noise
    const float sigma_earth_mag_sq = kSigmaEarthMagWalk * kSigmaEarthMagWalk;
    const float sigma_body_mag_bias_sq = kSigmaBodyMagBiasWalk * kSigmaBodyMagBiasWalk;
    const float sigma_wind_sq = kSigmaWindWalk * kSigmaWindWalk;
    const float sigma_baro_bias_sq = kSigmaBaroBiasWalk * kSigmaBaroBiasWalk;

    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        out(eskf::kIdxEarthMag + i, eskf::kIdxEarthMag + i)       = sigma_earth_mag_sq;
        out(eskf::kIdxBodyMagBias + i, eskf::kIdxBodyMagBias + i) = sigma_body_mag_bias_sq;
    }
    out(eskf::kIdxWindNE + 0, eskf::kIdxWindNE + 0) = sigma_wind_sq;
    out(eskf::kIdxWindNE + 1, eskf::kIdxWindNE + 1) = sigma_wind_sq;
    out(eskf::kIdxBaroBias, eskf::kIdxBaroBias)      = sigma_baro_bias_sq;
}

// ============================================================================
// dense_fpft_add: P = F * P * F^T + Qd (dense triple product)
// Static locals avoid stack pressure from temporaries (LL Entry 1).
// Used only by predict_dense() as verification reference.
// ============================================================================
static void dense_fpft_add(Mat24& p_mat, const Mat24& f_mat, const Mat24& qd_mat) {
    static Mat24 g_fpTemp;
    static Mat24 g_ftTemp;
    // FP = F * P
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            float sum = 0.0F;
            for (int32_t k = 0; k < eskf::kStateSize; ++k) {
                sum += f_mat.data[r][k] * p_mat.data[k][c];
            }
            g_fpTemp.data[r][c] = sum;
        }
    }
    // F^T
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            g_ftTemp.data[c][r] = f_mat.data[r][c];
        }
    }
    // P = FP * F^T + Qd
    for (int32_t r = 0; r < eskf::kStateSize; ++r) {
        for (int32_t c = 0; c < eskf::kStateSize; ++c) {
            float sum = 0.0F;
            for (int32_t k = 0; k < eskf::kStateSize; ++k) {
                sum += g_fpTemp.data[r][k] * g_ftTemp.data[k][c];
            }
            p_mat.data[r][c] = sum + qd_mat.data[r][c];
        }
    }
}

// ============================================================================
// predict: Codegen FPFT covariance propagation
// P = F * P * F^T + Q_d (Sola 2017, Eq. 269)
//
// Uses SymPy-generated flat scalar expansion (scripts/generate_fpft.py).
// CSE eliminates redundant sub-expressions. Q_d baked into generated code.
// Dense predict_dense() retained as verification reference (Test 8).
//
// History: Block-sparse tried first, 31% SLOWER (712us vs 542us).
// Codegen is the correct path (PX4/ArduPilot pattern).
// ============================================================================
void ESKF::predict(const Vec3& accel_meas, const Vec3& gyro_meas, float dt) {
    const Vec3 accel_body = accel_meas - accel_bias;
    const Vec3 gyro_body = gyro_meas - gyro_bias;

    propagate_nominal(accel_meas, gyro_meas, dt);

#ifdef ESKF_USE_BIERMAN
    // Codegen FPFT requires dense P — reconstruct from UD if needed.
    ensure_dense();
#endif

    // Codegen FPFT: SymPy-generated flat scalar expansion with CSE
    // Replaces dense O(N^3) triple product. Q_d baked in.
    const Mat3 rot_mat = quat_to_dcm(q);
    codegen_fpft(P.data, rot_mat.data,
                 accel_body.x, accel_body.y, accel_body.z,
                 gyro_body.x, gyro_body.y, gyro_body.z, dt);
    P.force_symmetric();

    // Phase Q delta: add (phase_scale - 1.0) * baseline_sigma^2 * dt to P diagonal.
    // Applied after codegen (which bakes baseline Q_d) and before clamp.
    if (phase_qr_) {
        apply_phase_q_delta(dt);
    }

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
    static Mat24 g_fDense;
    static Mat24 g_qdDense;
    build_F(g_fDense, q, accel_body, gyro_body, dt);
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
// Sparse P ← G*P*G^T where G = I except 3×3 attitude block G_a.
// Only rows/cols 0-2 change. Cost: ~450 MACs at N=24 vs ~27,648 dense.
void ESKF::reset_covariance_attitude(const float ga[3][3]) {
    // Step 1: GP[0:3][:] = G_a * P[0:3][:] (rows 0-2 of G*P)
    static float g_gpRows[3][24];  // NOLINT(readability-magic-numbers)
    for (int32_t i = 0; i < 3; ++i) {
        for (int32_t j = 0; j < eskf::kStateSize; ++j) {
            g_gpRows[i][j] = ga[i][0] * P.data[0][j]
                         + ga[i][1] * P.data[1][j]
                         + ga[i][2] * P.data[2][j];
        }
    }

    // Save original P columns 0-2 for rows >=3 (step 3 reads P[i>=3][k<3]
    // after step 2 has overwritten symmetric entries).
    static float g_pCol02[24][3];  // NOLINT(readability-magic-numbers)
    for (int32_t i = 3; i < eskf::kStateSize; ++i) {
        g_pCol02[i][0] = P.data[i][0];
        g_pCol02[i][1] = P.data[i][1];
        g_pCol02[i][2] = P.data[i][2];
    }

    // Step 2: cols j >= 3 — P_new[i<3][j] = gpRows[i][j]
    for (int32_t i = 0; i < 3; ++i) {
        for (int32_t j = 3; j < eskf::kStateSize; ++j) {
            P.data[i][j] = g_gpRows[i][j];
            P.data[j][i] = g_gpRows[i][j];
        }
    }

    // 3×3 attitude block: P_new[i][j] = sum_k gpRows[i][k] * ga[j][k]
    for (int32_t i = 0; i < 3; ++i) {
        for (int32_t j = i; j < 3; ++j) {
            float sum = 0.0F;
            for (int32_t k = 0; k < 3; ++k) {
                sum += g_gpRows[i][k] * ga[j][k];
            }
            P.data[i][j] = sum;
            P.data[j][i] = sum;
        }
    }

    // Step 3: rows >= 3, cols 0-2 — use saved original columns
    for (int32_t i = 3; i < eskf::kStateSize; ++i) {
        for (int32_t j = 0; j < 3; ++j) {
            float sum = 0.0F;
            for (int32_t k = 0; k < 3; ++k) {
                sum += g_pCol02[i][k] * ga[j][k];
            }
            P.data[i][j] = sum;
            P.data[j][i] = sum;
        }
    }
}

void ESKF::reset(const Vec3& delta_theta) {
    const Quat dq = Quat::from_small_angle(delta_theta);
    q = q * dq;
    q.normalize();

    // G_a = I - [deltaTheta/2]_x (attitude block of reset Jacobian)
    float ga[3][3];
    const Mat3 half_skew = skew(delta_theta * 0.5F);
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            ga[r][c] = ((r == c) ? 1.0F : 0.0F) - half_skew(r, c);
        }
    }

    reset_covariance_attitude(ga);
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

    // Extended states
    earth_mag.x += dx.data[eskf::kIdxEarthMag + 0][0];
    earth_mag.y += dx.data[eskf::kIdxEarthMag + 1][0];
    earth_mag.z += dx.data[eskf::kIdxEarthMag + 2][0];

    body_mag_bias.x += dx.data[eskf::kIdxBodyMagBias + 0][0];
    body_mag_bias.y += dx.data[eskf::kIdxBodyMagBias + 1][0];
    body_mag_bias.z += dx.data[eskf::kIdxBodyMagBias + 2][0];

    wind_n_ += dx.data[eskf::kIdxWindNE + 0][0];
    wind_e_ += dx.data[eskf::kIdxWindNE + 1][0];

    baro_bias_ += dx.data[eskf::kIdxBaroBias][0];

    const Vec3 delta_theta(dx.data[eskf::kIdxAttitude + 0][0],
                           dx.data[eskf::kIdxAttitude + 1][0],
                           dx.data[eskf::kIdxAttitude + 2][0]);
    reset(delta_theta);
}

// ============================================================================
// scalar_kalman_update: Joseph-form scalar measurement update
// Computes K = hValue * P[:,hIdx] / S, δx = K * innovation,
// P = (I - K*H)*P*(I - K*H)' + K*R*K', then injects δx.
// hValue: actual H-matrix entry (+1.0F or -1.0F). Generalizes to non-unit
// Jacobian entries without API change. See plan hValue correctness proof.
//
// O(N²) rank-1 Joseph form: since H is scalar (single non-zero entry),
// the triple product expands to:
//   P_new[i][j] = P[i][j] - K[i]*hv*P[h][j] - P[i][h]*hv*K[j] + K[i]*S*K[j]
// where h=hIdx, hv=hValue, S=hv²*P[h][h]+R. Avoids O(N³) dense matrix multiply.
// Mathematically identical to the full Joseph form.
// ============================================================================
void ESKF::scalar_kalman_update(int32_t h_idx, float h_value,
                                float innovation, float r) {
    const float s = h_value * h_value * P(h_idx, h_idx) + r;
    const float inv_s = 1.0F / s;

    // Kalman gain: K[i] = hValue * P[i][hIdx] / S
    static Mat<eskf::kStateSize, 1> g_gain;
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        g_gain.data[i][0] = h_value * P.data[i][h_idx] * inv_s;
    }

    // Error state correction: δx = K * innovation
    static Mat<eskf::kStateSize, 1> g_dx;
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        g_dx.data[i][0] = g_gain.data[i][0] * innovation;
    }

    // O(N²) rank-1 Joseph form (upper triangle, then copy to lower).
    // P_new[i][j] = P[i][j] - K[i]*hv*P_h[j] - P_h[i]*hv*K[j] + K[i]*S*K[j]
    // where P_h = original P[hIdx][:] row, saved before in-place update.
    static float g_phRow[eskf::kStateSize];
    for (int32_t j = 0; j < eskf::kStateSize; ++j) {
        g_phRow[j] = P.data[h_idx][j];
    }

    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        const float ki = g_gain.data[i][0];
        const float ki_s = ki * s;
        const float hv_phi = h_value * g_phRow[i];
        for (int32_t j = i; j < eskf::kStateSize; ++j) {
            const float kj = g_gain.data[j][0];
            const float hv_phj = h_value * g_phRow[j];
            P.data[i][j] = P.data[i][j]
                         - ki * hv_phj
                         - hv_phi * kj
                         + ki_s * kj;
            P.data[j][i] = P.data[i][j];  // symmetric
        }
    }

    inject_error_state(g_dx);
}

// ============================================================================
// Bierman measurement update path (ESKF_USE_BIERMAN)
// ============================================================================
#ifdef ESKF_USE_BIERMAN

void ESKF::ensure_dense() {
    if (g_pRepr == PRepr::DENSE) {
        return;
    }
    // Reconstruct P = U * D * U^T from UD factorization
    ud_to_dense(g_biermanUd, P.data);
    P.force_symmetric();
    clamp_covariance();

    // Post-reconstruct assertion (Council Req. #3): P diagonals >= 0
    // in debug builds. Negative diagonal after reconstruct means UD corruption.
#ifndef NDEBUG
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        assert(P.data[i][i] >= 0.0F);  // NOLINT(cert-dcl03-c)
    }
#endif

    g_pRepr = PRepr::DENSE;
}

void ESKF::ensure_ud() {
    if (g_pRepr == PRepr::UD) {
        return;
    }
    // Inhibited states have P diagonal = 0, which makes P semi-positive-definite.
    // ud_factorize requires strictly positive-definite. Temporarily set zero
    // diagonals to a tiny epsilon, factorize, then zero the D entries.
    // Bierman won't touch these states — D=0 means zero contribution to
    // alpha and zero Kalman gain.
    static constexpr float kFactorizeEps = 1e-30F;
    bool patched[eskf::kStateSize] = {};
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        if (P.data[i][i] <= 0.0F) {
            P.data[i][i] = kFactorizeEps;
            patched[i] = true;
        }
    }

    const bool ok = ud_factorize(g_biermanUd, P.data);

    // Restore patched diagonals to zero in both P and UD
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        if (patched[i]) {
            P.data[i][i] = 0.0F;
            g_biermanUd.D[i] = 0.0F;
        }
    }

    // If factorize failed despite patching, P is corrupt — healthy() will catch it.
    (void)ok;
    g_pRepr = PRepr::UD;
}

void ESKF::bierman_kalman_update(int32_t h_idx, float h_value,
                                  float innovation, float r) {
    ensure_ud();

    static float g_biermanDx[eskf::kStateSize];
    bierman_scalar_update(g_biermanUd, h_idx, h_value, innovation, r,
                          g_biermanDx);

    // Convert dx array to Mat<N,1> for inject_error_state
    static Mat<eskf::kStateSize, 1> g_dxMat;
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        g_dxMat.data[i][0] = g_biermanDx[i];
    }
    inject_error_state(g_dxMat);
}

#endif // ESKF_USE_BIERMAN

// ============================================================================
// update_baro: Barometric altitude measurement update
//
// Measurement model:
//   z = altitudeAglM (positive up)
//   h(x) = -p.z + baro_bias (NED down negated + bias when enabled)
//   H = [0 0 0 | 0 0 -1 | 0...0 | 0...0 | 0 0 | +1]  (1×24, +1 at [23] when enabled)
//   R = kRBaro = 0.033² ≈ 0.001089 m²
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
    // Council condition 1: reject non-finite input
    if (!std::isfinite(altitude_agl_m)) {
        return false;
    }

    // H has one non-zero element: H[0][kIdxPosition+2] = -1
    // When baro_bias is enabled, H also has +1 at index 23. However,
    // scalar_kalman_update() only handles single-entry H. This works because:
    //   - When inhibited (default): P[23][*] = 0, K[23] = 0 regardless
    //   - When enabled: baro_bias corrects via cross-covariance P[5][23]
    // Full dual-entry H update deferred until baro_bias proves useful.
    constexpr int32_t kHIdx = eskf::kIdxPosition + 2;  // index 5 (NED-down)
    constexpr float kHValue = -1.0F;

    // Innovation: y = z - h(x)
    // h(x) = -p.z + baro_bias (when baro bias enabled)
    // When inhibited, P[23][*] = 0, so Kalman gain for baro_bias state is zero.
    const float predicted = -p.z + (inhibit_baro_bias_ ? 0.0F : baro_bias_);
    const float innovation = altitude_agl_m - predicted;

    // Phase-aware R: use phase R when configured, baseline otherwise
    const float r = phase_qr_ ? r_active_.r_baro : kRBaro;

    // Innovation covariance: S = H²*P[5][5] + R
    const float s = P(kHIdx, kHIdx) + r;

    // Council condition 2: reject degenerate covariance
    if (s < kMinInnovationVariance) {
        return false;
    }

    // NIS for diagnostics
    last_baro_nis_ = (innovation * innovation) / s;

    // Innovation gate
    const float gate_threshold = kBaroInnovationGate * sqrtf(s);
    if (fabsf(innovation) > gate_threshold) {
        ++baro_total_rejects_;
        return false;
    }

    // Push NIS AFTER gate — gated readings have enormous NIS that corrupt the monitor
    if (phase_qr_) {
        innovation_channel_push(&innov_baro_, last_baro_nis_);
    }

    ++baro_total_accepts_;
#ifdef ESKF_USE_BIERMAN
    bierman_kalman_update(kHIdx, kHValue, innovation, r);
#else
    scalar_kalman_update(kHIdx, kHValue, innovation, r);
    clamp_covariance();
#endif
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
// update_mag_heading: Magnetometer heading measurement update
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
// UNOBSERVABLE: mag states 15-20 (earth_mag, body_mag_bias) have no H entries
// and no F coupling with this yaw-only model. They MUST remain inhibited until
// a proper 3-axis mag measurement model is implemented (Titan tier).
// Full 3-axis model: z_pred = R(q) * earth_mag + body_mag_bias, with H at
// attitude(0-2), earth_mag(15-17), body_mag_bias(18-20).
//
// Two-tier interference detection (council modification):
//   25-50% magnitude deviation: inflate R by 10x
//   >50% deviation: hard reject
//
// Sequential scalar update — same pattern as update_baro().
// Joseph form P update for numerical stability.
// Static locals for Mat24 temporaries (LL Entry 1).
// ============================================================================
// Compute effective R after two-tier interference detection.
// Returns negative if hard-rejected (>50% deviation).
static float mag_interference_r(float mag_norm, float expected_magnitude) {
    float r_effective = ESKF::kRMagHeading;
    if (expected_magnitude > 0.0F) {
        const float deviation = fabsf(mag_norm - expected_magnitude) / expected_magnitude;
        if (deviation > ESKF::kMagInterferenceRejectThreshold) { return -1.0F; }
        if (deviation > ESKF::kMagInterferenceThreshold) {
            r_effective = ESKF::kRMagHeading * ESKF::kMagInterferenceRScale;
        }
    }
    return r_effective;
}

// Compute effective mag R: interference → phase R → tilt inflation.
// Returns negative if hard-rejected (>50% interference or >60° tilt).
float ESKF::compute_mag_r(float mag_norm, float expected_magnitude,
                          float tilt) const {
    float r = mag_interference_r(mag_norm, expected_magnitude);
    if (r < 0.0F) { return -1.0F; }

    // Phase-aware R baseline
    if (phase_qr_) {
        const float ratio = r / kRMagHeading;
        r = r_active_.r_mag * ratio;
    }

    // Tilt-conditional R inflation
    if (tilt > kMagTiltMaxRad) { return -1.0F; }
    if (tilt > kMagTiltThresholdRad) {
        const float frac = (tilt - kMagTiltThresholdRad)
                         / (kMagTiltMaxRad - kMagTiltThresholdRad);
        r *= (1.0F + kMagTiltRInflationMax * frac);
    }
    return r;
}

bool ESKF::update_mag_heading(const Vec3& mag_body, float expected_magnitude,
                              float declination_rad) {
    if (!std::isfinite(mag_body.x) || !std::isfinite(mag_body.y) ||
        !std::isfinite(mag_body.z) || !std::isfinite(declination_rad)) {
        return false;
    }
    const float mag_norm = mag_body.norm();
    if (mag_norm < kMagMinMagnitude) { return false; }

    const Vec3 euler = q.to_euler();
    const float tilt = sqrtf(euler.x * euler.x + euler.y * euler.y);
    const float r_effective = compute_mag_r(mag_norm, expected_magnitude, tilt);
    if (r_effective < 0.0F) {
        ++mag_consecutive_rejects_;
        ++mag_total_rejects_;
        return false;
    }

    const Quat q_zero_yaw = Quat::from_euler(euler.x, euler.y, 0.0F);
    const Vec3 mag_level = q_zero_yaw.rotate(mag_body);

    // Measured heading (mag North + declination), predicted heading (Euler yaw)
    const float heading_measured = wrap_pi(-atan2f(mag_level.y, mag_level.x)
                                           + declination_rad);
    const float innovation = wrap_pi(heading_measured - euler.z);

    constexpr int32_t kHIdx = eskf::kIdxAttitude + 2;  // yaw index
    const float s = P(kHIdx, kHIdx) + r_effective;
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

    // Push NIS AFTER gate
    if (phase_qr_) {
        innovation_channel_push(&innov_mag_, last_mag_nis_);
    }

    mag_consecutive_rejects_ = 0;
    ++mag_total_accepts_;
#ifdef ESKF_USE_BIERMAN
    bierman_kalman_update(kHIdx, 1.0F, innovation, r_effective);
#else
    scalar_kalman_update(kHIdx, 1.0F, innovation, r_effective);
    clamp_covariance();
#endif
    return true;
}

// ============================================================================
// update_mag_3axis: Full 3D magnetometer fusion (Stage 3D, IVP-99)
//
// Measurement model:
//   z_observed = magBody (calibrated, body frame)
//   z_predicted = R(q) * earth_mag + body_mag_bias (body frame)
//   innovation = magBody - z_predicted (per axis)
//
// Sequential scalar updates: 3 for earth_mag NED (rotated to body),
// then 3 for body_mag_bias. Same pattern as GPS position/velocity.
// ============================================================================
// Sequential 3-axis scalar updates for mag states.
// Returns max NIS across all accepted axes.
float ESKF::fuse_mag_axes(const float innov[3], float r_per_axis) {
    float max_nis = 0.0F;
    // Earth mag states [15-17]
    for (int32_t axis = 0; axis < 3; ++axis) {
        const int32_t h_idx = eskf::kIdxEarthMag + axis;
        const float s = P(h_idx, h_idx) + r_per_axis;
        if (s >= kMinInnovationVariance &&
            fabsf(innov[axis]) <= 5.0F * sqrtf(s)) {
            const float nis = (innov[axis] * innov[axis]) / s;
            if (nis > max_nis) { max_nis = nis; }
#ifdef ESKF_USE_BIERMAN
            bierman_kalman_update(h_idx, 1.0f, innov[axis], r_per_axis);
#else
            scalar_kalman_update(h_idx, 1.0f, innov[axis], r_per_axis);
#endif
        }
    }
    // Body mag bias states [18-20]
    for (int32_t axis = 0; axis < 3; ++axis) {
        const int32_t h_idx = eskf::kIdxBodyMagBias + axis;
        const float s = P(h_idx, h_idx) + r_per_axis;
        if (s >= kMinInnovationVariance &&
            fabsf(innov[axis]) <= 5.0F * sqrtf(s)) {
#ifdef ESKF_USE_BIERMAN
            bierman_kalman_update(h_idx, 1.0f, innov[axis], r_per_axis);
#else
            scalar_kalman_update(h_idx, 1.0f, innov[axis], r_per_axis);
#endif
        }
    }
    return max_nis;
}

// Magnitude pre-check: reject if measured |B| deviates >25% from expected
static bool mag_magnitude_ok(const Vec3& mag_body, const Vec3& earth_field_ned) {
    const float expected_mag = earth_field_ned.norm();
    if (expected_mag < 1.0F) { return true; }  // No expected field to compare
    const float ratio = mag_body.norm() / expected_mag;
    return (ratio >= 0.75F && ratio <= 1.25F);
}

bool ESKF::update_mag_3axis(const Vec3& mag_body, const Vec3& earth_field_ned,
                            float r_per_axis) {
    if (inhibit_mag_states_) { return false; }
    if (!std::isfinite(mag_body.x) || !std::isfinite(mag_body.y) ||
        !std::isfinite(mag_body.z)) { return false; }

    if (!mag_magnitude_ok(mag_body, earth_field_ned)) {
        ++mag_consecutive_rejects_;
        ++mag_total_rejects_;
        return false;
    }

    // Predicted body-frame mag: R(q) * earth_mag + body_mag_bias
    const Vec3 mag_pred_body = q.rotate(earth_mag) + body_mag_bias;

    // Innovation in body frame (measured - predicted)
    const float innov[3] = {
        mag_body.x - mag_pred_body.x,
        mag_body.y - mag_pred_body.y,
        mag_body.z - mag_pred_body.z
    };

    // Fuse 3 earth_mag + 3 body_mag_bias via sequential scalar updates
    last_mag_nis_ = fuse_mag_axes(innov, r_per_axis);
    mag_consecutive_rejects_ = 0;
    ++mag_total_accepts_;

#ifndef ESKF_USE_BIERMAN
    clamp_covariance();
#endif
    return true;
}

// ============================================================================
// reset_mag_heading: Force-reset yaw after sustained rejection.
// ArduPilot alignYawAngle() / PX4 resetMagHeading() pattern.
// Zeroes yaw cross-covariances to prevent stale correlations from
// corrupting subsequent updates. Navigation states may briefly wobble
// after heading reset; settles in 2-3s with GPS aiding.
// ============================================================================
void ESKF::reset_mag_heading(float heading_measured) {
#ifdef ESKF_USE_BIERMAN
    ensure_dense();  // Modifies P directly
#endif
    constexpr int32_t kYawIdx = eskf::kIdxAttitude + 2;
    const Vec3 euler = q.to_euler();

    q = Quat::from_euler(euler.x, euler.y, heading_measured);
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
// update_zupt: Zero-velocity pseudo-measurement
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
bool ESKF::update_zupt(const Vec3& accel_meas, const Vec3& gyro_meas) {
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
        ++zupt_total_rejects_;
        return false;
    }

    last_zupt_active_ = true;

    // Three sequential scalar updates: v_n=0, v_e=0, v_d=0
    float max_nis = 0.0F;
    const float v_components[3] = { v.x, v.y, v.z };

    for (int32_t axis = 0; axis < 3; ++axis) {
        const int32_t k_h_idx = eskf::kIdxVelocity + axis;
        const float innovation = -v_components[axis];  // y = 0 - v[axis]
        const float s = P(k_h_idx, k_h_idx) + kRZupt;
        if (s >= kMinInnovationVariance) {
            // NIS
            const float nis = (innovation * innovation) / s;
            if (nis > max_nis) {
                max_nis = nis;
            }

            // Innovation gate
            const float gate_threshold = kZuptInnovationGate * sqrtf(s);
            if (fabsf(innovation) <= gate_threshold) {
#ifdef ESKF_USE_BIERMAN
                bierman_kalman_update(k_h_idx, 1.0F, innovation, kRZupt);
#else
                scalar_kalman_update(k_h_idx, 1.0F, innovation, kRZupt);
#endif
            }
        }
    }

    last_zupt_nis_ = max_nis;
    ++zupt_total_accepts_;
#ifndef ESKF_USE_BIERMAN
    clamp_covariance();
#endif
    return true;
}

// ============================================================================
// update_zupt (state-aware overload): When on_pad is true, skips IMU
// stationarity check and uses tighter R. The flight state machine guarantees
// stationarity in IDLE/ARMED — no need to infer it from sensor data.
// ArduPilot EKF3 onGround, PX4 ECL vehicle_at_rest.
// ============================================================================
bool ESKF::update_zupt(const Vec3& accel_meas, const Vec3& gyro_meas,
                        bool on_pad) {
    if (!on_pad) {
        return update_zupt(accel_meas, gyro_meas);
    }

    // On pad: skip stationarity check, use tight R
    if (!std::isfinite(accel_meas.x) || !std::isfinite(gyro_meas.x)) {
        last_zupt_active_ = false;
        return false;
    }

    last_zupt_active_ = true;

    float max_nis = 0.0F;
    const float v_components[3] = { v.x, v.y, v.z };

    for (int32_t axis = 0; axis < 3; ++axis) {
        const int32_t k_h_idx = eskf::kIdxVelocity + axis;
        const float innovation = -v_components[axis];
        const float s = P(k_h_idx, k_h_idx) + kRZuptOnPad;
        if (s >= kMinInnovationVariance) {
            const float nis = (innovation * innovation) / s;
            if (nis > max_nis) {
                max_nis = nis;
            }

            const float gate_threshold = kZuptInnovationGate * sqrtf(s);
            if (fabsf(innovation) <= gate_threshold) {
#ifdef ESKF_USE_BIERMAN
                bierman_kalman_update(k_h_idx, 1.0F, innovation, kRZuptOnPad);
#else
                scalar_kalman_update(k_h_idx, 1.0F, innovation, kRZuptOnPad);
#endif
            }
        }
    }

    last_zupt_nis_ = max_nis;
    ++zupt_total_accepts_;
#ifndef ESKF_USE_BIERMAN
    clamp_covariance();
#endif
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
bool ESKF::set_origin(double lat_rad, double lon_rad, float alt_m, float hdop) {
    if (has_origin_) {
        return false;  // Origin already set
    }
    if (hdop > kGpsMaxHdopForOrigin) {
        return false;  // Geometry too poor for origin
    }
#ifdef ESKF_USE_BIERMAN
    ensure_dense();  // Modifies P directly
#endif

    origin_lat_rad_ = lat_rad;
    origin_lon_rad_ = lon_rad;
    origin_alt_m_ = alt_m;
    cos_origin_lat_ = cosf(static_cast<float>(lat_rad));
    has_origin_ = true;

    // Council fix: Reset position/velocity P to GPS-derived uncertainty.
    // Without this, stale cross-covariances corrupt baro Kalman gain.
    const float r_pos = kSigmaGpsPosBase * fmaxf(hdop, 1.0F);
    const float p_pos = r_pos * r_pos;  // Position variance from GPS quality
    const float p_vel = kRGpsVel;       // Velocity variance (fixed)

    // Zero ALL cross-covariance terms involving position [3..5] and velocity [6..8]
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        for (int32_t j = eskf::kIdxPosition; j < eskf::kIdxPosition + kPosVelBlockSpan; ++j) {
            P.data[i][j] = 0.0F;
            P.data[j][i] = 0.0F;
        }
    }
    // Set position diagonal
    P.data[eskf::kIdxPosition + 0][eskf::kIdxPosition + 0] = p_pos;  // North
    P.data[eskf::kIdxPosition + 1][eskf::kIdxPosition + 1] = p_pos;  // East
    P.data[eskf::kIdxPosition + 2][eskf::kIdxPosition + 2] = p_pos * kSigmaGpsVertScale * kSigmaGpsVertScale;  // Down
    // Set velocity diagonal
    P.data[eskf::kIdxVelocity + 0][eskf::kIdxVelocity + 0] = p_vel;
    P.data[eskf::kIdxVelocity + 1][eskf::kIdxVelocity + 1] = p_vel;
    P.data[eskf::kIdxVelocity + 2][eskf::kIdxVelocity + 2] = p_vel;

    return true;
}

// ============================================================================
// reset_origin: Re-center NED frame, adjusting p for continuity
// Computes absolute geodetic from old origin + p, stores new origin,
// reprojects p into the new frame. Net effect: p becomes near-zero.
// Council condition C-7: absolute position must be preserved.
// ============================================================================
void ESKF::reset_origin(double new_lat_rad, double new_lon_rad, float new_alt_m) {
    if (!has_origin_) {
        return;
    }

    // Save current absolute geodetic position (reverse flat-earth, in double)
    double abs_lat = origin_lat_rad_
        + static_cast<double>(p.x) / static_cast<double>(kEarthRadius);
    double abs_lon = origin_lon_rad_
        + static_cast<double>(p.y) / (static_cast<double>(kEarthRadius)
                                       * static_cast<double>(cos_origin_lat_));
    float abs_alt = origin_alt_m_ - p.z;  // NED down: alt = origin_alt - down

    // Store new origin
    origin_lat_rad_ = new_lat_rad;
    origin_lon_rad_ = new_lon_rad;
    origin_alt_m_ = new_alt_m;
    cos_origin_lat_ = cosf(static_cast<float>(new_lat_rad));

    // Reproject absolute position into new frame
    p = geodetic_to_ned(abs_lat, abs_lon, abs_alt);
}

// ============================================================================
// reset_velocity: Zero velocity + reset P velocity block.
// Flight Director API for state transitions.
// Pattern: same as set_origin() P-reset for velocity block (lines 996-1010).
// ============================================================================
void ESKF::reset_velocity() {
#ifdef ESKF_USE_BIERMAN
    ensure_dense();
#endif
    v = Vec3();
    // Zero cross-covariances for velocity states [6..8]
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        for (int32_t j = eskf::kIdxVelocity;
             j < eskf::kIdxVelocity + eskf::kBlockSize; ++j) {
            P.data[i][j] = 0.0F;
            P.data[j][i] = 0.0F;
        }
    }
    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        P.data[eskf::kIdxVelocity + i][eskf::kIdxVelocity + i] =
            kInitPVelocity;
    }
}

// ============================================================================
// reset_position: Zero position + reset P position block.
// Flight Director API for state transitions.
// ============================================================================
void ESKF::reset_position() {
#ifdef ESKF_USE_BIERMAN
    ensure_dense();
#endif
    p = Vec3();
    // Zero cross-covariances for position states [3..5]
    for (int32_t i = 0; i < eskf::kStateSize; ++i) {
        for (int32_t j = eskf::kIdxPosition;
             j < eskf::kIdxPosition + eskf::kBlockSize; ++j) {
            P.data[i][j] = 0.0F;
            P.data[j][i] = 0.0F;
        }
    }
    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        P.data[eskf::kIdxPosition + i][eskf::kIdxPosition + i] =
            kInitPPosition;
    }
}

// ============================================================================
// geodetic_to_ned: Convert WGS84 geodetic to local NED frame
// Critical (council C-5): subtraction in double before float cast.
// Flat-earth approximation — valid within ~100km of origin.
// ArduPilot: Location::get_distance_NED(), same formula.
// ============================================================================
Vec3 ESKF::geodetic_to_ned(double lat_rad, double lon_rad, float alt_m) const {
    if (!has_origin_) {
        return Vec3(0.0F, 0.0F, 0.0F);
    }

    // Double-precision subtraction before float cast (council C-5)
    float north = static_cast<float>(
        (lat_rad - origin_lat_rad_) * static_cast<double>(kEarthRadius));
    float east = static_cast<float>(
        (lon_rad - origin_lon_rad_) * static_cast<double>(kEarthRadius)
        * static_cast<double>(cos_origin_lat_));
    float down = -(alt_m - origin_alt_m_);

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
bool ESKF::update_gps_position(const Vec3& gps_ned, float hdop, float vdop) {
    if (!std::isfinite(gps_ned.x) || !std::isfinite(gps_ned.y) ||
        !std::isfinite(gps_ned.z)) {
        return false;
    }
    if (!has_origin_) { return false; }

    // Compute R per axis: horizontal from HDOP, vertical from VDOP or scaled horiz
    // Phase-aware R: phase R replaces baseline before HDOP scaling
    const float r_base = phase_qr_ ? r_active_.r_gps_pos : kRGpsPosDefault;
    const float hdop_clamped = (hdop > 1.0F) ? hdop : 1.0F;
    const float r_horiz = r_base * hdop_clamped * hdop_clamped;
    float r_vert = 0.0F;
    if (vdop > 0.0F) {
        const float vdop_clamped = (vdop > 1.0F) ? vdop : 1.0F;
        const float sigma_v = kSigmaGpsPosBase * vdop_clamped;
        r_vert = sigma_v * sigma_v;
    } else {
        r_vert = r_horiz * kSigmaGpsVertScale * kSigmaGpsVertScale;
    }

    const float r_values[3] = { r_horiz, r_horiz, r_vert };
    const int32_t indices[3] = { eskf::kIdxPosN, eskf::kIdxPosE, eskf::kIdxPosD };
    const float p_components[3] = { p.x, p.y, p.z };
    const float gps_components[3] = { gps_ned.x, gps_ned.y, gps_ned.z };
    float max_nis = 0.0F;

    for (int32_t axis = 0; axis < 3; ++axis) {
        const int32_t k_h_idx = indices[axis];
        const float r_noise = r_values[axis];
        const float innovation = gps_components[axis] - p_components[axis];
        const float s = P(k_h_idx, k_h_idx) + r_noise;
        if (s >= kMinInnovationVariance) {
            const float nis = (innovation * innovation) / s;

            if (fabsf(innovation) <= kGpsPositionGate * sqrtf(s)) {
                // Track max NIS only from accepted readings
                if (nis > max_nis) { max_nis = nis; }

#ifdef ESKF_USE_BIERMAN
                bierman_kalman_update(k_h_idx, 1.0F, innovation, r_noise);
#else
                scalar_kalman_update(k_h_idx, 1.0F, innovation, r_noise);
#endif
            }
        }
    }

    last_gps_pos_nis_ = max_nis;

    // Push NIS AFTER gate
    if (phase_qr_ && max_nis > 0.0F) {
        innovation_channel_push(&innov_gps_pos_, last_gps_pos_nis_);
    }

    ++gps_pos_total_accepts_;
#ifndef ESKF_USE_BIERMAN
    clamp_covariance();
#endif
    return true;
}

// ============================================================================
// update_gps_velocity: 2 sequential scalar updates on velocity (N, E only)
//
// Vertical velocity from GPS is unreliable — baro + IMU handle vertical.
// ArduPilot EKF3: FuseVelPosNED() with velocity axes.
// Council C-2: only indices [6..7], not [6..8].
// ============================================================================
bool ESKF::update_gps_velocity(float v_north, float v_east) {
    // Guard: reject non-finite input
    if (!std::isfinite(v_north) || !std::isfinite(v_east)) {
        return false;
    }

    const int32_t indices[2] = { eskf::kIdxVelN, eskf::kIdxVelE };
    const float gps_vel[2] = { v_north, v_east };
    const float v_components[2] = { v.x, v.y };

    float max_nis = 0.0F;

    for (int32_t axis = 0; axis < 2; ++axis) {
        const int32_t k_h_idx = indices[axis];

        // Innovation: y = z - h(x) = gpsVel[axis] - v[axis]
        const float innovation = gps_vel[axis] - v_components[axis];

        // Phase-aware R
        const float r = phase_qr_ ? r_active_.r_gps_vel : kRGpsVel;

        // Innovation covariance: S = P[idx][idx] + R
        const float s = P(k_h_idx, k_h_idx) + r;
        if (s >= kMinInnovationVariance) {
            // NIS
            const float nis = (innovation * innovation) / s;

            // Innovation gate
            const float gate_threshold = kGpsVelocityGate * sqrtf(s);
            if (fabsf(innovation) <= gate_threshold) {
                // Track max NIS only from accepted readings
                if (nis > max_nis) {
                    max_nis = nis;
                }

#ifdef ESKF_USE_BIERMAN
                bierman_kalman_update(k_h_idx, 1.0F, innovation, r);
#else
                scalar_kalman_update(k_h_idx, 1.0F, innovation, r);
#endif
            }
        }
    }

    last_gps_vel_nis_ = max_nis;

    // Push NIS AFTER gate
    if (phase_qr_ && max_nis > 0.0F) {
        innovation_channel_push(&innov_gps_vel_, last_gps_vel_nis_);
    }

    ++gps_vel_total_accepts_;
#ifndef ESKF_USE_BIERMAN
    clamp_covariance();
#endif
    return true;
}

// ============================================================================
// zero_p_block: Zero P rows/columns for a contiguous block of states.
// Used by inhibit flag control and clamp_covariance().
// ============================================================================
void ESKF::zero_p_block(int32_t start_idx, int32_t count) {
    for (int32_t i = start_idx; i < start_idx + count; ++i) {
        for (int32_t j = 0; j < eskf::kStateSize; ++j) {
            P.data[i][j] = 0.0F;
            P.data[j][i] = 0.0F;
        }
    }
}

// ============================================================================
// clamp_covariance: Diagonal clamping per council RF-2
// Prevents P from growing unbounded during GPS-denied operation.
// R-8: Codegen adds Q_d to all 24 states (including inhibited ones).
// This function zeroes the inhibited blocks back to 0 after propagation.
// ============================================================================
void ESKF::clamp_core_covariance() {
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
    // Core bias blocks are not clamped — they should converge via updates
}

void ESKF::clamp_extended_covariance() {
    // Codegen adds Q_d to all states. Inhibited blocks must be re-zeroed
    // here — the codegen doesn't know about inhibit flags.
    if (inhibit_mag_states_) {
        zero_p_block(eskf::kIdxEarthMag, kMagBlockSpan);  // earth_mag(3) + body_mag_bias(3)
    } else {
        for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
            if (P(eskf::kIdxEarthMag + i, eskf::kIdxEarthMag + i) > kClampPEarthMag) {
                P(eskf::kIdxEarthMag + i, eskf::kIdxEarthMag + i) = kClampPEarthMag;
            }
            if (P(eskf::kIdxBodyMagBias + i, eskf::kIdxBodyMagBias + i) > kClampPBodyMagBias) {
                P(eskf::kIdxBodyMagBias + i, eskf::kIdxBodyMagBias + i) = kClampPBodyMagBias;
            }
        }
    }

    if (inhibit_wind_states_) {
        zero_p_block(eskf::kIdxWindNE, 2);
    } else {
        for (int32_t i = 0; i < 2; ++i) {
            if (P(eskf::kIdxWindNE + i, eskf::kIdxWindNE + i) > kClampPWind) {
                P(eskf::kIdxWindNE + i, eskf::kIdxWindNE + i) = kClampPWind;
            }
        }
    }

    if (inhibit_baro_bias_) {
        zero_p_block(eskf::kIdxBaroBias, 1);
    } else {
        if (P(eskf::kIdxBaroBias, eskf::kIdxBaroBias) > kClampPBaroBias) {
            P(eskf::kIdxBaroBias, eskf::kIdxBaroBias) = kClampPBaroBias;
        }
    }
}

void ESKF::clamp_covariance() {
    clamp_core_covariance();
    clamp_extended_covariance();
}

// ============================================================================
// healthy: Health check per council RF-3
// Returns false if any of:
//   - P contains NaN or Inf
//   - Enabled P diagonals are not strictly positive
//   - Quaternion norm is far from 1
//   - Biases exceed physical limits
//
// R-1: Inhibit-aware. Inhibited state blocks have P diagonal = 0 (by design).
// diagonal_positive() would return false. Instead, check only enabled state
// block diagonals for strict positivity.
// ============================================================================
bool ESKF::healthy() const {
    // Bierman note: when P is in UD form, stale dense diagonals are still
    // valid for health checks (Bierman preserves positive-definiteness).

    // P must be finite everywhere (including inhibited blocks, which are 0)
    if (!P.is_finite()) {
        return false;
    }

    // Core states [0..14]: always require positive diagonal
    for (int32_t i = 0; i < eskf::kIdxEarthMag; ++i) {
        if (P.data[i][i] <= 0.0F) {
            return false;
        }
    }

    // Extended states: only check diagonal when NOT inhibited
    if (!inhibit_mag_states_) {
        for (int32_t i = eskf::kIdxEarthMag; i < eskf::kIdxEarthMag + kMagBlockSpan; ++i) {
            if (P.data[i][i] <= 0.0F) {
                return false;
            }
        }
    }
    if (!inhibit_wind_states_) {
        for (int32_t i = eskf::kIdxWindNE; i < eskf::kIdxWindNE + 2; ++i) {
            if (P.data[i][i] <= 0.0F) {
                return false;
            }
        }
    }
    if (!inhibit_baro_bias_) {
        if (P.data[eskf::kIdxBaroBias][eskf::kIdxBaroBias] <= 0.0F) {
            return false;
        }
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

    // Velocity sentinel: catches silent sensor fault divergence (LL Entry 29)
    if (v.norm() >= kMaxHealthyVelocity) {
        return false;
    }

    return true;
}

// ============================================================================
// check_p_growth: Periodic P-diagonal growth rate check.
// Returns false if position or velocity P diags grew >10× in 30s (diverging).
// Reset cycling guard: >2 resets in 5 min → permanently degraded.
// ============================================================================
// Snapshot P-diagonals for position and velocity into baseline arrays.
void ESKF::snapshot_p_growth_baseline() {
    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        p_growth_baseline_pos_[i] =
            P.data[eskf::kIdxPosition + i][eskf::kIdxPosition + i];
        p_growth_baseline_vel_[i] =
            P.data[eskf::kIdxVelocity + i][eskf::kIdxVelocity + i];
    }
}

// Record a P-growth reset event, flag degraded if cycling.
void ESKF::record_p_growth_reset(uint32_t now_us) {
    if (p_growth_reset_count_ == 0) {
        p_growth_first_reset_us_ = now_us;
    }
    p_growth_reset_count_++;
    if (p_growth_reset_count_ > kPGrowthMaxResetsInWindow &&
        now_us - p_growth_first_reset_us_ < kPGrowthResetWindowUs) {
        p_growth_degraded_ = true;
    }
    p_growth_baseline_set_ = false;
}

bool ESKF::check_p_growth(uint32_t now_us) {
    if (p_growth_degraded_) {
        return false;
    }

    if (!p_growth_baseline_set_) {
        snapshot_p_growth_baseline();
        p_growth_last_check_us_ = now_us;
        p_growth_baseline_set_ = true;
        return true;
    }

    if (now_us - p_growth_last_check_us_ < kPGrowthCheckIntervalUs) {
        return true;
    }

    // Check growth ratio for each axis
    for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
        float pos_p = P.data[eskf::kIdxPosition + i][eskf::kIdxPosition + i];
        float vel_p = P.data[eskf::kIdxVelocity + i][eskf::kIdxVelocity + i];

        if (p_growth_baseline_pos_[i] > kPGrowthBaselineEpsilon &&
            pos_p > p_growth_baseline_pos_[i] * kPGrowthRatioThreshold) {
            record_p_growth_reset(now_us);
            return false;
        }
        if (p_growth_baseline_vel_[i] > kPGrowthBaselineEpsilon &&
            vel_p > p_growth_baseline_vel_[i] * kPGrowthRatioThreshold) {
            record_p_growth_reset(now_us);
            return false;
        }
    }

    // Update baseline snapshot
    snapshot_p_growth_baseline();
    p_growth_last_check_us_ = now_us;

    // Clear reset counter if window expired
    if (p_growth_reset_count_ > 0 &&
        now_us - p_growth_first_reset_us_ >= kPGrowthResetWindowUs) {
        p_growth_reset_count_ = 0;
    }

    return true;
}

// ============================================================================
// reset_p_growth_baseline: Clear baseline state after filter re-init.
// ============================================================================
void ESKF::reset_p_growth_baseline() {
    p_growth_baseline_set_ = false;
    p_growth_degraded_ = false;
    p_growth_reset_count_ = 0;
}

// ============================================================================
// set_inhibit_mag: Enable/disable earth_mag[15-17] + body_mag_bias[18-20]
// R-7: When enabling, assert P is zero before setting initial variance.
// When disabling, zero P block to prevent stale cross-covariances.
// UNOBSERVABLE: mag states 15-20 require 3-axis mag model (Titan tier).
// Do NOT enable until full 3-axis measurement update is implemented.
// ============================================================================
void ESKF::set_inhibit_mag(bool inhibit) {
    if (inhibit == inhibit_mag_states_) {
        return;  // No change
    }
#ifdef ESKF_USE_BIERMAN
    ensure_dense();  // Inhibit modifies P directly
#endif
    inhibit_mag_states_ = inhibit;
    if (inhibit) {
        // Disabling: zero P block + nominal states
        zero_p_block(eskf::kIdxEarthMag, kMagBlockSpan);
        earth_mag = Vec3();
        body_mag_bias = Vec3();
    } else {
        // Enabling: set P diagonal to initial variance
        zero_p_block(eskf::kIdxEarthMag, kMagBlockSpan);  // Clear any stale cross-covariances
        for (int32_t i = 0; i < eskf::kBlockSize; ++i) {
            P(eskf::kIdxEarthMag + i, eskf::kIdxEarthMag + i) = kInitPEarthMag;
            P(eskf::kIdxBodyMagBias + i, eskf::kIdxBodyMagBias + i) = kInitPBodyMagBias;
        }
    }
}

// ============================================================================
// set_inhibit_wind: Enable/disable wind_NE[21-22]
// ============================================================================
void ESKF::set_inhibit_wind(bool inhibit) {
    if (inhibit == inhibit_wind_states_) {
        return;
    }
#ifdef ESKF_USE_BIERMAN
    ensure_dense();
#endif
    inhibit_wind_states_ = inhibit;
    if (inhibit) {
        zero_p_block(eskf::kIdxWindNE, 2);
        wind_n_ = 0.0F;
        wind_e_ = 0.0F;
    } else {
        zero_p_block(eskf::kIdxWindNE, 2);
        P(eskf::kIdxWindNE + 0, eskf::kIdxWindNE + 0) = kInitPWind;
        P(eskf::kIdxWindNE + 1, eskf::kIdxWindNE + 1) = kInitPWind;
    }
}

// ============================================================================
// set_inhibit_baro_bias: Enable/disable baro_bias[23]
// ============================================================================
void ESKF::set_inhibit_baro_bias(bool inhibit) {
    if (inhibit == inhibit_baro_bias_) {
        return;
    }
#ifdef ESKF_USE_BIERMAN
    ensure_dense();
#endif
    inhibit_baro_bias_ = inhibit;
    if (inhibit) {
        zero_p_block(eskf::kIdxBaroBias, 1);
        baro_bias_ = 0.0F;
    } else {
        zero_p_block(eskf::kIdxBaroBias, 1);
        P(eskf::kIdxBaroBias, eskf::kIdxBaroBias) = kInitPBaroBias;
    }
}

// ============================================================================
// Phase-aware Q/R
// ============================================================================

void ESKF::set_phase_qr(const PhaseQRTable* table) {
    phase_qr_ = table;
    if (table) {
        current_phase_ = 0;
        prev_phase_ = 0;
        q_ramp_alpha_ = 1.0F;
        q_ramp_remaining_ = 0;
        update_active_qr();
        innovation_channel_init(&innov_baro_);
        innovation_channel_init(&innov_mag_);
        innovation_channel_init(&innov_gps_pos_);
        innovation_channel_init(&innov_gps_vel_);
    }
}

void ESKF::notify_phase_change(uint8_t new_phase) {
    if (!phase_qr_ || new_phase >= kPhaseCount) {
        return;
    }
    if (new_phase == current_phase_) {
        return;
    }
    prev_phase_ = current_phase_;
    current_phase_ = new_phase;
    q_ramp_remaining_ = phase_qr_->ramp_steps;
    q_ramp_alpha_ = 0.0F;
    update_active_qr();
}

void ESKF::update_active_qr() {
    if (!phase_qr_) { return; }

    const auto& curr = phase_qr_->phases[current_phase_];
    const auto& prev = phase_qr_->phases[prev_phase_];
    const float a = q_ramp_alpha_;

    // Lerp Q scales
    q_active_.attitude   = prev.q_scale.attitude   + a * (curr.q_scale.attitude   - prev.q_scale.attitude);
    q_active_.velocity   = prev.q_scale.velocity   + a * (curr.q_scale.velocity   - prev.q_scale.velocity);
    q_active_.accel_bias = prev.q_scale.accel_bias + a * (curr.q_scale.accel_bias - prev.q_scale.accel_bias);
    q_active_.gyro_bias  = prev.q_scale.gyro_bias  + a * (curr.q_scale.gyro_bias  - prev.q_scale.gyro_bias);

    // Lerp R values
    r_active_.r_baro    = prev.r.r_baro    + a * (curr.r.r_baro    - prev.r.r_baro);
    r_active_.r_mag     = prev.r.r_mag     + a * (curr.r.r_mag     - prev.r.r_mag);
    r_active_.r_gps_pos = prev.r.r_gps_pos + a * (curr.r.r_gps_pos - prev.r.r_gps_pos);
    r_active_.r_gps_vel = prev.r.r_gps_vel + a * (curr.r.r_gps_vel - prev.r.r_gps_vel);
}

void ESKF::apply_phase_q_delta(float dt) {
    // Advance ramp
    if (q_ramp_remaining_ > 0) {
        --q_ramp_remaining_;
        const uint8_t total = phase_qr_->ramp_steps;
        q_ramp_alpha_ = 1.0f - static_cast<float>(q_ramp_remaining_)
                              / static_cast<float>(total);
        update_active_qr();
    }

    // Innovation adaptation: multiply Q scale by innovation-driven factor.
    // Freeze adaptation during ramp (Council A4 — avoid reacting to transition transients).
    float innov_scale_att = 1.0F;
    float innov_scale_vel = 1.0F;
    if (q_ramp_remaining_ == 0) {
        // Use max of relevant channels for each Q group
        innov_scale_att = innovation_channel_q_scale(&innov_mag_);
        const float baro_s = innovation_channel_q_scale(&innov_baro_);
        const float gps_s = innovation_channel_q_scale(&innov_gps_pos_);
        innov_scale_vel = (baro_s > gps_s) ? baro_s : gps_s;
    }

    // Additive delta: P[i][i] += baseline_sigma^2 * (effective_scale - 1.0) * dt
    // Only add when scale > 1.0 (delta > 0) — cannot reduce P diagonal.
    const float eff_att = q_active_.attitude * innov_scale_att;
    const float eff_vel = q_active_.velocity * innov_scale_vel;
    const float eff_ab  = q_active_.accel_bias;
    const float eff_gb  = q_active_.gyro_bias;

    if (eff_att > 1.0F) {
        const float delta = kSigmaGyro * kSigmaGyro * (eff_att - 1.0F) * dt;
        for (int32_t i = eskf::kIdxAttitude; i < eskf::kIdxAttitude + 3; ++i) {
            P(i, i) += delta;
        }
    }
    if (eff_vel > 1.0F) {
        const float delta = kSigmaAccel * kSigmaAccel * (eff_vel - 1.0F) * dt;
        for (int32_t i = eskf::kIdxVelocity; i < eskf::kIdxVelocity + 3; ++i) {
            P(i, i) += delta;
        }
    }
    if (eff_ab > 1.0F) {
        const float delta = kSigmaAccelBiasWalk * kSigmaAccelBiasWalk * (eff_ab - 1.0F) * dt;
        for (int32_t i = eskf::kIdxAccelBias; i < eskf::kIdxAccelBias + 3; ++i) {
            P(i, i) += delta;
        }
    }
    if (eff_gb > 1.0F) {
        const float delta = kSigmaGyroBiasWalk * kSigmaGyroBiasWalk * (eff_gb - 1.0F) * dt;
        for (int32_t i = eskf::kIdxGyroBias; i < eskf::kIdxGyroBias + 3; ++i) {
            P(i, i) += delta;
        }
    }
}

} // namespace rc
