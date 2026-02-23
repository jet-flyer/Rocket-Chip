// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include <gtest/gtest.h>
#include "math/quat.h"

#include <cmath>

using rc::Quat;
using rc::Vec3;

constexpr float kTol = 1e-5f;
constexpr float kPi = 3.14159265f;
constexpr float kPiOver2 = 1.5707963f;

// ============================================================================
// Construction and Identity
// ============================================================================

TEST(QuatTest, DefaultIsIdentity) {
    Quat q;
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuatTest, IdentityComposition) {
    // q_id * q == q
    Quat q_id;
    Quat q(0.7071068f, 0.0f, 0.7071068f, 0.0f);  // 90 deg about Y
    Quat result = q_id * q;
    EXPECT_NEAR(result.w, q.w, kTol);
    EXPECT_NEAR(result.x, q.x, kTol);
    EXPECT_NEAR(result.y, q.y, kTol);
    EXPECT_NEAR(result.z, q.z, kTol);

    // q * q_id == q
    result = q * q_id;
    EXPECT_NEAR(result.w, q.w, kTol);
    EXPECT_NEAR(result.x, q.x, kTol);
    EXPECT_NEAR(result.y, q.y, kTol);
    EXPECT_NEAR(result.z, q.z, kTol);
}

// ============================================================================
// Rotation Tests
// ============================================================================

TEST(QuatTest, Rotate90DegreesAroundZ) {
    // 90 deg around Z: should map X-axis to Y-axis
    Quat q = Quat::from_axis_angle(Vec3(0.0f, 0.0f, 1.0f), kPiOver2);
    Vec3 result = q.rotate(Vec3(1.0f, 0.0f, 0.0f));

    EXPECT_NEAR(result.x, 0.0f, kTol);
    EXPECT_NEAR(result.y, 1.0f, kTol);
    EXPECT_NEAR(result.z, 0.0f, kTol);
}

TEST(QuatTest, Rotate90DegreesAroundX) {
    // 90 deg around X: should map Y-axis to Z-axis
    Quat q = Quat::from_axis_angle(Vec3(1.0f, 0.0f, 0.0f), kPiOver2);
    Vec3 result = q.rotate(Vec3(0.0f, 1.0f, 0.0f));

    EXPECT_NEAR(result.x, 0.0f, kTol);
    EXPECT_NEAR(result.y, 0.0f, kTol);
    EXPECT_NEAR(result.z, 1.0f, kTol);
}

TEST(QuatTest, Rotate180DegreesAroundZ) {
    // 180 deg around Z: should map X-axis to -X-axis
    Quat q = Quat::from_axis_angle(Vec3(0.0f, 0.0f, 1.0f), kPi);
    Vec3 result = q.rotate(Vec3(1.0f, 0.0f, 0.0f));

    EXPECT_NEAR(result.x, -1.0f, kTol);
    EXPECT_NEAR(result.y, 0.0f, kTol);
    EXPECT_NEAR(result.z, 0.0f, kTol);
}

TEST(QuatTest, RotateMatchesHamiltonProduct) {
    // q.rotate(v) must match q * [0,v] * q_conj
    Quat q = Quat::from_euler(0.3f, 0.5f, 1.2f);
    Vec3 v(1.0f, 2.0f, 3.0f);

    Vec3 r1 = q.rotate(v);

    // Manual Hamilton product: q * [0,v] * q*
    Quat qv(0.0f, v.x, v.y, v.z);
    Quat qr = q * qv * q.conjugate();

    EXPECT_NEAR(r1.x, qr.x, kTol);
    EXPECT_NEAR(r1.y, qr.y, kTol);
    EXPECT_NEAR(r1.z, qr.z, kTol);
}

TEST(QuatTest, RotateMatchesDCM) {
    // q.rotate(v) must match DCM * v for same quaternion
    Quat q = Quat::from_euler(0.3f, 0.5f, 1.2f);
    Vec3 v(1.0f, 2.0f, 3.0f);

    Vec3 r_quat = q.rotate(v);

    float dcm[9];
    q.to_rotation_matrix(dcm);
    Vec3 r_dcm(
        dcm[0] * v.x + dcm[1] * v.y + dcm[2] * v.z,
        dcm[3] * v.x + dcm[4] * v.y + dcm[5] * v.z,
        dcm[6] * v.x + dcm[7] * v.y + dcm[8] * v.z
    );

    EXPECT_NEAR(r_quat.x, r_dcm.x, kTol);
    EXPECT_NEAR(r_quat.y, r_dcm.y, kTol);
    EXPECT_NEAR(r_quat.z, r_dcm.z, kTol);
}

// ============================================================================
// Double Cover Property
// ============================================================================

TEST(QuatTest, DoubleCoverProperty) {
    // q and -q must produce identical rotations
    Quat q = Quat::from_euler(0.3f, 0.5f, 1.2f);
    Quat neg_q(-q.w, -q.x, -q.y, -q.z);
    Vec3 v(1.0f, 2.0f, 3.0f);

    Vec3 r1 = q.rotate(v);
    Vec3 r2 = neg_q.rotate(v);

    EXPECT_NEAR(r1.x, r2.x, kTol);
    EXPECT_NEAR(r1.y, r2.y, kTol);
    EXPECT_NEAR(r1.z, r2.z, kTol);
}

// ============================================================================
// Euler Round-Trip
// ============================================================================

TEST(QuatTest, EulerRoundTrip) {
    // from_euler -> to_euler should recover original angles
    // Avoiding gimbal lock at +/-90 deg pitch
    const float roll = 0.3f;
    const float pitch = 0.5f;
    const float yaw = 1.2f;

    Quat q = Quat::from_euler(roll, pitch, yaw);
    Vec3 euler = q.to_euler();

    EXPECT_NEAR(euler.x, roll, kTol);
    EXPECT_NEAR(euler.y, pitch, kTol);
    EXPECT_NEAR(euler.z, yaw, kTol);
}

TEST(QuatTest, EulerRoundTripZero) {
    Quat q = Quat::from_euler(0.0f, 0.0f, 0.0f);
    Vec3 euler = q.to_euler();

    EXPECT_NEAR(euler.x, 0.0f, kTol);
    EXPECT_NEAR(euler.y, 0.0f, kTol);
    EXPECT_NEAR(euler.z, 0.0f, kTol);
}

TEST(QuatTest, EulerRoundTripNegativeAngles) {
    const float roll = -0.7f;
    const float pitch = -0.3f;
    const float yaw = -2.1f;

    Quat q = Quat::from_euler(roll, pitch, yaw);
    Vec3 euler = q.to_euler();

    EXPECT_NEAR(euler.x, roll, kTol);
    EXPECT_NEAR(euler.y, pitch, kTol);
    EXPECT_NEAR(euler.z, yaw, kTol);
}

// ============================================================================
// from_small_angle (ESKF core operation)
// ============================================================================

TEST(QuatTest, FromSmallAngleZeroIsIdentity) {
    // quat_from_small_angle([0,0,0]) -> identity quaternion
    Quat q = Quat::from_small_angle(Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_NEAR(q.w, 1.0f, kTol);
    EXPECT_NEAR(q.x, 0.0f, kTol);
    EXPECT_NEAR(q.y, 0.0f, kTol);
    EXPECT_NEAR(q.z, 0.0f, kTol);
}

TEST(QuatTest, FromSmallAngleSmallRotation) {
    // Small rotation about Z: should approximate from_axis_angle
    const float dtheta = 0.01f;
    Quat q_small = Quat::from_small_angle(Vec3(0.0f, 0.0f, dtheta));
    Quat q_exact = Quat::from_axis_angle(Vec3(0.0f, 0.0f, 1.0f), dtheta);

    // For small angles, first-order approximation should be very close
    EXPECT_NEAR(q_small.w, q_exact.w, 1e-4f);
    EXPECT_NEAR(q_small.x, q_exact.x, 1e-4f);
    EXPECT_NEAR(q_small.y, q_exact.y, 1e-4f);
    EXPECT_NEAR(q_small.z, q_exact.z, 1e-4f);
}

// ============================================================================
// Normalize Stability (drift accumulation test)
// ============================================================================

TEST(QuatTest, NormalizeAfter1000Multiplies) {
    // After 1000 multiplies, norm should still be ~1.0 if we normalize each step
    Quat q = Quat::from_axis_angle(Vec3(0.0f, 0.0f, 1.0f), 0.01f);
    Quat result;  // identity

    for (int32_t i = 0; i < 1000; ++i) {
        result = result * q;
        result.normalize();
    }

    EXPECT_NEAR(result.norm(), 1.0f, 1e-6f);
}

TEST(QuatTest, NormDriftsWithoutNormalization) {
    // Without normalization, norm drifts from 1.0 after many multiplies
    Quat q = Quat::from_axis_angle(Vec3(0.0f, 0.0f, 1.0f), 0.01f);
    Quat result;  // identity

    for (int32_t i = 0; i < 1000; ++i) {
        result = result * q;
        // deliberately NOT normalizing
    }

    // Norm should have drifted from 1.0
    float n = result.norm();
    EXPECT_GT(fabsf(n - 1.0f), 1e-6f);
}

// ============================================================================
// Conjugate and Inverse
// ============================================================================

TEST(QuatTest, ConjugateMultiplyIsNormSquared) {
    // q * q_conj should give [|q|^2, 0, 0, 0]
    Quat q = Quat::from_euler(0.3f, 0.5f, 1.2f);
    Quat result = q * q.conjugate();

    float n2 = q.norm_sq();
    EXPECT_NEAR(result.w, n2, kTol);
    EXPECT_NEAR(result.x, 0.0f, kTol);
    EXPECT_NEAR(result.y, 0.0f, kTol);
    EXPECT_NEAR(result.z, 0.0f, kTol);
}

TEST(QuatTest, InverseProducesIdentity) {
    // q * q_inv should give identity
    Quat q = Quat::from_euler(0.3f, 0.5f, 1.2f);
    Quat result = q * q.inverse();

    EXPECT_NEAR(result.w, 1.0f, kTol);
    EXPECT_NEAR(result.x, 0.0f, kTol);
    EXPECT_NEAR(result.y, 0.0f, kTol);
    EXPECT_NEAR(result.z, 0.0f, kTol);
}

// ============================================================================
// from_two_vectors
// ============================================================================

TEST(QuatTest, FromTwoVectorsAligned) {
    // Aligned vectors -> identity
    Vec3 v(1.0f, 0.0f, 0.0f);
    Quat q = Quat::from_two_vectors(v, v);
    EXPECT_NEAR(q.w, 1.0f, kTol);
    EXPECT_NEAR(q.x, 0.0f, kTol);
    EXPECT_NEAR(q.y, 0.0f, kTol);
    EXPECT_NEAR(q.z, 0.0f, kTol);
}

TEST(QuatTest, FromTwoVectorsOpposite) {
    // Opposite vectors -> 180 deg rotation
    Vec3 from(1.0f, 0.0f, 0.0f);
    Vec3 to(-1.0f, 0.0f, 0.0f);
    Quat q = Quat::from_two_vectors(from, to);

    // Should rotate from to to
    Vec3 result = q.rotate(from);
    EXPECT_NEAR(result.x, to.x, kTol);
    EXPECT_NEAR(result.y, to.y, kTol);
    EXPECT_NEAR(result.z, to.z, kTol);
}

TEST(QuatTest, FromTwoVectorsArbitrary) {
    Vec3 from(1.0f, 0.0f, 0.0f);
    Vec3 to(0.0f, 1.0f, 0.0f);
    Quat q = Quat::from_two_vectors(from, to);

    Vec3 result = q.rotate(from);
    EXPECT_NEAR(result.x, to.x, kTol);
    EXPECT_NEAR(result.y, to.y, kTol);
    EXPECT_NEAR(result.z, to.z, kTol);
}

// ============================================================================
// DCM Properties
// ============================================================================

TEST(QuatTest, RotationMatrixIsOrthogonal) {
    // DCM rows should be orthonormal: R * R^T = I
    Quat q = Quat::from_euler(0.3f, 0.5f, 1.2f);
    float m[9];
    q.to_rotation_matrix(m);

    // Check row 0 dot row 1 ~= 0
    float d01 = m[0] * m[3] + m[1] * m[4] + m[2] * m[5];
    EXPECT_NEAR(d01, 0.0f, kTol);

    // Check row 0 norm ~= 1
    float n0 = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    EXPECT_NEAR(n0, 1.0f, kTol);

    // Check row 1 norm ~= 1
    float n1 = sqrtf(m[3] * m[3] + m[4] * m[4] + m[5] * m[5]);
    EXPECT_NEAR(n1, 1.0f, kTol);

    // Check row 2 norm ~= 1
    float n2 = sqrtf(m[6] * m[6] + m[7] * m[7] + m[8] * m[8]);
    EXPECT_NEAR(n2, 1.0f, kTol);
}
