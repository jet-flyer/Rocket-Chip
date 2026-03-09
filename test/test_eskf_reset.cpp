// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Pre-Stage 8: ESKF reset_velocity() and reset_position() tests.
//
// Validates that the Flight Director reset APIs correctly zero state,
// reset P cross-covariances, and preserve unrelated states/biases.

#include <gtest/gtest.h>

#include <cmath>

#include "fusion/eskf.h"
#include "fusion/eskf_state.h"
#include "math/vec3.h"

namespace {

const rc::Vec3 kAccelStationary(0.0f, 0.0f, -rc::ESKF::kGravity);
const rc::Vec3 kGyroZero(0.0f, 0.0f, 0.0f);
constexpr float kDt = 0.005f;  // 200Hz

// Helper: init and propagate a few steps to build up non-trivial state
rc::ESKF make_eskf_with_state() {
    rc::ESKF eskf;
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroZero));
    // Propagate with slight acceleration to build non-zero velocity/position
    rc::Vec3 accelTilted(0.5f, -0.3f, -rc::ESKF::kGravity);
    for (int i = 0; i < 20; ++i) {
        eskf.predict(accelTilted, kGyroZero, kDt);
    }
    return eskf;
}

// ============================================================================
// reset_velocity tests
// ============================================================================

TEST(ESKFReset, ResetVelocity_ZerosV) {
    auto eskf = make_eskf_with_state();
    ASSERT_GT(eskf.v.norm(), 0.001f);  // Precondition: v is non-zero
    eskf.reset_velocity();
    EXPECT_FLOAT_EQ(eskf.v.x, 0.0f);
    EXPECT_FLOAT_EQ(eskf.v.y, 0.0f);
    EXPECT_FLOAT_EQ(eskf.v.z, 0.0f);
}

TEST(ESKFReset, ResetVelocity_ResetsP) {
    auto eskf = make_eskf_with_state();
    eskf.reset_velocity();

    // Diagonal should be kInitPVelocity
    for (int32_t i = 0; i < rc::eskf::kBlockSize; ++i) {
        int32_t idx = rc::eskf::kIdxVelocity + i;
        EXPECT_FLOAT_EQ(eskf.P.data[idx][idx], rc::ESKF::kInitPVelocity)
            << "Velocity P diagonal[" << i << "] mismatch";
    }

    // Cross-covariance terms involving velocity should be zero
    for (int32_t i = 0; i < rc::eskf::kStateSize; ++i) {
        for (int32_t j = rc::eskf::kIdxVelocity;
             j < rc::eskf::kIdxVelocity + rc::eskf::kBlockSize; ++j) {
            if (i == j) continue;  // Skip diagonal
            EXPECT_FLOAT_EQ(eskf.P.data[i][j], 0.0f)
                << "Cross-cov P[" << i << "][" << j << "] not zero";
            EXPECT_FLOAT_EQ(eskf.P.data[j][i], 0.0f)
                << "Cross-cov P[" << j << "][" << i << "] not zero";
        }
    }
}

TEST(ESKFReset, ResetVelocity_PreservesOtherStates) {
    auto eskf = make_eskf_with_state();
    // Snapshot pre-reset state
    auto qBefore = eskf.q;
    auto pBefore = eskf.p;
    auto abBefore = eskf.accel_bias;
    auto gbBefore = eskf.gyro_bias;

    eskf.reset_velocity();

    // Attitude preserved
    EXPECT_FLOAT_EQ(eskf.q.w, qBefore.w);
    EXPECT_FLOAT_EQ(eskf.q.x, qBefore.x);
    EXPECT_FLOAT_EQ(eskf.q.y, qBefore.y);
    EXPECT_FLOAT_EQ(eskf.q.z, qBefore.z);
    // Position preserved
    EXPECT_FLOAT_EQ(eskf.p.x, pBefore.x);
    EXPECT_FLOAT_EQ(eskf.p.y, pBefore.y);
    EXPECT_FLOAT_EQ(eskf.p.z, pBefore.z);
    // Biases preserved (council unanimous: never reset biases)
    EXPECT_FLOAT_EQ(eskf.accel_bias.x, abBefore.x);
    EXPECT_FLOAT_EQ(eskf.accel_bias.y, abBefore.y);
    EXPECT_FLOAT_EQ(eskf.accel_bias.z, abBefore.z);
    EXPECT_FLOAT_EQ(eskf.gyro_bias.x, gbBefore.x);
    EXPECT_FLOAT_EQ(eskf.gyro_bias.y, gbBefore.y);
    EXPECT_FLOAT_EQ(eskf.gyro_bias.z, gbBefore.z);
    // Filter remains healthy
    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// reset_position tests
// ============================================================================

TEST(ESKFReset, ResetPosition_ZerosP) {
    auto eskf = make_eskf_with_state();
    ASSERT_GT(eskf.p.norm(), 0.0001f);  // Precondition: p is non-zero
    eskf.reset_position();
    EXPECT_FLOAT_EQ(eskf.p.x, 0.0f);
    EXPECT_FLOAT_EQ(eskf.p.y, 0.0f);
    EXPECT_FLOAT_EQ(eskf.p.z, 0.0f);
}

TEST(ESKFReset, ResetPosition_ResetsP) {
    auto eskf = make_eskf_with_state();
    eskf.reset_position();

    // Diagonal should be kInitPPosition
    for (int32_t i = 0; i < rc::eskf::kBlockSize; ++i) {
        int32_t idx = rc::eskf::kIdxPosition + i;
        EXPECT_FLOAT_EQ(eskf.P.data[idx][idx], rc::ESKF::kInitPPosition)
            << "Position P diagonal[" << i << "] mismatch";
    }

    // Cross-covariance terms involving position should be zero
    for (int32_t i = 0; i < rc::eskf::kStateSize; ++i) {
        for (int32_t j = rc::eskf::kIdxPosition;
             j < rc::eskf::kIdxPosition + rc::eskf::kBlockSize; ++j) {
            if (i == j) continue;
            EXPECT_FLOAT_EQ(eskf.P.data[i][j], 0.0f)
                << "Cross-cov P[" << i << "][" << j << "] not zero";
            EXPECT_FLOAT_EQ(eskf.P.data[j][i], 0.0f)
                << "Cross-cov P[" << j << "][" << i << "] not zero";
        }
    }
}

TEST(ESKFReset, ResetPosition_PreservesOtherStates) {
    auto eskf = make_eskf_with_state();
    auto qBefore = eskf.q;
    auto vBefore = eskf.v;
    auto abBefore = eskf.accel_bias;
    auto gbBefore = eskf.gyro_bias;

    eskf.reset_position();

    // Attitude preserved
    EXPECT_FLOAT_EQ(eskf.q.w, qBefore.w);
    EXPECT_FLOAT_EQ(eskf.q.x, qBefore.x);
    // Velocity preserved
    EXPECT_FLOAT_EQ(eskf.v.x, vBefore.x);
    EXPECT_FLOAT_EQ(eskf.v.y, vBefore.y);
    EXPECT_FLOAT_EQ(eskf.v.z, vBefore.z);
    // Biases preserved
    EXPECT_FLOAT_EQ(eskf.accel_bias.x, abBefore.x);
    EXPECT_FLOAT_EQ(eskf.gyro_bias.x, gbBefore.x);
    // Healthy
    EXPECT_TRUE(eskf.healthy());
}

}  // namespace
