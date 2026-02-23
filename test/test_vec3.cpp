// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include <gtest/gtest.h>
#include "math/vec3.h"

using rc::Vec3;

// ============================================================================
// Construction
// ============================================================================

TEST(Vec3Test, DefaultConstructsToZero) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3Test, ComponentConstruction) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

// ============================================================================
// Arithmetic
// ============================================================================

TEST(Vec3Test, Addition) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b;
    EXPECT_FLOAT_EQ(c.x, 5.0f);
    EXPECT_FLOAT_EQ(c.y, 7.0f);
    EXPECT_FLOAT_EQ(c.z, 9.0f);
}

TEST(Vec3Test, Subtraction) {
    Vec3 a(4.0f, 5.0f, 6.0f);
    Vec3 b(1.0f, 2.0f, 3.0f);
    Vec3 c = a - b;
    EXPECT_FLOAT_EQ(c.x, 3.0f);
    EXPECT_FLOAT_EQ(c.y, 3.0f);
    EXPECT_FLOAT_EQ(c.z, 3.0f);
}

TEST(Vec3Test, ScalarMultiplication) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    Vec3 r = v * 2.0f;
    EXPECT_FLOAT_EQ(r.x, 2.0f);
    EXPECT_FLOAT_EQ(r.y, 4.0f);
    EXPECT_FLOAT_EQ(r.z, 6.0f);
}

// ============================================================================
// Dot and Cross Product
// ============================================================================

TEST(Vec3Test, DotProduct) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    EXPECT_FLOAT_EQ(a.dot(b), 32.0f);  // 1*4 + 2*5 + 3*6
}

TEST(Vec3Test, DotProductOrthogonal) {
    Vec3 x(1.0f, 0.0f, 0.0f);
    Vec3 y(0.0f, 1.0f, 0.0f);
    EXPECT_FLOAT_EQ(x.dot(y), 0.0f);
}

TEST(Vec3Test, CrossProductBasisVectors) {
    Vec3 x(1.0f, 0.0f, 0.0f);
    Vec3 y(0.0f, 1.0f, 0.0f);
    Vec3 z = x.cross(y);
    EXPECT_FLOAT_EQ(z.x, 0.0f);
    EXPECT_FLOAT_EQ(z.y, 0.0f);
    EXPECT_FLOAT_EQ(z.z, 1.0f);
}

TEST(Vec3Test, CrossProductAnticommutative) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 ab = a.cross(b);
    Vec3 ba = b.cross(a);
    EXPECT_FLOAT_EQ(ab.x, -ba.x);
    EXPECT_FLOAT_EQ(ab.y, -ba.y);
    EXPECT_FLOAT_EQ(ab.z, -ba.z);
}

TEST(Vec3Test, CrossProductSelfIsZero) {
    Vec3 v(3.0f, 7.0f, 11.0f);
    Vec3 c = v.cross(v);
    EXPECT_NEAR(c.x, 0.0f, 1e-6f);
    EXPECT_NEAR(c.y, 0.0f, 1e-6f);
    EXPECT_NEAR(c.z, 0.0f, 1e-6f);
}

// ============================================================================
// Norm and Normalize
// ============================================================================

TEST(Vec3Test, NormOfUnitVector) {
    Vec3 x(1.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(x.norm(), 1.0f);
}

TEST(Vec3Test, NormOfKnownVector) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.norm(), 5.0f);
}

TEST(Vec3Test, NormalizeProducesUnitLength) {
    Vec3 v(3.0f, 4.0f, 5.0f);
    Vec3 n = v.normalized();
    EXPECT_NEAR(n.norm(), 1.0f, 1e-6f);
}

TEST(Vec3Test, NormalizeZeroVectorReturnsZero) {
    Vec3 zero;
    Vec3 n = zero.normalized();
    EXPECT_FLOAT_EQ(n.x, 0.0f);
    EXPECT_FLOAT_EQ(n.y, 0.0f);
    EXPECT_FLOAT_EQ(n.z, 0.0f);
}

TEST(Vec3Test, NormalizePreservesDirection) {
    Vec3 v(2.0f, 0.0f, 0.0f);
    Vec3 n = v.normalized();
    EXPECT_NEAR(n.x, 1.0f, 1e-6f);
    EXPECT_NEAR(n.y, 0.0f, 1e-6f);
    EXPECT_NEAR(n.z, 0.0f, 1e-6f);
}

// ============================================================================
// Negate and Scalar Divide (IVP-39 expansion)
// ============================================================================

TEST(Vec3Test, Negate) {
    Vec3 v(1.0f, -2.0f, 3.0f);
    Vec3 n = -v;
    EXPECT_FLOAT_EQ(n.x, -1.0f);
    EXPECT_FLOAT_EQ(n.y, 2.0f);
    EXPECT_FLOAT_EQ(n.z, -3.0f);
}

TEST(Vec3Test, NegateZero) {
    Vec3 v;
    Vec3 n = -v;
    EXPECT_FLOAT_EQ(n.x, 0.0f);
    EXPECT_FLOAT_EQ(n.y, 0.0f);
    EXPECT_FLOAT_EQ(n.z, 0.0f);
}

TEST(Vec3Test, ScalarDivide) {
    Vec3 v(2.0f, 4.0f, 6.0f);
    Vec3 r = v / 2.0f;
    EXPECT_FLOAT_EQ(r.x, 1.0f);
    EXPECT_FLOAT_EQ(r.y, 2.0f);
    EXPECT_FLOAT_EQ(r.z, 3.0f);
}

TEST(Vec3Test, CompoundAssignAdd) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    v += Vec3(4.0f, 5.0f, 6.0f);
    EXPECT_FLOAT_EQ(v.x, 5.0f);
    EXPECT_FLOAT_EQ(v.y, 7.0f);
    EXPECT_FLOAT_EQ(v.z, 9.0f);
}

TEST(Vec3Test, CompoundAssignSub) {
    Vec3 v(4.0f, 5.0f, 6.0f);
    v -= Vec3(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 3.0f);
    EXPECT_FLOAT_EQ(v.y, 3.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST(Vec3Test, CompoundAssignMul) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    v *= 3.0f;
    EXPECT_FLOAT_EQ(v.x, 3.0f);
    EXPECT_FLOAT_EQ(v.y, 6.0f);
    EXPECT_FLOAT_EQ(v.z, 9.0f);
}
