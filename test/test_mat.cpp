#include <gtest/gtest.h>
#include "math/mat.h"
#include "fusion/eskf_state.h"

#include <cmath>

using rc::Mat;
using rc::Mat3;
using rc::Mat15;

constexpr float kTol = 1e-5f;

// ============================================================================
// Basic Operations
// ============================================================================

TEST(MatTest, ZeroInitialized) {
    Mat3 m;
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            EXPECT_FLOAT_EQ(m(r, c), 0.0f);
        }
    }
}

TEST(MatTest, Identity3x3) {
    Mat3 I = Mat3::identity();
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            EXPECT_FLOAT_EQ(I(r, c), (r == c) ? 1.0f : 0.0f);
        }
    }
}

TEST(MatTest, MultiplyByIdentity) {
    // A * I == A
    Mat3 A;
    A(0, 0) = 1.0f; A(0, 1) = 2.0f; A(0, 2) = 3.0f;
    A(1, 0) = 4.0f; A(1, 1) = 5.0f; A(1, 2) = 6.0f;
    A(2, 0) = 7.0f; A(2, 1) = 8.0f; A(2, 2) = 9.0f;

    Mat3 I = Mat3::identity();
    Mat3 result = A * I;

    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(result(r, c), A(r, c), kTol);
        }
    }
}

TEST(MatTest, KnownMultiplication) {
    // [[1,2],[3,4]] * [[5,6],[7,8]] = [[19,22],[43,50]]
    Mat<2, 2> A;
    A(0, 0) = 1.0f; A(0, 1) = 2.0f;
    A(1, 0) = 3.0f; A(1, 1) = 4.0f;

    Mat<2, 2> B;
    B(0, 0) = 5.0f; B(0, 1) = 6.0f;
    B(1, 0) = 7.0f; B(1, 1) = 8.0f;

    Mat<2, 2> C = A * B;
    EXPECT_FLOAT_EQ(C(0, 0), 19.0f);
    EXPECT_FLOAT_EQ(C(0, 1), 22.0f);
    EXPECT_FLOAT_EQ(C(1, 0), 43.0f);
    EXPECT_FLOAT_EQ(C(1, 1), 50.0f);
}

TEST(MatTest, RectangularMultiply) {
    // (2x3) * (3x1) -> (2x1)
    Mat<2, 3> A;
    A(0, 0) = 1.0f; A(0, 1) = 2.0f; A(0, 2) = 3.0f;
    A(1, 0) = 4.0f; A(1, 1) = 5.0f; A(1, 2) = 6.0f;

    Mat<3, 1> x;
    x(0, 0) = 1.0f; x(1, 0) = 2.0f; x(2, 0) = 3.0f;

    Mat<2, 1> result = A * x;
    EXPECT_FLOAT_EQ(result(0, 0), 14.0f);  // 1+4+9
    EXPECT_FLOAT_EQ(result(1, 0), 32.0f);  // 4+10+18
}

TEST(MatTest, TransposeTransposeIsOriginal) {
    // A^T^T == A
    Mat3 A;
    A(0, 0) = 1.0f; A(0, 1) = 2.0f; A(0, 2) = 3.0f;
    A(1, 0) = 4.0f; A(1, 1) = 5.0f; A(1, 2) = 6.0f;
    A(2, 0) = 7.0f; A(2, 1) = 8.0f; A(2, 2) = 9.0f;

    Mat3 ATT = A.transposed().transposed();
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            EXPECT_FLOAT_EQ(ATT(r, c), A(r, c));
        }
    }
}

TEST(MatTest, Addition) {
    Mat<2, 2> A, B;
    A(0, 0) = 1.0f; A(0, 1) = 2.0f;
    A(1, 0) = 3.0f; A(1, 1) = 4.0f;
    B(0, 0) = 5.0f; B(0, 1) = 6.0f;
    B(1, 0) = 7.0f; B(1, 1) = 8.0f;

    Mat<2, 2> C = A + B;
    EXPECT_FLOAT_EQ(C(0, 0), 6.0f);
    EXPECT_FLOAT_EQ(C(0, 1), 8.0f);
    EXPECT_FLOAT_EQ(C(1, 0), 10.0f);
    EXPECT_FLOAT_EQ(C(1, 1), 12.0f);
}

TEST(MatTest, ScalarMultiply) {
    Mat<2, 2> A;
    A(0, 0) = 1.0f; A(0, 1) = 2.0f;
    A(1, 0) = 3.0f; A(1, 1) = 4.0f;

    Mat<2, 2> C = A * 3.0f;
    EXPECT_FLOAT_EQ(C(0, 0), 3.0f);
    EXPECT_FLOAT_EQ(C(0, 1), 6.0f);
    EXPECT_FLOAT_EQ(C(1, 0), 9.0f);
    EXPECT_FLOAT_EQ(C(1, 1), 12.0f);
}

TEST(MatTest, Trace) {
    Mat3 A;
    A(0, 0) = 1.0f; A(1, 1) = 5.0f; A(2, 2) = 9.0f;
    EXPECT_FLOAT_EQ(A.trace(), 15.0f);
}

// ============================================================================
// Symmetry Enforcement
// ============================================================================

TEST(MatTest, ForceSymmetric) {
    Mat3 A;
    A(0, 0) = 1.0f; A(0, 1) = 2.0f; A(0, 2) = 3.0f;
    A(1, 0) = 4.0f; A(1, 1) = 5.0f; A(1, 2) = 6.0f;
    A(2, 0) = 7.0f; A(2, 1) = 8.0f; A(2, 2) = 9.0f;

    A.force_symmetric();

    // Check symmetry: A(r,c) == A(c,r)
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            EXPECT_FLOAT_EQ(A(r, c), A(c, r));
        }
    }

    // Check averaged values
    EXPECT_FLOAT_EQ(A(0, 1), 3.0f);  // (2+4)/2
    EXPECT_FLOAT_EQ(A(0, 2), 5.0f);  // (3+7)/2
    EXPECT_FLOAT_EQ(A(1, 2), 7.0f);  // (6+8)/2
}

// ============================================================================
// FPFT (covariance propagation)
// ============================================================================

TEST(MatTest, FPFTDense3x3) {
    // Known result for small matrix
    Mat3 F;
    F(0, 0) = 1.0f; F(0, 1) = 0.1f; F(0, 2) = 0.0f;
    F(1, 0) = 0.0f; F(1, 1) = 1.0f; F(1, 2) = 0.0f;
    F(2, 0) = 0.0f; F(2, 1) = 0.0f; F(2, 2) = 1.0f;

    Mat3 P = Mat3::identity();

    Mat3 result = rc::fpft_dense(F, P);

    // F*I*F^T = F*F^T
    Mat3 expected = F * F.transposed();
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(result(r, c), expected(r, c), kTol);
        }
    }
}

TEST(MatTest, FPFTPreservesSymmetry) {
    // If P is symmetric, F*P*F^T should be symmetric
    Mat3 F;
    F(0, 0) = 1.0f; F(0, 1) = 0.1f; F(0, 2) = 0.05f;
    F(1, 0) = 0.0f; F(1, 1) = 1.0f; F(1, 2) = 0.1f;
    F(2, 0) = 0.0f; F(2, 1) = 0.0f; F(2, 2) = 1.0f;

    Mat3 P;
    P(0, 0) = 2.0f; P(0, 1) = 0.5f; P(0, 2) = 0.1f;
    P(1, 0) = 0.5f; P(1, 1) = 3.0f; P(1, 2) = 0.2f;
    P(2, 0) = 0.1f; P(2, 1) = 0.2f; P(2, 2) = 1.0f;

    Mat3 result = rc::fpft_dense(F, P);

    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(result(r, c), result(c, r), kTol);
        }
    }
}

// ============================================================================
// Joseph Form Update
// ============================================================================

TEST(MatTest, JosephFormWellConditioned) {
    // Joseph form should match standard form for well-conditioned case
    // Standard: P_new = P - K*H*P
    // Joseph: P_new = (I-KH)*P*(I-KH)^T + K*R*K^T
    Mat<2, 2> P;
    P(0, 0) = 1.0f; P(0, 1) = 0.1f;
    P(1, 0) = 0.1f; P(1, 1) = 1.0f;

    Mat<1, 2> H;
    H(0, 0) = 1.0f; H(0, 1) = 0.0f;

    Mat<1, 1> R;
    R(0, 0) = 0.5f;

    // Compute K = P*H^T * (H*P*H^T + R)^-1
    auto PHt = P * H.transposed();
    float S = H(0, 0) * PHt(0, 0) + H(0, 1) * PHt(1, 0) + R(0, 0);
    Mat<2, 1> K;
    K(0, 0) = PHt(0, 0) / S;
    K(1, 0) = PHt(1, 0) / S;

    // Standard form: P_new = (I - K*H) * P
    Mat<2, 2> I = Mat<2, 2>::identity();
    Mat<2, 2> P_standard = (I - K * H) * P;

    // Joseph form
    Mat<2, 2> P_joseph = rc::joseph_update(P, K, H, R);

    // Should be very close for well-conditioned case
    for (int32_t r = 0; r < 2; ++r) {
        for (int32_t c = 0; c < 2; ++c) {
            EXPECT_NEAR(P_joseph(r, c), P_standard(r, c), 1e-4f);
        }
    }
}

TEST(MatTest, JosephFormPreservesPDUnderStress) {
    // Joseph form should preserve positive-definiteness even when
    // standard form loses it due to numerical issues.
    // Run 1000 update cycles with aggressive gains.
    Mat<2, 2> P;
    P(0, 0) = 100.0f; P(0, 1) = 50.0f;
    P(1, 0) = 50.0f;  P(1, 1) = 100.0f;

    Mat<1, 2> H;
    H(0, 0) = 1.0f; H(0, 1) = 0.0f;

    Mat<1, 1> R;
    R(0, 0) = 0.01f;  // Very small R -> aggressive updates

    for (int32_t i = 0; i < 1000; ++i) {
        // Compute K
        auto PHt = P * H.transposed();
        float S = H(0, 0) * PHt(0, 0) + H(0, 1) * PHt(1, 0) + R(0, 0);
        Mat<2, 1> K;
        K(0, 0) = PHt(0, 0) / S;
        K(1, 0) = PHt(1, 0) / S;

        P = rc::joseph_update(P, K, H, R);
    }

    // P should still be symmetric and have positive diagonal
    EXPECT_TRUE(P.diagonal_positive());
    EXPECT_TRUE(P.is_finite());
    for (int32_t r = 0; r < 2; ++r) {
        for (int32_t c = 0; c < 2; ++c) {
            EXPECT_NEAR(P(r, c), P(c, r), 1e-4f);
        }
    }
}

TEST(MatTest, JosephFormSymmetricOutput) {
    Mat<2, 2> P;
    P(0, 0) = 2.0f; P(0, 1) = 0.5f;
    P(1, 0) = 0.5f; P(1, 1) = 3.0f;

    Mat<1, 2> H;
    H(0, 0) = 1.0f; H(0, 1) = 0.5f;

    Mat<1, 1> R;
    R(0, 0) = 1.0f;

    auto PHt = P * H.transposed();
    float S = H(0, 0) * PHt(0, 0) + H(0, 1) * PHt(1, 0) + R(0, 0);
    Mat<2, 1> K;
    K(0, 0) = PHt(0, 0) / S;
    K(1, 0) = PHt(1, 0) / S;

    Mat<2, 2> P_new = rc::joseph_update(P, K, H, R);

    for (int32_t r = 0; r < 2; ++r) {
        for (int32_t c = 0; c < 2; ++c) {
            EXPECT_NEAR(P_new(r, c), P_new(c, r), kTol);
        }
    }
}

// ============================================================================
// Scalar Update
// ============================================================================

TEST(MatTest, ScalarUpdateReducesUncertainty) {
    // After a measurement update, trace(P) should decrease
    Mat<2, 2> P;
    P(0, 0) = 10.0f; P(0, 1) = 0.0f;
    P(1, 0) = 0.0f;  P(1, 1) = 10.0f;

    Mat<2, 1> x;
    x(0, 0) = 1.0f;
    x(1, 0) = 0.0f;

    Mat<1, 2> H;
    H(0, 0) = 1.0f; H(0, 1) = 0.0f;

    float z = 1.5f;
    float R = 1.0f;

    float trace_before = P.trace();
    auto result = rc::scalar_update(P, x, H, z, R);
    Mat<2, 2> P_new = rc::joseph_update_scalar(P, result.K, H, R);
    float trace_after = P_new.trace();

    EXPECT_LT(trace_after, trace_before);
}

TEST(MatTest, ScalarUpdateNISComputation) {
    Mat<2, 2> P;
    P(0, 0) = 1.0f; P(0, 1) = 0.0f;
    P(1, 0) = 0.0f; P(1, 1) = 1.0f;

    Mat<2, 1> x;
    x(0, 0) = 0.0f;
    x(1, 0) = 0.0f;

    Mat<1, 2> H;
    H(0, 0) = 1.0f; H(0, 1) = 0.0f;

    float z = 2.0f;
    float R = 1.0f;

    auto result = rc::scalar_update(P, x, H, z, R);

    // Innovation = z - H*x = 2 - 0 = 2
    EXPECT_NEAR(result.innovation, 2.0f, kTol);

    // S = H*P*H^T + R = 1*1*1 + 1 = 2
    EXPECT_NEAR(result.S, 2.0f, kTol);

    // NIS = innovation^2 / S = 4 / 2 = 2
    EXPECT_NEAR(result.nis, 2.0f, kTol);
}

// ============================================================================
// Cholesky Decomposition
// ============================================================================

TEST(MatTest, CholeskyKnownMatrix) {
    // A = [[4, 2], [2, 3]]
    // L = [[2, 0], [1, sqrt(2)]]
    Mat<2, 2> A;
    A(0, 0) = 4.0f; A(0, 1) = 2.0f;
    A(1, 0) = 2.0f; A(1, 1) = 3.0f;

    Mat<2, 2> L;
    bool ok = rc::cholesky(A, L);
    EXPECT_TRUE(ok);

    EXPECT_NEAR(L(0, 0), 2.0f, kTol);
    EXPECT_NEAR(L(0, 1), 0.0f, kTol);
    EXPECT_NEAR(L(1, 0), 1.0f, kTol);
    EXPECT_NEAR(L(1, 1), sqrtf(2.0f), kTol);
}

TEST(MatTest, CholeskyReconstruction) {
    // L * L^T should give back the original matrix
    Mat3 A;
    A(0, 0) = 4.0f; A(0, 1) = 2.0f; A(0, 2) = 1.0f;
    A(1, 0) = 2.0f; A(1, 1) = 5.0f; A(1, 2) = 3.0f;
    A(2, 0) = 1.0f; A(2, 1) = 3.0f; A(2, 2) = 6.0f;

    Mat3 L;
    bool ok = rc::cholesky(A, L);
    EXPECT_TRUE(ok);

    Mat3 LLT = L * L.transposed();
    for (int32_t r = 0; r < 3; ++r) {
        for (int32_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(LLT(r, c), A(r, c), kTol);
        }
    }
}

TEST(MatTest, CholeskyFailsForNonPD) {
    // Non-positive-definite matrix should fail
    Mat<2, 2> A;
    A(0, 0) = 1.0f;  A(0, 1) = 10.0f;
    A(1, 0) = 10.0f; A(1, 1) = 1.0f;

    Mat<2, 2> L;
    bool ok = rc::cholesky(A, L);
    EXPECT_FALSE(ok);
}

// ============================================================================
// 15x15 Operations (ESKF-sized)
// ============================================================================

TEST(MatTest, Identity15x15) {
    Mat15 I = Mat15::identity();
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            EXPECT_FLOAT_EQ(I(r, c), (r == c) ? 1.0f : 0.0f);
        }
    }
}

TEST(MatTest, Multiply15x15ByIdentity) {
    // P * I == P for 15x15
    Mat15 P;
    // Fill with some values
    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            P(r, c) = static_cast<float>(r * 15 + c) * 0.01f;
        }
    }

    Mat15 I = Mat15::identity();
    Mat15 result = P * I;

    for (int32_t r = 0; r < 15; ++r) {
        for (int32_t c = 0; c < 15; ++c) {
            EXPECT_NEAR(result(r, c), P(r, c), kTol);
        }
    }
}

TEST(MatTest, FPFT15x15DiagonalPositive) {
    // Propagation should preserve positive diagonal if input is PD
    Mat15 P = Mat15::identity();
    Mat15 F = Mat15::identity();
    // Add small off-diagonal coupling
    F(rc::eskf::kIdxPosition, rc::eskf::kIdxVelocity) = 0.005f;
    F(rc::eskf::kIdxVelocity, rc::eskf::kIdxAccelBias) = -0.005f;

    Mat15 result = rc::fpft_dense(F, P);
    EXPECT_TRUE(result.diagonal_positive());
    EXPECT_TRUE(result.is_finite());
}

// ============================================================================
// Named State Indices
// ============================================================================

TEST(MatTest, StateIndicesAreContiguous) {
    // Verify indices form contiguous non-overlapping blocks
    EXPECT_EQ(rc::eskf::kIdxAttitude, 0);
    EXPECT_EQ(rc::eskf::kIdxPosition, 3);
    EXPECT_EQ(rc::eskf::kIdxVelocity, 6);
    EXPECT_EQ(rc::eskf::kIdxAccelBias, 9);
    EXPECT_EQ(rc::eskf::kIdxGyroBias, 12);
    EXPECT_EQ(rc::eskf::kStateSize, 15);

    // Each block is exactly 3 elements
    EXPECT_EQ(rc::eskf::kIdxPosition - rc::eskf::kIdxAttitude, rc::eskf::kBlockSize);
    EXPECT_EQ(rc::eskf::kIdxVelocity - rc::eskf::kIdxPosition, rc::eskf::kBlockSize);
    EXPECT_EQ(rc::eskf::kIdxAccelBias - rc::eskf::kIdxVelocity, rc::eskf::kBlockSize);
    EXPECT_EQ(rc::eskf::kIdxGyroBias - rc::eskf::kIdxAccelBias, rc::eskf::kBlockSize);
    EXPECT_EQ(rc::eskf::kStateSize - rc::eskf::kIdxGyroBias, rc::eskf::kBlockSize);
}
