// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_MATH_MAT_H
#define ROCKETCHIP_MATH_MAT_H

// Mat<R,C>: Compile-time sized matrix for sensor fusion.
// Host-build, float32, no heap. No Pico SDK.
// Reference: Sola (2017) for ESKF matrix operations.

#include <cmath>
#include <cstdint>
#ifndef NDEBUG
#include <cstdlib>
#endif

namespace rc {

template <int32_t Rows, int32_t Cols>
struct Mat {
    float data[Rows][Cols]{};

    // Element access
    float& operator()(int32_t r, int32_t c) {
#ifndef NDEBUG
        if (r < 0 || r >= Rows || c < 0 || c >= Cols) { std::abort(); }
#endif
        return data[r][c];
    }
    const float& operator()(int32_t r, int32_t c) const {
#ifndef NDEBUG
        if (r < 0 || r >= Rows || c < 0 || c >= Cols) { std::abort(); }
#endif
        return data[r][c];
    }

    // Addition
    Mat operator+(const Mat& rhs) const {
        Mat result;
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t c = 0; c < Cols; ++c) {
                result.data[r][c] = data[r][c] + rhs.data[r][c];
            }
        }
        return result;
    }

    // Subtraction
    Mat operator-(const Mat& rhs) const {
        Mat result;
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t c = 0; c < Cols; ++c) {
                result.data[r][c] = data[r][c] - rhs.data[r][c];
            }
        }
        return result;
    }

    // Scalar multiply
    Mat operator*(float s) const {
        Mat result;
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t c = 0; c < Cols; ++c) {
                result.data[r][c] = data[r][c] * s;
            }
        }
        return result;
    }

    // Matrix multiply: (Rows x Cols) * (Cols x P) -> (Rows x P)
    template <int32_t P>
    Mat<Rows, P> operator*(const Mat<Cols, P>& rhs) const {
        Mat<Rows, P> result;
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t p = 0; p < P; ++p) {
                float sum = 0.0F;
                for (int32_t k = 0; k < Cols; ++k) {
                    sum += data[r][k] * rhs.data[k][p];
                }
                result.data[r][p] = sum;
            }
        }
        return result;
    }

    // Transpose: (Rows x Cols) -> (Cols x Rows)
    Mat<Cols, Rows> transposed() const {
        Mat<Cols, Rows> result;
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t c = 0; c < Cols; ++c) {
                result.data[c][r] = data[r][c];
            }
        }
        return result;
    }

    // Force symmetric: P = (P + P^T) / 2
    // Only valid for square matrices (compile-time enforced by usage)
    void force_symmetric() {
        static_assert(Rows == Cols, "force_symmetric requires square matrix");
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t c = r + 1; c < Cols; ++c) {
                float avg = (data[r][c] + data[c][r]) * 0.5F;
                data[r][c] = avg;
                data[c][r] = avg;
            }
        }
    }

    // Check if all elements are finite (no NaN, no Inf)
    bool is_finite() const {
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t c = 0; c < Cols; ++c) {
                if (!std::isfinite(data[r][c])) {
                    return false;
                }
            }
        }
        return true;
    }

    // Check diagonal positive (for covariance matrices)
    bool diagonal_positive() const {
        static_assert(Rows == Cols, "diagonal_positive requires square matrix");
        for (int32_t i = 0; i < Rows; ++i) {
            if (data[i][i] <= 0.0F) {
                return false;
            }
        }
        return true;
    }

    // Trace (sum of diagonal)
    float trace() const {
        static_assert(Rows == Cols, "trace requires square matrix");
        float sum = 0.0F;
        for (int32_t i = 0; i < Rows; ++i) {
            sum += data[i][i];
        }
        return sum;
    }

    // ---- In-place mutations (avoid return-by-value for large matrices, LL Entry 1) ----

    // Zero all elements in place
    void set_zero() {
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t c = 0; c < Cols; ++c) {
                data[r][c] = 0.0F;
            }
        }
    }

    // Set to identity in place (square matrices only)
    void set_identity() {
        static_assert(Rows == Cols, "set_identity requires square matrix");
        set_zero();
        for (int32_t i = 0; i < Rows; ++i) {
            data[i][i] = 1.0F;
        }
    }

    // Multiply all elements by scalar in place
    void scale(float s) {
        for (int32_t r = 0; r < Rows; ++r) {
            for (int32_t c = 0; c < Cols; ++c) {
                data[r][c] *= s;
            }
        }
    }

    // ---- Static constructors ----

    static Mat zeros() {
        return Mat{};  // zero-initialized by default
    }

    static Mat identity() {
        static_assert(Rows == Cols, "identity requires square matrix");
        Mat result;
        for (int32_t i = 0; i < Rows; ++i) {
            result.data[i][i] = 1.0F;
        }
        return result;
    }
};

// Common type aliases
using Mat3 = Mat<3, 3>;
using Mat15 = Mat<15, 15>;
using Vec15 = Mat<15, 1>;
using Mat24 = Mat<24, 24>;
using Vec24 = Mat<24, 1>;

// ============================================================================
// ESKF-specific free functions
// ============================================================================

// Dense F*P*F^T for host tests (test_mat.cpp). Flight sparse check is
// dense_fpft_add in eskf.cpp, not this helper.
template <int32_t N>
Mat<N, N> fpft_dense(const Mat<N, N>& F, const Mat<N, N>& P) {
    // Compute F*P first, then (F*P)*F^T
    Mat<N, N> FP = F * P;
    return FP * F.transposed();
}

// Gain/innovation scalars only — does not write P or x.
// H is 1xN, R is scalar. Avoids matrix inversion.
//
// Returns {K, innovation, S, nis}. K is Nx1, the rest are scalar.
template <int32_t N>
struct ScalarUpdateResult {
    Mat<N, 1> K;          // Kalman gain (Nx1)
    float innovation;     // z - H*x (scalar)
    float S;              // Innovation covariance H*P*H^T + R (scalar)
    float nis;            // innovation^2 / S, or 0 when S <= 1e-30
};

// N is a compile-time size (host tests use 2/3/15; not the flight UD path).
template <int32_t N>
ScalarUpdateResult<N> scalar_update(
    const Mat<N, N>& P,
    const Mat<N, 1>& x,
    const Mat<1, N>& H,
    float z,
    float R)
{
    // PHt = P * H^T  (Nx1)
    Mat<N, 1> PHt = P * H.transposed();

    // S = H * P * H^T + R  (scalar)
    float S = 0.0F;
    for (int32_t i = 0; i < N; ++i) {
        S += H(0, i) * PHt(i, 0);
    }
    S += R;

    // Innovation: z - H*x  (scalar)
    float Hx = 0.0F;
    for (int32_t i = 0; i < N; ++i) {
        Hx += H(0, i) * x(i, 0);
    }
    float innovation = z - Hx;

    // K = PHt / S; if S <= 1e-30, inv_S = 0 so K is zero.
    float inv_S = (S > 1e-30F) ? (1.0F / S) : 0.0F;
    Mat<N, 1> K;
    for (int32_t i = 0; i < N; ++i) {
        K(i, 0) = PHt(i, 0) * inv_S;
    }

    float nis = (S > 1e-30F) ? (innovation * innovation / S) : 0.0F;

    return {K, innovation, S, nis};
}

// Cholesky A = L L^T into L. false on a non-positive pivot (L left partial).
// N is compile-time (host tests 2/3).
template <int32_t N>
bool cholesky(const Mat<N, N>& A, Mat<N, N>& L) {
    L = Mat<N, N>::zeros();

    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = 0; j <= i; ++j) {
            float sum = 0.0F;
            for (int32_t k = 0; k < j; ++k) {
                sum += L(i, k) * L(j, k);
            }

            if (i == j) {
                float diag = A(i, i) - sum;
                if (diag <= 0.0F) {
                    return false;  // Non-positive pivot; L left partial
                }
                L(i, j) = sqrtf(diag);
            } else {
                L(i, j) = (A(i, j) - sum) / L(j, j);
            }
        }
    }
    return true;
}

} // namespace rc

#endif // ROCKETCHIP_MATH_MAT_H
