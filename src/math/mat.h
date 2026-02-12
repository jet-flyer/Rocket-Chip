#ifndef ROCKETCHIP_MATH_MAT_H
#define ROCKETCHIP_MATH_MAT_H

// Mat<R,C>: Compile-time sized matrix for sensor fusion.
// Pure C++ — no Pico SDK dependencies. Must compile on any host.
//
// All float, no double. Static storage, no heap allocation.
// Reference: Sola (2017) for ESKF matrix operations.

#include <cmath>
#include <cstdint>

namespace rc {

template <int32_t Rows, int32_t Cols>
struct Mat {
    float data[Rows][Cols]{};

    // Element access
    float& operator()(int32_t r, int32_t c) { return data[r][c]; }
    const float& operator()(int32_t r, int32_t c) const { return data[r][c]; }

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
                float sum = 0.0f;
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
                float avg = (data[r][c] + data[c][r]) * 0.5f;
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
            if (data[i][i] <= 0.0f) {
                return false;
            }
        }
        return true;
    }

    // Trace (sum of diagonal)
    float trace() const {
        static_assert(Rows == Cols, "trace requires square matrix");
        float sum = 0.0f;
        for (int32_t i = 0; i < Rows; ++i) {
            sum += data[i][i];
        }
        return sum;
    }

    // ---- Static constructors ----

    static Mat zeros() {
        return Mat{};  // zero-initialized by default
    }

    static Mat identity() {
        static_assert(Rows == Cols, "identity requires square matrix");
        Mat result;
        for (int32_t i = 0; i < Rows; ++i) {
            result.data[i][i] = 1.0f;
        }
        return result;
    }
};

// Common type aliases
using Mat3 = Mat<3, 3>;
using Mat15 = Mat<15, 15>;
using Vec15 = Mat<15, 1>;

// ============================================================================
// ESKF-specific free functions
// ============================================================================

// Dense F*P*F^T — used as verification path against sparse version.
// P must be square NxN, F must be NxN.
template <int32_t N>
Mat<N, N> fpft_dense(const Mat<N, N>& F, const Mat<N, N>& P) {
    // Compute F*P first, then (F*P)*F^T
    Mat<N, N> FP = F * P;
    return FP * F.transposed();
}

// Joseph form covariance update:
// P_new = (I - K*H) * P * (I - K*H)^T + K*R*K^T
// Numerically stable — maintains positive definiteness.
// Symmetry enforced after update.
//
// Template params: N = state dim, M = measurement dim
template <int32_t N, int32_t M>
Mat<N, N> joseph_update(
    const Mat<N, N>& P,
    const Mat<N, M>& K,
    const Mat<M, N>& H,
    const Mat<M, M>& R)
{
    Mat<N, N> I = Mat<N, N>::identity();
    Mat<N, N> IKH = I - K * H;                     // (I - K*H)
    Mat<N, N> IKHP = IKH * P;                      // (I - K*H) * P
    Mat<N, N> term1 = IKHP * IKH.transposed();     // (I-KH)*P*(I-KH)^T
    Mat<N, M> KR = K * R;                           // K*R  (NxM)
    Mat<N, N> term2 = KR * K.transposed();          // K*R*K^T  (NxM)*(MxN) = NxN
    Mat<N, N> result = term1 + term2;
    result.force_symmetric();
    return result;
}

// Scalar measurement update — optimized path when H is a row vector (1xN)
// and R is a scalar. Avoids matrix inversion entirely.
//
// Returns: {K, innovation, S} where K is Nx1 gain, innovation is scalar,
// S is innovation covariance (scalar).
template <int32_t N>
struct ScalarUpdateResult {
    Mat<N, 1> K;          // Kalman gain (Nx1)
    float innovation;     // z - H*x (scalar)
    float S;              // Innovation covariance H*P*H^T + R (scalar)
    float nis;            // Normalized innovation squared: innovation^2 / S
};

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
    float S = 0.0f;
    for (int32_t i = 0; i < N; ++i) {
        S += H(0, i) * PHt(i, 0);
    }
    S += R;

    // Innovation: z - H*x  (scalar)
    float Hx = 0.0f;
    for (int32_t i = 0; i < N; ++i) {
        Hx += H(0, i) * x(i, 0);
    }
    float innovation = z - Hx;

    // Kalman gain: K = PHt / S  (Nx1)
    float inv_S = (S > 1e-30f) ? (1.0f / S) : 0.0f;
    Mat<N, 1> K;
    for (int32_t i = 0; i < N; ++i) {
        K(i, 0) = PHt(i, 0) * inv_S;
    }

    float nis = (S > 1e-30f) ? (innovation * innovation / S) : 0.0f;

    return {K, innovation, S, nis};
}

// Joseph form for scalar update — takes precomputed K and H
template <int32_t N>
Mat<N, N> joseph_update_scalar(
    const Mat<N, N>& P,
    const Mat<N, 1>& K,
    const Mat<1, N>& H,
    float R)
{
    // Convert to matrix form and use full Joseph update
    Mat<1, 1> R_mat;
    R_mat(0, 0) = R;
    return joseph_update(P, K, H, R_mat);
}

// Cholesky decomposition: A = L * L^T
// Returns lower triangular L. A must be symmetric positive definite.
// Returns false if matrix is not positive definite.
template <int32_t N>
bool cholesky(const Mat<N, N>& A, Mat<N, N>& L) {
    L = Mat<N, N>::zeros();

    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = 0; j <= i; ++j) {
            float sum = 0.0f;
            for (int32_t k = 0; k < j; ++k) {
                sum += L(i, k) * L(j, k);
            }

            if (i == j) {
                float diag = A(i, i) - sum;
                if (diag <= 0.0f) {
                    return false;  // Not positive definite
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
