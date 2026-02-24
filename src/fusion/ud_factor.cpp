// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file ud_factor.cpp
 * @brief UD factorization for 24-state ESKF covariance.
 *
 * Thornton WMGS temporal update + Bierman scalar measurement update.
 * Three Thornton precision variants (f32, mixed f32/f64, f64 accum).
 *
 * All hot functions placed in .time_critical SRAM section for fair
 * comparison with codegen FPFT (LL Entry 30: XIP cache is only 2KB).
 */

#include "fusion/ud_factor.h"

#include <cmath>
#include <cstring>

namespace rc {

// N = 24 (state size) used throughout.
static constexpr int32_t N = 24;

// =========================================================================
// Utility functions
// =========================================================================

void ud_from_diagonal(UD24& ud, const float diag[24]) {
    std::memset(ud.U, 0, sizeof(ud.U));
    for (int32_t i = 0; i < N; ++i) {
        ud.U[i][i] = 1.0f;  // Unit diagonal
        ud.D[i] = diag[i];
    }
}

void ud_to_dense(const UD24& ud, float P[24][24]) {
    // P = U * D * U^T
    // P[i][j] = sum_k( U[i][k] * D[k] * U[j][k] )
    // Only need k >= max(i,j) since U is upper triangular.
    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = i; j < N; ++j) {
            float sum = 0.0f;
            // U[i][k] is nonzero for k >= i, U[j][k] for k >= j.
            // Since j >= i, iterate k from j to N-1.
            for (int32_t k = j; k < N; ++k) {
                sum += ud.U[i][k] * ud.D[k] * ud.U[j][k];
            }
            P[i][j] = sum;
            P[j][i] = sum;
        }
    }
}

bool ud_all_positive(const UD24& ud) {
    for (int32_t i = 0; i < N; ++i) {
        // Use !(D > 0) instead of (D <= 0) to catch NaN (NaN comparisons
        // return false, so NaN <= 0 is false — would slip through).
        if (!(ud.D[i] > 0.0f)) {
            return false;
        }
    }
    return true;
}

bool ud_factorize(UD24& ud, const float P[24][24]) {
    // Modified Cholesky: P = U * D * U^T (UDU^T decomposition).
    // Process columns from right to left.
    //
    // For j = N-1 down to 0:
    //   D[j] = P[j][j] - sum_{k=j+1}^{N-1} U[j][k]^2 * D[k]
    //   For i = 0 to j-1:
    //     U[i][j] = (P[i][j] - sum_{k=j+1}^{N-1} U[i][k]*D[k]*U[j][k]) / D[j]

    // Start with U = 0
    std::memset(ud.U, 0, sizeof(ud.U));

    for (int32_t j = N - 1; j >= 0; --j) {
        // Compute D[j]
        float dj = P[j][j];
        for (int32_t k = j + 1; k < N; ++k) {
            dj -= ud.U[j][k] * ud.U[j][k] * ud.D[k];
        }
        if (dj <= 0.0f) {
            return false;  // Not positive definite
        }
        ud.D[j] = dj;
        ud.U[j][j] = 1.0f;  // Unit diagonal

        // Compute U[i][j] for i < j
        const float inv_dj = 1.0f / dj;
        for (int32_t i = 0; i < j; ++i) {
            float uij = P[i][j];
            for (int32_t k = j + 1; k < N; ++k) {
                uij -= ud.U[i][k] * ud.D[k] * ud.U[j][k];
            }
            ud.U[i][j] = uij * inv_dj;
        }
    }
    return true;
}

// =========================================================================
// Thornton WMGS temporal update — float32 accumulators
//
// Algorithm: Ramos et al. (arXiv:2203.06105), G = I simplification.
// W = F * U, then modified weighted Gram-Schmidt from j=N-1 down to 0.
//
// SRAM placement for fair comparison with codegen FPFT.
// =========================================================================

// Static workspace — 2,304 bytes (LL Entry 1)
static float g_W[N][N];
// D_old snapshot — Thornton WMGS requires unmodified D values during sweep.
static float g_D_old[N];

__attribute__((section(".time_critical.thornton_f32")))
void thornton_f32(UD24& ud, const float F[24][24], const float Qd[24]) {
    // Step 1: W = F * U  (U is upper triangular: U[r][c] nonzero for r <= c)
    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = 0; j < N; ++j) {
            float sum = 0.0f;
            for (int32_t k = 0; k <= j; ++k) {
                sum += F[i][k] * ud.U[k][j];
            }
            g_W[i][j] = sum;
        }
    }

    // Snapshot D before in-place WMGS sweep.
    // The dj and c accumulators need the *old* D values for all k.
    // Without this, D[k] for k > j has already been overwritten.
    std::memcpy(g_D_old, ud.D, sizeof(g_D_old));

    // Step 2: Modified weighted Gram-Schmidt (Thornton)
    for (int32_t j = N - 1; j >= 0; --j) {
        float dj = Qd[j];
        for (int32_t k = 0; k < N; ++k) {
            dj += g_W[j][k] * g_W[j][k] * g_D_old[k];
        }
        ud.D[j] = dj;

        if (dj < 1e-30f) {
            for (int32_t i = 0; i < j; ++i) {
                ud.U[i][j] = 0.0f;
            }
            continue;
        }

        const float inv_dj = 1.0f / dj;

        for (int32_t i = 0; i < j; ++i) {
            float c = 0.0f;
            for (int32_t k = 0; k < N; ++k) {
                c += g_W[i][k] * g_D_old[k] * g_W[j][k];
            }

            ud.U[i][j] = c * inv_dj;

            const float uij = ud.U[i][j];
            for (int32_t k = 0; k < N; ++k) {
                g_W[i][k] -= uij * g_W[j][k];
            }
        }
    }

    for (int32_t i = 0; i < N; ++i) {
        ud.U[i][i] = 1.0f;
    }
}

// =========================================================================
// Thornton WMGS — mixed precision (double accumulators, float storage)
// =========================================================================

// Separate workspace to avoid aliasing concerns during parallel testing
static float g_W_mixed[N][N];
static float g_D_old_mixed[N];

__attribute__((section(".time_critical.thornton_mixed")))
void thornton_mixed(UD24& ud, const float F[24][24], const float Qd[24]) {
    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = 0; j < N; ++j) {
            float sum = 0.0f;
            for (int32_t k = 0; k <= j; ++k) {
                sum += F[i][k] * ud.U[k][j];
            }
            g_W_mixed[i][j] = sum;
        }
    }

    std::memcpy(g_D_old_mixed, ud.D, sizeof(g_D_old_mixed));

    for (int32_t j = N - 1; j >= 0; --j) {
        double dj = static_cast<double>(Qd[j]);
        for (int32_t k = 0; k < N; ++k) {
            dj += static_cast<double>(g_W_mixed[j][k])
                * static_cast<double>(g_W_mixed[j][k])
                * static_cast<double>(g_D_old_mixed[k]);
        }
        ud.D[j] = static_cast<float>(dj);

        if (dj < 1e-30) {
            for (int32_t i = 0; i < j; ++i) {
                ud.U[i][j] = 0.0f;
            }
            continue;
        }

        const double inv_dj = 1.0 / dj;

        for (int32_t i = 0; i < j; ++i) {
            double c = 0.0;
            for (int32_t k = 0; k < N; ++k) {
                c += static_cast<double>(g_W_mixed[i][k])
                   * static_cast<double>(g_D_old_mixed[k])
                   * static_cast<double>(g_W_mixed[j][k]);
            }
            ud.U[i][j] = static_cast<float>(c * inv_dj);

            const float uij = ud.U[i][j];
            for (int32_t k = 0; k < N; ++k) {
                g_W_mixed[i][k] -= uij * g_W_mixed[j][k];
            }
        }
    }

    for (int32_t i = 0; i < N; ++i) {
        ud.U[i][i] = 1.0f;
    }
}

// =========================================================================
// Thornton WMGS — float64 accumulators throughout inner loops
// =========================================================================

static float g_W_f64[N][N];
static float g_D_old_f64[N];

__attribute__((section(".time_critical.thornton_f64")))
void thornton_f64(UD24& ud, const float F[24][24], const float Qd[24]) {
    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = 0; j < N; ++j) {
            double sum = 0.0;
            for (int32_t k = 0; k <= j; ++k) {
                sum += static_cast<double>(F[i][k])
                     * static_cast<double>(ud.U[k][j]);
            }
            g_W_f64[i][j] = static_cast<float>(sum);
        }
    }

    std::memcpy(g_D_old_f64, ud.D, sizeof(g_D_old_f64));

    for (int32_t j = N - 1; j >= 0; --j) {
        double dj = static_cast<double>(Qd[j]);
        for (int32_t k = 0; k < N; ++k) {
            dj += static_cast<double>(g_W_f64[j][k])
                * static_cast<double>(g_W_f64[j][k])
                * static_cast<double>(g_D_old_f64[k]);
        }
        ud.D[j] = static_cast<float>(dj);

        if (dj < 1e-30) {
            for (int32_t i = 0; i < j; ++i) {
                ud.U[i][j] = 0.0f;
            }
            continue;
        }

        const double inv_dj = 1.0 / dj;

        for (int32_t i = 0; i < j; ++i) {
            double c = 0.0;
            for (int32_t k = 0; k < N; ++k) {
                c += static_cast<double>(g_W_f64[i][k])
                   * static_cast<double>(g_D_old_f64[k])
                   * static_cast<double>(g_W_f64[j][k]);
            }
            ud.U[i][j] = static_cast<float>(c * inv_dj);

            const float uij = ud.U[i][j];
            for (int32_t k = 0; k < N; ++k) {
                g_W_f64[i][k] -= uij * g_W_f64[j][k];
            }
        }
    }

    for (int32_t i = 0; i < N; ++i) {
        ud.U[i][i] = 1.0f;
    }
}

// =========================================================================
// Bierman scalar measurement update
//
// Exploits sparse H (single nonzero entry at hIdx with value hValue).
// f = U^T * h simplifies to reading column hIdx of U scaled by hValue.
//
// Bierman (1977), Grewal & Andrews (2015) Chapter 6.
// =========================================================================

// Bierman workspace — static to avoid stack pressure (LL Entry 1).
static float g_bf[N];
static float g_bg[N];
static float g_bK[N];
static float g_balpha[N];

// Compute f = U^T * H^T and g = D * f for sparse H with single nonzero
// entry at hIdx.  f[i] = U[hIdx][i] * hValue for i >= hIdx, else 0.
// See Bierman (1977): U is upper triangular, H^T is column with hValue
// at index hIdx.
static void bierman_compute_fg(const UD24& ud, int32_t hIdx, float hValue) {
    for (int32_t i = 0; i < N; ++i) {
        g_bf[i] = (i >= hIdx) ? ud.U[hIdx][i] * hValue : 0.0f;
        g_bg[i] = ud.D[i] * g_bf[i];
    }
}

// Forward pass: update U, D, and build unnormalized gain K.
static void bierman_forward_pass(UD24& ud, float r) {
    g_balpha[0] = r + g_bf[0] * g_bg[0];
    g_bK[0] = g_bg[0];

    for (int32_t j = 1; j < N; ++j) {
        g_balpha[j] = g_balpha[j - 1] + g_bf[j] * g_bg[j];

        if (g_balpha[j - 1] < 1e-30f) {
            g_bK[j] = g_bg[j];
            continue;
        }

        const float lambda = -g_bf[j] / g_balpha[j - 1];
        ud.D[j] = ud.D[j] * g_balpha[j - 1] / g_balpha[j];

        for (int32_t i = 0; i < j; ++i) {
            const float u_save = ud.U[i][j];
            ud.U[i][j] = u_save + lambda * g_bK[i];
            g_bK[i] = g_bK[i] + g_bg[j] * u_save;
        }
        g_bK[j] = g_bg[j];
    }

    // D[0] update (missed in loop above which starts at j=1)
    if (g_balpha[0] > 1e-30f) {
        ud.D[0] = ud.D[0] * r / g_balpha[0];
    }
}

__attribute__((section(".time_critical.bierman")))
void bierman_scalar_update(UD24& ud, int32_t hIdx, float hValue,
                           float innovation, float r, float dx[24]) {
    bierman_compute_fg(ud, hIdx, hValue);
    bierman_forward_pass(ud, r);

    // Normalize gain and compute error state correction
    const float alpha_last = g_balpha[N - 1];
    if (alpha_last > 1e-30f) {
        const float inv_alpha = 1.0f / alpha_last;
        for (int32_t i = 0; i < N; ++i) {
            g_bK[i] *= inv_alpha;
            dx[i] = g_bK[i] * innovation;
        }
    } else {
        for (int32_t i = 0; i < N; ++i) {
            dx[i] = 0.0f;
        }
    }
}

} // namespace rc
