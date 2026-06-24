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

// Minimum D-element guard for numerical stability.
// Below this threshold, the U column is zeroed to avoid division by near-zero.
static constexpr float  kMinDFloat  = 1e-30F;

// =========================================================================
// Utility functions
// =========================================================================

void ud_to_dense(const UD24& ud, float P[24][24]) {  // NOLINT(readability-magic-numbers)
    // P = U * D * U^T
    // P[i][j] = sum_k( U[i][k] * D[k] * U[j][k] )
    // Only need k >= max(i,j) since U is upper triangular.
    for (int32_t i = 0; i < N; ++i) {
        for (int32_t j = i; j < N; ++j) {
            float sum = 0.0F;
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

bool ud_factorize(UD24& ud, const float P[24][24]) {  // NOLINT(readability-magic-numbers)
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
        if (dj <= 0.0F) {
            return false;  // Not positive definite
        }
        ud.D[j] = dj;
        ud.U[j][j] = 1.0F;  // Unit diagonal

        // Compute U[i][j] for i < j
        const float inv_dj = 1.0F / dj;
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
        g_bf[i] = (i >= hIdx) ? ud.U[hIdx][i] * hValue : 0.0F;
        g_bg[i] = ud.D[i] * g_bf[i];
    }
}

// Forward pass: update U, D, and build unnormalized gain K.
static void bierman_forward_pass(UD24& ud, float r) {
    g_balpha[0] = r + g_bf[0] * g_bg[0];
    g_bK[0] = g_bg[0];

    for (int32_t j = 1; j < N; ++j) {
        g_balpha[j] = g_balpha[j - 1] + g_bf[j] * g_bg[j];

        if (g_balpha[j - 1] < kMinDFloat) {
            g_bK[j] = g_bg[j];
        } else {
            const float lambda = -g_bf[j] / g_balpha[j - 1];
            ud.D[j] = ud.D[j] * g_balpha[j - 1] / g_balpha[j];

            for (int32_t i = 0; i < j; ++i) {
                const float u_save = ud.U[i][j];
                ud.U[i][j] = u_save + lambda * g_bK[i];
                g_bK[i] = g_bK[i] + g_bg[j] * u_save;
            }
            g_bK[j] = g_bg[j];
        }
    }

    // D[0] update (missed in loop above which starts at j=1)
    if (g_balpha[0] > 1e-30F) {
        ud.D[0] = ud.D[0] * r / g_balpha[0];
    }
}

__attribute__((section(".time_critical.bierman")))
void bierman_scalar_update(UD24& ud, int32_t hIdx, float hValue,
                           float innovation, float r, float dx[24]) {  // NOLINT(readability-magic-numbers)
    bierman_compute_fg(ud, hIdx, hValue);
    bierman_forward_pass(ud, r);

    // Normalize gain and compute error state correction
    const float alpha_last = g_balpha[N - 1];
    if (alpha_last > 1e-30F) {
        const float inv_alpha = 1.0F / alpha_last;
        for (int32_t i = 0; i < N; ++i) {
            g_bK[i] *= inv_alpha;
            dx[i] = g_bK[i] * innovation;
        }
    } else {
        for (int32_t i = 0; i < N; ++i) {
            dx[i] = 0.0F;
        }
    }
}

} // namespace rc
