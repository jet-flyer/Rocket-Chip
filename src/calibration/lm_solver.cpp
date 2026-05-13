// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Linear-algebra primitives for the LM solver. See lm_solver.h for the
// template-based solver itself.

#include "lm_solver.h"

namespace {

// Matrix-inverse working buffer: largest case is 9x18 augmented [src | I].
constexpr uint8_t kMaxMatDim   = 9;
constexpr uint8_t kMaxAugWidth = kMaxMatDim * 2;
constexpr float   kSingularityThreshold = 1e-10F;

// Forward elimination with partial pivoting on flat augmented matrix [A|I].
bool forward_eliminate(float* aug, uint8_t n, uint8_t augWidth) {
    for (uint8_t col = 0; col < n; col++) {
        uint8_t maxRow = col;
        float   maxVal = fabsf(aug[col * augWidth + col]);
        for (uint8_t r = col + 1; r < n; r++) {
            float val = fabsf(aug[r * augWidth + col]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = r;
            }
        }
        if (maxVal < kSingularityThreshold) {
            return false;
        }
        if (maxRow != col) {
            for (uint8_t c = 0; c < augWidth; c++) {
                float tmp = aug[col * augWidth + c];
                aug[col * augWidth + c]    = aug[maxRow * augWidth + c];
                aug[maxRow * augWidth + c] = tmp;
            }
        }
        float pivot = aug[col * augWidth + col];
        for (uint8_t r = col + 1; r < n; r++) {
            float factor = aug[r * augWidth + col] / pivot;
            for (uint8_t c = col; c < augWidth; c++) {
                aug[r * augWidth + c] -= factor * aug[col * augWidth + c];
            }
        }
    }
    return true;
}

void back_substitute(float* aug, uint8_t n, uint8_t augWidth) {
    for (int8_t col = static_cast<int8_t>(n - 1); col >= 0; col--) {
        float pivot = aug[col * augWidth + col];
        for (uint8_t c = 0; c < augWidth; c++) {
            aug[col * augWidth + c] /= pivot;
        }
        for (int8_t r = static_cast<int8_t>(col - 1); r >= 0; r--) {
            float factor = aug[r * augWidth + col];
            for (uint8_t c = 0; c < augWidth; c++) {
                aug[r * augWidth + c] -= factor * aug[col * augWidth + c];
            }
        }
    }
}

}  // namespace

bool mat_inverse(const float* src, float* dst, uint8_t n) {
    if (n > kMaxMatDim) {
        return false;
    }
    uint8_t augWidth = n * 2;
    static float g_aug[kMaxMatDim * kMaxAugWidth];

    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            g_aug[r * augWidth + c]     = src[r * n + c];
            g_aug[r * augWidth + c + n] = (r == c) ? 1.0F : 0.0F;
        }
    }
    if (!forward_eliminate(g_aug, n, augWidth)) {
        return false;
    }
    back_substitute(g_aug, n, augWidth);
    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            dst[r * n + c] = g_aug[r * augWidth + c + n];
        }
    }
    return true;
}

bool lm_compute_step(const float* params, float* newParams,
                     const float* jtjInv, const float* jtfi, uint8_t numParams) {
    for (uint8_t i = 0; i < numParams; i++) {
        float delta = 0.0F;
        for (uint8_t j = 0; j < numParams; j++) {
            delta += jtjInv[i * numParams + j] * jtfi[j];
        }
        newParams[i] = params[i] - delta;
        if (isnan(newParams[i]) || isinf(newParams[i])) { return false; }
    }
    return true;
}
