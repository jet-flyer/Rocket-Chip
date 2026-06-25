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
bool forward_eliminate(float* aug, uint8_t n, uint8_t aug_width) {
    for (uint8_t col = 0; col < n; col++) {
        uint8_t max_row = col;
        float   max_val = fabsf(aug[col * aug_width + col]);
        for (uint8_t r = col + 1; r < n; r++) {
            float val = fabsf(aug[r * aug_width + col]);
            if (val > max_val) {
                max_val = val;
                max_row = r;
            }
        }
        if (max_val < kSingularityThreshold) {
            return false;
        }
        if (max_row != col) {
            for (uint8_t c = 0; c < aug_width; c++) {
                float tmp = aug[col * aug_width + c];
                aug[col * aug_width + c]    = aug[max_row * aug_width + c];
                aug[max_row * aug_width + c] = tmp;
            }
        }
        float pivot = aug[col * aug_width + col];
        for (uint8_t r = col + 1; r < n; r++) {
            float factor = aug[r * aug_width + col] / pivot;
            for (uint8_t c = col; c < aug_width; c++) {
                aug[r * aug_width + c] -= factor * aug[col * aug_width + c];
            }
        }
    }
    return true;
}

void back_substitute(float* aug, uint8_t n, uint8_t aug_width) {
    for (int8_t col = static_cast<int8_t>(n - 1); col >= 0; col--) {
        float pivot = aug[col * aug_width + col];
        for (uint8_t c = 0; c < aug_width; c++) {
            aug[col * aug_width + c] /= pivot;
        }
        for (int8_t r = static_cast<int8_t>(col - 1); r >= 0; r--) {
            float factor = aug[r * aug_width + col];
            for (uint8_t c = 0; c < aug_width; c++) {
                aug[r * aug_width + c] -= factor * aug[col * aug_width + c];
            }
        }
    }
}

}  // namespace

bool mat_inverse(const float* src, float* dst, uint8_t n) {
    if (n > kMaxMatDim) {
        return false;
    }
    uint8_t aug_width = n * 2;
    static float g_aug[kMaxMatDim * kMaxAugWidth];

    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            g_aug[r * aug_width + c]     = src[r * n + c];
            g_aug[r * aug_width + c + n] = (r == c) ? 1.0F : 0.0F;
        }
    }
    if (!forward_eliminate(g_aug, n, aug_width)) {
        return false;
    }
    back_substitute(g_aug, n, aug_width);
    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            dst[r * n + c] = g_aug[r * aug_width + c + n];
        }
    }
    return true;
}

bool lm_compute_step(const float* params, float* new_params,
                     const float* jtj_inv, const float* jtfi, uint8_t num_params) {
    for (uint8_t i = 0; i < num_params; i++) {
        float delta = 0.0F;
        for (uint8_t j = 0; j < num_params; j++) {
            delta += jtj_inv[i * num_params + j] * jtfi[j];
        }
        new_params[i] = params[i] - delta;
        if (isnan(new_params[i]) || isinf(new_params[i])) { return false; }
    }
    return true;
}
