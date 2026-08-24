// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_UD_FACTOR_H
#define ROCKETCHIP_FUSION_UD_FACTOR_H

// 24-state UD: P = U D U^T. Factorize can fail mid-write (D<=0).
// Bierman floors tiny D or skips a column; D>0 is not guaranteed after every call.

#include <cstdint>

#include "fusion/eskf_state.h"

namespace rc {

// 24×24 UD factored covariance (dimension = eskf::kStateSize).
// U: unit upper triangular (diagonal = 1, lower triangle = 0, upper stored).
// D: diagonal vector.
// Total storage: 2,400 bytes (vs 2,304 for dense P).
struct UD24 {
    float U[eskf::kStateSize][eskf::kStateSize];  // Unit upper triangular
    float D[eskf::kStateSize];                    // Diagonal
};

// Reconstruct dense P = U * D * U^T.
void ud_to_dense(const UD24& ud, float P[eskf::kStateSize][eskf::kStateSize]);

// Factorize dense symmetric P into UD form (modified Cholesky).
// Used for hybrid codegen+Bierman path: codegen updates dense P,
// then factorize into UD for Bierman measurement update.
// Returns false if P is not positive-definite (any D[i] <= 0).
bool ud_factorize(UD24& ud, const float P[eskf::kStateSize][eskf::kStateSize]);

// In-place scalar update. H has one nonzero at hIdx in [0, kStateSize).
// Core 0 only. Out-of-range hIdx is a no-op (debug abort).

void bierman_scalar_update(UD24& ud, int32_t hIdx, float hValue,
                           float innovation, float r, float dx[eskf::kStateSize]);

} // namespace rc

#endif // ROCKETCHIP_FUSION_UD_FACTOR_H
