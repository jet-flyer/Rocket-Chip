// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_UD_FACTOR_H
#define ROCKETCHIP_FUSION_UD_FACTOR_H

// UD factorization for 24-state ESKF covariance matrix.
// P = U * D * U^T where U is unit upper triangular, D is diagonal.
//
// Maintains positive-definiteness by construction (all D[i] > 0).
//
// Production path: codegen FPFT (predict) + Bierman scalar measurement
// update (Bierman 1977). Phase-1 benchmark concluded codegen+Bierman is
// numerically stable and faster than Thornton WMGS alternatives — see
// CHANGELOG 2026 entry "UD factorization + DCP float64 benchmark."

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

// =========================================================================
// Bierman scalar measurement update.
// Updates U,D in-place from a single scalar measurement.
//
// hIdx: index of the single non-zero H entry.
// hValue: value of H at that entry (+1.0 or -1.0).
// innovation: measurement residual (z - h(x)).
// r: scalar measurement noise variance.
// dx[24]: output error-state correction vector.
// =========================================================================

void bierman_scalar_update(UD24& ud, int32_t hIdx, float hValue,
                           float innovation, float r, float dx[eskf::kStateSize]);

} // namespace rc

#endif // ROCKETCHIP_FUSION_UD_FACTOR_H
