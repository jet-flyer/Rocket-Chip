// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_FUSION_UD_FACTOR_H
#define ROCKETCHIP_FUSION_UD_FACTOR_H

// UD factorization for 24-state ESKF covariance matrix.
// P = U * D * U^T where U is unit upper triangular, D is diagonal.
//
// Maintains positive-definiteness by construction (all D[i] > 0).
//
// Algorithms:
//   Thornton WMGS temporal update — Ramos et al. (arXiv:2203.06105)
//   Bierman scalar measurement update — Bierman (1977)
//
// Titan-tier investigation — current Core architecture (codegen FPFT + Joseph)
// is adequate for hobby rockets. UD is for certification paths (DO-178C).

#include <cstdint>

namespace rc {

// 24×24 UD factored covariance.
// U: unit upper triangular (diagonal = 1, lower triangle = 0, upper stored).
// D: diagonal vector (24 floats).
// Total storage: 2,400 bytes (vs 2,304 for dense P).
struct UD24 {
    float U[24][24];  // Unit upper triangular
    float D[24];      // Diagonal
};

// Initialize UD from a diagonal matrix (U = I, D = diag).
// Used for ESKF init where P starts diagonal.
void ud_from_diagonal(UD24& ud, const float diag[24]);

// Reconstruct dense P = U * D * U^T.
// Output: P[24][24] (symmetric).
void ud_to_dense(const UD24& ud, float P[24][24]);

// Check all D[i] > 0 (positive-definiteness).
bool ud_all_positive(const UD24& ud);

// Factorize dense symmetric P into UD form (modified Cholesky).
// Used for hybrid codegen+Bierman path: codegen updates dense P,
// then factorize into UD for Bierman measurement update.
// Returns false if P is not positive-definite (any D[i] <= 0).
bool ud_factorize(UD24& ud, const float P[24][24]);

// =========================================================================
// Thornton WMGS temporal update: propagate U,D through F and Q_d.
// P_new = F * P * F^T + G * Q_d * G^T  (G = I simplification)
//
// Three precision variants — differ only in inner-loop accumulator type.
// All modify ud in-place.
//
// F: 24×24 state transition matrix (from ESKF::build_F).
// Qd: 24-element diagonal of discrete process noise (Q_c * dt).
// =========================================================================

void thornton_f32(UD24& ud, const float F[24][24], const float Qd[24]);
void thornton_mixed(UD24& ud, const float F[24][24], const float Qd[24]);
void thornton_f64(UD24& ud, const float F[24][24], const float Qd[24]);

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
                           float innovation, float r, float dx[24]);

} // namespace rc

#endif // ROCKETCHIP_FUSION_UD_FACTOR_H
