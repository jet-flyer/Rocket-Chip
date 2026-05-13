// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_CALIBRATION_LM_SOLVER_H
#define ROCKETCHIP_CALIBRATION_LM_SOLVER_H

// Levenberg-Marquardt solver used by mag sphere-fit and ellipsoid-fit
// calibration. Pure-function module: all working state (samples, JtJ buffer,
// inverse buffer) is passed in by the caller — no file-scope globals. This
// is what makes the solver host-testable in isolation.
//
// Extracted from calibration_manager.cpp during R-6c of the 2026-05-07
// master standards audit. Two goals:
//   1. Replace function-pointer dispatch (P10 Rule 9 deviation FP-1) with
//      template dispatch. Same machine code, no pointer-to-function in scope.
//   2. Separate algorithm from state so the math is unit-testable.

#include <math.h>
#include <stdint.h>
#include <string.h>

// LM damping constants (ArduPilot + council consensus 2026-02-10).
constexpr float kMagLmLambdaInit = 1.0F;
constexpr float kMagLmLambdaUp   = 10.0F;
constexpr float kMagLmLambdaDown = 0.1F;

// Largest param vector across all callers (mag ellipsoid = 9, sphere = 4,
// accel 6-pos = 6). Stack buffers inside the solver size to this.
constexpr uint8_t kLmMaxParams = 9;

// Linear-algebra primitives (defined in lm_solver.cpp). Public so both the
// solver templates and the host tests can reach them.
bool mat_inverse(const float* src, float* dst, uint8_t n);
bool lm_compute_step(const float* params, float* newParams,
                     const float* jtjInv, const float* jtfi, uint8_t numParams);

// Compute mean squared residuals over samples[0..numSamples-1].
template <typename ResFn>
float lm_mean_sq_residuals(const float (*samples)[3], uint16_t numSamples,
                           const float* params, ResFn residualFn) {
    float sum = 0.0F;
    for (uint16_t i = 0; i < numSamples; i++) {
        float r = residualFn(samples[i], params);
        sum += r * r;
    }
    return sum / static_cast<float>(numSamples);
}

// Accumulate J^T*J and J^T*residual for one LM iteration.
// Writes to caller-owned jtj (numParams x numParams) and jtfi (numParams).
template <typename ResFn, typename JacFn>
void lm_accumulate_jtj(const float (*samples)[3], uint16_t numSamples,
                       const float* params, uint8_t numParams,
                       float* jtj, float* jtfi,
                       ResFn residualFn, JacFn jacobianFn) {
    float jacob[kLmMaxParams];
    memset(jtj, 0, numParams * numParams * sizeof(float));
    memset(jtfi, 0, numParams * sizeof(float));
    for (uint16_t i = 0; i < numSamples; i++) {
        float r = residualFn(samples[i], params);
        jacobianFn(samples[i], params, jacob);
        for (uint8_t row = 0; row < numParams; row++) {
            jtfi[row] += jacob[row] * r;
            for (uint8_t col = 0; col < numParams; col++) {
                jtj[row * numParams + col] += jacob[row] * jacob[col];
            }
        }
    }
}

// Run LM iterations on params[0..numParams-1] using samples[0..numSamples-1].
// jtj and jtjInv are caller-owned scratch buffers sized for numParams x numParams.
// On return, bestParams holds the best fit and *bestFitness holds RMS^2.
// Template parameters: ResFn = float(const float[3], const float*),
//                      JacFn = void (const float[3], const float*, float*).
template <typename ResFn, typename JacFn>
void lm_solve(const float (*samples)[3], uint16_t numSamples,
              float* params, float* bestParams, float* bestFitness,
              uint8_t numParams, uint8_t maxIter,
              float* jtj, float* jtjInv,
              ResFn residualFn, JacFn jacobianFn) {
    float lambda = kMagLmLambdaInit;
    float jtfi[kLmMaxParams];
    for (uint8_t iter = 0; iter < maxIter; iter++) {
        lm_accumulate_jtj(samples, numSamples, params, numParams,
                          jtj, jtfi, residualFn, jacobianFn);

        for (uint8_t d = 0; d < numParams; d++) {
            jtj[d * numParams + d] += lambda;
        }
        if (!mat_inverse(jtj, jtjInv, numParams)) { break; }

        float newParams[kLmMaxParams];
        if (!lm_compute_step(params, newParams, jtjInv, jtfi, numParams)) { break; }

        float fitness = lm_mean_sq_residuals(samples, numSamples, newParams, residualFn);
        if (fitness < *bestFitness) {
            *bestFitness = fitness;
            memcpy(bestParams, newParams, numParams * sizeof(float));
            memcpy(params, newParams, numParams * sizeof(float));
            lambda *= kMagLmLambdaDown;
        } else {
            lambda *= kMagLmLambdaUp;
        }
    }
}

#endif  // ROCKETCHIP_CALIBRATION_LM_SOLVER_H
