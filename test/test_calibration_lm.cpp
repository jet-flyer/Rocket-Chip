// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Host regression coverage for the Levenberg-Marquardt solver used by mag
// calibration (src/calibration/lm_solver.{h,cpp}).
//
// Context: R-6c of the 2026-05-07 master standards audit eliminated the
// FP-1 deviation (function-pointer dispatch vs P10 Rule 9) by extracting the
// LM math into a pure-function module that takes its samples + JtJ buffers
// as explicit parameters and dispatches residual/jacobian via template
// instantiation. Before R-6c, this code had ZERO host-side coverage — the
// only proving ground was a hand-graded bench mag-cal cycle.
//
// Council review (JPL + ArduPilot + Professor, 2026-05-13) directed adding
// this regression as part of R-6c per HW_GATE_DISCIPLINE Rule 7 (the
// refactor's verification path depended on infrastructure that didn't
// exist). Scope: smoke + iteration-cap + degenerate-input guard, with
// deterministic seeds so a future agent can't quietly change the test to
// mask a regression.
//
// **Tolerances in this file are part of the FP-1 -> P10-Rule-9 verification
// chain.** If you change a tolerance, the rationale lives in
// ACCEPTED_STANDARDS_DEVIATIONS.md's Resolved section + the dated audit
// report's `## Remediation` section. Update both before bumping numbers.

#include <gtest/gtest.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "src/calibration/lm_solver.h"

namespace {

// Determinstic sample-generation seed. NAMED so a future agent can't quietly
// change it. If you do change it, capture a fresh baseline + document why
// in the test file header.
constexpr uint32_t kSampleSeed = 0xC0FFEE1AU;

// Tolerances (council Amendment 1 + 2 + 3). Tight enough to catch a
// regression in the LM iteration; loose enough to absorb float-reordering
// from the template-dispatch refactor. The R-6c bytes-on-wire identity
// claim is that the recovered params under template dispatch match the
// pre-refactor function-pointer dispatch to within these tolerances.
constexpr float kSphereOffsetTol  = 1e-3F;  // µT
constexpr float kSphereRadiusTol  = 1e-3F;  // µT
constexpr float kEllipsoidOffsetTol = 1e-2F;  // µT
constexpr float kEllipsoidDiagTol   = 1e-2F;  // unitless scale
constexpr float kEllipsoidOffdiagTol = 1e-3F;
constexpr float kRmsResidualMax     = 1.0F;   // µT — gross sanity

// Mag calibration constants (mirror calibration_manager.cpp's view; these are
// the parameters the LM solver is configured for).
constexpr uint8_t  kMagSphereParams      = 4;
constexpr uint8_t  kMagEllipsoidParams   = 9;
constexpr uint8_t  kMagSphereIterations  = 10;
constexpr uint8_t  kMagEllipsoidIterations = 20;
constexpr float    kMinVectorLength      = 1e-6F;
constexpr float    kMagExpectedRadius    = 50.0F;  // µT (typical Earth field)

// --- Residual + Jacobian functors (copies of the production code paths,
//     intentionally duplicated so the test pins their behavior). ---

float calc_sphere_residual(const float sample[3], const float* params) {
    float sx = sample[0] + params[1];
    float sy = sample[1] + params[2];
    float sz = sample[2] + params[3];
    float len = sqrtf(sx*sx + sy*sy + sz*sz);
    return params[0] - len;
}

void calc_sphere_jacobian(const float sample[3], const float* params, float* jacob) {
    float sx = sample[0] + params[1];
    float sy = sample[1] + params[2];
    float sz = sample[2] + params[3];
    float len = sqrtf(sx*sx + sy*sy + sz*sz);
    if (len < kMinVectorLength) {
        memset(jacob, 0, kMagSphereParams * sizeof(float));
        return;
    }
    jacob[0] = 1.0F;
    jacob[1] = -sx / len;
    jacob[2] = -sy / len;
    jacob[3] = -sz / len;
}

float calc_residual_mag(const float sample[3], const float* params) {
    float sx = sample[0] + params[0];
    float sy = sample[1] + params[1];
    float sz = sample[2] + params[2];
    float a = params[3] * sx + params[6] * sy + params[7] * sz;
    float b = params[6] * sx + params[4] * sy + params[8] * sz;
    float c = params[7] * sx + params[8] * sy + params[5] * sz;
    float len = sqrtf(a*a + b*b + c*c);
    return kMagExpectedRadius - len;
}

void calc_jacobian_mag(const float sample[3], const float* params, float* jacob) {
    float sx = sample[0] + params[0];
    float sy = sample[1] + params[1];
    float sz = sample[2] + params[2];
    float a = params[3] * sx + params[6] * sy + params[7] * sz;
    float b = params[6] * sx + params[4] * sy + params[8] * sz;
    float c = params[7] * sx + params[8] * sy + params[5] * sz;
    float len = sqrtf(a*a + b*b + c*c);
    if (len < kMinVectorLength) {
        memset(jacob, 0, kMagEllipsoidParams * sizeof(float));
        return;
    }
    jacob[0] = -((params[3]*a + params[6]*b + params[7]*c) / len);
    jacob[1] = -((params[6]*a + params[4]*b + params[8]*c) / len);
    jacob[2] = -((params[7]*a + params[8]*b + params[5]*c) / len);
    jacob[3] = -(sx * a / len);
    jacob[4] = -(sy * b / len);
    jacob[5] = -(sz * c / len);
    jacob[6] = -((sy*a + sx*b) / len);
    jacob[7] = -((sz*a + sx*c) / len);
    jacob[8] = -((sz*b + sy*c) / len);
}

// --- Sample generator (Fibonacci-sphere distribution + applied distortion). ---

struct Rng {
    uint32_t state;
    explicit Rng(uint32_t seed) : state(seed) {}
    float next01() {
        state = state * 1664525U + 1013904223U;
        return static_cast<float>(state) / static_cast<float>(UINT32_MAX);
    }
};

// Generate `n` points on a unit sphere, scale to radius `r`, apply linear
// distortion `M`, then add bias `offset`. The recovered LM params should
// invert that distortion and bias.
void generate_distorted_sphere(float (*out)[3], uint16_t n,
                                float radius,
                                const float offset[3],
                                const float M[9],
                                Rng& rng,
                                float noise = 0.0F) {
    const float golden = 3.14159265358979323846F * (3.0F - sqrtf(5.0F));
    for (uint16_t i = 0; i < n; i++) {
        float y = 1.0F - (static_cast<float>(i) / static_cast<float>(n - 1)) * 2.0F;
        float r = sqrtf(1.0F - y * y);
        float theta = golden * static_cast<float>(i);
        float ux = cosf(theta) * r;
        float uy = y;
        float uz = sinf(theta) * r;

        float sx = radius * ux;
        float sy = radius * uy;
        float sz = radius * uz;

        // Apply linear distortion M (3x3, row-major).
        float dx = M[0]*sx + M[1]*sy + M[2]*sz;
        float dy = M[3]*sx + M[4]*sy + M[5]*sz;
        float dz = M[6]*sx + M[7]*sy + M[8]*sz;

        // Add bias offset (the LM solver should recover -offset).
        dx -= offset[0];
        dy -= offset[1];
        dz -= offset[2];

        if (noise > 0.0F) {
            dx += (rng.next01() * 2.0F - 1.0F) * noise;
            dy += (rng.next01() * 2.0F - 1.0F) * noise;
            dz += (rng.next01() * 2.0F - 1.0F) * noise;
        }

        out[i][0] = dx;
        out[i][1] = dy;
        out[i][2] = dz;
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Smoke: sphere fit recovers known offset on noise-free input
// ---------------------------------------------------------------------------
TEST(LmSolver, SphereFitRecoversKnownOffset) {
    constexpr uint16_t kN = 300;
    float samples[kN][3];
    const float gtRadius = 50.0F;
    const float gtOffset[3] = {0.1F, -0.05F, 0.2F};
    const float identity[9] = {1,0,0, 0,1,0, 0,0,1};

    Rng rng(kSampleSeed);
    generate_distorted_sphere(samples, kN, gtRadius, gtOffset, identity, rng);

    float params[kMagSphereParams] = {gtRadius * 0.9F, 0, 0, 0};
    float bestParams[kMagSphereParams];
    memcpy(bestParams, params, sizeof(params));
    float bestFitness = lm_mean_sq_residuals(samples, kN, params, calc_sphere_residual);

    float jtj[kMagSphereParams * kMagSphereParams];
    float jtjInv[kMagSphereParams * kMagSphereParams];

    lm_solve(samples, kN, params, bestParams, &bestFitness,
             kMagSphereParams, kMagSphereIterations,
             jtj, jtjInv,
             calc_sphere_residual, calc_sphere_jacobian);

    EXPECT_NEAR(bestParams[0], gtRadius,    kSphereRadiusTol);
    EXPECT_NEAR(bestParams[1], gtOffset[0], kSphereOffsetTol);
    EXPECT_NEAR(bestParams[2], gtOffset[1], kSphereOffsetTol);
    EXPECT_NEAR(bestParams[3], gtOffset[2], kSphereOffsetTol);
    EXPECT_LT(sqrtf(bestFitness), kRmsResidualMax);
}

// ---------------------------------------------------------------------------
// Smoke: ellipsoid fit recovers diagonal scale + small off-diagonal
// ---------------------------------------------------------------------------
//
// Generator convention: the hardware applies a forward distortion
// `raw = M_hw * sphere_point - bias`. The residual evaluates
// `M_params * (sample + offset_params) ≈ radius`, so the LM should recover:
//   - `offset_params  = +bias`
//   - `M_params       = M_hw^-1`
//
// To keep assertions readable, choose `M_hw` whose inverse is exactly
// known: a pure diagonal `M_hw = diag(d0, d1, d2)` has `M_hw^-1 = diag(1/d0, 1/d1, 1/d2)`.
TEST(LmSolver, EllipsoidFitRecoversDiagonalAndOffdiagonal) {
    constexpr uint16_t kN = 300;
    float samples[kN][3];
    const float gtRadius = kMagExpectedRadius;
    const float gtOffset[3] = {0.5F, -0.3F, 0.7F};
    // Pure diagonal: M_hw[i,i] = 1/expected[i]. Solver should recover expected[i].
    const float gtDiag[3] = {0.95F, 1.05F, 0.98F};
    const float gtM[9] = {
        1.0F / gtDiag[0], 0.0F,             0.0F,
        0.0F,             1.0F / gtDiag[1], 0.0F,
        0.0F,             0.0F,             1.0F / gtDiag[2]
    };

    Rng rng(kSampleSeed);
    generate_distorted_sphere(samples, kN, gtRadius, gtOffset, gtM, rng);

    // Seed at identity (typical production usage seeds offset from sphere fit).
    float params[kMagEllipsoidParams] = {
        0.0F, 0.0F, 0.0F,  // offset
        1.0F, 1.0F, 1.0F,  // diag
        0.0F, 0.0F, 0.0F   // offdiag
    };
    float bestParams[kMagEllipsoidParams];
    memcpy(bestParams, params, sizeof(params));
    float bestFitness = lm_mean_sq_residuals(samples, kN, params, calc_residual_mag);

    float jtj[kMagEllipsoidParams * kMagEllipsoidParams];
    float jtjInv[kMagEllipsoidParams * kMagEllipsoidParams];

    lm_solve(samples, kN, params, bestParams, &bestFitness,
             kMagEllipsoidParams, kMagEllipsoidIterations,
             jtj, jtjInv,
             calc_residual_mag, calc_jacobian_mag);

    // Offset recovers to +bias.
    EXPECT_NEAR(bestParams[0], gtOffset[0], kEllipsoidOffsetTol);
    EXPECT_NEAR(bestParams[1], gtOffset[1], kEllipsoidOffsetTol);
    EXPECT_NEAR(bestParams[2], gtOffset[2], kEllipsoidOffsetTol);

    // Diagonal recovers to gtDiag (which is M_hw^-1).
    EXPECT_NEAR(bestParams[3], gtDiag[0], kEllipsoidDiagTol);
    EXPECT_NEAR(bestParams[4], gtDiag[1], kEllipsoidDiagTol);
    EXPECT_NEAR(bestParams[5], gtDiag[2], kEllipsoidDiagTol);

    // Pure-diagonal generator => offdiag should stay near zero.
    EXPECT_LT(fabsf(bestParams[6]), kEllipsoidOffdiagTol);
    EXPECT_LT(fabsf(bestParams[7]), kEllipsoidOffdiagTol);
    EXPECT_LT(fabsf(bestParams[8]), kEllipsoidOffdiagTol);

    EXPECT_LT(sqrtf(bestFitness), kRmsResidualMax);
}

// ---------------------------------------------------------------------------
// Iteration-cap guard: solver must NOT exceed maxIter (no infinite loop)
// ---------------------------------------------------------------------------
TEST(LmSolver, RespectsIterationCap) {
    // Pathological: feed mostly-degenerate input that won't converge, verify
    // we exit by maxIter cap rather than spinning. Use 2 iterations as a
    // tight cap that's well below normal sphere convergence (~10).
    constexpr uint16_t kN = 64;
    float samples[kN][3];
    Rng rng(kSampleSeed);
    // All samples on a small patch — sphere fit is under-constrained but
    // shouldn't NaN out.
    for (uint16_t i = 0; i < kN; i++) {
        samples[i][0] = 1.0F + rng.next01() * 0.01F;
        samples[i][1] = rng.next01() * 0.01F;
        samples[i][2] = rng.next01() * 0.01F;
    }

    float params[kMagSphereParams] = {1.0F, 0, 0, 0};
    float bestParams[kMagSphereParams];
    memcpy(bestParams, params, sizeof(params));
    float bestFitness = lm_mean_sq_residuals(samples, kN, params, calc_sphere_residual);
    float jtj[kMagSphereParams * kMagSphereParams];
    float jtjInv[kMagSphereParams * kMagSphereParams];

    const uint8_t kTightCap = 2;
    lm_solve(samples, kN, params, bestParams, &bestFitness,
             kMagSphereParams, kTightCap,
             jtj, jtjInv,
             calc_sphere_residual, calc_sphere_jacobian);

    // If the solver respected the cap, this test exits (it ran). No NaN/inf
    // in the output is the implicit assertion — gtest catches asserts mid-run.
    for (uint8_t i = 0; i < kMagSphereParams; i++) {
        EXPECT_FALSE(isnan(bestParams[i]));
        EXPECT_FALSE(isinf(bestParams[i]));
    }
}

// ---------------------------------------------------------------------------
// Degenerate input: all-zero samples shouldn't crash or produce NaN
// ---------------------------------------------------------------------------
TEST(LmSolver, DegenerateAllZerosGracefullyTerminates) {
    constexpr uint16_t kN = 32;
    float samples[kN][3];
    memset(samples, 0, sizeof(samples));

    float params[kMagSphereParams] = {1.0F, 0, 0, 0};
    float bestParams[kMagSphereParams];
    memcpy(bestParams, params, sizeof(params));
    float bestFitness = lm_mean_sq_residuals(samples, kN, params, calc_sphere_residual);
    float jtj[kMagSphereParams * kMagSphereParams];
    float jtjInv[kMagSphereParams * kMagSphereParams];

    lm_solve(samples, kN, params, bestParams, &bestFitness,
             kMagSphereParams, kMagSphereIterations,
             jtj, jtjInv,
             calc_sphere_residual, calc_sphere_jacobian);

    // No NaN/inf in output, no crash. The solver may not converge to anything
    // meaningful, but it must not produce poisoned floats.
    for (uint8_t i = 0; i < kMagSphereParams; i++) {
        EXPECT_FALSE(isnan(bestParams[i]));
        EXPECT_FALSE(isinf(bestParams[i]));
    }
}

// ---------------------------------------------------------------------------
// mat_inverse direct check: 3x3 identity round-trip
// ---------------------------------------------------------------------------
TEST(LmSolver, MatInverseIdentityRoundtrip) {
    constexpr uint8_t n = 3;
    const float identity[9] = {1,0,0, 0,1,0, 0,0,1};
    float out[9];
    EXPECT_TRUE(mat_inverse(identity, out, n));
    for (uint8_t i = 0; i < 9; i++) {
        EXPECT_FLOAT_EQ(out[i], identity[i]);
    }
}

TEST(LmSolver, MatInverseRejectsSingular) {
    constexpr uint8_t n = 3;
    const float singular[9] = {1,2,3, 2,4,6, 7,8,9};  // row 2 = 2*row 1
    float out[9];
    EXPECT_FALSE(mat_inverse(singular, out, n));
}
