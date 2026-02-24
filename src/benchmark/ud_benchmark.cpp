// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file ud_benchmark.cpp
 * @brief UD Factorization + DCP Float64 Benchmark.
 *
 * Standalone binary — no watchdog, no Core 1, no sensors.
 * Flash via debug probe, connect serial (COM7) to read results.
 *
 * Phase 1 (Tests 1-2): Gate decision for UD investment.
 *   Test 1: DCP raw throughput characterization
 *   Test 2: Current P stability over 100K predict steps
 *
 * Phase 2 (Tests 3-5): Only if Phase 1 shows P instability.
 *   Test 3: Thornton + Bierman timing
 *   Test 3b: Hybrid codegen+Bierman timing
 *   Test 4: Numerical accuracy comparison (3 paths)
 *   Test 5: Full ESKF cycle timing
 *
 * Council review: CONSENSUS approve with modifications.
 * Framed as Titan-tier investigation — Core arch is adequate.
 */

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/structs/systick.h"

#include "fusion/eskf.h"
#include "fusion/eskf_codegen.h"
#include "fusion/eskf_state.h"
#include "fusion/ud_factor.h"
#include "math/mat.h"
#include "math/quat.h"
#include "math/vec3.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>  // placement new

// Local helper: quaternion to 3×3 rotation matrix (Mat3).
// Same as the file-local function in eskf.cpp.
static rc::Mat3 quat_to_dcm(const rc::Quat& q) {
    float m9[9];
    q.to_rotation_matrix(m9);
    rc::Mat3 rotMat;
    rotMat(0, 0) = m9[0]; rotMat(0, 1) = m9[1]; rotMat(0, 2) = m9[2];
    rotMat(1, 0) = m9[3]; rotMat(1, 1) = m9[4]; rotMat(1, 2) = m9[5]; // NOLINT
    rotMat(2, 0) = m9[6]; rotMat(2, 1) = m9[7]; rotMat(2, 2) = m9[8]; // NOLINT
    return rotMat;
}

// =========================================================================
// Configuration
// =========================================================================

static constexpr int32_t kDcpIterations = 1000;
static constexpr int32_t kStabilitySteps = 100000;
static constexpr int32_t kStabilityReportInterval = 1000;
// Test 4 runs 3 parallel paths (codegen + 2× Thornton) at ~10ms/step.
// 10K steps = ~100s, enough to show divergence. 100K would take ~17 min.
static constexpr int32_t kAccuracySteps = 10000;
static constexpr int32_t kAccuracyReportInterval = 1000;
static constexpr int32_t kTimingIterations = 100;
static constexpr float kBenchmarkDt = 0.005f;  // 200Hz

// Synthetic IMU data (stationary with small gyro drift)
static constexpr float kAccelX = 0.0f;
static constexpr float kAccelY = 0.0f;
static constexpr float kAccelZ = -9.80665f;  // Gravity (sensor frame, flat)
static constexpr float kGyroX = 0.001f;      // Tiny drift
static constexpr float kGyroY = -0.002f;
static constexpr float kGyroZ = 0.0005f;

static const char* kBuildTag = "ud-bench-6";

// =========================================================================
// SysTick cycle counter helpers
// =========================================================================

static void systick_init() {
    // Enable SysTick with processor clock, no interrupt, max reload
    systick_hw->csr = 0;            // Disable
    systick_hw->rvr = 0x00FFFFFF;   // 24-bit max reload
    systick_hw->cvr = 0;            // Clear current
    systick_hw->csr = 0x5;          // Enable, processor clock, no interrupt
}

static inline uint32_t systick_read() {
    return systick_hw->cvr;
}

// SysTick counts DOWN. Elapsed = (start - end) & 0xFFFFFF.
static inline uint32_t systick_elapsed(uint32_t start, uint32_t end) {
    return (start - end) & 0x00FFFFFF;
}

// =========================================================================
// Static allocations (LL Entry 1: >1KB must be static)
// =========================================================================

static rc::ESKF g_eskf;

// Reset g_eskf without stack-allocating a temporary ESKF (~2.3KB).
// `g_eskf = ESKF{}` creates a stack temporary that overflows 4KB SCRATCH_Y.
// Placement new reconstructs in-place: zero stack cost.
static void reset_eskf() {
    g_eskf.~ESKF();
    new (&g_eskf) rc::ESKF();
}

static rc::UD24 g_ud;
static rc::UD24 g_ud_b;   // Path B
static rc::UD24 g_ud_c;   // Path C

// F and Qd workspaces for Thornton
static rc::Mat24 g_F;
static rc::Mat24 g_Qc;
static float g_Qd[24];



// =========================================================================
// Noinline wrappers for codegen_fpft calls.
// codegen_fpft uses ~1.8KB of stack. When called from a test function
// that also has a large frame (many locals across multiple blocks),
// the combined stack exceeds 4KB SCRATCH_Y. Noinline prevents the
// compiler from merging these frames.
// =========================================================================

static void __attribute__((noinline)) bench_codegen_fpft() {
    reset_eskf();
    const rc::Vec3 a{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 g{0.0f, 0.0f, 0.0f};
    g_eskf.init(a, g);
    const rc::Mat3 R = quat_to_dcm(g_eskf.q);

    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        codegen_fpft(g_eskf.P.data, R.data,
                     kAccelX, kAccelY, kAccelZ,
                     kGyroX, kGyroY, kGyroZ,
                     kBenchmarkDt);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = g_eskf.P(0, 0);
    (void)sink;
    printf("Codegen FPFT:      %llu us/iter (reference)\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void __attribute__((noinline)) bench_hybrid_cycle() {
    reset_eskf();
    const rc::Vec3 a{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 g{0.0f, 0.0f, 0.0f};
    g_eskf.init(a, g);
    const rc::Mat3 R = quat_to_dcm(g_eskf.q);
    static float dx[24];

    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        codegen_fpft(g_eskf.P.data, R.data,
                     kAccelX, kAccelY, kAccelZ,
                     kGyroX, kGyroY, kGyroZ,
                     kBenchmarkDt);
        g_eskf.P.force_symmetric();
        rc::ud_factorize(g_ud, g_eskf.P.data);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosD, -1.0f,
                                   0.5f, rc::ESKF::kRBaro, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxYaw, 1.0f,
                                   0.01f, rc::ESKF::kRMagHeading, dx);
        rc::ud_to_dense(g_ud, g_eskf.P.data);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = g_eskf.P(0, 0);
    (void)sink;
    printf("Hybrid (codegen+factorize+Bierman): %llu us/cycle\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

// =========================================================================
// Test 1: DCP Raw Throughput Characterization
// =========================================================================

// Helper: time a single-op loop and print result.
static void bench_f32_mac() {
    volatile float f32_b = 0.999f, f32_c = 1.001f;
    float acc = 0.0f;
    uint32_t start = systick_read();
    for (int32_t i = 0; i < kDcpIterations; ++i) {
        acc += static_cast<float>(f32_b) * static_cast<float>(f32_c);
    }
    uint32_t cycles = systick_elapsed(start, systick_read());
    volatile float sink = acc;
    (void)sink;
    printf("f32 MAC:           %lu cycles / %ld iter = %.1f cycles/op\n",
           (unsigned long)cycles, kDcpIterations,
           static_cast<double>(cycles) / kDcpIterations);
}

static void bench_f64_mac() {
    volatile double f64_b = 0.999, f64_c = 1.001;
    double acc = 0.0;
    uint32_t start = systick_read();
    for (int32_t i = 0; i < kDcpIterations; ++i) {
        acc += static_cast<double>(f64_b) * static_cast<double>(f64_c);
    }
    uint32_t cycles = systick_elapsed(start, systick_read());
    volatile double sink = acc;
    (void)sink;
    printf("f64 MAC (DCP):     %lu cycles / %ld iter = %.1f cycles/op\n",
           (unsigned long)cycles, kDcpIterations,
           static_cast<double>(cycles) / kDcpIterations);
}

static void bench_promoted_mac() {
    volatile float f32_b = 0.999f, f32_c = 1.001f;
    double acc = 0.0;
    uint32_t start = systick_read();
    for (int32_t i = 0; i < kDcpIterations; ++i) {
        acc += static_cast<double>(static_cast<float>(f32_b))
             * static_cast<double>(static_cast<float>(f32_c));
    }
    uint32_t cycles = systick_elapsed(start, systick_read());
    volatile double sink = acc;
    (void)sink;
    printf("f32->f64 promoted: %lu cycles / %ld iter = %.1f cycles/op\n",
           (unsigned long)cycles, kDcpIterations,
           static_cast<double>(cycles) / kDcpIterations);
}

static void bench_demotion() {
    volatile double f64_a = 1.0;
    float acc = 0.0f;
    uint32_t start = systick_read();
    for (int32_t i = 0; i < kDcpIterations; ++i) {
        acc = static_cast<float>(static_cast<double>(f64_a) + 0.001);
    }
    uint32_t cycles = systick_elapsed(start, systick_read());
    volatile float sink = acc;
    (void)sink;
    printf("f64->f32 demotion: %lu cycles / %ld iter = %.1f cycles/op\n",
           (unsigned long)cycles, kDcpIterations,
           static_cast<double>(cycles) / kDcpIterations);
}

static void bench_chained_f64() {
    volatile double b0 = 1.001, b1 = 0.999, b2 = 1.002, b3 = 0.998;
    volatile double c0 = 0.997, c1 = 1.003, c2 = 0.996, c3 = 1.004;
    double acc = 0.0;
    uint32_t start = systick_read();
    for (int32_t i = 0; i < kDcpIterations; ++i) {
        acc += static_cast<double>(b0) * static_cast<double>(c0);
        acc += static_cast<double>(b1) * static_cast<double>(c1);
        acc += static_cast<double>(b2) * static_cast<double>(c2);
        acc += static_cast<double>(b3) * static_cast<double>(c3);
        acc += static_cast<double>(b0) * static_cast<double>(c1);
        acc += static_cast<double>(b1) * static_cast<double>(c0);
        acc += static_cast<double>(b2) * static_cast<double>(c3);
        acc += static_cast<double>(b3) * static_cast<double>(c2);
    }
    uint32_t cycles = systick_elapsed(start, systick_read());
    volatile double sink = acc;
    (void)sink;
    printf("Chained f64 MAC:   %lu cycles / %ld×8 = %.1f cycles/op\n",
           (unsigned long)cycles, kDcpIterations,
           static_cast<double>(cycles) / (kDcpIterations * 8));
}

static void test1_dcp_throughput() {
    printf("\n--- Test 1: DCP Throughput ---\n");
    bench_f32_mac();
    bench_f64_mac();
    bench_promoted_mac();
    bench_demotion();
    bench_chained_f64();
}

// =========================================================================
// Test 2: Current P Stability over 100K predict steps
// =========================================================================

// Measure raw asymmetry of P before force_symmetric()
static float measure_asymmetry(const rc::Mat24& P) {
    float maxAsym = 0.0f;
    for (int32_t i = 0; i < 24; ++i) {
        for (int32_t j = i + 1; j < 24; ++j) {
            float diff = std::fabs(P(i, j) - P(j, i));
            if (diff > maxAsym) {
                maxAsym = diff;
            }
        }
    }
    return maxAsym;
}

// Find min/max diagonal of core states [0..14]
static float min_core_diagonal(const rc::Mat24& P) {
    float minD = P(0, 0);
    for (int32_t i = 1; i < 15; ++i) {
        if (P(i, i) < minD) {
            minD = P(i, i);
        }
    }
    return minD;
}

static float max_core_diagonal(const rc::Mat24& P) {
    float maxD = P(0, 0);
    for (int32_t i = 1; i < 15; ++i) {
        if (P(i, i) > maxD) {
            maxD = P(i, i);
        }
    }
    return maxD;
}

// Run kStabilitySteps predict cycles, reporting diagnostics every
// kStabilityReportInterval steps.  Returns step of first negative diagonal
// (-1 if none).
static int32_t test2_run_steps() {
    int32_t firstNegStep = -1;
    const rc::Vec3 accelMeas{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroMeas{kGyroX, kGyroY, kGyroZ};

    for (int32_t step = 1; step <= kStabilitySteps; ++step) {
        // Run predict (codegen_fpft + force_symmetric + clamp).
        // Asymmetry measured AFTER force_symmetric — gives a lower bound.
        g_eskf.predict(accelMeas, gyroMeas, kBenchmarkDt);

        float minD = min_core_diagonal(g_eskf.P);
        float maxD = max_core_diagonal(g_eskf.P);

        if (minD < 0.0f && firstNegStep < 0) {
            firstNegStep = step;
        }

        if (step % kStabilityReportInterval == 0) {
            float rawAsym = measure_asymmetry(g_eskf.P);
            float condProxy = (minD > 0.0f) ? (maxD / minD) : -1.0f;
            printf("%5ld | %13.6e | %13.6e | %13.6e | %.2e\n",
                   (long)step,
                   static_cast<double>(minD),
                   static_cast<double>(maxD),
                   static_cast<double>(rawAsym),
                   static_cast<double>(condProxy));
        }
    }
    return firstNegStep;
}

static void test2_p_stability() {
    printf("\n--- Test 2: Current P Stability (100K steps) ---\n");

    const rc::Vec3 accelInit{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroInit{0.0f, 0.0f, 0.0f};
    reset_eskf();
    bool initOk = g_eskf.init(accelInit, gyroInit);
    printf("ESKF init: %s\n", initOk ? "OK" : "FAIL");
    if (!initOk) {
        printf("Cannot proceed — init failed\n");
        return;
    }

    printf("Step | MinDiag(core) | MaxDiag(core) | MaxAsym       | CondProxy\n");
    printf("-----+--------------+--------------+--------------+----------\n");

    uint64_t totalStart = time_us_64();
    int32_t firstNegStep = test2_run_steps();
    uint64_t totalElapsed = time_us_64() - totalStart;

    printf("\nSteps before negative diagonal: %s\n",
           (firstNegStep < 0) ? "none in 100000" : "FAILED");
    if (firstNegStep >= 0) {
        printf("  First negative at step %ld\n", (long)firstNegStep);
    }
    printf("Total time: %llu ms\n",
           (unsigned long long)(totalElapsed / 1000));
    printf("ESKF healthy: %s\n", g_eskf.healthy() ? "YES" : "NO");
}

// =========================================================================
// Test 3: UD Implementation Timing
//
// Each sub-test is a separate noinline function to keep call-chain stack
// depth within the 4KB SCRATCH_Y limit. codegen_fpft alone uses ~1.8KB.
// =========================================================================

// Static workspace for Test 3 init diagonal — shared across sub-tests.
static float g_t3_initDiag[24];

// Prepare F, Qd, and initDiag for Test 3 sub-tests.
static void __attribute__((noinline)) test3_setup() {
    for (int32_t i = 0; i < 3; ++i) {
        g_t3_initDiag[rc::eskf::kIdxAttitude + i] = rc::ESKF::kInitPAttitude;
        g_t3_initDiag[rc::eskf::kIdxPosition + i] = rc::ESKF::kInitPPosition;
        g_t3_initDiag[rc::eskf::kIdxVelocity + i] = rc::ESKF::kInitPVelocity;
        g_t3_initDiag[rc::eskf::kIdxAccelBias + i] = rc::ESKF::kInitPAccelBias;
        g_t3_initDiag[rc::eskf::kIdxGyroBias + i] = rc::ESKF::kInitPGyroBias;
        g_t3_initDiag[rc::eskf::kIdxEarthMag + i] = rc::ESKF::kInitPEarthMag;
        g_t3_initDiag[rc::eskf::kIdxBodyMagBias + i] = rc::ESKF::kInitPBodyMagBias;
    }
    g_t3_initDiag[rc::eskf::kIdxWindNE + 0] = rc::ESKF::kInitPWind;
    g_t3_initDiag[rc::eskf::kIdxWindNE + 1] = rc::ESKF::kInitPWind;
    g_t3_initDiag[rc::eskf::kIdxBaroBias] = rc::ESKF::kInitPBaroBias;

    // Use g_eskf (static) to build F — no local ESKF on stack.
    reset_eskf();
    const rc::Vec3 accelInit{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroInit{0.0f, 0.0f, 0.0f};
    g_eskf.init(accelInit, gyroInit);

    rc::ESKF::build_F(g_F, g_eskf.q,
                       rc::Vec3{kAccelX, kAccelY, kAccelZ},
                       rc::Vec3{kGyroX, kGyroY, kGyroZ},
                       kBenchmarkDt);

    rc::ESKF::build_Qc(g_Qc);
    for (int32_t i = 0; i < 24; ++i) {
        g_Qd[i] = g_Qc(i, i) * kBenchmarkDt;
    }
}

static void __attribute__((noinline)) bench_thornton_f32() {
    rc::ud_from_diagonal(g_ud, g_t3_initDiag);
    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        rc::thornton_f32(g_ud, g_F.data, g_Qd);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = g_ud.D[0];
    (void)sink;
    printf("Thornton f32:      %llu us/iter\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void __attribute__((noinline)) bench_thornton_mixed() {
    rc::ud_from_diagonal(g_ud, g_t3_initDiag);
    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        rc::thornton_mixed(g_ud, g_F.data, g_Qd);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = g_ud.D[0];
    (void)sink;
    printf("Thornton mixed:    %llu us/iter\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void __attribute__((noinline)) bench_thornton_f64() {
    rc::ud_from_diagonal(g_ud, g_t3_initDiag);
    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        rc::thornton_f64(g_ud, g_F.data, g_Qd);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = g_ud.D[0];
    (void)sink;
    printf("Thornton f64:      %llu us/iter\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void __attribute__((noinline)) bench_bierman_scalar() {
    rc::ud_from_diagonal(g_ud, g_t3_initDiag);
    static float dx[24];
    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosD, -1.0f,
                                   0.5f, rc::ESKF::kRBaro, dx);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = dx[0];
    (void)sink;
    printf("Bierman scalar:    %llu us/iter\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void __attribute__((noinline)) bench_joseph_scalar() {
    reset_eskf();
    const rc::Vec3 accelInit{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroInit{0.0f, 0.0f, 0.0f};
    g_eskf.init(accelInit, gyroInit);
    // One predict to get non-trivial P (calls codegen_fpft internally).
    const rc::Vec3 accelMeas{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroMeas{kGyroX, kGyroY, kGyroZ};
    g_eskf.predict(accelMeas, gyroMeas, kBenchmarkDt);

    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        g_eskf.update_baro(0.0f);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = g_eskf.P(0, 0);
    (void)sink;
    printf("Joseph scalar:     %llu us/iter (reference, includes inject)\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void test3_ud_timing() {
    printf("\n--- Test 3: Implementation Timing ---\n");

    test3_setup();
    bench_thornton_f32();
    bench_thornton_mixed();
    bench_thornton_f64();
    bench_bierman_scalar();
    bench_codegen_fpft();
    bench_joseph_scalar();
    bench_hybrid_cycle();
}

// =========================================================================
// Test 4: Numerical Accuracy Comparison
// =========================================================================

static float frobenius_diff(const float A[24][24], const float B[24][24]) {
    double sum = 0.0;
    for (int32_t i = 0; i < 24; ++i) {
        for (int32_t j = 0; j < 24; ++j) {
            double d = static_cast<double>(A[i][j]) - static_cast<double>(B[i][j]);
            sum += d * d;
        }
    }
    return static_cast<float>(std::sqrt(sum));
}

static float max_diag_diff(const float A[24][24], const float B[24][24]) {
    float maxD = 0.0f;
    for (int32_t i = 0; i < 24; ++i) {
        float d = std::fabs(A[i][i] - B[i][i]);
        if (d > maxD) maxD = d;
    }
    return maxD;
}

// Path A ESKF — static to avoid stack temporary from assignment.
static rc::ESKF g_eskfA;

static void __attribute__((noinline)) reset_eskfA() {
    g_eskfA.~ESKF();
    new (&g_eskfA) rc::ESKF();
}

static float P_B[24][24];
static float P_C[24][24];

static void __attribute__((noinline)) test4_main_loop() {
    printf("Step  | MaxDiagDiff(A-C) | Frobenius(A-C) | MinDiag(A) | D_pos(B) | D_pos(C)\n");
    printf("------+-----------------+---------------+-----------+---------+---------\n");

    for (int32_t step = 1; step <= kAccuracySteps; ++step) {
        const rc::Vec3 accelMeas{kAccelX, kAccelY, kAccelZ};
        const rc::Vec3 gyroMeas{kGyroX, kGyroY, kGyroZ};

        // Path A: full predict (evolves quaternion, rebuilds F internally).
        g_eskfA.predict(accelMeas, gyroMeas, kBenchmarkDt);

        // Rebuild F from Path A's evolved quaternion so UD paths use the
        // same F as codegen. Without this, UD paths use a stale F from
        // step 0 → comparison is meaningless → Frobenius diverges to NaN.
        rc::ESKF::build_F(g_F, g_eskfA.q, accelMeas, gyroMeas, kBenchmarkDt);

        // Paths B & C: Thornton predict with matched F.
        rc::thornton_f32(g_ud_b, g_F.data, g_Qd);
        rc::thornton_mixed(g_ud_c, g_F.data, g_Qd);

        if (step % kAccuracyReportInterval == 0) {
            rc::ud_to_dense(g_ud_b, P_B);
            rc::ud_to_dense(g_ud_c, P_C);

            float diagDiff = max_diag_diff(g_eskfA.P.data, P_C);
            float frobDiff = frobenius_diff(g_eskfA.P.data, P_C);
            float minDiagA = min_core_diagonal(g_eskfA.P);
            bool dPosB = rc::ud_all_positive(g_ud_b);
            bool dPosC = rc::ud_all_positive(g_ud_c);

            printf("%5ld | %15.6e | %13.6e | %9.3e | %7s | %7s\n",
                   (long)step,
                   static_cast<double>(diagDiff),
                   static_cast<double>(frobDiff),
                   static_cast<double>(minDiagA),
                   dPosB ? "OK" : "FAIL",
                   dPosC ? "OK" : "FAIL");
        }
    }
}

static void __attribute__((noinline)) test4_pathological() {
    printf("\n--- Pathological: P[3][3]=1e6, P[9][9]=1e-8 ---\n");

    reset_eskfA();
    const rc::Vec3 accelInit{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroInit{0.0f, 0.0f, 0.0f};
    g_eskfA.init(accelInit, gyroInit);
    g_eskfA.P(3, 3) = 1e6f;
    g_eskfA.P(9, 9) = 1e-8f;

    float initDiag[24];
    for (int32_t i = 0; i < 24; ++i) {
        initDiag[i] = g_eskfA.P(i, i);
    }
    rc::ud_from_diagonal(g_ud_b, initDiag);
    rc::ud_from_diagonal(g_ud_c, initDiag);

    printf("Step  | MaxDiagDiff(A-C) | MinDiag(A) | D_pos(C)\n");
    printf("------+-----------------+-----------+---------\n");

    for (int32_t step = 1; step <= 10000; ++step) {
        const rc::Vec3 accelMeas{kAccelX, kAccelY, kAccelZ};
        const rc::Vec3 gyroMeas{kGyroX, kGyroY, kGyroZ};
        g_eskfA.predict(accelMeas, gyroMeas, kBenchmarkDt);
        rc::ESKF::build_F(g_F, g_eskfA.q, accelMeas, gyroMeas, kBenchmarkDt);
        rc::thornton_f32(g_ud_b, g_F.data, g_Qd);
        rc::thornton_mixed(g_ud_c, g_F.data, g_Qd);

        if (step % 1000 == 0) {
            rc::ud_to_dense(g_ud_c, P_C);
            float diagDiff = max_diag_diff(g_eskfA.P.data, P_C);
            float minDiagA = min_core_diagonal(g_eskfA.P);
            bool dPosC = rc::ud_all_positive(g_ud_c);
            printf("%5ld | %15.6e | %9.3e | %7s\n",
                   (long)step,
                   static_cast<double>(diagDiff),
                   static_cast<double>(minDiagA),
                   dPosC ? "OK" : "FAIL");
        }
    }
}

static void test4_accuracy() {
    printf("\n--- Test 4: Numerical Accuracy (10K steps) ---\n");

    const rc::Vec3 accelInit{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroInit{0.0f, 0.0f, 0.0f};

    // Path A: codegen + Joseph
    reset_eskfA();
    g_eskfA.init(accelInit, gyroInit);

    // Paths B & C: UD from same initial P
    float initDiag[24];
    for (int32_t i = 0; i < 24; ++i) {
        initDiag[i] = g_eskfA.P(i, i);
    }
    rc::ud_from_diagonal(g_ud_b, initDiag);
    rc::ud_from_diagonal(g_ud_c, initDiag);

    // Build F and Qd
    rc::ESKF::build_F(g_F, g_eskfA.q,
                       rc::Vec3{kAccelX, kAccelY, kAccelZ},
                       rc::Vec3{kGyroX, kGyroY, kGyroZ},
                       kBenchmarkDt);
    rc::ESKF::build_Qc(g_Qc);
    for (int32_t i = 0; i < 24; ++i) {
        g_Qd[i] = g_Qc(i, i) * kBenchmarkDt;
    }

    test4_main_loop();
    test4_pathological();
}

// =========================================================================
// Test 5: Full Cycle Timing
// =========================================================================

// Each Test 5 variant is noinline — predict() calls codegen_fpft (~1.8KB).
static void __attribute__((noinline)) bench_cycle_current() {
    const rc::Vec3 accelInit{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroInit{0.0f, 0.0f, 0.0f};
    const rc::Vec3 accelMeas{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroMeas{kGyroX, kGyroY, kGyroZ};

    reset_eskf();
    g_eskf.init(accelInit, gyroInit);
    g_eskf.set_origin(0.0, 0.0, 0.0f, 1.0f);

    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        g_eskf.predict(accelMeas, gyroMeas, kBenchmarkDt);
        g_eskf.update_baro(0.0f);
        g_eskf.update_mag_heading(rc::Vec3{20.0f, 5.0f, 40.0f}, 45.0f, 0.0f);
        g_eskf.update_gps_position(rc::Vec3{0.0f, 0.0f, 0.0f}, 1.5f, 2.0f);
        g_eskf.update_gps_velocity(0.0f, 0.0f);
        g_eskf.update_zupt(accelMeas, gyroMeas);
    }
    uint64_t elapsed = time_us_64() - start;
    printf("Current (codegen+Joseph): %llu us/cycle\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void __attribute__((noinline)) bench_cycle_ud_f32() {
    const rc::Vec3 accelInit{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroInit{0.0f, 0.0f, 0.0f};
    const rc::Vec3 accelMeas{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroMeas{kGyroX, kGyroY, kGyroZ};

    reset_eskf();
    g_eskf.init(accelInit, gyroInit);

    float diag[24];
    for (int32_t i = 0; i < 24; ++i) diag[i] = g_eskf.P(i, i);
    rc::ud_from_diagonal(g_ud, diag);

    rc::ESKF::build_F(g_F, g_eskf.q, accelMeas, gyroMeas, kBenchmarkDt);
    rc::ESKF::build_Qc(g_Qc);
    for (int32_t i = 0; i < 24; ++i) g_Qd[i] = g_Qc(i, i) * kBenchmarkDt;

    static float dx[24];

    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        rc::thornton_f32(g_ud, g_F.data, g_Qd);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosD, -1.0f,
                                   0.5f, rc::ESKF::kRBaro, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxYaw, 1.0f,
                                   0.01f, rc::ESKF::kRMagHeading, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosN, 1.0f,
                                   0.0f, rc::ESKF::kRGpsPosDefault, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosE, 1.0f,
                                   0.0f, rc::ESKF::kRGpsPosDefault, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosD, 1.0f,
                                   0.0f, rc::ESKF::kRGpsPosDefault, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelN, 1.0f,
                                   0.0f, rc::ESKF::kRGpsVel, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelE, 1.0f,
                                   0.0f, rc::ESKF::kRGpsVel, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelN, 1.0f,
                                   0.0f, rc::ESKF::kRZupt, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelE, 1.0f,
                                   0.0f, rc::ESKF::kRZupt, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelD, 1.0f,
                                   0.0f, rc::ESKF::kRZupt, dx);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = g_ud.D[0];
    (void)sink;
    printf("UD f32 (Thornton+Bierman): %llu us/cycle\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void __attribute__((noinline)) bench_cycle_ud_mixed() {
    const rc::Vec3 accelInit{kAccelX, kAccelY, kAccelZ};
    const rc::Vec3 gyroInit{0.0f, 0.0f, 0.0f};

    reset_eskf();
    g_eskf.init(accelInit, gyroInit);

    float diag[24];
    for (int32_t i = 0; i < 24; ++i) diag[i] = g_eskf.P(i, i);
    rc::ud_from_diagonal(g_ud, diag);

    static float dx[24];

    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kTimingIterations; ++i) {
        rc::thornton_mixed(g_ud, g_F.data, g_Qd);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosD, -1.0f,
                                   0.5f, rc::ESKF::kRBaro, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxYaw, 1.0f,
                                   0.01f, rc::ESKF::kRMagHeading, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosN, 1.0f,
                                   0.0f, rc::ESKF::kRGpsPosDefault, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosE, 1.0f,
                                   0.0f, rc::ESKF::kRGpsPosDefault, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxPosD, 1.0f,
                                   0.0f, rc::ESKF::kRGpsPosDefault, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelN, 1.0f,
                                   0.0f, rc::ESKF::kRGpsVel, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelE, 1.0f,
                                   0.0f, rc::ESKF::kRGpsVel, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelN, 1.0f,
                                   0.0f, rc::ESKF::kRZupt, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelE, 1.0f,
                                   0.0f, rc::ESKF::kRZupt, dx);
        rc::bierman_scalar_update(g_ud, rc::eskf::kIdxVelD, 1.0f,
                                   0.0f, rc::ESKF::kRZupt, dx);
    }
    uint64_t elapsed = time_us_64() - start;
    volatile float sink = g_ud.D[0];
    (void)sink;
    printf("UD mixed (Thornton+Bierman): %llu us/cycle\n",
           (unsigned long long)(elapsed / kTimingIterations));
}

static void test5_full_cycle() {
    printf("\n--- Test 5: Full Cycle Timing ---\n");

    bench_cycle_current();
    bench_cycle_ud_f32();
    bench_cycle_ud_mixed();
    bench_hybrid_cycle();
}

// =========================================================================
// Main
// =========================================================================

int main() {
    stdio_init_all();

    // Wait for USB CDC connection (standalone benchmark, OK to block)
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);  // Settle

    systick_init();

    printf("\n=== UD FACTORIZATION BENCHMARK ===\n");
    printf("Build: %s (%s %s)\n", kBuildTag, __DATE__, __TIME__);
    printf("Board: RP2350 HSTX Feather @ 150MHz\n");
    printf("Optimization: -O2\n");
    printf("Phase 1: DCP throughput + P stability (100K steps)\n");
    printf("Phase 2: UD timing + accuracy + full cycle\n\n");

    // ---- Phase 1 ----
    test1_dcp_throughput();
    test2_p_stability();

    // ---- Phase 2 ----
    // Always run all tests to collect data — the gate decision
    // is made by the user based on Phase 1 results.
    test3_ud_timing();
    test4_accuracy();
    test5_full_cycle();

    printf("\n=== Benchmark Complete ===\n");

    // Idle forever (no watchdog)
    while (true) {
        sleep_ms(1000);
    }
}
