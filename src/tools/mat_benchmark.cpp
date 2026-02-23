// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file mat_benchmark.cpp
 * @brief Standalone matrix operation benchmark for IVP-40 gate.
 *
 * Measures execution time of Mat<N,N> multiply and FPFT operations
 * at 15x15, 18x18, and 24x24 to inform ESKF state count decisions.
 *
 * Separate binary — no watchdog, no Core 1, no sensors.
 * Flash via debug probe, connect serial to read results.
 */

#include "pico/stdlib.h"
#include "pico/time.h"
#include "math/mat.h"
#include "fusion/baro_kf.h"
#include <cstdio>

// Number of iterations per benchmark (enough for stable average)
static constexpr int32_t kIterations = 100;

// Fill a matrix with pseudo-random values (deterministic, no rand())
template <int32_t N>
static void fill_matrix(rc::Mat<N, N>& m) {
    float val = 0.1f;
    for (int32_t r = 0; r < N; ++r) {
        for (int32_t c = 0; c < N; ++c) {
            m(r, c) = val;
            val += 0.037f;  // Irrational-ish step to avoid patterns
            if (val > 1.0f) val -= 1.0f;
        }
    }
}

// Out-parameter multiply: result = a * b.
// Avoids returning large Mat by value (stack overflow on 4KB SCRATCH_Y).
template <int32_t N>
static void multiply_into(rc::Mat<N, N>& result,
                           const rc::Mat<N, N>& a,
                           const rc::Mat<N, N>& b) {
    for (int32_t r = 0; r < N; ++r) {
        for (int32_t p = 0; p < N; ++p) {
            float sum = 0.0f;
            for (int32_t k = 0; k < N; ++k) {
                sum += a(r, k) * b(k, p);
            }
            result(r, p) = sum;
        }
    }
}

// Out-parameter transpose: result = a^T
template <int32_t N>
static void transpose_into(rc::Mat<N, N>& result, const rc::Mat<N, N>& a) {
    for (int32_t r = 0; r < N; ++r) {
        for (int32_t c = 0; c < N; ++c) {
            result(r, c) = a(c, r);
        }
    }
}

// Out-parameter FPFT: result = F * P * F^T
// Uses two static temporaries to avoid stack allocation.
template <int32_t N>
static void fpft_into(rc::Mat<N, N>& result,
                       const rc::Mat<N, N>& f,
                       const rc::Mat<N, N>& p) {
    // All temps static (LL Entry 1: >1KB objects)
    static rc::Mat<N, N> fp;
    static rc::Mat<N, N> ft;
    multiply_into(fp, f, p);
    transpose_into(ft, f);
    multiply_into(result, fp, ft);
}

template <int32_t N>
static void benchmark_multiply(const char* label) {
    // Static allocation per LL Entry 1 (>1KB objects)
    static rc::Mat<N, N> a;
    static rc::Mat<N, N> b;
    static rc::Mat<N, N> result;

    fill_matrix(a);
    fill_matrix(b);

    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kIterations; ++i) {
        multiply_into(result, a, b);
    }
    uint64_t elapsed = time_us_64() - start;

    // Use result to prevent optimization
    float checksum = result(0, 0) + result(N - 1, N - 1);
    printf("  %s multiply: %llu us/iter (checksum=%.3f)\n",
           label, (unsigned long long)(elapsed / kIterations), (double)checksum);
}

template <int32_t N>
static void benchmark_fpft(const char* label) {
    // Static allocation per LL Entry 1
    static rc::Mat<N, N> f;
    static rc::Mat<N, N> p;
    static rc::Mat<N, N> result;

    fill_matrix(f);

    // Make P symmetric positive definite: P = F*F^T + I
    static rc::Mat<N, N> ft;
    transpose_into(ft, f);
    multiply_into(p, f, ft);
    for (int32_t i = 0; i < N; ++i) {
        p(i, i) += 1.0f;
    }

    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kIterations; ++i) {
        fpft_into(result, f, p);
    }
    uint64_t elapsed = time_us_64() - start;

    float checksum = result(0, 0) + result(N - 1, N - 1);
    printf("  %s FPFT:     %llu us/iter (checksum=%.3f)\n",
           label, (unsigned long long)(elapsed / kIterations), (double)checksum);
}

static void benchmark_baro_kf() {
    rc::BaroKF kf;
    kf.init(100.0f);

    // Predict benchmark
    uint64_t start = time_us_64();
    for (int32_t i = 0; i < kIterations; ++i) {
        kf.predict(0.02f);  // 50Hz
    }
    uint64_t elapsed = time_us_64() - start;
    printf("  BaroKF predict: %llu us/iter\n",
           (unsigned long long)(elapsed / kIterations));

    // Update benchmark
    kf.init(100.0f);  // Reset
    kf.predict(0.02f);
    start = time_us_64();
    for (int32_t i = 0; i < kIterations; ++i) {
        kf.update(100.0f + 0.01f * (float)i);
    }
    elapsed = time_us_64() - start;
    printf("  BaroKF update:  %llu us/iter\n",
           (unsigned long long)(elapsed / kIterations));
}

int main() {
    stdio_init_all();

    // Wait for USB CDC connection (standalone tool, OK to block)
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);  // Settle

    printf("\n=== RocketChip Matrix Benchmark (IVP-40) ===\n");
    printf("Build: %s %s\n", __DATE__, __TIME__);
    printf("Iterations per test: %ld\n", (long)kIterations);
    printf("Optimization: -O2\n\n");

    printf("Matrix Multiply (A*B):\n");
    benchmark_multiply<15>("15x15");
    benchmark_multiply<18>("18x18");
    benchmark_multiply<24>("24x24");

    printf("\nFPFT (F*P*F^T):\n");
    benchmark_fpft<15>("15x15");
    benchmark_fpft<18>("18x18");
    benchmark_fpft<24>("24x24");

    printf("\nBaro KF (2-state):\n");
    benchmark_baro_kf();

    printf("\n=== Benchmark Complete ===\n");

    // Idle forever (no watchdog to worry about)
    while (true) {
        sleep_ms(1000);
    }
}
