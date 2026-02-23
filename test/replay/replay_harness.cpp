// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IVP-42c: Standalone ESKF replay harness.
// Reads input CSV (Section 5.3 format), feeds ESKF, writes output CSV.
// Output becomes change-indication regression baseline (ESKF_TESTING_GUIDE.md S5.4).
//
// Usage: replay_harness <input.csv> <output.csv>

#include <clocale>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "replay/sensor_log.h"
#include "replay/output_log.h"
#include "fusion/eskf.h"
#include "math/vec3.h"

static void print_eskf_config(FILE* f) {
    fprintf(f, "ESKF configuration:\n");
    fprintf(f, "  kGravity         = %.9g\n", static_cast<double>(rc::ESKF::kGravity));
    fprintf(f, "  kSigmaGyro       = %.9g\n", static_cast<double>(rc::ESKF::kSigmaGyro));
    fprintf(f, "  kSigmaAccel      = %.9g\n", static_cast<double>(rc::ESKF::kSigmaAccel));
    fprintf(f, "  kSigmaGyroBiasWalk  = %.9g\n", static_cast<double>(rc::ESKF::kSigmaGyroBiasWalk));
    fprintf(f, "  kSigmaAccelBiasWalk = %.9g\n", static_cast<double>(rc::ESKF::kSigmaAccelBiasWalk));
    fprintf(f, "  kInitPAttitude   = %.9g\n", static_cast<double>(rc::ESKF::kInitPAttitude));
    fprintf(f, "  kInitPPosition   = %.9g\n", static_cast<double>(rc::ESKF::kInitPPosition));
    fprintf(f, "  kInitPVelocity   = %.9g\n", static_cast<double>(rc::ESKF::kInitPVelocity));
    fprintf(f, "  kInitPAccelBias  = %.9g\n", static_cast<double>(rc::ESKF::kInitPAccelBias));
    fprintf(f, "  kInitPGyroBias   = %.9g\n", static_cast<double>(rc::ESKF::kInitPGyroBias));
}

int main(int argc, char* argv[]) {
    // Council CR-3: locale-independent decimal formatting
    setlocale(LC_NUMERIC, "C");

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <input.csv> <output.csv>\n", argv[0]);
        return 1;
    }

    const char* input_path = argv[1];
    const char* output_path = argv[2];

    // Open input
    rc::test::SensorLog log(input_path);
    if (!log.is_open()) {
        fprintf(stderr, "Error: cannot open input file: %s\n", input_path);
        return 1;
    }

    // Read first sample
    rc::test::SensorSample sample{};
    if (!log.read_next(sample)) {
        fprintf(stderr, "Error: no data rows in input file\n");
        return 1;
    }

    // Init ESKF with stationary accel and zero gyro
    rc::ESKF eskf;
    const rc::Vec3 accel_stationary(0.0f, 0.0f, -rc::ESKF::kGravity);
    const rc::Vec3 gyro_zero(0.0f, 0.0f, 0.0f);
    if (!eskf.init(accel_stationary, gyro_zero)) {
        fprintf(stderr, "Error: ESKF init failed (stationarity check)\n");
        return 1;
    }

    // Open output
    FILE* out = fopen(output_path, "w");
    if (!out) {
        fprintf(stderr, "Error: cannot open output file: %s\n", output_path);
        return 1;
    }

    // Write header + row 0 (initial state)
    rc::test::write_output_header(out);
    rc::test::write_output_row(out, sample.timestamp_us, eskf);

    // Aiding sensor counters (council CR-5)
    int32_t baro_count = 0;
    int32_t gps_count = 0;
    int32_t mag_count = 0;

    if (!std::isnan(sample.baro_alt_m)) ++baro_count;
    if (!std::isnan(sample.gps_fix))    ++gps_count;
    if (!std::isnan(sample.mx))         ++mag_count;

    // Replay loop
    uint64_t prev_ts = sample.timestamp_us;
    int32_t row_count = 1;  // Already wrote row 0

    while (log.read_next(sample)) {
        float dt = static_cast<float>(sample.timestamp_us - prev_ts) * 1e-6f;
        prev_ts = sample.timestamp_us;

        rc::Vec3 accel(sample.ax, sample.ay, sample.az);
        rc::Vec3 gyro(sample.gx, sample.gy, sample.gz);
        eskf.predict(accel, gyro, dt);

        // IVP-44: Magnetometer heading update (when mag data available)
        if (!std::isnan(sample.mx) && !std::isnan(sample.my) &&
            !std::isnan(sample.mz)) {
            rc::Vec3 mag_body(sample.mx, sample.my, sample.mz);
            // expected_magnitude=0 skips interference check (no cal reference in replay).
            // declination_rad=0 (magnetic heading, no GPS-derived WMM lookup in replay).
            eskf.update_mag_heading(mag_body, 0.0f, 0.0f);
        }

        rc::test::write_output_row(out, sample.timestamp_us, eskf);
        ++row_count;

        // Count aiding sensor availability
        if (!std::isnan(sample.baro_alt_m)) ++baro_count;
        if (!std::isnan(sample.gps_fix))    ++gps_count;
        if (!std::isnan(sample.mx))         ++mag_count;
    }

    fclose(out);

    // Summary to stderr (council CR-5, CR-6)
    fprintf(stderr, "Replay complete: %s\n", input_path);
    fprintf(stderr, "  Rows written: %d\n", row_count);
    fprintf(stderr, "  Final position: [%.4f, %.4f, %.4f] m\n",
            static_cast<double>(eskf.p.x),
            static_cast<double>(eskf.p.y),
            static_cast<double>(eskf.p.z));
    fprintf(stderr, "  Healthy: %s\n", eskf.healthy() ? "yes" : "no");
    fprintf(stderr, "  Aiding sensor rows — baro: %d, GPS: %d, mag: %d\n",
            baro_count, gps_count, mag_count);
    print_eskf_config(stderr);

    return 0;
}
