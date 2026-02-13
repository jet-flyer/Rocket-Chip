// IVP-42c: Replay regression test.
// Re-runs ESKF replay on each canonical trajectory and compares output
// against committed reference CSVs. Any behavioral change in the ESKF
// produces a diff, catching regressions automatically.
//
// Uses combined relative+absolute tolerance (council CR-2):
//   |a - b| <= max(1e-4, 1e-4 * max(|a|, |b|))
// This gives absolute tolerance near zero and relative for large values.

#include <gtest/gtest.h>

#include <clocale>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "replay/sensor_log.h"
#include "fusion/eskf.h"
#include "math/vec3.h"

namespace {

// Council CR-2: combined relative+absolute tolerance
bool approx_eq(float a, float b) {
    return std::fabs(a - b) <=
           std::max(1e-4f, 1e-4f * std::max(std::fabs(a), std::fabs(b)));
}

// Parse one row of reference CSV into float array.
// Returns number of columns parsed, or 0 on failure.
struct RefRow {
    uint64_t timestamp_us;
    float cols[31];  // qw..P14 (columns 1-31)
    int32_t ncols;
};

bool parse_ref_row(const std::string& line, RefRow& row) {
    const char* p = line.c_str();
    int32_t col = 0;

    // Column 0: timestamp
    row.timestamp_us = strtoull(p, nullptr, 10);
    while (*p && *p != ',') ++p;
    if (*p == ',') ++p;
    ++col;

    // Columns 1-31: float values
    int32_t float_idx = 0;
    while (*p && float_idx < 31) {
        row.cols[float_idx] = std::strtof(p, nullptr);
        while (*p && *p != ',') ++p;
        if (*p == ',') ++p;
        ++float_idx;
        ++col;
    }

    row.ncols = float_idx;
    return float_idx >= 31;
}

// Run replay on input CSV, return output rows as RefRow vector.
std::vector<RefRow> run_replay(const std::string& input_path) {
    std::vector<RefRow> result;

    rc::test::SensorLog log(input_path);
    if (!log.is_open()) return result;

    rc::test::SensorSample sample{};
    if (!log.read_next(sample)) return result;

    rc::ESKF eskf;
    const rc::Vec3 accel_stationary(0.0f, 0.0f, -rc::ESKF::kGravity);
    const rc::Vec3 gyro_zero(0.0f, 0.0f, 0.0f);
    if (!eskf.init(accel_stationary, gyro_zero)) return result;

    // Row 0: initial state
    auto make_row = [](uint64_t ts, const rc::ESKF& e) -> RefRow {
        RefRow r{};
        r.timestamp_us = ts;
        r.cols[0]  = e.q.w;
        r.cols[1]  = e.q.x;
        r.cols[2]  = e.q.y;
        r.cols[3]  = e.q.z;
        r.cols[4]  = e.p.x;
        r.cols[5]  = e.p.y;
        r.cols[6]  = e.p.z;
        r.cols[7]  = e.v.x;
        r.cols[8]  = e.v.y;
        r.cols[9]  = e.v.z;
        r.cols[10] = e.accel_bias.x;
        r.cols[11] = e.accel_bias.y;
        r.cols[12] = e.accel_bias.z;
        r.cols[13] = e.gyro_bias.x;
        r.cols[14] = e.gyro_bias.y;
        r.cols[15] = e.gyro_bias.z;
        for (int32_t i = 0; i < 15; ++i) {
            r.cols[16 + i] = e.P(i, i);
        }
        r.ncols = 31;
        return r;
    };

    result.push_back(make_row(sample.timestamp_us, eskf));

    uint64_t prev_ts = sample.timestamp_us;
    while (log.read_next(sample)) {
        float dt = static_cast<float>(sample.timestamp_us - prev_ts) * 1e-6f;
        prev_ts = sample.timestamp_us;

        rc::Vec3 accel(sample.ax, sample.ay, sample.az);
        rc::Vec3 gyro(sample.gx, sample.gy, sample.gz);
        eskf.predict(accel, gyro, dt);

        // IVP-44: mag heading update (matches replay_harness.cpp)
        if (!std::isnan(sample.mx) && !std::isnan(sample.my) &&
            !std::isnan(sample.mz)) {
            rc::Vec3 mag_body(sample.mx, sample.my, sample.mz);
            eskf.update_mag_heading(mag_body, 0.0f, 0.0f);
        }

        result.push_back(make_row(sample.timestamp_us, eskf));
    }

    return result;
}

// Load reference CSV, return RefRow vector.
std::vector<RefRow> load_reference(const std::string& ref_path) {
    std::vector<RefRow> result;

    std::ifstream file(ref_path);
    if (!file.is_open()) return result;

    std::string line;
    // Skip header
    if (!std::getline(file, line)) return result;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        RefRow row{};
        if (parse_ref_row(line, row)) {
            result.push_back(row);
        }
    }

    return result;
}

const char* kColNames[] = {
    "qw", "qx", "qy", "qz",
    "pn", "pe", "pd",
    "vn", "ve", "vd",
    "ab_x", "ab_y", "ab_z",
    "gb_x", "gb_y", "gb_z",
    "P00", "P01", "P02", "P03", "P04",
    "P05", "P06", "P07", "P08", "P09",
    "P10", "P11", "P12", "P13", "P14"
};

class ReplayRegressionTest : public ::testing::TestWithParam<std::string> {
protected:
    void SetUp() override {
        // Council CR-3: locale-independent decimal parsing
        setlocale(LC_NUMERIC, "C");
    }
};

TEST_P(ReplayRegressionTest, MatchesReference) {
    const std::string& name = GetParam();
    const std::string input_path = "test/data/" + name + ".csv";
    const std::string ref_path = "test/data/reference/" + name + "_ref.csv";

    // Run replay
    auto replay = run_replay(input_path);
    ASSERT_GT(replay.size(), 0u)
        << "Replay produced no output for " << input_path;

    // Load reference
    auto reference = load_reference(ref_path);
    ASSERT_GT(reference.size(), 0u)
        << "Could not load reference file: " << ref_path;

    // Compare row counts
    ASSERT_EQ(replay.size(), reference.size())
        << "Row count mismatch: replay=" << replay.size()
        << " reference=" << reference.size();

    // Compare row-by-row, column-by-column
    for (size_t row = 0; row < replay.size(); ++row) {
        ASSERT_EQ(replay[row].timestamp_us, reference[row].timestamp_us)
            << "Timestamp mismatch at row " << row;

        for (int32_t col = 0; col < 31; ++col) {
            float actual = replay[row].cols[col];
            float expected = reference[row].cols[col];

            EXPECT_TRUE(approx_eq(actual, expected))
                << "Mismatch at row " << row
                << " col " << col << " (" << kColNames[col] << ")"
                << ": expected=" << expected
                << " actual=" << actual
                << " delta=" << std::fabs(actual - expected);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Trajectories,
    ReplayRegressionTest,
    ::testing::Values(
        "static_1min",
        "const_velocity_30s",
        "const_accel_30s",
        "const_turn_10s",
        "banked_turn_10s"
    ),
    [](const ::testing::TestParamInfo<std::string>& info) {
        return info.param;
    }
);

} // namespace
