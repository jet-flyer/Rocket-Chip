#ifndef ROCKETCHIP_TEST_REPLAY_SENSOR_LOG_H
#define ROCKETCHIP_TEST_REPLAY_SENSOR_LOG_H

// Streaming CSV sensor data reader for ESKF replay harness.
// Reads the Section 5.3 format from ESKF_TESTING_GUIDE.md row-by-row.
// Distinct from csv_loader.h (batch unit test loader) — see IVP-42c plan.
//
// Design intent: streaming access for replay loops, all 28 columns parsed,
// NaN preserved (for aiding sensor availability detection), monotonic
// timestamp validation. Positioned for future binary format support (Stage 7)
// without modifying the unit test utility.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace rc {
namespace test {

struct SensorSample {
    uint64_t timestamp_us;

    // IMU (indices 1-6)
    float ax, ay, az;       // Accelerometer (m/s^2)
    float gx, gy, gz;       // Gyroscope (rad/s)

    // Aiding sensors (indices 7-17) — NaN = not available this timestep
    float baro_alt_m;
    float gps_lat_1e7, gps_lon_1e7, gps_alt_m;
    float gps_vn, gps_ve, gps_vd;
    float gps_fix;
    float mx, my, mz;       // Magnetometer

    // Ground truth (indices 18-27)
    // Test-domain only — on-board binary format will not include these.
    float truth_qw, truth_qx, truth_qy, truth_qz;
    float truth_pn, truth_pe, truth_pd;
    float truth_vn, truth_ve, truth_vd;
};

class SensorLog {
public:
    explicit SensorLog(const std::string& path)
        : file_(path), header_skipped_(false), last_timestamp_us_(0) {
        if (!file_.is_open()) {
            return;
        }
        // Skip header row
        std::string header;
        if (std::getline(file_, header)) {
            header_skipped_ = true;
        }
    }

    bool is_open() const { return file_.is_open() && header_skipped_; }

    void reset() {
        file_.clear();
        file_.seekg(0);
        // Re-skip header
        std::string header;
        std::getline(file_, header);
        last_timestamp_us_ = 0;
    }

    // Read next row. Returns false at EOF or on error (with stderr diagnostic).
    bool read_next(SensorSample& sample) {
        std::string line;
        while (std::getline(file_, line)) {
            if (line.empty()) continue;

            // Parse all 28 columns
            const char* p = line.c_str();
            float fields[28]{};
            uint64_t ts = 0;
            int32_t col = 0;

            while (*p && col < 28) {
                const char* start = p;
                while (*p && *p != ',') ++p;

                if (col == 0) {
                    // Timestamp: parse as uint64_t
                    ts = strtoull(start, nullptr, 10);
                } else {
                    // Float field: preserve NaN
                    if (start[0] == 'n' || start[0] == 'N') {
                        fields[col] = NAN;
                    } else {
                        fields[col] = std::strtof(start, nullptr);
                    }
                }

                ++col;
                if (*p == ',') ++p;
            }

            if (col < 28) continue;  // Malformed row, skip

            // Council CR-4: validate monotonic timestamps
            if (last_timestamp_us_ > 0 && ts <= last_timestamp_us_) {
                fprintf(stderr, "sensor_log: non-monotonic timestamp %llu"
                        " <= %llu, skipping row\n",
                        static_cast<unsigned long long>(ts),
                        static_cast<unsigned long long>(last_timestamp_us_));
                continue;
            }
            last_timestamp_us_ = ts;

            // Fill sample
            sample.timestamp_us = ts;

            sample.ax = fields[1];
            sample.ay = fields[2];
            sample.az = fields[3];
            sample.gx = fields[4];
            sample.gy = fields[5];
            sample.gz = fields[6];

            sample.baro_alt_m   = fields[7];
            sample.gps_lat_1e7  = fields[8];
            sample.gps_lon_1e7  = fields[9];
            sample.gps_alt_m    = fields[10];
            sample.gps_vn       = fields[11];
            sample.gps_ve       = fields[12];
            sample.gps_vd       = fields[13];
            sample.gps_fix      = fields[14];
            sample.mx           = fields[15];
            sample.my           = fields[16];
            sample.mz           = fields[17];

            sample.truth_qw = fields[18];
            sample.truth_qx = fields[19];
            sample.truth_qy = fields[20];
            sample.truth_qz = fields[21];
            sample.truth_pn = fields[22];
            sample.truth_pe = fields[23];
            sample.truth_pd = fields[24];
            sample.truth_vn = fields[25];
            sample.truth_ve = fields[26];
            sample.truth_vd = fields[27];

            return true;
        }
        return false;  // EOF
    }

private:
    std::ifstream file_;
    bool header_skipped_;
    uint64_t last_timestamp_us_;
};

} // namespace test
} // namespace rc

#endif // ROCKETCHIP_TEST_REPLAY_SENSOR_LOG_H
