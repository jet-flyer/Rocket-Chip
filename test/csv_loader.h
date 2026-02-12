#ifndef ROCKETCHIP_TEST_CSV_LOADER_H
#define ROCKETCHIP_TEST_CSV_LOADER_H

// Minimal CSV loader for ESKF trajectory test data.
// Parses the Section 5.3 format from ESKF_TESTING_GUIDE.md.
// No external dependencies — just <fstream>, <string>, <vector>, <cstdlib>.

#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace rc {
namespace test {

struct Sample {
    // IMU columns (indices 1-6)
    float ax, ay, az;
    float gx, gy, gz;

    // Ground truth (indices 18-27)
    float truth_qw, truth_qx, truth_qy, truth_qz;
    float truth_pn, truth_pe, truth_pd;
    float truth_vn, truth_ve, truth_vd;
};

// Parse a single float field from a CSV token.
// Returns 0.0f for "nan" fields.
inline float parse_field(const char* s) {
    if (s[0] == 'n' || s[0] == 'N') return 0.0f;  // nan
    return std::strtof(s, nullptr);
}

// Load CSV file, return vector of Samples.
// Skips the header row. Parses IMU + truth columns by index.
inline std::vector<Sample> load_csv(const std::string& path) {
    std::vector<Sample> samples;
    std::ifstream file(path);
    if (!file.is_open()) return samples;

    std::string line;
    // Skip header
    if (!std::getline(file, line)) return samples;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        // Split by comma into tokens
        const char* p = line.c_str();
        float fields[28]{};
        int col = 0;

        while (*p && col < 28) {
            const char* start = p;
            while (*p && *p != ',') ++p;
            fields[col] = parse_field(start);
            ++col;
            if (*p == ',') ++p;
        }

        if (col < 28) continue;  // malformed row

        Sample s{};
        s.ax = fields[1];
        s.ay = fields[2];
        s.az = fields[3];
        s.gx = fields[4];
        s.gy = fields[5];
        s.gz = fields[6];

        s.truth_qw = fields[18];
        s.truth_qx = fields[19];
        s.truth_qy = fields[20];
        s.truth_qz = fields[21];
        s.truth_pn = fields[22];
        s.truth_pe = fields[23];
        s.truth_pd = fields[24];
        s.truth_vn = fields[25];
        s.truth_ve = fields[26];
        s.truth_vd = fields[27];

        samples.push_back(s);
    }

    return samples;
}

} // namespace test
} // namespace rc

#endif // ROCKETCHIP_TEST_CSV_LOADER_H
