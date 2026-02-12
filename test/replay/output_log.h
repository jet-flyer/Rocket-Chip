#ifndef ROCKETCHIP_TEST_REPLAY_OUTPUT_LOG_H
#define ROCKETCHIP_TEST_REPLAY_OUTPUT_LOG_H

// CSV writer for ESKF output (state + P diagonal).
// Used by both replay_harness.cpp and test_replay_regression.cpp.
//
// Output format: 32 columns (see IVP-42c plan for full spec).
// Float format: %.9g (council CR-1: FLT_DECIMAL_DIG = 9, minimum for
// guaranteed single-precision round-trip).
//
// IVP-43 extension point: append nis_baro as column 32 when baro
// measurement update lands.

#include <cstdint>
#include <cstdio>

#include "fusion/eskf.h"
#include "fusion/eskf_state.h"

namespace rc {
namespace test {

inline void write_output_header(FILE* f) {
    fprintf(f,
        "timestamp_us,qw,qx,qy,qz,pn,pe,pd,vn,ve,vd,"
        "ab_x,ab_y,ab_z,gb_x,gb_y,gb_z,"
        "P00,P01,P02,P03,P04,P05,P06,P07,P08,P09,P10,P11,P12,P13,P14\n");
}

inline void write_output_row(FILE* f, uint64_t ts, const ESKF& eskf) {
    fprintf(f, "%llu", static_cast<unsigned long long>(ts));

    // Quaternion (4)
    fprintf(f, ",%.9g,%.9g,%.9g,%.9g",
            static_cast<double>(eskf.q.w),
            static_cast<double>(eskf.q.x),
            static_cast<double>(eskf.q.y),
            static_cast<double>(eskf.q.z));

    // Position NED (3)
    fprintf(f, ",%.9g,%.9g,%.9g",
            static_cast<double>(eskf.p.x),
            static_cast<double>(eskf.p.y),
            static_cast<double>(eskf.p.z));

    // Velocity NED (3)
    fprintf(f, ",%.9g,%.9g,%.9g",
            static_cast<double>(eskf.v.x),
            static_cast<double>(eskf.v.y),
            static_cast<double>(eskf.v.z));

    // Accel bias (3)
    fprintf(f, ",%.9g,%.9g,%.9g",
            static_cast<double>(eskf.accel_bias.x),
            static_cast<double>(eskf.accel_bias.y),
            static_cast<double>(eskf.accel_bias.z));

    // Gyro bias (3)
    fprintf(f, ",%.9g,%.9g,%.9g",
            static_cast<double>(eskf.gyro_bias.x),
            static_cast<double>(eskf.gyro_bias.y),
            static_cast<double>(eskf.gyro_bias.z));

    // P diagonal (15)
    for (int32_t i = 0; i < rc::eskf::kStateSize; ++i) {
        fprintf(f, ",%.9g", static_cast<double>(eskf.P(i, i)));
    }

    fprintf(f, "\n");
}

} // namespace test
} // namespace rc

#endif // ROCKETCHIP_TEST_REPLAY_OUTPUT_LOG_H
