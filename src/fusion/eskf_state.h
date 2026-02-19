#ifndef ROCKETCHIP_FUSION_ESKF_STATE_H
#define ROCKETCHIP_FUSION_ESKF_STATE_H

// ESKF state vector indices.
// Pure C++ — no Pico SDK dependencies.
//
// 15-state error-state Kalman filter per Sola (2017) S5.
// Error state: dx = [d_theta(3), d_p(3), d_v(3), d_a_bias(3), d_g_bias(3)]
// Named indices prevent off-by-one errors in F_x, H, and K construction.

#include <cstdint>

namespace rc::eskf {

constexpr int32_t kIdxAttitude  = 0;   // delta_theta [0..2]
constexpr int32_t kIdxPosition  = 3;   // delta_p     [3..5]
constexpr int32_t kIdxVelocity  = 6;   // delta_v     [6..8]
constexpr int32_t kIdxAccelBias = 9;   // delta_a_bias [9..11]
constexpr int32_t kIdxGyroBias  = 12;  // delta_g_bias [12..14]
constexpr int32_t kStateSize    = 15;  // Total error state dimension

// Block sizes (all 3-element subvectors)
constexpr int32_t kBlockSize = 3;

// Per-axis named indices (used in measurement update H-vector construction)
constexpr int32_t kIdxPosN = kIdxPosition + 0;   // 3
constexpr int32_t kIdxPosE = kIdxPosition + 1;   // 4
constexpr int32_t kIdxPosD = kIdxPosition + 2;   // 5
constexpr int32_t kIdxVelN = kIdxVelocity + 0;   // 6
constexpr int32_t kIdxVelE = kIdxVelocity + 1;   // 7
constexpr int32_t kIdxVelD = kIdxVelocity + 2;   // 8
constexpr int32_t kIdxYaw  = kIdxAttitude + 2;   // 2

} // namespace rc::eskf

#endif // ROCKETCHIP_FUSION_ESKF_STATE_H
