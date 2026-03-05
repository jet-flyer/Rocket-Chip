// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file log_decimator.h
 * @brief Box-car averaging decimator for FusedState (200Hz→50Hz)
 *
 * Accumulates N samples of FusedState and outputs the average.
 * Float fields: arithmetic mean.
 * Quaternion: component-wise accumulate with sign flip for antipodal
 *   protection (Markley 2007), then normalize.
 * Integer fields (GPS lat/lon, fix, sats): pass-through from final sample.
 * met_ms: final sample's value.
 *
 * SDK-independent — compiles on host and target.
 *
 * IVP-52c: Decimation + Main Loop Integration (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_LOG_DECIMATOR_H
#define ROCKETCHIP_LOG_DECIMATOR_H

#include "rocketchip/fused_state.h"
#include <stdint.h>

namespace rc {

struct LogDecimator {
    FusedState accum;      // Running accumulator
    uint32_t   count;      // Samples accumulated so far
    uint32_t   ratio;      // Decimation ratio (e.g., 4 for 200→50Hz)
    bool       initialized;
};

/**
 * @brief Initialize decimator with given ratio
 * @param dec   Decimator state (caller-owned)
 * @param ratio Decimation ratio (e.g., 4 for 200Hz→50Hz)
 */
void decimator_init(LogDecimator* dec, uint32_t ratio);

/**
 * @brief Feed one sample to the decimator
 * @param dec   Initialized decimator
 * @param input Current FusedState sample
 * @param out   Output averaged FusedState (only valid when return is true)
 * @return true when a decimated output is ready (every ratio-th sample)
 */
bool decimator_push(LogDecimator* dec, const FusedState& input, FusedState& out);

} // namespace rc

#endif // ROCKETCHIP_LOG_DECIMATOR_H
