// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Box-car averaging decimator for FusedState.
// Listed floats: mean. Quaternion: Markley 2007 then normalize. Listed integers: last sample.
// confident / confidence_div_deg / uncertain_ms: first sample of the window.

#ifndef ROCKETCHIP_LOG_DECIMATOR_H
#define ROCKETCHIP_LOG_DECIMATOR_H

#include "rocketchip/fused_state.h"
#include <stdint.h>

namespace rc {

struct LogDecimator {
    FusedState accum;      // Running accumulator
    uint32_t   count;      // Samples accumulated so far
    uint32_t   ratio;      // Samples per output (caller-supplied)
    bool       initialized;
};

void decimator_init(LogDecimator* dec, uint32_t ratio);

// true when a decimated output is ready (every ratio-th sample)
bool decimator_push(LogDecimator* dec, const FusedState& input, FusedState& out);

} // namespace rc

#endif // ROCKETCHIP_LOG_DECIMATOR_H
