// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Pre-arm fail auto-clear (~3 s yellow double-flash).
// remaining==0 stays 0; state_changed clears; else decrement.
// AO_Notify_post_prearm_fail() resets to kPreArmFailTicks (not a tick)
// so repeated ARM rejects keep the visual up.
//============================================================================
#ifndef ROCKETCHIP_PREARM_FAIL_TICKS_H
#define ROCKETCHIP_PREARM_FAIL_TICKS_H

#include <stdint.h>

namespace rc {

// AO_Notify's tick handler runs at ~33 Hz (every 3 ticks at 100 Hz base),
// so 99 ticks ≈ 3.0 seconds of yellow double-flash visibility before the
// pre-arm-fail intent auto-clears.
static constexpr uint32_t kPreArmFailTicks = 99U;

// Pure helper — no globals, no side effects. Host-testable.
inline uint32_t prearm_fail_tick_next(uint32_t remaining, bool state_changed) {
    if (state_changed) {
        return 0U;
    }
    if (remaining == 0U) {
        return 0U;
    }
    return remaining - 1U;
}

} // namespace rc

#endif // ROCKETCHIP_PREARM_FAIL_TICKS_H
