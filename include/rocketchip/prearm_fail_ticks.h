// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Pre-arm Fail Auto-Clear — Pure Helper (Stage L IVP-L3)
//
// Drives the ~3-second visibility window for PhaseIntent::kPreArmFail.
// AO_Notify stores a tick counter; this pure function computes the next
// counter value given the current remaining count and whether a state
// change just occurred.
//
// Semantics:
//   - remaining == 0  → intent cleared, stay at 0
//   - state_changed   → immediate clear (return 0)
//   - otherwise       → decrement (return max(remaining - 1, 0))
//
// Post-flow: every time AO_Notify_post_prearm_fail() is invoked, the
// counter is RESET to kPreArmFailTicks (not decremented). This
// refreshes the full 3-second window so rapid-fire ARM rejections
// don't prematurely clear the previous visual (JPL council 2026-04-18).
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
