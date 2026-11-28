// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Boot hardware check for PSRAM (plan section 5). Counted only when the
// board pack fits PSRAM (board::kPsramAvailable), like the radio (only
// when SPI came up) and the status LED (only when fitted). PASS only when
// the logger would use it: detected, self-test OK and flash-safe test OK
// (the same predicate main.cpp uses to give the logger the PSRAM ring).

#ifndef ROCKETCHIP_PSRAM_CHECK_POLICY_H
#define ROCKETCHIP_PSRAM_CHECK_POLICY_H

#include <stdint.h>

namespace rc {

enum class PsramCheck : uint8_t {
    kNotCounted,  // board has no PSRAM: not a check, not a pass
    kPass,
    kFail,        // fitted but not detected, or a self-test failed
};

constexpr PsramCheck psram_check(bool fitted, uint32_t size_bytes,
                                 bool self_test_ok, bool flash_safe_ok) {
    if (!fitted) {
        return PsramCheck::kNotCounted;
    }
    return ((size_bytes > 0U) && self_test_ok && flash_safe_ok)
               ? PsramCheck::kPass
               : PsramCheck::kFail;
}

}  // namespace rc

#endif  // ROCKETCHIP_PSRAM_CHECK_POLICY_H
