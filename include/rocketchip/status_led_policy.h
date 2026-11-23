// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Status LED backend selection from board-pack constants. Pure (no SDK).
// WS2812 wins if both a NeoPixel count and a discrete RGB are set; packs
// should not advertise both.

#ifndef ROCKETCHIP_STATUS_LED_POLICY_H
#define ROCKETCHIP_STATUS_LED_POLICY_H

#include <stdint.h>

namespace rc {

enum class StatusLedKind : uint8_t {
    kNone = 0,
    kWs2812,
    kDiscreteRgb,
};

constexpr StatusLedKind status_led_kind(uint8_t neo_pixel_count,
                                        bool rgb_led_available) {
    if (neo_pixel_count > 0U) {
        return StatusLedKind::kWs2812;
    }
    if (rgb_led_available) {
        return StatusLedKind::kDiscreteRgb;
    }
    return StatusLedKind::kNone;
}

} // namespace rc

#endif // ROCKETCHIP_STATUS_LED_POLICY_H
