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

// True when the pack's single onboard LED pin is one channel of its RGB
// status LED. The status LED then owns that pin as PWM; nothing else may
// claim it as a GPIO or report it as a separate LED.
constexpr bool onboard_led_is_rgb_channel(bool rgb_led_available,
                                          uint8_t led_pin, uint8_t red_pin,
                                          uint8_t green_pin, uint8_t blue_pin) {
    return rgb_led_available &&
           (led_pin == red_pin || led_pin == green_pin || led_pin == blue_pin);
}

// Boot hardware check: a status LED counts (PASS/FAIL) only when the pack
// fits one, and it is named for what is fitted.
constexpr bool status_led_counted(StatusLedKind kind) {
    return kind != StatusLedKind::kNone;
}

constexpr const char* status_led_label(StatusLedKind kind) {
    switch (kind) {
    case StatusLedKind::kWs2812:      return "NeoPixel";
    case StatusLedKind::kDiscreteRgb: return "RGB status LED";
    case StatusLedKind::kNone:
    default:                          return "none";
    }
}

} // namespace rc

#endif // ROCKETCHIP_STATUS_LED_POLICY_H
