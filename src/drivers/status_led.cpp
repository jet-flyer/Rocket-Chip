// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Status LED output: dispatch to WS2812 or a discrete RGB PWM backend
// chosen by board-pack constants (no per-board #ifdefs).

#include "status_led.h"
#include "rocketchip/board.h"
#include "rocketchip/status_led_policy.h"
#include "rocketchip/rgb_led_animate.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

namespace {

constexpr uint16_t kRgbPwmWrap = 255;
constexpr float kRgbPwmClkDiv = 32.0F;  // ~18 kHz at 150 MHz

bool g_ready = false;
bool g_ws2812 = false;
ws2812_mode_t g_mode = WS2812_MODE_OFF;
ws2812_rgb_t g_base = kColorOff;
ws2812_rgb_t g_alt = kColorOff;
uint32_t g_modeStartMs = 0;
uint32_t g_altHalfMs = rc::kRgbAlternateHalfMs;

constexpr rc::StatusLedKind kind() {
    return rc::status_led_kind(board::kNeoPixelCount, board::kRgbLedAvailable);
}

rc::RgbAnimMode to_anim(ws2812_mode_t mode) {
    switch (mode) {
    case WS2812_MODE_SOLID:        return rc::RgbAnimMode::kSolid;
    case WS2812_MODE_BREATHE:      return rc::RgbAnimMode::kBreathe;
    case WS2812_MODE_BLINK:        return rc::RgbAnimMode::kBlink;
    case WS2812_MODE_BLINK_FAST:   return rc::RgbAnimMode::kBlinkFast;
    case WS2812_MODE_RAINBOW:      return rc::RgbAnimMode::kRainbow;
    case WS2812_MODE_ALTERNATE:    return rc::RgbAnimMode::kAlternate;
    case WS2812_MODE_DOUBLE_FLASH: return rc::RgbAnimMode::kDoubleFlash;
    case WS2812_MODE_PIXELS:       return rc::RgbAnimMode::kSolid;
    case WS2812_MODE_OFF:
    default:                       return rc::RgbAnimMode::kOff;
    }
}

rc::Rgb8 to_rgb8(ws2812_rgb_t c) {
    return rc::Rgb8{c.r, c.g, c.b};
}

void rgb_pwm_write_pin(uint8_t pin, uint8_t value) {
    uint16_t level = value;
    if constexpr (board::kLedRgbActiveLow) {
        level = static_cast<uint16_t>(kRgbPwmWrap - level);
    }
    pwm_set_gpio_level(pin, level);
}

void rgb_write(rc::Rgb8 c) {
    rgb_pwm_write_pin(board::kLedRedPin, c.r);
    rgb_pwm_write_pin(board::kLedGreenPin, c.g);
    rgb_pwm_write_pin(board::kLedBluePin, c.b);
}

void rgb_init_pins() {
    const uint8_t pins[3] = {
        board::kLedRedPin, board::kLedGreenPin, board::kLedBluePin
    };
    bool slice_done[NUM_PWM_SLICES] = {};
    for (uint8_t pin : pins) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        const uint slice = pwm_gpio_to_slice_num(pin);
        if (!slice_done[slice]) {
            pwm_config cfg = pwm_get_default_config();
            pwm_config_set_wrap(&cfg, kRgbPwmWrap);
            pwm_config_set_clkdiv(&cfg, kRgbPwmClkDiv);
            pwm_init(slice, &cfg, true);
            slice_done[slice] = true;
        }
    }
    rgb_write(rc::Rgb8{0, 0, 0});
}

void rgb_apply_now() {
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    const uint32_t elapsed = now - g_modeStartMs;
    const rc::Rgb8 out = rc::rgb_led_color(to_anim(g_mode), to_rgb8(g_base),
                                          to_rgb8(g_alt), elapsed, g_altHalfMs);
    rgb_write(out);
}

} // namespace

bool status_led_init(void) {
    g_ready = false;
    g_ws2812 = false;
    g_mode = WS2812_MODE_OFF;
    g_base = kColorOff;
    g_alt = kColorOff;
    g_modeStartMs = 0;
    g_altHalfMs = rc::kRgbAlternateHalfMs;

    if constexpr (kind() == rc::StatusLedKind::kWs2812) {
        g_ready = ws2812_status_init(pio0, board::kNeoPixelPin,
                                     board::kNeoPixelCount);
        g_ws2812 = g_ready;
        return g_ready;
    } else if constexpr (kind() == rc::StatusLedKind::kDiscreteRgb) {
        rgb_init_pins();
        g_ready = true;
        return true;
    } else {
        return false;
    }
}

bool status_led_ws2812_ready(void) {
    return g_ws2812;
}

void status_led_set_mode(ws2812_mode_t mode, ws2812_rgb_t color) {
    if (!g_ready) {
        return;
    }
    g_mode = mode;
    g_base = color;
    g_modeStartMs = to_ms_since_boot(get_absolute_time());
    if constexpr (kind() == rc::StatusLedKind::kWs2812) {
        ws2812_set_mode(mode, color);
    } else if constexpr (kind() == rc::StatusLedKind::kDiscreteRgb) {
        rgb_apply_now();
    }
}

void status_led_set_mode_alternate(ws2812_rgb_t a, ws2812_rgb_t b,
                                   uint32_t halfPeriodMs) {
    if (!g_ready) {
        return;
    }
    g_mode = WS2812_MODE_ALTERNATE;
    g_base = a;
    g_alt = b;
    g_altHalfMs = halfPeriodMs;  // 0 remaps to 250 ms in rgb_led_color
    g_modeStartMs = to_ms_since_boot(get_absolute_time());
    if constexpr (kind() == rc::StatusLedKind::kWs2812) {
        ws2812_set_mode_alternate(a, b, halfPeriodMs);
    } else if constexpr (kind() == rc::StatusLedKind::kDiscreteRgb) {
        rgb_apply_now();
    }
}

void status_led_update(void) {
    if (!g_ready) {
        return;
    }
    if constexpr (kind() == rc::StatusLedKind::kWs2812) {
        ws2812_update();
    } else if constexpr (kind() == rc::StatusLedKind::kDiscreteRgb) {
        rgb_apply_now();
    }
}

void status_led_set_rssi_bar(int16_t rssi_dbm, bool searching) {
    if (!g_ready) {
        return;
    }
    if constexpr (kind() == rc::StatusLedKind::kWs2812) {
        ws2812_set_rssi_bar(rssi_dbm, searching);
    } else if constexpr (kind() == rc::StatusLedKind::kDiscreteRgb) {
        if (searching || rssi_dbm == 0) {
            status_led_set_mode(WS2812_MODE_OFF, kColorOff);
            return;
        }
        // Locked: solid green. Brightness is not RSSI-graded on one LED.
        (void)rssi_dbm;
        status_led_set_mode(WS2812_MODE_SOLID, kColorGreen);
    }
}

void status_led_set_sweep_bar(ws2812_rgb_t color) {
    if (!g_ready) {
        return;
    }
    if constexpr (kind() == rc::StatusLedKind::kWs2812) {
        ws2812_set_sweep_bar(color);
    } else if constexpr (kind() == rc::StatusLedKind::kDiscreteRgb) {
        status_led_set_mode(WS2812_MODE_BLINK, color);
    }
}

void status_led_set_flash_bar(ws2812_rgb_t color, bool on) {
    if (!g_ready) {
        return;
    }
    if constexpr (kind() == rc::StatusLedKind::kWs2812) {
        ws2812_set_flash_bar(color, on);
    } else if constexpr (kind() == rc::StatusLedKind::kDiscreteRgb) {
        status_led_set_mode(on ? WS2812_MODE_SOLID : WS2812_MODE_OFF,
                            on ? color : kColorOff);
    }
}
