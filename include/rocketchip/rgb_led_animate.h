// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Discrete-RGB frame of a status-LED mode. Pure (no SDK). Timings and
// HSV match src/drivers/ws2812_status.cpp so the RGB backend can follow
// the same patterns the WS2812 driver uses.

#ifndef ROCKETCHIP_RGB_LED_ANIMATE_H
#define ROCKETCHIP_RGB_LED_ANIMATE_H

#include <stdint.h>
#include <math.h>

namespace rc {

enum class RgbAnimMode : uint8_t {
    kOff = 0,
    kSolid,
    kBreathe,
    kBlink,
    kBlinkFast,
    kRainbow,
    kAlternate,
    kDoubleFlash,
};

struct Rgb8 {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

inline constexpr uint32_t kRgbBreathePeriodMs    = 2000;
inline constexpr uint32_t kRgbBlinkOnMs          = 500;
inline constexpr uint32_t kRgbBlinkOffMs         = 500;
inline constexpr uint32_t kRgbFastBlinkOnMs      = 100;
inline constexpr uint32_t kRgbFastBlinkOffMs     = 100;
inline constexpr uint32_t kRgbAlternateHalfMs    = 250;
inline constexpr uint32_t kRgbDoubleFlashP0OnMs  = 100;
inline constexpr uint32_t kRgbDoubleFlashP1OffMs = 100;
inline constexpr uint32_t kRgbDoubleFlashP2OnMs  = 100;
inline constexpr uint32_t kRgbDoubleFlashP3OffMs = 700;
inline constexpr uint32_t kRgbDoubleFlashPeriodMs =
    kRgbDoubleFlashP0OnMs + kRgbDoubleFlashP1OffMs +
    kRgbDoubleFlashP2OnMs + kRgbDoubleFlashP3OffMs;
inline constexpr float kRgbRainbowCyclePeriodMs = 6000.0F;
inline constexpr float kRgbRainbowSaturation    = 100.0F;
inline constexpr float kRgbRainbowValue         = 25.0F;
inline constexpr float kRgbBreatheMinScale      = 0.1F;
inline constexpr float kRgbBreatheRange         = 0.9F;
inline constexpr float kRgbPi                   = 3.14159F;
inline constexpr float kRgbAchromaticThreshold  = 0.001F;
inline constexpr float kRgbMaxChannelValue      = 255.0F;
inline constexpr float kRgbPercentScale         = 100.0F;
inline constexpr float kRgbHueFull              = 360.0F;
inline constexpr float kRgbHueSector            = 60.0F;

// Discrete RGB LED pins, in color order (R, G, B).
struct RgbLedPins {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct PwmPinLevel {
    uint8_t pin;
    uint16_t level;
};

struct RgbPwmLevels {
    PwmPinLevel r;
    PwmPinLevel g;
    PwmPinLevel b;
};

// PWM counter wrap for the RGB backend: one count per 8-bit duty step.
inline constexpr uint16_t kRgbPwmWrap = 255;

// 8-bit duty to a PWM compare level. An active-low channel is lit when the
// pin is low, so its level is inverted against the wrap.
constexpr uint16_t rgb_pwm_level(uint8_t value, bool active_low) {
    return active_low ? static_cast<uint16_t>(kRgbPwmWrap - value)
                      : static_cast<uint16_t>(value);
}

// Color channel -> pin and level. c.r drives pins.r, c.g pins.g, c.b pins.b.
constexpr RgbPwmLevels rgb_pwm_levels(Rgb8 c, RgbLedPins pins, bool active_low) {
    return RgbPwmLevels{
        PwmPinLevel{pins.r, rgb_pwm_level(c.r, active_low)},
        PwmPinLevel{pins.g, rgb_pwm_level(c.g, active_low)},
        PwmPinLevel{pins.b, rgb_pwm_level(c.b, active_low)},
    };
}

inline Rgb8 rgb8_scale(Rgb8 c, float scale) {
    auto ch = [scale](uint8_t v) -> uint8_t {
        const float x = static_cast<float>(v) * scale;
        if (x <= 0.0F) {
            return 0;
        }
        if (x >= 255.0F) {
            return 255;
        }
        return static_cast<uint8_t>(x);
    };
    return Rgb8{ch(c.r), ch(c.g), ch(c.b)};
}

inline Rgb8 rgb_hsv_to_rgb(float h, float s, float v) {
    Rgb8 rgb = {0, 0, 0};
    h = fmodf(h, kRgbHueFull);
    if (h < 0.0F) {
        h += kRgbHueFull;
    }
    s = s / kRgbPercentScale;
    v = v / kRgbPercentScale;

    if (s < kRgbAchromaticThreshold) {
        const uint8_t gray = static_cast<uint8_t>(v * kRgbMaxChannelValue);
        rgb.r = rgb.g = rgb.b = gray;
        return rgb;
    }

    const float c = v * s;
    const float x = c * (1.0F - fabsf(fmodf(h / kRgbHueSector, 2.0F) - 1.0F));
    const float m = v - c;
    float r1 = 0.0F;
    float g1 = 0.0F;
    float b1 = 0.0F;
    if (h < kRgbHueSector) {
        r1 = c; g1 = x; b1 = 0.0F;
    } else if (h < 120.0F) {
        r1 = x; g1 = c; b1 = 0.0F;
    } else if (h < 180.0F) {
        r1 = 0.0F; g1 = c; b1 = x;
    } else if (h < 240.0F) {
        r1 = 0.0F; g1 = x; b1 = c;
    } else if (h < 300.0F) {
        r1 = x; g1 = 0.0F; b1 = c;
    } else {
        r1 = c; g1 = 0.0F; b1 = x;
    }
    rgb.r = static_cast<uint8_t>((r1 + m) * kRgbMaxChannelValue);
    rgb.g = static_cast<uint8_t>((g1 + m) * kRgbMaxChannelValue);
    rgb.b = static_cast<uint8_t>((b1 + m) * kRgbMaxChannelValue);
    return rgb;
}

// alt_half_ms is the kAlternate half-period; 0 maps to kRgbAlternateHalfMs
// (same remap as ws2812_set_mode_alternate).
inline Rgb8 rgb_led_color(RgbAnimMode mode, Rgb8 base, Rgb8 alt,
                          uint32_t elapsed_ms,
                          uint32_t alt_half_ms = kRgbAlternateHalfMs) {
    switch (mode) {
    case RgbAnimMode::kOff:
        return Rgb8{0, 0, 0};
    case RgbAnimMode::kSolid:
        return base;
    case RgbAnimMode::kBreathe: {
        const float phase = static_cast<float>(elapsed_ms) /
                            static_cast<float>(kRgbBreathePeriodMs);
        float scale = (sinf(phase * 2.0F * kRgbPi) + 1.0F) / 2.0F;
        scale = kRgbBreatheMinScale + scale * kRgbBreatheRange;
        return rgb8_scale(base, scale);
    }
    case RgbAnimMode::kBlink: {
        const uint32_t period = kRgbBlinkOnMs + kRgbBlinkOffMs;
        return ((elapsed_ms % period) < kRgbBlinkOnMs) ? base : Rgb8{0, 0, 0};
    }
    case RgbAnimMode::kBlinkFast: {
        const uint32_t period = kRgbFastBlinkOnMs + kRgbFastBlinkOffMs;
        return ((elapsed_ms % period) < kRgbFastBlinkOnMs) ? base
                                                           : Rgb8{0, 0, 0};
    }
    case RgbAnimMode::kRainbow: {
        const float hue = fmodf(static_cast<float>(elapsed_ms) /
                                    kRgbRainbowCyclePeriodMs * kRgbHueFull,
                                kRgbHueFull);
        return rgb_hsv_to_rgb(hue, kRgbRainbowSaturation, kRgbRainbowValue);
    }
    case RgbAnimMode::kAlternate: {
        const uint32_t half = (alt_half_ms != 0U) ? alt_half_ms
                                                  : kRgbAlternateHalfMs;
        const uint32_t period = half * 2U;
        return ((elapsed_ms % period) < half) ? base : alt;
    }
    case RgbAnimMode::kDoubleFlash: {
        const uint32_t phase = elapsed_ms % kRgbDoubleFlashPeriodMs;
        const uint32_t p1 = kRgbDoubleFlashP0OnMs;
        const uint32_t p2 = p1 + kRgbDoubleFlashP1OffMs;
        const uint32_t p3 = p2 + kRgbDoubleFlashP2OnMs;
        const bool on = (phase < p1) || (phase >= p2 && phase < p3);
        return on ? base : Rgb8{0, 0, 0};
    }
    }
    return Rgb8{0, 0, 0};
}

} // namespace rc

#endif // ROCKETCHIP_RGB_LED_ANIMATE_H
