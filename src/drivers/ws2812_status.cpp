// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file ws2812_status.c
 * @brief WS2812 NeoPixel status LED driver implementation
 *
 * Prior Art:
 *   - Pico SDK examples/pio/ws2812 (PIO program for WS2812 protocol)
 *   - WS2812B datasheet (timing requirements: T0H/T1H/T0L/T1L)
 *   - Adafruit NeoPixel library (color format, brightness scaling)
 */

#include "ws2812_status.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "ws2812.pio.h"
#include <math.h>

// WS2812 protocol constants
constexpr float    kWs2812BitRate       = 800000.0F;  // 800kHz data rate
constexpr uint32_t kDefaultBreathePeriodMs = 2000;
constexpr uint32_t kDefaultBlinkOnMs    = 500;
constexpr uint32_t kDefaultBlinkOffMs   = 500;
constexpr uint32_t kFastBlinkPeriodMs   = 200;  // 5Hz = 100ms on + 100ms off
constexpr uint32_t kFastBlinkOnMs       = 100;
constexpr uint32_t kDefaultAlternateHalfMs = 250;  // Stage L: 250ms each = 2Hz toggle
// Double-flash: 100ms on, 100ms off, 100ms on, 700ms off = 1s full cycle (Stage L, AP-parity pre-arm-fail shape)
constexpr uint32_t kDoubleFlashP0OnMs   = 100;   // first pulse on
constexpr uint32_t kDoubleFlashP1OffMs  = 100;   // gap between pulses
constexpr uint32_t kDoubleFlashP2OnMs   = 100;   // second pulse on
constexpr uint32_t kDoubleFlashP3OffMs  = 700;   // long pause
constexpr uint32_t kDoubleFlashPeriodMs = kDoubleFlashP0OnMs + kDoubleFlashP1OffMs +
                                           kDoubleFlashP2OnMs + kDoubleFlashP3OffMs;
constexpr float    kRainbowCyclePeriodMs = 6000.0F;  // Full HSV rotation period
constexpr float    kRainbowSaturation   = 100.0F;
constexpr float    kRainbowValue        = 25.0F;     // Max brightness for rainbow mode
constexpr float    kBreatheMinScale     = 0.1F;      // Minimum brightness during breathe
constexpr float    kBreatheRange        = 0.9F;       // 1.0 - kBreatheMinScale
constexpr float    kPi                  = 3.14159F;
constexpr float    kAchromaticThreshold = 0.001F;     // Below this saturation, treat as gray
constexpr float    kMaxChannelValue     = 255.0F;
constexpr float    kPercentScale        = 100.0F;
constexpr float    kHueFull             = 360.0F;
constexpr float    kHueSector           = 60.0F;      // HSV sector width (360/6)
constexpr uint32_t kGrbGreenShift       = 24;         // GRB bit position: green in upper byte

// ============================================================================
// Private State
// ============================================================================

static struct {
    PIO pio;
    uint sm;
    uint offset;
    bool initialized;
    uint8_t numLeds;       // Number of LEDs in chain (board::kNeoPixelCount)

    // Per-pixel buffer for multi-LED patterns (RSSI bar, etc.)
    ws2812_rgb_t pixels[8];            // Max 8 LEDs per chain

    // Current mode and color
    ws2812_mode_t mode;
    ws2812_rgb_t baseColor;
    ws2812_rgb_t altColor;   // Stage L: secondary color for MODE_ALTERNATE
    uint8_t brightness;

    // Pattern timing
    uint32_t breathePeriodMs;
    uint32_t blinkOnMs;
    uint32_t blinkOffMs;
    uint32_t alternateHalfMs;  // Stage L: time each color shows in MODE_ALTERNATE

    // Animation state
    uint32_t lastUpdateMs;
    uint32_t phaseStartMs;
    bool blinkState;
    float rainbowHue;
} g_state = {
    .pio = nullptr,
    .sm = 0,
    .offset = 0,
    .initialized = false,
    .numLeds = 1,
    .pixels = {},
    .mode = WS2812_MODE_OFF,
    .baseColor = {0, 0, 0},
    .altColor = {0, 0, 0},
    .brightness = 255,
    .breathePeriodMs = kDefaultBreathePeriodMs,
    .blinkOnMs = kDefaultBlinkOnMs,
    .blinkOffMs = kDefaultBlinkOffMs,
    .alternateHalfMs = kDefaultAlternateHalfMs,
    .lastUpdateMs = 0,
    .phaseStartMs = 0,
    .blinkState = false,
    .rainbowHue = 0.0F,
};

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Send raw RGB data to WS2812 via PIO
 */
static void send_pixel(uint8_t r, uint8_t g, uint8_t b) {
    if (!g_state.initialized) {
        return;
    }

    // Apply global brightness
    if (g_state.brightness < 255) {
        r = (r * g_state.brightness) >> 8;
        g = (g * g_state.brightness) >> 8;
        b = (b * g_state.brightness) >> 8;
    }

    // WS2812 expects GRB order, data in upper 24 bits
    uint32_t grb = (static_cast<uint32_t>(g) << kGrbGreenShift) | (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(b) << 8);

    // [S3] Send same color to all LEDs in chain (1 on Feather, 5 on Fruit Jam).
    // Per-LED control (RSSI bar, etc.) is Stage 7 scope via status engine.
    for (uint8_t i = 0; i < g_state.numLeds; ++i) {
        pio_sm_put_blocking(g_state.pio, g_state.sm, grb);
    }
}

// ============================================================================
// Per-Pixel Functions (RSSI bar for Fruit Jam 5-LED strip)
// ============================================================================

void ws2812_set_pixel_rgb(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (!g_state.initialized || index >= g_state.numLeds) { return; }
    g_state.pixels[index] = {r, g, b};
}

void ws2812_show() {
    if (!g_state.initialized) { return; }

    for (uint8_t i = 0; i < g_state.numLeds; ++i) {
        ws2812_rgb_t px = g_state.pixels[i];
        // Apply global brightness
        if (g_state.brightness < 255) {
            px.r = (px.r * g_state.brightness) >> 8;
            px.g = (px.g * g_state.brightness) >> 8;
            px.b = (px.b * g_state.brightness) >> 8;
        }
        uint32_t grb = (static_cast<uint32_t>(px.g) << kGrbGreenShift)
                      | (static_cast<uint32_t>(px.r) << 16)
                      | (static_cast<uint32_t>(px.b) << 8);
        pio_sm_put_blocking(g_state.pio, g_state.sm, grb);
    }
}

void ws2812_set_rssi_bar(int16_t rssi, bool no_signal) {
    if (!g_state.initialized) { return; }

    // Clear all pixels
    for (uint8_t i = 0; i < g_state.numLeds; ++i) {
        g_state.pixels[i] = kColorOff;
    }

    if (no_signal) {
        // No signal: pixel 0 = dim red
        g_state.pixels[0] = {0x10, 0x00, 0x00};
        ws2812_show();
        return;
    }

    // Map RSSI to lit pixel count (1-5 for 5-LED strip)
    // -40 dBm = excellent (5 pixels), -120 dBm = barely detectable (1 pixel)
    uint8_t max_px = g_state.numLeds;
    uint8_t lit;
    if (rssi >= -60) {
        lit = max_px;        // 5/5: excellent
    } else if (rssi >= -70) {
        lit = (max_px >= 4) ? 4 : max_px;  // 4/5: good
    } else if (rssi >= -80) {
        lit = (max_px >= 3) ? 3 : max_px;  // 3/5: ok
    } else if (rssi >= -95) {
        lit = (max_px >= 2) ? 2 : max_px;  // 2/5: weak
    } else {
        lit = 1;             // 1/5: very weak
    }

    // Color: green (strong) → yellow (mid) → red (weak)
    for (uint8_t i = 0; i < lit; ++i) {
        if (i < (lit / 2)) {
            g_state.pixels[i] = kColorGreen;   // Lower LEDs green
        } else if (i == lit - 1 && rssi < -80) {
            g_state.pixels[i] = kColorRed;     // Top LED red when weak
        } else {
            g_state.pixels[i] = (rssi < -70) ? kColorYellow : kColorGreen;
        }
    }

    ws2812_show();
}

// Stage T IVP-T5.5 sub 2g — KITT/Cylon sweep on LED bar.
// Advances one step per call — caller throttles cadence. Position + direction
// persist in file-scope statics. Lights exactly 1 pixel at a time.
void ws2812_set_sweep_bar(ws2812_rgb_t color) {
    if (!g_state.initialized) { return; }
    static uint8_t s_pos = 0;
    static int8_t  s_dir = 1;
    const uint8_t n = g_state.numLeds;
    if (n == 0) { return; }
    // Edge guard — single-LED strips just show the color solid.
    if (n == 1) {
        g_state.pixels[0] = color;
        ws2812_show();
        return;
    }
    for (uint8_t i = 0; i < n; ++i) { g_state.pixels[i] = kColorOff; }
    g_state.pixels[s_pos] = color;
    ws2812_show();
    // Advance + bounce at ends.
    if (s_pos == 0)      { s_dir = 1; }
    if (s_pos == n - 1U) { s_dir = -1; }
    s_pos = static_cast<uint8_t>(
        static_cast<int>(s_pos) + static_cast<int>(s_dir));
}

/**
 * @brief Apply brightness scaling to a color
 */
static ws2812_rgb_t apply_brightness(ws2812_rgb_t color, float scale) {
    ws2812_rgb_t result;
    result.r = static_cast<uint8_t>(static_cast<float>(color.r) * scale);
    result.g = static_cast<uint8_t>(static_cast<float>(color.g) * scale);
    result.b = static_cast<uint8_t>(static_cast<float>(color.b) * scale);
    return result;
}

// ============================================================================
// Initialization
// ============================================================================

bool ws2812_status_init(PIO pio, uint pin, uint8_t num_leds) {
    if (g_state.initialized) {
        return true;  // Already initialized
    }

    // Claim a state machine and load the program
    uint sm = 0;
    uint offset = 0;

    if (!pio_claim_free_sm_and_add_program_for_gpio_range(
            &ws2812_program, &pio, &sm, &offset, pin, 1, true)) {
        return false;
    }

    // Initialize PIO program (800kHz, RGB mode)
    ws2812_program_init(pio, sm, offset, pin, kWs2812BitRate, false);

    g_state.pio = pio;
    g_state.sm = sm;
    g_state.offset = offset;
    g_state.initialized = true;
    g_state.numLeds = (num_leds > 0) ? num_leds : 1;
    g_state.lastUpdateMs = to_ms_since_boot(get_absolute_time());
    g_state.phaseStartMs = g_state.lastUpdateMs;

    // Start with LEDs off
    send_pixel(0, 0, 0);

    return true;
}

void ws2812_status_deinit() {
    if (!g_state.initialized) {
        return;
    }

    // Turn off LED
    send_pixel(0, 0, 0);

    // Release PIO resources
    pio_remove_program_and_unclaim_sm(&ws2812_program, g_state.pio,
                                       g_state.sm, g_state.offset);
    g_state.initialized = false;
}

// ============================================================================
// Color Setting
// ============================================================================

void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    g_state.mode = WS2812_MODE_SOLID;
    g_state.baseColor.r = r;
    g_state.baseColor.g = g;
    g_state.baseColor.b = b;
    send_pixel(r, g, b);
}

void ws2812_set_hsv(float h, float s, float v) {
    ws2812_rgb_t rgb = ws2812_hsv_to_rgb(h, s, v);
    ws2812_set_rgb(rgb.r, rgb.g, rgb.b);
}

void ws2812_set_hex(uint32_t hex) {
    uint8_t r = (hex >> 16) & 0xFF;
    uint8_t g = (hex >> 8) & 0xFF;
    uint8_t b = hex & 0xFF;
    ws2812_set_rgb(r, g, b);
}

void ws2812_off() {
    g_state.mode = WS2812_MODE_OFF;
    g_state.baseColor = ws2812_rgb_t{0, 0, 0};
    send_pixel(0, 0, 0);
}

// ============================================================================
// Pattern Modes
// ============================================================================

void ws2812_set_mode(ws2812_mode_t mode, ws2812_rgb_t color) {
    g_state.mode = mode;
    g_state.baseColor = color;
    g_state.phaseStartMs = to_ms_since_boot(get_absolute_time());
    g_state.blinkState = true;
    g_state.rainbowHue = 0.0F;

    // Immediately show initial state
    switch (mode) {
        case WS2812_MODE_OFF:
            send_pixel(0, 0, 0);
            break;
        case WS2812_MODE_SOLID:
        case WS2812_MODE_BLINK:
        case WS2812_MODE_BLINK_FAST:
        case WS2812_MODE_ALTERNATE:
        case WS2812_MODE_DOUBLE_FLASH:
            send_pixel(color.r, color.g, color.b);
            break;
        default:
            break;
    }
}

void ws2812_set_mode_alternate(ws2812_rgb_t a, ws2812_rgb_t b,
                               uint32_t half_period_ms) {
    g_state.mode = WS2812_MODE_ALTERNATE;
    g_state.baseColor = a;
    g_state.altColor = b;
    g_state.alternateHalfMs = (half_period_ms > 0U) ? half_period_ms
                                                  : kDefaultAlternateHalfMs;
    g_state.phaseStartMs = to_ms_since_boot(get_absolute_time());
    g_state.blinkState = true;   // true = showing baseColor
    send_pixel(a.r, a.g, a.b);
}

void ws2812_set_breathe_period(uint32_t period_ms) {
    g_state.breathePeriodMs = period_ms;
}

void ws2812_set_blink_timing(uint32_t on_ms, uint32_t off_ms) {
    g_state.blinkOnMs = on_ms;
    g_state.blinkOffMs = off_ms;
}

void ws2812_set_brightness(uint8_t brightness) {
    g_state.brightness = brightness;
}

// ============================================================================
// Update Helpers
// ============================================================================

static void update_breathe(uint32_t elapsed) {
    float phase = static_cast<float>(elapsed) / static_cast<float>(g_state.breathePeriodMs);
    float scale = (sinf(phase * 2.0F * kPi) + 1.0F) / 2.0F;
    scale = kBreatheMinScale + scale * kBreatheRange;
    ws2812_rgb_t color = apply_brightness(g_state.baseColor, scale);
    send_pixel(color.r, color.g, color.b);
}

static void update_blink(uint32_t elapsed, uint32_t on_ms, uint32_t off_ms) {
    uint32_t period = on_ms + off_ms;
    uint32_t phase = elapsed % period;
    bool should_be_on = (phase < on_ms);

    if (should_be_on != g_state.blinkState) {
        g_state.blinkState = should_be_on;
        if (should_be_on) {
            send_pixel(g_state.baseColor.r, g_state.baseColor.g,
                      g_state.baseColor.b);
        } else {
            send_pixel(0, 0, 0);
        }
    }
}

static void update_rainbow(uint32_t elapsed) {
    g_state.rainbowHue = fmodf(static_cast<float>(elapsed) / kRainbowCyclePeriodMs * kHueFull, kHueFull);
    ws2812_rgb_t color = ws2812_hsv_to_rgb(g_state.rainbowHue, kRainbowSaturation, kRainbowValue);
    send_pixel(color.r, color.g, color.b);
}

// Stage L: two-color alternation. `blinkState` true = showing baseColor,
// false = showing altColor. Edge-triggered (only sends when boundary crossed)
// to avoid hammering the PIO FIFO every tick.
static void update_alternate(uint32_t elapsed, uint32_t half_ms) {
    uint32_t period = half_ms * 2U;
    if (period == 0U) return;
    bool show_base = ((elapsed % period) < half_ms);
    if (show_base != g_state.blinkState) {
        g_state.blinkState = show_base;
        const ws2812_rgb_t& c = show_base ? g_state.baseColor : g_state.altColor;
        send_pixel(c.r, c.g, c.b);
    }
}

// Stage L: double-flash for pre-arm fail. 100ms on, 100ms off, 100ms on, 700ms off.
// Edge-triggered like update_blink. Uses baseColor for the on-frames.
static void update_double_flash(uint32_t elapsed) {
    uint32_t phase = elapsed % kDoubleFlashPeriodMs;
    bool should_be_on;
    if (phase < kDoubleFlashP0OnMs) {
        should_be_on = true;
    } else if (phase < kDoubleFlashP0OnMs + kDoubleFlashP1OffMs) {
        should_be_on = false;
    } else if (phase < kDoubleFlashP0OnMs + kDoubleFlashP1OffMs + kDoubleFlashP2OnMs) {
        should_be_on = true;
    } else {
        should_be_on = false;
    }
    if (should_be_on != g_state.blinkState) {
        g_state.blinkState = should_be_on;
        if (should_be_on) {
            send_pixel(g_state.baseColor.r, g_state.baseColor.g, g_state.baseColor.b);
        } else {
            send_pixel(0, 0, 0);
        }
    }
}

// ============================================================================
// Update
// ============================================================================

void ws2812_update() {
    if (!g_state.initialized) {
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = now - g_state.phaseStartMs;

    switch (g_state.mode) {
        case WS2812_MODE_OFF:
        case WS2812_MODE_SOLID:
            // Nothing to update — color already set or off
            break;

        case WS2812_MODE_BREATHE:
            update_breathe(elapsed);
            break;

        case WS2812_MODE_BLINK:
            update_blink(elapsed, g_state.blinkOnMs, g_state.blinkOffMs);
            break;

        case WS2812_MODE_BLINK_FAST:
            update_blink(elapsed, kFastBlinkOnMs, kFastBlinkPeriodMs - kFastBlinkOnMs);
            break;

        case WS2812_MODE_RAINBOW:
            update_rainbow(elapsed);
            break;

        case WS2812_MODE_ALTERNATE:
            update_alternate(elapsed, g_state.alternateHalfMs);
            break;

        case WS2812_MODE_DOUBLE_FLASH:
            update_double_flash(elapsed);
            break;
    }

    g_state.lastUpdateMs = now;
}

// ============================================================================
// Utility
// ============================================================================

// NOLINTBEGIN(readability-magic-numbers) — standard HSV-to-RGB conversion
// Sector boundaries (0, 60, 120, 180, 240, 300, 360) are inherent to the HSV color model.
ws2812_rgb_t ws2812_hsv_to_rgb(float h, float s, float v) {
    ws2812_rgb_t rgb = {0, 0, 0};

    // Normalize inputs
    h = fmodf(h, kHueFull);
    if (h < 0) {
        h += kHueFull;
    }
    s = s / kPercentScale;
    v = v / kPercentScale;

    if (s < kAchromaticThreshold) {
        uint8_t gray = static_cast<uint8_t>(v * kMaxChannelValue);
        rgb.r = rgb.g = rgb.b = gray;
        return rgb;
    }

    float c = v * s;
    float x = c * (1.0F - fabsf(fmodf(h / kHueSector, 2.0F) - 1.0F));
    float m = v - c;

    float r1 = 0.0F;
    float g1 = 0.0F;
    float b1 = 0.0F;

    if (h < 60.0F) {
        r1 = c; g1 = x; b1 = 0;
    } else if (h < 120.0F) {
        r1 = x; g1 = c; b1 = 0;
    } else if (h < 180.0F) {
        r1 = 0; g1 = c; b1 = x;
    } else if (h < 240.0F) {
        r1 = 0; g1 = x; b1 = c;
    } else if (h < 300.0F) {
        r1 = x; g1 = 0; b1 = c;
    } else {
        r1 = c; g1 = 0; b1 = x;
    }

    rgb.r = static_cast<uint8_t>((r1 + m) * kMaxChannelValue);
    rgb.g = static_cast<uint8_t>((g1 + m) * kMaxChannelValue);
    rgb.b = static_cast<uint8_t>((b1 + m) * kMaxChannelValue);

    return rgb;
}
// NOLINTEND(readability-magic-numbers)
