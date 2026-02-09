/**
 * @file ws2812_status.c
 * @brief WS2812 NeoPixel status LED driver implementation
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

// ============================================================================
// Private State
// ============================================================================

static struct {
    PIO pio;
    uint sm;
    uint offset;
    bool initialized;

    // Current mode and color
    ws2812_mode_t mode;
    ws2812_rgb_t base_color;
    uint8_t brightness;

    // Pattern timing
    uint32_t breathe_period_ms;
    uint32_t blink_on_ms;
    uint32_t blink_off_ms;

    // Animation state
    uint32_t last_update_ms;
    uint32_t phase_start_ms;
    bool blink_state;
    float rainbow_hue;
} g_state = {
    .pio = nullptr,
    .sm = 0,
    .offset = 0,
    .initialized = false,
    .mode = WS2812_MODE_OFF,
    .base_color = {0, 0, 0},
    .brightness = 255,
    .breathe_period_ms = kDefaultBreathePeriodMs,
    .blink_on_ms = kDefaultBlinkOnMs,
    .blink_off_ms = kDefaultBlinkOffMs,
    .last_update_ms = 0,
    .phase_start_ms = 0,
    .blink_state = false,
    .rainbow_hue = 0.0F,
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
    uint32_t grb = ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
    pio_sm_put_blocking(g_state.pio, g_state.sm, grb);
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

bool ws2812_status_init(PIO pio, uint pin) {
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
    g_state.last_update_ms = to_ms_since_boot(get_absolute_time());
    g_state.phase_start_ms = g_state.last_update_ms;

    // Start with LED off
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
    g_state.base_color.r = r;
    g_state.base_color.g = g;
    g_state.base_color.b = b;
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
    g_state.base_color = ws2812_rgb_t{0, 0, 0};
    send_pixel(0, 0, 0);
}

// ============================================================================
// Pattern Modes
// ============================================================================

void ws2812_set_mode(ws2812_mode_t mode, ws2812_rgb_t color) {
    g_state.mode = mode;
    g_state.base_color = color;
    g_state.phase_start_ms = to_ms_since_boot(get_absolute_time());
    g_state.blink_state = true;
    g_state.rainbow_hue = 0.0F;

    // Immediately show initial state
    switch (mode) {
        case WS2812_MODE_OFF:
            send_pixel(0, 0, 0);
            break;
        case WS2812_MODE_SOLID:
            send_pixel(color.r, color.g, color.b);
            break;
        case WS2812_MODE_BLINK:
        case WS2812_MODE_BLINK_FAST:
            send_pixel(color.r, color.g, color.b);
            break;
        default:
            break;
    }
}

void ws2812_set_breathe_period(uint32_t period_ms) {
    g_state.breathe_period_ms = period_ms;
}

void ws2812_set_blink_timing(uint32_t on_ms, uint32_t off_ms) {
    g_state.blink_on_ms = on_ms;
    g_state.blink_off_ms = off_ms;
}

void ws2812_set_brightness(uint8_t brightness) {
    g_state.brightness = brightness;
}

// ============================================================================
// Update
// ============================================================================

void ws2812_update() {
    if (!g_state.initialized) {
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = now - g_state.phase_start_ms;

    switch (g_state.mode) {
        case WS2812_MODE_OFF:
            // Nothing to update
            break;

        case WS2812_MODE_SOLID:
            // Nothing to update - color already set
            break;

        case WS2812_MODE_BREATHE: {
            // Sinusoidal brightness pulsing
            float phase = (float)elapsed / (float)g_state.breathe_period_ms;
            float scale = (sinf(phase * 2.0F * kPi) + 1.0F) / 2.0F;
            scale = kBreatheMinScale + scale * kBreatheRange;
            ws2812_rgb_t color = apply_brightness(g_state.base_color, scale);
            send_pixel(color.r, color.g, color.b);
            break;
        }

        case WS2812_MODE_BLINK: {
            uint32_t period = g_state.blink_on_ms + g_state.blink_off_ms;
            uint32_t phase = elapsed % period;
            bool should_be_on = (phase < g_state.blink_on_ms);

            if (should_be_on != g_state.blink_state) {
                g_state.blink_state = should_be_on;
                if (should_be_on) {
                    send_pixel(g_state.base_color.r, g_state.base_color.g,
                              g_state.base_color.b);
                } else {
                    send_pixel(0, 0, 0);
                }
            }
            break;
        }

        case WS2812_MODE_BLINK_FAST: {
            // 5Hz blinking
            uint32_t phase = elapsed % kFastBlinkPeriodMs;
            bool should_be_on = (phase < kFastBlinkOnMs);

            if (should_be_on != g_state.blink_state) {
                g_state.blink_state = should_be_on;
                if (should_be_on) {
                    send_pixel(g_state.base_color.r, g_state.base_color.g,
                              g_state.base_color.b);
                } else {
                    send_pixel(0, 0, 0);
                }
            }
            break;
        }

        case WS2812_MODE_RAINBOW: {
            // Smooth rainbow cycle
            g_state.rainbow_hue = fmodf((float)elapsed / kRainbowCyclePeriodMs * kHueFull, kHueFull);
            ws2812_rgb_t color = ws2812_hsv_to_rgb(g_state.rainbow_hue, kRainbowSaturation, kRainbowValue);
            send_pixel(color.r, color.g, color.b);
            break;
        }
    }

    g_state.last_update_ms = now;
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
        uint8_t gray = (uint8_t)(v * kMaxChannelValue);
        rgb.r = rgb.g = rgb.b = gray;
        return rgb;
    }

    float c = v * s;
    float x = c * (1.0F - fabsf(fmodf(h / kHueSector, 2.0F) - 1.0F));
    float m = v - c;

    float r1, g1, b1;

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

    rgb.r = (uint8_t)((r1 + m) * kMaxChannelValue);
    rgb.g = (uint8_t)((g1 + m) * kMaxChannelValue);
    rgb.b = (uint8_t)((b1 + m) * kMaxChannelValue);

    return rgb;
}
// NOLINTEND(readability-magic-numbers)
