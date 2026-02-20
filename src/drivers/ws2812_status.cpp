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
constexpr uint32_t kGrbGreenShift       = 24;         // GRB bit position: green in upper byte

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
    ws2812_rgb_t baseColor;
    uint8_t brightness;

    // Pattern timing
    uint32_t breathePeriodMs;
    uint32_t blinkOnMs;
    uint32_t blinkOffMs;

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
    .mode = WS2812_MODE_OFF,
    .baseColor = {0, 0, 0},
    .brightness = 255,
    .breathePeriodMs = kDefaultBreathePeriodMs,
    .blinkOnMs = kDefaultBlinkOnMs,
    .blinkOffMs = kDefaultBlinkOffMs,
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
    g_state.lastUpdateMs = to_ms_since_boot(get_absolute_time());
    g_state.phaseStartMs = g_state.lastUpdateMs;

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
            send_pixel(color.r, color.g, color.b);
            break;
        default:
            break;
    }
}

void ws2812_set_breathe_period(uint32_t periodMs) {
    g_state.breathePeriodMs = periodMs;
}

void ws2812_set_blink_timing(uint32_t onMs, uint32_t offMs) {
    g_state.blinkOnMs = onMs;
    g_state.blinkOffMs = offMs;
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

static void update_blink(uint32_t elapsed, uint32_t onMs, uint32_t offMs) {
    uint32_t period = onMs + offMs;
    uint32_t phase = elapsed % period;
    bool shouldBeOn = (phase < onMs);

    if (shouldBeOn != g_state.blinkState) {
        g_state.blinkState = shouldBeOn;
        if (shouldBeOn) {
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
