/**
 * @file ws2812_status.c
 * @brief WS2812 NeoPixel status LED driver implementation
 */

#include "ws2812_status.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "ws2812.pio.h"
#include <math.h>

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
    .pio = NULL,
    .sm = 0,
    .offset = 0,
    .initialized = false,
    .mode = WS2812_MODE_OFF,
    .base_color = {0, 0, 0},
    .brightness = 255,
    .breathe_period_ms = 2000,
    .blink_on_ms = 500,
    .blink_off_ms = 500,
    .last_update_ms = 0,
    .phase_start_ms = 0,
    .blink_state = false,
    .rainbow_hue = 0.0f,
};

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Send raw RGB data to WS2812 via PIO
 */
static void send_pixel(uint8_t r, uint8_t g, uint8_t b) {
    if (!g_state.initialized) return;

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
    result.r = (uint8_t)(color.r * scale);
    result.g = (uint8_t)(color.g * scale);
    result.b = (uint8_t)(color.b * scale);
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
    ws2812_program_init(pio, sm, offset, pin, 800000.0f, false);

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

void ws2812_status_deinit(void) {
    if (!g_state.initialized) return;

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

void ws2812_off(void) {
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
    g_state.rainbow_hue = 0.0f;

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

void ws2812_update(void) {
    if (!g_state.initialized) return;

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
            float scale = (sinf(phase * 2.0f * 3.14159f) + 1.0f) / 2.0f;
            // Minimum 10% brightness so LED doesn't fully turn off
            scale = 0.1f + scale * 0.9f;
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
            // 5Hz blinking (100ms on, 100ms off)
            uint32_t phase = elapsed % 200;
            bool should_be_on = (phase < 100);

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
            // Smooth rainbow cycle - 6 second full rotation
            g_state.rainbow_hue = fmodf((float)elapsed / 6000.0f * 360.0f, 360.0f);
            ws2812_rgb_t color = ws2812_hsv_to_rgb(g_state.rainbow_hue, 100.0f, 25.0f);
            send_pixel(color.r, color.g, color.b);
            break;
        }
    }

    g_state.last_update_ms = now;
}

// ============================================================================
// Utility
// ============================================================================

ws2812_rgb_t ws2812_hsv_to_rgb(float h, float s, float v) {
    ws2812_rgb_t rgb = {0, 0, 0};

    // Normalize inputs
    h = fmodf(h, 360.0f);
    if (h < 0) h += 360.0f;
    s = s / 100.0f;
    v = v / 100.0f;

    if (s < 0.001f) {
        // Achromatic (gray)
        uint8_t gray = (uint8_t)(v * 255.0f);
        rgb.r = rgb.g = rgb.b = gray;
        return rgb;
    }

    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;

    float r1, g1, b1;

    if (h < 60.0f) {
        r1 = c;
        g1 = x;
        b1 = 0;
    } else if (h < 120.0f) {
        r1 = x;
        g1 = c;
        b1 = 0;
    } else if (h < 180.0f) {
        r1 = 0;
        g1 = c;
        b1 = x;
    } else if (h < 240.0f) {
        r1 = 0;
        g1 = x;
        b1 = c;
    } else if (h < 300.0f) {
        r1 = x;
        g1 = 0;
        b1 = c;
    } else {
        r1 = c;
        g1 = 0;
        b1 = x;
    }

    rgb.r = (uint8_t)((r1 + m) * 255.0f);
    rgb.g = (uint8_t)((g1 + m) * 255.0f);
    rgb.b = (uint8_t)((b1 + m) * 255.0f);

    return rgb;
}
