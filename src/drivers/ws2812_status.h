// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file ws2812_status.h
 * @brief WS2812 NeoPixel status LED driver with smooth transitions
 *
 * Provides ArduPilot-style status indication patterns:
 * - Solid colors for states
 * - Breathing/pulsing for activity
 * - Blinking for warnings/errors
 * - Smooth color transitions via HSV
 */

#ifndef ROCKETCHIP_WS2812_STATUS_H
#define ROCKETCHIP_WS2812_STATUS_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

// ============================================================================
// Types
// ============================================================================

/**
 * @brief RGB color structure
 */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_rgb_t;

/**
 * @brief Status LED pattern modes
 */
typedef enum {
    WS2812_MODE_OFF,        ///< LED off
    WS2812_MODE_SOLID,      ///< Solid color
    WS2812_MODE_BREATHE,    ///< Smooth brightness pulsing
    WS2812_MODE_BLINK,      ///< On/off blinking
    WS2812_MODE_BLINK_FAST, ///< Fast blinking (error indication)
    WS2812_MODE_RAINBOW,    ///< Smooth rainbow cycle
} ws2812_mode_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize WS2812 status LED(s)
 * @param pio PIO instance (pio0, pio1, or pio2)
 * @param pin GPIO pin for data (board::kNeoPixelPin)
 * @param num_leds Number of LEDs in chain (board::kNeoPixelCount, default 1)
 * @return true on success
 */
bool ws2812_status_init(PIO pio, uint pin, uint8_t num_leds = 1);

/**
 * @brief Deinitialize and release PIO resources
 */
void ws2812_status_deinit(void);

// ============================================================================
// Color Setting
// ============================================================================

/**
 * @brief Set color using RGB values
 * @param r Red (0-255)
 * @param g Green (0-255)
 * @param b Blue (0-255)
 */
void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set color using HSV values (smooth color wheel)
 * @param h Hue (0-360 degrees)
 * @param s Saturation (0-100%)
 * @param v Value/brightness (0-100%)
 */
void ws2812_set_hsv(float h, float s, float v);

/**
 * @brief Set color using hex value (0xRRGGBB)
 * @param hex 24-bit color value
 */
void ws2812_set_hex(uint32_t hex);

/**
 * @brief Turn LED off
 */
void ws2812_off(void);

/**
 * @brief Set individual pixel color (IVP-97: RSSI bar)
 * @param index Pixel index (0 = first LED in chain)
 * @param r Red (0-255)
 * @param g Green (0-255)
 * @param b Blue (0-255)
 */
void ws2812_set_pixel_rgb(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Flush pixel buffer to LEDs (call after setting individual pixels)
 */
void ws2812_show(void);

/**
 * @brief Set RSSI bar pattern on multi-LED strip (IVP-97)
 *
 * Maps RSSI value (-120 to -40 dBm) to 1-5 lit pixels.
 * Strong (>-70): green. Marginal (-70 to -90): yellow. Weak (<-90): red.
 * No signal: all off with slow red pulse.
 *
 * @param rssi   RSSI in dBm (negative, e.g., -80)
 * @param no_signal true if no packets received recently (>5s gap)
 */
void ws2812_set_rssi_bar(int16_t rssi, bool no_signal);

// ============================================================================
// Pattern Modes
// ============================================================================

/**
 * @brief Set pattern mode
 * @param mode Pattern mode
 * @param color Base color for the pattern
 */
void ws2812_set_mode(ws2812_mode_t mode, ws2812_rgb_t color);

/**
 * @brief Set breathing/pulse speed
 * @param periodMs Full breath cycle period in milliseconds
 */
void ws2812_set_breathe_period(uint32_t periodMs);

/**
 * @brief Set blink timing
 * @param onMs On time in milliseconds
 * @param offMs Off time in milliseconds
 */
void ws2812_set_blink_timing(uint32_t onMs, uint32_t offMs);

/**
 * @brief Set global brightness (dimming)
 * @param brightness 0-255 (0 = off, 255 = full brightness)
 */
void ws2812_set_brightness(uint8_t brightness);

// ============================================================================
// Update (call from task loop)
// ============================================================================

/**
 * @brief Update LED state - call periodically (e.g., 50Hz)
 *
 * This handles pattern animations. Call from the main loop.
 */
void ws2812_update(void);

// ============================================================================
// Utility
// ============================================================================

/**
 * @brief Convert HSV to RGB
 * @param h Hue (0-360)
 * @param s Saturation (0-100)
 * @param v Value (0-100)
 * @return RGB color
 */
ws2812_rgb_t ws2812_hsv_to_rgb(float h, float s, float v);

// ============================================================================
// Predefined Colors (dimmed for status use)
// ============================================================================

constexpr ws2812_rgb_t kColorOff     = {0x00, 0x00, 0x00};
constexpr ws2812_rgb_t kColorRed     = {0x40, 0x00, 0x00};
constexpr ws2812_rgb_t kColorGreen   = {0x00, 0x40, 0x00};
constexpr ws2812_rgb_t kColorBlue    = {0x00, 0x00, 0x40};
constexpr ws2812_rgb_t kColorYellow  = {0x40, 0x40, 0x00};
constexpr ws2812_rgb_t kColorCyan    = {0x00, 0x40, 0x40};
constexpr ws2812_rgb_t kColorMagenta = {0x40, 0x00, 0x40};
constexpr ws2812_rgb_t kColorOrange  = {0x40, 0x20, 0x00};
constexpr ws2812_rgb_t kColorWhite   = {0x40, 0x40, 0x40};

#endif // ROCKETCHIP_WS2812_STATUS_H
