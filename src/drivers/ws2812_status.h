// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// WS2812 NeoPixel status LED driver with smooth transitions
// Provides ArduPilot-style status indication patterns:
// - Solid colors for states
// - Breathing/pulsing for activity
// - Blinking for warnings/errors
// - Smooth color transitions via HSV

#ifndef ROCKETCHIP_WS2812_STATUS_H
#define ROCKETCHIP_WS2812_STATUS_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

// ============================================================================
// Types
// ============================================================================

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_rgb_t;

typedef enum {
    WS2812_MODE_OFF,        // LED off
    WS2812_MODE_SOLID,      // Solid color
    WS2812_MODE_BREATHE,    // Smooth brightness pulsing
    WS2812_MODE_BLINK,      // On/off blinking
    WS2812_MODE_BLINK_FAST, // Fast blinking (error indication)
    WS2812_MODE_RAINBOW,    // Smooth rainbow cycle
    WS2812_MODE_ALTERNATE,  // Two-color alternation (Stage L — beacon overlay)
    WS2812_MODE_DOUBLE_FLASH, // Two short pulses + long pause (Stage L — pre-arm fail)
} ws2812_mode_t;

// ============================================================================
// Initialization
// ============================================================================

// num_leds 0: no-op false. Above 8 (pixels[] capacity): clamp.
// `pio` is overwritten by pio_claim_free_sm_and_add_program_for_gpio_range
// (SDK picks the block).
bool ws2812_status_init(PIO pio, uint pin, uint8_t num_leds = 1);

void ws2812_status_deinit(void);

// ============================================================================
// Color Setting
// ============================================================================

void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b);

void ws2812_set_hsv(float h, float s, float v);

void ws2812_set_hex(uint32_t hex);

void ws2812_off(void);

void ws2812_set_pixel_rgb(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

void ws2812_show(void);

// Maps RSSI to 1–N lit pixels (N = strip length).
// Thresholds in the .cpp: >= -60 all lit, then -70/-80/-95 steps.
// Lit pixels mixed green/yellow/red by index, not a whole-bar color band.
// no_signal: pixel 0 dim red, rest off. Not a pulse.
void ws2812_set_rssi_bar(int16_t rssi, bool no_signal);

// One step per call; caller sets cadence. Color is the caller's choice.
// Position + direction persist in file-scope statics.
void ws2812_set_sweep_bar(ws2812_rgb_t color);

// ============================================================================
// Pattern Modes
// ============================================================================

void ws2812_set_mode(ws2812_mode_t mode, ws2812_rgb_t color);

// 0: keep last period (divide-by-zero guard).
void ws2812_set_breathe_period(uint32_t periodMs);

// 0 on either side: keep last timings.
void ws2812_set_blink_timing(uint32_t onMs, uint32_t offMs);

// Swaps `a` and `b` every halfPeriodMs. No C++ default; 0 remaps in the
// .cpp to kDefaultAlternateHalfMs (250). Full a→b→a period is 2*halfPeriodMs.
void ws2812_set_mode_alternate(ws2812_rgb_t a, ws2812_rgb_t b,
                               uint32_t halfPeriodMs);

void ws2812_set_brightness(uint8_t brightness);

// ============================================================================
// Update (call from task loop)
// ============================================================================

// This handles pattern animations. Call from the main loop.
void ws2812_update(void);

// ============================================================================
// Utility
// ============================================================================

// RGB color
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
