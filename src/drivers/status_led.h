// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Board-selected status LED output: WS2812 and/or a discrete active-low
// RGB LED. The engine talks only to this API; pack constants pick the
// backend. Color and mode types match the WS2812 driver so existing
// patterns stay one mapping.

#ifndef ROCKETCHIP_STATUS_LED_H
#define ROCKETCHIP_STATUS_LED_H

#include "drivers/ws2812_status.h"
#include <stdint.h>

// Initialize the backend selected by board::kNeoPixelCount / kRgbLedAvailable.
// Returns true if a backend was started.
bool status_led_init(void);

// True when the WS2812 backend is the one in use and init succeeded.
bool status_led_ws2812_ready(void);

void status_led_set_mode(ws2812_mode_t mode, ws2812_rgb_t color);
void status_led_set_mode_alternate(ws2812_rgb_t a, ws2812_rgb_t b,
                                   uint32_t halfPeriodMs);
void status_led_update(void);

// Multi-pixel station chain animations. On a single RGB LED these collapse
// to a solid or blink of the same color family.
void status_led_set_rssi_bar(int16_t rssi_dbm, bool searching);
void status_led_set_sweep_bar(ws2812_rgb_t color);
void status_led_set_flash_bar(ws2812_rgb_t color, bool on);

#endif // ROCKETCHIP_STATUS_LED_H
