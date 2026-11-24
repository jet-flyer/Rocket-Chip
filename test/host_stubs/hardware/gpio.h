// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Host stub: counting GPIO calls used by spi_bus.cpp, diag and board packs.

#ifndef ROCKETCHIP_TEST_HOST_STUB_HARDWARE_GPIO_H
#define ROCKETCHIP_TEST_HOST_STUB_HARDWARE_GPIO_H

#include "pico/stdlib.h"
#include "host_stub_counters.h"

enum gpio_function_t { GPIO_FUNC_SPI = 1 };

inline void gpio_put(uint pin, bool value) {
    (void)pin;
    (void)value;
    ++host_stub::g_gpioCalls;
}

inline bool gpio_get(uint pin) {
    (void)pin;
    ++host_stub::g_gpioCalls;
    return host_stub::g_gpioLevel;
}

inline void gpio_set_function(uint pin, gpio_function_t fn) {
    (void)pin;
    (void)fn;
    ++host_stub::g_gpioCalls;
}

#endif  // ROCKETCHIP_TEST_HOST_STUB_HARDWARE_GPIO_H
