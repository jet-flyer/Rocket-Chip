// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "safety/pyro_edge_logger.h"

#ifndef ROCKETCHIP_HOST_TEST

#include "hardware/gpio.h"
#include "pico/time.h"
#include "rocketchip/rc_log.h"

namespace rc {

static PyroEdgeEvent s_buffer[kPyroEdgeBufferSize];
static volatile uint32_t s_count = 0;
static uint8_t s_droguPin = 0;
static uint8_t s_mainPin = 0;

static void gpio_edge_callback(uint gpio, uint32_t events) {
    if (s_count >= kPyroEdgeBufferSize) { return; }
    uint32_t idx = s_count;
    s_buffer[idx].timestamp_us = time_us_64();
    s_buffer[idx].gpio_pin = static_cast<uint8_t>(gpio);
    s_buffer[idx].rising = (events & GPIO_IRQ_EDGE_RISE) != 0;
    s_count = idx + 1;
}

void pyro_edge_logger_init(uint8_t drogue_pin, uint8_t main_pin) {
    s_droguPin = drogue_pin;
    s_mainPin = main_pin;
    s_count = 0;

    gpio_set_irq_enabled_with_callback(
        drogue_pin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        gpio_edge_callback
    );
    gpio_set_irq_enabled(
        main_pin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );
}

uint32_t pyro_edge_logger_count() {
    return s_count;
}

const PyroEdgeEvent* pyro_edge_logger_get(uint32_t index) {
    if (index >= s_count) { return nullptr; }
    return &s_buffer[index];
}

void pyro_edge_logger_dump_cli() {
    uint32_t n = s_count;
    if (n == 0) {
        rc::rc_log("Pyro log: empty\n");
        return;
    }
    rc::rc_log("Pyro log: %lu events\n", (unsigned long)n);
    for (uint32_t i = 0; i < n; ++i) {
        const auto& e = s_buffer[i];
        const char* pin_name = (e.gpio_pin == s_droguPin) ? "DROGUE" :
                               (e.gpio_pin == s_mainPin)  ? "MAIN"   : "?";
        rc::rc_log("  [%llu us] GPIO%u (%s) %s\n",
                   (unsigned long long)e.timestamp_us,
                   e.gpio_pin, pin_name,
                   e.rising ? "RISE" : "FALL");
    }
}

}  // namespace rc

#else  // ROCKETCHIP_HOST_TEST

namespace rc {

void pyro_edge_logger_init(uint8_t, uint8_t) {}
uint32_t pyro_edge_logger_count() { return 0; }
const PyroEdgeEvent* pyro_edge_logger_get(uint32_t) { return nullptr; }
void pyro_edge_logger_dump_cli() {}

}  // namespace rc

#endif
