// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Pyro GPIO edge logger (IVP-130).
// Captures rising/falling edges on pyro GPIO pins with microsecond timestamps.
// Flight-binary essential — pyro timing is forensic data for post-flight analysis.
// Non-invasive: one GPIO ISR callback, static ring buffer, no side effects on flight logic.
#ifndef ROCKETCHIP_SAFETY_PYRO_EDGE_LOGGER_H
#define ROCKETCHIP_SAFETY_PYRO_EDGE_LOGGER_H

#include <stdint.h>

namespace rc {

struct PyroEdgeEvent {
    uint64_t timestamp_us;
    uint8_t  gpio_pin;
    bool     rising;
};

static constexpr uint32_t kPyroEdgeBufferSize = 64;

void pyro_edge_logger_init(uint8_t drogue_pin, uint8_t main_pin);
uint32_t pyro_edge_logger_count();
const PyroEdgeEvent* pyro_edge_logger_get(uint32_t index);
void pyro_edge_logger_dump_cli();

}  // namespace rc

#endif
