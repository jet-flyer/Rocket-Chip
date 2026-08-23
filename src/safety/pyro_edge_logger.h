// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// WIP — not a flight product. Do not call pyro_edge_logger_init() from
// flight boot (WN-274 / rem WB R-14).
//
// IVP-130 bench helper: GPIO edge capture on PIO backup-timer pins
// (GPIO 12/13 on this pack). Those pins are not connected to pyro
// hardware. FD "pyro fire" does not drive them.
//
// 64-slot fill-and-stop (not a ring). Only consumer: debug-menu dump.
// Not PCM / flight log / telemetry. Not "forensic ready."
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

// Bench-only. Must not be called from init_pio_safety / main until this
// WIP is finished (pyro HW on the pins + a real log consumer).
void pyro_edge_logger_init(uint8_t drogue_pin, uint8_t main_pin);
uint32_t pyro_edge_logger_count();
const PyroEdgeEvent* pyro_edge_logger_get(uint32_t index);
void pyro_edge_logger_dump_cli();

}  // namespace rc

#endif
