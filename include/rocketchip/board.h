// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Compile-time board selector (PICO_BOARD → board_*.h, `board::` constants).
// Adding a board: new board_<name>.h, #elif on the SDK macro, check
// docs/hardware/BOARD_COMPARISON.md. Unknown PICO_BOARD must not inherit
// Feather pins.

#ifndef ROCKETCHIP_BOARD_H
#define ROCKETCHIP_BOARD_H

#include "pico/stdlib.h"

#if defined(ADAFRUIT_FRUIT_JAM)
    #include "board_fruit_jam.h"
#elif defined(ADAFRUIT_FEATHER_RP2350)
    #include "board_feather_rp2350.h"
#elif defined(PIMORONI_TINY2350)
    // WIP / unsupported until TINY_2350_BRINGUP_OK. Plus pack only;
    // no board_tiny_2350.h.
    #include "board_tiny_2350_plus.h"
#elif defined(RASPBERRYPI_PICO2)
    // WIP / unsupported until PICO2_BRINGUP_OK.
    #include "board_pico2.h"
#else
    // Fail-closed: unknown PICO_BOARD must not silently inherit Feather pins
    // (WN-020). Host tests are not a board — they compile Feather constants.
#ifdef ROCKETCHIP_HOST_TEST
    #include "board_feather_rp2350.h"
#else
#error "Unsupported PICO_BOARD. Add a board_*.h pack and selector clause; do not default to Feather HSTX."
#endif
#endif

#endif // ROCKETCHIP_BOARD_H
