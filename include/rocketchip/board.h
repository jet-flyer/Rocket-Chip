// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file board.h
 * @brief Compile-time board selector — includes correct board header
 *
 * Selection based on PICO_BOARD CMake variable, which the Pico SDK sets
 * as a preprocessor define via the board header include chain.
 *
 * Stage J: Fruit Jam HAL — board abstraction for same-binary builds.
 * All board-specific constants live in board_*.h headers under the
 * `board::` namespace. Drivers and main.cpp use `board::kFoo` instead
 * of hardcoded values.
 *
 * Adding a new board:
 *   1. Create include/rocketchip/board_<name>.h with all board:: constants
 *   2. Add an #elif clause below matching the SDK board detection macro
 *   3. Verify pin assignments against docs/hardware/BOARD_COMPARISON.md
 */

#ifndef ROCKETCHIP_BOARD_H
#define ROCKETCHIP_BOARD_H

#include "pico/stdlib.h"

#if defined(ADAFRUIT_FRUIT_JAM)
    #include "board_fruit_jam.h"
#elif defined(ADAFRUIT_FEATHER_RP2350)
    #include "board_feather_rp2350.h"
#elif defined(PIMORONI_TINY2350)
    // Pimoroni Tiny 2350+ — scaffolding, gated by TINY_2350_BRINGUP_OK
    // (Stage 16C IVP-143). Base Tiny 2350 variant (board_tiny_2350.h)
    // deferred until Plus variant completes hardware bring-up.
    #include "board_tiny_2350_plus.h"
#elif defined(RASPBERRYPI_PICO2)
    // Raspberry Pi Pico 2 — scaffolding, gated by PICO2_BRINGUP_OK
    // (Stage 16C IVP-143).
    #include "board_pico2.h"
#else
    // Default to Feather RP2350 HSTX — the original flight board
    #include "board_feather_rp2350.h"
#endif

#endif // ROCKETCHIP_BOARD_H
