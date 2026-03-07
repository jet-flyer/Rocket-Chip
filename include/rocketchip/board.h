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
#else
    // Default to Feather RP2350 HSTX — the original flight board
    #include "board_feather_rp2350.h"
#endif

#endif // ROCKETCHIP_BOARD_H
