// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Board constants for Pimoroni Tiny 2350+ (the Plus variant)
// RP2350A (QFN-60), 4 MB flash (Pimoroni Tiny 2350). Pins live in
// board_tiny_2350_common.h; this file sets PSRAM flag + name.
// PSRAM CS is GPIO 21 in common — same pin as I2C SCL; not allowlisted.
// WIP until TINY_2350_BRINGUP_OK (WN-027/028).

#ifndef ROCKETCHIP_BOARD_TINY_2350_PLUS_H
#define ROCKETCHIP_BOARD_TINY_2350_PLUS_H

#include "board_tiny_2350_common.h"

namespace board {

// PSRAM off until CS is allowlisted and not on I2C SCL (GPIO 21).
inline constexpr bool        kPsramAvailable = false;
inline constexpr const char* kBoardName      = "Pimoroni Tiny 2350+";

} // namespace board

#endif // ROCKETCHIP_BOARD_TINY_2350_PLUS_H
