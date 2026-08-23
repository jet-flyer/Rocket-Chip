// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Board constants for Pimoroni Tiny 2350+ (the Plus variant)
// RP2350A (QFN-60), 8 MB flash, onboard PSRAM. Pin map largely
// shared with the base Tiny 2350; variant overrides are explicit
// below.
// WIP / unsupported until TINY_2350_BRINGUP_OK (WN-027/028).
// Pin map in board_tiny_2350_common.h is unverified.

#ifndef ROCKETCHIP_BOARD_TINY_2350_PLUS_H
#define ROCKETCHIP_BOARD_TINY_2350_PLUS_H

#include "board_tiny_2350_common.h"

#ifndef TINY_2350_BRINGUP_OK
#error "Tiny 2350+ is WIP/unsupported. Define TINY_2350_BRINGUP_OK only after a documented pin-map allowlist."
#endif

namespace board {

// Variant-specific overrides for Tiny 2350+ (the Plus).
inline constexpr bool        kPsramAvailable = true;
inline constexpr const char* kBoardName      = "Pimoroni Tiny 2350+";

} // namespace board

#endif // ROCKETCHIP_BOARD_TINY_2350_PLUS_H
