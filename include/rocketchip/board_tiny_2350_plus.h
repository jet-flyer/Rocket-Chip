// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file board_tiny_2350_plus.h
 * @brief Board constants for Pimoroni Tiny 2350+ (the Plus variant)
 *
 * RP2350A (QFN-60), 8 MB flash, onboard PSRAM. Pin map largely
 * shared with the base Tiny 2350; variant overrides are explicit
 * below.
 *
 * Stage 16C IVP-143 scaffolding. The pin map inherited from
 * board_tiny_2350_common.h is datasheet-sourced and has NOT been
 * verified on hardware. Bring-up is gated behind the
 * TINY_2350_BRINGUP_OK define — a future bring-up IVP must define
 * that symbol after verifying each pin physically.
 */

#ifndef ROCKETCHIP_BOARD_TINY_2350_PLUS_H
#define ROCKETCHIP_BOARD_TINY_2350_PLUS_H

#include "board_tiny_2350_common.h"

#ifndef TINY_2350_BRINGUP_OK
#error "Tiny_2350 pin map not yet verified on hardware. Define TINY_2350_BRINGUP_OK after hardware bring-up."
#endif

namespace board {

// Variant-specific overrides for Tiny 2350+ (the Plus).
inline constexpr bool        kPsramAvailable = true;
inline constexpr const char* kBoardName      = "Pimoroni Tiny 2350+";

} // namespace board

#endif // ROCKETCHIP_BOARD_TINY_2350_PLUS_H
