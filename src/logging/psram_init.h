// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// APS6404L-3SQR on QMI CS1 — board-coupled (WN-216), pin from board::kPsramCsPin.
// Cached XIP 0x11000000; uncached 0x15000000 for crash-recovery header writes.
// Volatile: power loss wipes it; flash flight table is durable.

#ifndef ROCKETCHIP_PSRAM_INIT_H
#define ROCKETCHIP_PSRAM_INIT_H

#include <stdint.h>
#include <stddef.h>

namespace rc {

// PSRAM memory layout
static constexpr uint32_t kPsramCachedBase   = 0x11000000U;
static constexpr uint32_t kPsramUncachedBase = 0x15000000U;
static constexpr uint32_t kPsramExpectedSize = 8U * 1024U * 1024U;  // 8MB

// Must be called BEFORE Core 1 launch and flash operations.
// Direct-mode windows run from SRAM; clock_get_hz / timing math run
// with XIP still up (Arduino-Pico discussion 3431).
size_t psram_init(uint32_t cs_pin);

// Tests offset 0, size/2, and size-4 to catch addressing issues.
bool psram_self_test(size_t size);

// Pointer to cached PSRAM base, or nullptr if not initialized
uint8_t* psram_base_ptr();

// Pointer to uncached PSRAM base, or nullptr if not initialized
uint8_t* psram_uncached_base_ptr();

// Size in bytes, or 0 if not initialized
size_t psram_get_size();

// Council req. #2 (hard gate): Write known pattern → flash_safe_execute()
// erase+program → read back → byte-for-byte verify. Validates SDK's
// QMI M1 save/restore for our CS1-not-in-FLASH_DEVINFO case.
bool psram_flash_safe_test();

} // namespace rc

#endif // ROCKETCHIP_PSRAM_INIT_H
