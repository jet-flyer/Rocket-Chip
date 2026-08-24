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

// Three-point addressing check (offset 0, size/2, size-4) through the
// uncached alias so XIP cache cannot satisfy the readback.
bool psram_self_test(size_t size);

// Pointer to cached PSRAM base, or nullptr if not initialized
uint8_t* psram_base_ptr();

// Pointer to uncached PSRAM base, or nullptr if not initialized
uint8_t* psram_uncached_base_ptr();

// Size in bytes, or 0 if not initialized
size_t psram_get_size();

// Hard gate for placing the flight ring in PSRAM: write a known pattern,
// flash_safe_execute() erase of the dedicated test sector, read back.
// Erase (not program) is enough: RP2350 datasheet §5.4.8.10/11 — erase
// and program use the same QMI direct-mode window and CS1 restore.
// AO_Logger requires this plus psram_self_test.
bool psram_flash_safe_test();

} // namespace rc

#endif // ROCKETCHIP_PSRAM_INIT_H
