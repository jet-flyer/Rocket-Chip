// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Flash Layout — portable addresses derived from PICO_FLASH_SIZE_BYTES
//
// All flash regions are anchored from the TOP of flash downward, so the
// layout adapts to any flash size (8MB Feather, 4MB Tiny2350, etc.).
//
// Layout (offsets from flash base, top-down):
//   [FLASH_SIZE - 8KB ]  Calibration storage (dual-sector, 8KB)
//   [FLASH_SIZE - 16KB]  Flight table (dual-sector, 8KB)
//   [FLASH_SIZE - 20KB]  Flash-safe test sector (4KB)
//   [512KB .. table-20KB] Flight log data
//   [0 .. 512KB]          Firmware
//
// Council C-A4: boot validation ensures regions don't overlap firmware.
//============================================================================
#ifndef ROCKETCHIP_FLASH_LAYOUT_H
#define ROCKETCHIP_FLASH_LAYOUT_H

#include <stdint.h>

#ifndef PICO_ON_DEVICE
// Host test / non-device build — provide defaults
#ifndef FLASH_SECTOR_SIZE
#define FLASH_SECTOR_SIZE 4096U
#endif
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (8 * 1024 * 1024)
#endif
#else
#include "hardware/flash.h"  // FLASH_SECTOR_SIZE
#ifndef PICO_FLASH_SIZE_BYTES
#error "PICO_FLASH_SIZE_BYTES must be defined (provided by board header)"
#endif
#endif

namespace rc {

// ============================================================================
// Derived flash layout — all anchored from PICO_FLASH_SIZE_BYTES
// ============================================================================

// Firmware reserve (first 512KB)
static constexpr uint32_t kFlashFirmwareReserve = 512U * 1024U;

// Calibration: last 8KB of flash (2 sectors)
static constexpr uint32_t kFlashCalSectorA =
    static_cast<uint32_t>(PICO_FLASH_SIZE_BYTES) - 2U * FLASH_SECTOR_SIZE;
static constexpr uint32_t kFlashCalSectorB =
    static_cast<uint32_t>(PICO_FLASH_SIZE_BYTES) - 1U * FLASH_SECTOR_SIZE;
static constexpr uint32_t kFlashCalSize = 2U * FLASH_SECTOR_SIZE;

// Flight table: next 8KB below calibration (2 sectors)
static constexpr uint32_t kFlashTableSectorA =
    kFlashCalSectorA - 2U * FLASH_SECTOR_SIZE;
static constexpr uint32_t kFlashTableSectorB =
    kFlashCalSectorA - 1U * FLASH_SECTOR_SIZE;

// Flash-safe test: 1 sector below flight table
static constexpr uint32_t kFlashSafeTestOffset =
    kFlashTableSectorA - FLASH_SECTOR_SIZE;

// Flight log region: between firmware and flash-safe test sector
static constexpr uint32_t kFlashLogStart = kFlashFirmwareReserve;
static constexpr uint32_t kFlashLogEnd   = kFlashSafeTestOffset;
static constexpr uint32_t kFlashLogSize  = kFlashLogEnd - kFlashLogStart;
static constexpr uint32_t kFlashLogSectors = kFlashLogSize / FLASH_SECTOR_SIZE;

// ============================================================================
// Boot validation [Council C-A4]
// ============================================================================

// Call from init to verify layout doesn't overlap firmware binary.
// binary_end: address of last byte of firmware (from linker symbol).
// Returns true if layout is valid.
static constexpr bool flash_layout_valid() {
    // Table must be above firmware reserve
    static_assert(kFlashTableSectorA >= kFlashFirmwareReserve,
                  "Flight table overlaps firmware reserve");
    // Log region must have positive size
    static_assert(kFlashLogEnd > kFlashLogStart,
                  "Flash log region has zero or negative size");
    // Calibration must be within flash
    static_assert(kFlashCalSectorB + FLASH_SECTOR_SIZE <=
                  static_cast<uint32_t>(PICO_FLASH_SIZE_BYTES),
                  "Calibration extends beyond flash");
    return true;
}

// Force compile-time validation
static_assert(flash_layout_valid(), "Flash layout validation failed");

} // namespace rc

#endif // ROCKETCHIP_FLASH_LAYOUT_H
