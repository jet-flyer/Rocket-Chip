/**
 * @file flash_map.h
 * @brief Flash memory layout for RocketChip RP2350
 *
 * Defines physical flash addresses for Tier 1 storage (AP_FlashStorage).
 * See docs/AP_HAL_RP2350_PLAN.md for architecture details.
 */

#pragma once

#include <stdint.h>

namespace rocketchip {

// ============================================================================
// Flash Memory Layout (8MB flash on Feather RP2350 HSTX)
// ============================================================================
//
// 0x10000000  +-----------------+
//             |   Bootloader    |  16KB
// 0x10004000  +-----------------+
//             |                 |
//             |    Firmware     |  512KB
//             |                 |
// 0x10084000  +-----------------+
//             | Storage Sect A  |  4KB  --+-- AP_FlashStorage (Tier 1)
// 0x10085000  +-----------------+         |   Dual-sector wear leveling
//             | Storage Sect B  |  4KB  --+
// 0x10086000  +-----------------+
//             |                 |
//             |    LittleFS     |  ~7.5MB (Tier 2A - future)
//             |  (Flight Logs)  |
//             |                 |
// 0x10800000  +-----------------+  (End of 8MB)

// Flash base address (XIP mapped)
constexpr uint32_t kFlashBase = 0x10000000;

// Total flash size (8MB on Feather RP2350 HSTX)
constexpr uint32_t kFlashSize = 8 * 1024 * 1024;

// Flash sector size for RP2350 (erase granularity)
constexpr uint32_t kFlashSectorSize = 4096;

// Flash page size (write granularity)
constexpr uint32_t kFlashPageSize = 256;

// ============================================================================
// Tier 1 Storage: AP_FlashStorage (calibration, config, missions)
// ============================================================================

// Firmware reservation (bootloader + application)
// 16KB bootloader + 512KB firmware = 528KB, round to 532KB (0x85000 - 0x4000 = 0x81000)
constexpr uint32_t kFirmwareEnd = 0x10084000;

// Storage sector A (4KB)
constexpr uint32_t kStorageSectorA = 0x10084000;
constexpr uint32_t kStorageSectorAOffset = kStorageSectorA - kFlashBase;

// Storage sector B (4KB)
constexpr uint32_t kStorageSectorB = 0x10085000;
constexpr uint32_t kStorageSectorBOffset = kStorageSectorB - kFlashBase;

// Total storage size (what AP_FlashStorage manages)
// This is the logical storage size, not flash sector size
constexpr uint16_t kStorageSize = 4096;

// ============================================================================
// Tier 2A Storage: LittleFS (future - flight logs)
// ============================================================================

// LittleFS starts after storage sectors
constexpr uint32_t kLittleFsStart = 0x10086000;
constexpr uint32_t kLittleFsStartOffset = kLittleFsStart - kFlashBase;

// LittleFS size (remaining flash)
constexpr uint32_t kLittleFsSize = (kFlashBase + kFlashSize) - kLittleFsStart;

}  // namespace rocketchip
