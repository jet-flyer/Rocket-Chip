// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file psram_init.h
 * @brief APS6404L-3SQR PSRAM initialization via QMI CS1
 *
 * Initializes 8MB QSPI PSRAM on the Adafruit Feather RP2350 HSTX.
 * GPIO 8 → XIP_CS1, QPI mode (0xEB read, 0x38 write).
 *
 * Based on SparkFun sparkfun-pico and AudioMorphology/PSRAM (MIT license).
 * Both derived from Arduino-Pico (earlephilhower).
 *
 * XIP memory map for CS1 (PSRAM):
 *   0x11000000  Cached XIP access
 *   0x15000000  Uncached XIP access
 *
 * Council req. #1: Use uncached alias (0x15000000) for crash recovery
 *   header writes to avoid cache coherency issues.
 * Council req. #3: PSRAM is volatile — power loss erases it. Flash
 *   flight table is the durable record.
 *
 * IVP-52a: PSRAM Init + Self-Test (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_PSRAM_INIT_H
#define ROCKETCHIP_PSRAM_INIT_H

#include <stdint.h>
#include <stddef.h>

namespace rc {

// PSRAM memory layout
static constexpr uint32_t kPsramCachedBase   = 0x11000000U;
static constexpr uint32_t kPsramUncachedBase = 0x15000000U;
static constexpr uint32_t kPsramExpectedSize = 8U * 1024U * 1024U;  // 8MB

/**
 * @brief Initialize PSRAM via QMI CS1
 * @param cs_pin GPIO pin for CS1 (8 on Adafruit Feather RP2350 HSTX)
 * @return Detected PSRAM size in bytes, or 0 on failure
 *
 * Must be called BEFORE stdio_init_all() and flash operations.
 * The entire function runs from SRAM (not flash).
 */
size_t psram_init(uint32_t cs_pin);

/**
 * @brief Self-test: write/read pattern at 3 addresses
 * @param size PSRAM size in bytes (from psram_init)
 * @return true if all 3 test points pass
 *
 * Tests offset 0, size/2, and size-4 to catch addressing issues.
 */
bool psram_self_test(size_t size);

/**
 * @brief Get pointer to PSRAM base (cached)
 * @return Pointer to cached PSRAM base, or nullptr if not initialized
 */
uint8_t* psram_base_ptr();

/**
 * @brief Get pointer to PSRAM base (uncached — for crash recovery header)
 * @return Pointer to uncached PSRAM base, or nullptr if not initialized
 */
uint8_t* psram_uncached_base_ptr();

/**
 * @brief Get detected PSRAM size
 * @return Size in bytes, or 0 if not initialized
 */
size_t psram_get_size();

/**
 * @brief Verify PSRAM data survives a flash_safe_execute() cycle
 * @return true if data integrity preserved after flash erase+program
 *
 * Council req. #2 (hard gate): Write known pattern → flash_safe_execute()
 * erase+program → read back → byte-for-byte verify. Validates SDK's
 * QMI M1 save/restore for our CS1-not-in-FLASH_DEVINFO case.
 */
bool psram_flash_safe_test();

} // namespace rc

#endif // ROCKETCHIP_PSRAM_INIT_H
