// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file flash_flush.h
 * @brief PSRAM/SRAM ring buffer → flash flush engine + flight table I/O
 *
 * Sector-by-sector flush: reads frames from ring buffer, packs into
 * 4KB sectors, erases + programs flash via flash_safe_execute().
 * Watchdog kicked between sectors (5s timeout, ~45ms per sector).
 *
 * Council req. #1: xip_cache_clean_all() before flush to write-back
 * dirty PSRAM cache lines.
 *
 * IVP-53b: Flash Flush (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_FLASH_FLUSH_H
#define ROCKETCHIP_FLASH_FLUSH_H

#include <stdint.h>
#include "flight_table.h"
#include "ring_buffer.h"
#include "rocketchip/telemetry_state.h"

namespace rc {

// ============================================================================
// Flush result
// ============================================================================

enum class FlushResult : uint8_t {
    kOk = 0,
    kNoFrames,            // Ring buffer empty
    kFlashFull,           // Not enough sectors remaining
    kTableFull,           // 32 flights already stored
    kEraseError,          // flash_safe_execute erase failed
    kWriteError,          // flash_safe_execute program failed
    kTableSaveError,      // Flight table save failed
    kNotInitialized,      // Ring buffer or table not ready
};

// ============================================================================
// Flight table flash I/O (dual-sector pattern)
// ============================================================================

/**
 * @brief Load flight table from flash (dual-sector, higher sequence wins)
 * @param state  Flight table state to populate
 * @return true if valid table loaded from flash, false if starting fresh
 */
bool flight_table_load(FlightTableState* state);

/**
 * @brief Save flight table to flash (alternate sector, sequence bump)
 * @param state  Flight table state to persist
 * @return true on success
 */
bool flight_table_save(FlightTableState* state);

/**
 * @brief Erase both flight table sectors in flash
 * @return true on success
 *
 * Used by CLI 'E' erase-all command after flight_table_erase_all().
 */
bool flight_table_erase_flash();

/**
 * @brief Erase all flight log data sectors (0x080000–0x7FBFFF)
 * @param table           Flight table state (used to determine how many sectors to erase)
 * @param kick_watchdog  Callback to kick watchdog between sectors (may be nullptr)
 * @return true on success
 *
 * Only erases sectors that have been used (based on next_free_sector).
 * Watchdog MUST be kicked between sectors.
 */
bool flight_log_erase_all(const FlightTableState* table, void (*kick_watchdog)());

// ============================================================================
// Ring → Flash flush
// ============================================================================

/**
 * @brief Flush ring buffer contents to flash
 * @param rb             Ring buffer to read from
 * @param table          Flight table state (updated with new entry)
 * @param metadata       Flight metadata (UTC anchor etc.)
 * @param summary        Flight summary (max alt, speed, etc.)
 * @param log_rate_hz    Logging rate (25 or 50)
 * @param kick_watchdog  Callback to kick watchdog between sectors
 * @return FlushResult status
 *
 * Sequence:
 * 1. Snapshot stored frame count
 * 2. Check flash capacity
 * 3. xip_cache_clean_all() (council req. #1)
 * 4. Per sector: read frames → 4KB buffer → erase → program → kick watchdog
 * 5. Build FlightLogEntry → flight_table_add_entry() → flight_table_save()
 * 6. ring_reset()
 */
FlushResult flush_ring_to_flash(RingBuffer* rb,
                                FlightTableState* table,
                                const FlightMetadata* metadata,
                                const FlightSummary* summary,
                                uint8_t log_rate_hz,
                                void (*kick_watchdog)());

} // namespace rc

#endif // ROCKETCHIP_FLASH_FLUSH_H
