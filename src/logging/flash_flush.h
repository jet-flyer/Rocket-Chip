// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// PSRAM/SRAM ring buffer → flash flush engine + flight table I/O
// Sector-by-sector flush: reads frames from ring buffer, packs into
// 4KB sectors, erases + programs flash via flash_safe_execute().
// Watchdog kicked between sectors (5s timeout, ~45ms per sector).
// xip_cache_clean_all() before flush (write-back dirty PSRAM cache lines).

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

// true if valid table loaded from flash, false if starting fresh
bool flight_table_load(FlightTableState* state);

bool flight_table_save(FlightTableState* state);

// Used by CLI 'E' erase-all command after flight_table_erase_all().
bool flight_table_erase_flash();

// Non-null table: erase used sectors [0, next_free). Null: all log sectors.
// pio_watchdog_feed between sectors.
bool flight_log_erase_all(const FlightTableState* table);

// ============================================================================
// Ring → Flash flush
// ============================================================================

// Sequence:
// 1. Snapshot stored frame count
// 2. Check flash capacity
// 3. xip_cache_clean_all() (council req. #1)
// 4. Per sector: read frames → 4KB buffer → erase → program → feed PIO watchdog
// 5. Build FlightLogEntry → flight_table_add_entry() → flight_table_save()
// 6. ring_reset()
FlushResult flush_ring_to_flash(RingBuffer* rb,
                                FlightTableState* table,
                                const FlightMetadata* metadata,
                                const FlightSummary* summary,
                                uint8_t log_rate_hz);

} // namespace rc

#endif // ROCKETCHIP_FLASH_FLUSH_H
