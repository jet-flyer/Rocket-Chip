// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// PSRAM/SRAM ring buffer → flash flush engine + flight table I/O
// Sector-by-sector flush: reads frames from ring buffer, packs into
// 4KB sectors, erases + programs flash via flash_safe_execute().
// The caller does one sector for each call (one AO_RCOS tick), so the
// QV loop runs between sectors. Watchdog kicked after each sector.
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
    kBusy,                // A flush job is already in progress
};

// Result of one flush_job_step call. done: the job ended (result tells how).
struct FlushStep {
    FlushResult result;
    bool done;
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
// flush_job_begin:
// 1. Snapshot stored frame count
// 2. Check flash capacity
// 3. xip_cache_clean_all() (council req. #1)
// flush_job_step (one call for each sector):
// 4. Read frames → 4KB buffer → erase → program → feed PIO watchdog
//    Short ring_read_sequential returns kWriteError (no 0xFF-padded success).
// Last flush_job_step, after the last sector:
// 5. Build FlightLogEntry → flight_table_add_entry() → flight_table_save()
// 6. ring_reset()
// The caller must stop ring pushes from begin until the step that is done
// (AO_Logger_set_ring_hold). kOk from begin: the job started.
FlushResult flush_job_begin(RingBuffer* rb,
                            FlightTableState* table,
                            const FlightMetadata* metadata,
                            const FlightSummary* summary,
                            uint8_t log_rate_hz);

// One sector, or the table save after the last sector. An error ends the job.
FlushStep flush_job_step();

bool flush_job_active();

// Frames of the current (or last) job.
uint32_t flush_job_stored();

} // namespace rc

#endif // ROCKETCHIP_FLASH_FLUSH_H
