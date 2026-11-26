// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// PSRAM/SRAM ring buffer → flash flush engine + flight table I/O
// Sector-by-sector flush: reads frames from ring buffer, packs into
// 4KB sectors, erases + programs flash via flash_safe_execute(). Each
// program is one flash_safe_execute call for each 256-byte page.
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
    kBusy,                // A flush, erase, or download job is in progress
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

// ============================================================================
// Erase all flights (CLI 'x'), one flash operation for each call
// ============================================================================
// Order (logging/flash_job_plan.h): table sector A, table sector B, RAM
// table reset + save, then each used log sector. AO_RCOS calls
// erase_job_step one time each tick. The ring buffer is not touched.

struct EraseProgress {
    uint32_t done;         // Steps done
    uint32_t total;        // All steps
    uint32_t log_sectors;  // Log sectors in the job
    bool report;           // A new tenth is done: print progress
};

// kOk: the job started. kBusy: a flush or an erase is in progress.
// kNotInitialized: no loaded table.
FlushResult erase_job_begin(FlightTableState* table);

// One operation. done with kOk: all flights erased. An error ends the job.
FlushStep erase_job_step();

bool erase_job_active();

// Stop before the next operation (the table on flash stays valid).
void erase_job_abort();

// Progress for the job in progress (or the last job).
EraseProgress erase_job_progress();

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
// kBusy if a flush or an erase job is in progress.
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
