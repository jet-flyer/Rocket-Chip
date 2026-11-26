// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Flash job plans (FSK open items A and B). Pure code: no Pico SDK and
// no QP, so the host tests can use it. logging/flash_flush.cpp does the
// flash operations; this module only gives the order of the operations.
//
// Erase plan (CLI 'x'). One flash operation for each AO_RCOS tick:
//   step 0      erase flight-table sector A
//   step 1      erase flight-table sector B
//   step 2      reset the RAM table, then save it (sector A)
//   step 3..    erase one used flight-log sector
// The table goes first. If the job stops (error, ARM, power loss), the
// table on flash never points at an erased log sector. A log sector that
// stays programmed is safe: the flush erases each sector before it
// programs it.
//
// Program split: one flash_safe_execute call for each 256-byte page, so
// one IRQ-off window is one page program (about 0.6 ms), not a 4 KB
// sector (about 9 ms).
//
// Download plan (CLI 'd'). AO_RCOS calls download_job_step one time each
// tick (logging/flight_download.h). One step sends a bounded part of the
// flight from XIP flash to USB CDC, so no handler blocks for long. Order
// on the wire: the log text that waits, the RCBIN line, the payload (the
// frame runs of flight_log_layout.h), then RCEND:<crc32>.
#ifndef ROCKETCHIP_LOGGING_FLASH_JOB_PLAN_H
#define ROCKETCHIP_LOGGING_FLASH_JOB_PLAN_H

#include <stdint.h>

namespace rc {

enum class EraseOpKind : uint8_t {
    kNone = 0,      // No job, or the job ended
    kTableSector,   // Erase one flight-table sector (offset)
    kTableReset,    // Clear the RAM table and save it
    kLogSector,     // Erase one flight-log sector (offset)
};

struct EraseOp {
    EraseOpKind kind;
    uint32_t offset;  // Flash byte offset. 0 for kTableReset and kNone.
};

// Flash layout for one erase job (byte offsets).
struct EraseLayout {
    uint32_t table_a;
    uint32_t table_b;
    uint32_t log_start;     // First flight-log sector
    uint32_t sector_size;
    uint32_t log_sectors;   // Used log sectors to erase
};

struct ErasePlan {
    EraseLayout layout;
    uint32_t step;          // Next step (0 .. total)
    uint32_t total;         // kEraseFixedSteps + log_sectors
    uint32_t decile;        // Last progress tenth that was reported
    bool active;
    bool failed;            // The job ended on a failed operation
};

enum class EraseBegin : uint8_t {
    kOk = 0,
    kBusy,      // An erase or a flush job is in progress
};

// Steps before the log sectors: table A, table B, table reset.
inline constexpr uint32_t kEraseFixedSteps = 3U;

// Used log sectors from the first log sector to the next free sector.
// 0 if used_end is not after first_sector.
uint32_t erase_log_sector_count(uint32_t first_sector, uint32_t used_end);

// Start a job. other_job_active: a flush or a download job is in progress.
// A job that is in progress gives kBusy and does not change.
EraseBegin erase_plan_begin(ErasePlan& plan, const EraseLayout& layout,
                            bool other_job_active);

// The operation for the current step. kNone when no job is active.
EraseOp erase_plan_op(const ErasePlan& plan);

// Record the result of the current operation. A failed operation ends
// the job. Returns true when the job ended (completed or failed).
bool erase_plan_advance(ErasePlan& plan, bool op_ok);

// Stop the job now (for example, the vehicle left the ground state).
void erase_plan_abort(ErasePlan& plan);

// Steps that are done, for progress text.
uint32_t erase_plan_done(const ErasePlan& plan);

// True one time for each new tenth of the job that is done (10%, 20% ...
// 100%). Call after erase_plan_advance.
bool erase_plan_progress_due(ErasePlan& plan);

// Page-sized program calls for len bytes (rounded up). 0 if page is 0.
uint32_t flash_program_chunk_count(uint32_t len, uint32_t page);

// Does the AO_RCOS UI read the USB keys itself (so the CLI must not)?
// cal_ui_active: a cal, prompt, or job state is set.
// background_job_active: that state is the 'l' flush, the 'x' erase, or
// the 'd' download job. The jobs do not read keys, so the CLI must read
// them during a job. If not, the busy rules never run and the keys wait
// in USB.
bool cal_ui_owns_keys(bool cal_ui_active, bool background_job_active);

enum class DownloadPhase : uint8_t {
    kIdle = 0,   // No job
    kDrainLog,   // Send the rc_log text that waits, before the header
    kHeader,     // Send the RCBIN line
    kPayload,    // Send frame bytes
    kFooter,     // Send the RCEND line
};

enum class DownloadEnd : uint8_t {
    kNone = 0,      // The job runs, or no job started
    kOk,            // All payload bytes went out (RCEND is next or sent)
    kStalled,       // No bytes went out for the stall timeout
    kDisconnected,  // The host closed the port
    kAborted,       // download_job_abort
};

enum class DownloadBegin : uint8_t {
    kOk = 0,
    kBusy,       // A download, flush, or erase job is in progress
    kBadLayout,  // frame_size is 0
};

struct DownloadPlan {
    uint32_t frame_count;
    uint32_t frame_size;
    uint32_t frames_done;       // Frames in runs that are fully sent
    uint32_t run_sent;          // Bytes sent of the current run
    uint32_t bytes_sent;        // Payload bytes sent
    uint32_t crc;               // Running CRC-32 (not inverted yet)
    uint32_t last_progress_us;  // Last time bytes went out (or the start)
    DownloadPhase phase;
    DownloadEnd end;
};

// The part of the payload to send next. offset: bytes from the start of
// the flight's first sector. len 0: nothing to send now.
struct DownloadChunk {
    uint32_t offset;
    uint32_t len;
};

// Start a job. other_job_active: a flush or an erase job is in progress.
// A job that is in progress gives kBusy and does not change.
DownloadBegin download_plan_begin(DownloadPlan& plan, uint32_t frame_count,
                                  uint32_t frame_size, uint32_t now_us,
                                  bool other_job_active);

bool download_plan_active(const DownloadPlan& plan);

// kDrainLog to kHeader: the log text that waited is sent.
void download_plan_log_drained(DownloadPlan& plan, uint32_t now_us);

// kHeader to kPayload (kFooter if the flight has no frames).
void download_plan_header_sent(DownloadPlan& plan, uint32_t now_us);

// The next chunk in kPayload: at most room and cap bytes, and never past
// the end of the current frame run (the next run starts after a gap).
DownloadChunk download_plan_chunk(const DownloadPlan& plan, uint32_t room,
                                  uint32_t cap);

// Record n bytes of the chunk that went out (bytes: the same data). Adds
// them to the CRC. After the last byte, the phase is kFooter (end kOk).
void download_plan_sent(DownloadPlan& plan, const uint8_t* bytes,
                        uint32_t n, uint32_t now_us);

// kFooter to kIdle: the RCEND line is sent.
void download_plan_footer_sent(DownloadPlan& plan);

// True if no bytes went out for timeout_us (time_us_32 wrap is safe).
bool download_plan_stalled(const DownloadPlan& plan, uint32_t now_us,
                           uint32_t timeout_us);

// Stop the job now (stall, disconnect, abort). The phase is kIdle.
void download_plan_fail(DownloadPlan& plan, DownloadEnd why);

// The CRC-32 of the payload bytes sent (final value).
uint32_t download_plan_crc(const DownloadPlan& plan);

// Payload bytes in the flight: frame_count * frame_size.
uint32_t download_plan_total_bytes(const DownloadPlan& plan);

} // namespace rc

#endif // ROCKETCHIP_LOGGING_FLASH_JOB_PLAN_H
