// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// CLI 'd' flight download job (logging/flight_download.h). Reads the
// flight through XIP; no flash operations. Wire order: the rc_log text
// that waits, the RCBIN line, the payload, then RCEND:<crc32>. From the
// RCBIN line to the RCEND line, rc_log text stays in its ring
// (rc_log_hold_cdc_binary), so no text gets into the binary stream. That
// text goes out after RCEND.
#include "logging/flight_download.h"

#include "logging/flash_flush.h"
#include "rocketchip/rc_log.h"

#include "hardware/regs/addressmap.h"
#include "pico/time.h"
#include "tusb.h"

#include <stddef.h>

namespace rc {
namespace {

// One step sends for at most kStepBudgetUs and kStepMaxBytes. The USB
// CDC TX buffer (1024 bytes) is the usual limit inside one step.
constexpr uint32_t kStepBudgetUs = 2000U;
constexpr uint32_t kStepMaxBytes = 4096U;
constexpr uint32_t kChunkMaxBytes = 512U;  // One tud_cdc_write call
// No bytes went out for this long: stop with RCERR:STALL.
constexpr uint32_t kStallTimeoutUs = 2000000U;
constexpr size_t kLineBytes = 48U;  // RCBIN or RCEND line

DownloadPlan g_plan{};
uint32_t g_startSector = 0U;
char g_line[kLineBytes] = {};
uint32_t g_lineLen = 0U;

const uint8_t* flight_base() {
    return reinterpret_cast<const uint8_t*>(
        XIP_BASE + (g_startSector * kFlashSectorSize));
}

// The host parses whole lines, so send all of the line or none of it.
bool send_line() {
    if (tud_cdc_write_available() < g_lineLen) {
        return false;
    }
    (void)tud_cdc_write(g_line, g_lineLen);  // Room was checked above
    tud_cdc_write_flush();
    return true;
}

const char* end_name(DownloadEnd why) {
    switch (why) {
    case DownloadEnd::kStalled:
        return "STALL";
    case DownloadEnd::kDisconnected:
        return "DISCONNECT";
    case DownloadEnd::kAborted:
        return "ABORT";
    case DownloadEnd::kNone:
    case DownloadEnd::kOk:
        break;
    }
    return "FAIL";
}

// Stop the job. rc_log text goes to USB again; RCERR goes through rc_log.
void fail_job(DownloadEnd why) {
    const uint32_t frames = g_plan.frames_done;
    download_plan_fail(g_plan, why);
    rc_log_hold_cdc_binary(0);
    rc_log("\nRCERR:%s:%lu\n", end_name(why),
           static_cast<unsigned long>(frames));
}

// Send the rc_log text that waits. When the ring is empty, hold it, so
// the RCBIN line is the next text on the wire.
void step_drain_log(uint32_t now_us) {
    rc_log_drain_to_cdc();
    if (rc_log_pending_bytes() == 0U) {
        rc_log_hold_cdc_binary(1);
        download_plan_log_drained(g_plan, now_us);
    }
}

// Send payload bytes until the step bound. When the TX buffer is full,
// wait inside the bound: the USB IRQ empties the buffer.
void step_payload(uint32_t start_us) {
    const uint8_t* const base = flight_base();
    uint32_t sent = 0U;
    while (g_plan.phase == DownloadPhase::kPayload && sent < kStepMaxBytes &&
           (time_us_32() - start_us) < kStepBudgetUs) {
        const uint32_t left = kStepMaxBytes - sent;
        const uint32_t cap = (left < kChunkMaxBytes) ? left : kChunkMaxBytes;
        const DownloadChunk chunk =
            download_plan_chunk(g_plan, tud_cdc_write_available(), cap);
        if (chunk.len == 0U) {
            continue;
        }
        const uint8_t* const src = base + chunk.offset;
        const uint32_t n = tud_cdc_write(src, chunk.len);
        download_plan_sent(g_plan, src, n, time_us_32());
        sent += n;
    }
    if (sent > 0U) {
        tud_cdc_write_flush();
    }
}

// Send RCEND:<crc32>. Returns true when the job ended.
bool step_footer() {
    g_lineLen = static_cast<uint32_t>(rc_snprintf(
        g_line, sizeof(g_line), "RCEND:%08lX\n",
        static_cast<unsigned long>(download_plan_crc(g_plan))));
    if (!send_line()) {
        return false;
    }
    download_plan_footer_sent(g_plan);
    rc_log_hold_cdc_binary(0);
    return true;
}

} // namespace

DownloadBegin download_job_begin(const FlightLogEntry& entry,
                                 uint32_t flight_num) {
    const DownloadBegin begin = download_plan_begin(
        g_plan, entry.frame_count, entry.frame_size, time_us_32(),
        flush_job_active() || erase_job_active());
    if (begin != DownloadBegin::kOk) {
        return begin;
    }
    g_startSector = entry.start_sector;
    g_lineLen = static_cast<uint32_t>(rc_snprintf(
        g_line, sizeof(g_line), "RCBIN:%lu:%lu:%u:%lu\n",
        static_cast<unsigned long>(flight_num),
        static_cast<unsigned long>(entry.frame_count),
        static_cast<unsigned>(entry.log_rate_hz),
        static_cast<unsigned long>(entry.frame_size)));
    return begin;
}

bool download_job_step() {
    if (!download_plan_active(g_plan)) {
        return true;
    }
    const uint32_t now_us = time_us_32();
    if (!tud_cdc_connected()) {
        fail_job(DownloadEnd::kDisconnected);
        return true;
    }
    if (download_plan_stalled(g_plan, now_us, kStallTimeoutUs)) {
        fail_job(DownloadEnd::kStalled);
        return true;
    }
    switch (g_plan.phase) {
    case DownloadPhase::kDrainLog:
        step_drain_log(now_us);
        break;
    case DownloadPhase::kHeader:
        if (send_line()) {
            download_plan_header_sent(g_plan, now_us);
        }
        break;
    case DownloadPhase::kPayload:
        step_payload(now_us);
        break;
    case DownloadPhase::kFooter:
    case DownloadPhase::kIdle:
        break;
    }
    if (g_plan.phase == DownloadPhase::kFooter) {
        return step_footer();
    }
    return false;
}

bool download_job_active() {
    return download_plan_active(g_plan);
}

void download_job_abort() {
    if (download_plan_active(g_plan)) {
        fail_job(DownloadEnd::kAborted);
    }
}

DownloadProgress download_job_progress() {
    return DownloadProgress{g_plan.bytes_sent,
                            download_plan_total_bytes(g_plan)};
}

} // namespace rc
