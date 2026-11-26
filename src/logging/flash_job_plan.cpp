// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Flash job plans: erase order and page split (see flash_job_plan.h).

#include "logging/flash_job_plan.h"

#include "logging/crc32.h"
#include "logging/flight_log_layout.h"

namespace rc {

namespace {
constexpr uint32_t kProgressTenths = 10U;
} // namespace

uint32_t erase_log_sector_count(uint32_t first_sector, uint32_t used_end) {
    return (used_end > first_sector) ? (used_end - first_sector) : 0U;
}

EraseBegin erase_plan_begin(ErasePlan& plan, const EraseLayout& layout,
                            bool other_job_active) {
    if (plan.active || other_job_active) {
        return EraseBegin::kBusy;
    }
    plan = ErasePlan{};
    plan.layout = layout;
    plan.total = kEraseFixedSteps + layout.log_sectors;
    plan.active = true;
    return EraseBegin::kOk;
}

EraseOp erase_plan_op(const ErasePlan& plan) {
    if (!plan.active || (plan.step >= plan.total)) {
        return {EraseOpKind::kNone, 0U};
    }
    if (plan.step == 0U) {
        return {EraseOpKind::kTableSector, plan.layout.table_a};
    }
    if (plan.step == 1U) {
        return {EraseOpKind::kTableSector, plan.layout.table_b};
    }
    if (plan.step == 2U) {
        return {EraseOpKind::kTableReset, 0U};
    }
    const uint32_t i = plan.step - kEraseFixedSteps;
    return {EraseOpKind::kLogSector,
            plan.layout.log_start + (i * plan.layout.sector_size)};
}

bool erase_plan_advance(ErasePlan& plan, bool op_ok) {
    if (!plan.active) {
        return true;
    }
    if (!op_ok) {
        plan.active = false;
        plan.failed = true;
        return true;
    }
    plan.step++;
    if (plan.step >= plan.total) {
        plan.active = false;
        return true;
    }
    return false;
}

void erase_plan_abort(ErasePlan& plan) {
    plan.active = false;
}

uint32_t erase_plan_done(const ErasePlan& plan) {
    return plan.step;
}

bool erase_plan_progress_due(ErasePlan& plan) {
    if (plan.total == 0U) {
        return false;
    }
    const uint32_t tenth = (plan.step * kProgressTenths) / plan.total;
    if (tenth > plan.decile) {
        plan.decile = tenth;
        return true;
    }
    return false;
}

uint32_t flash_program_chunk_count(uint32_t len, uint32_t page) {
    if (page == 0U) {
        return 0U;
    }
    return (len / page) + (((len % page) != 0U) ? 1U : 0U);
}

bool cal_ui_owns_keys(bool cal_ui_active, bool background_job_active) {
    return cal_ui_active && !background_job_active;
}

namespace {
constexpr uint32_t kCrc32Init = 0xFFFFFFFFU;  // Also the final XOR
} // namespace

DownloadBegin download_plan_begin(DownloadPlan& plan, uint32_t frame_count,
                                  uint32_t frame_size, uint32_t now_us,
                                  bool other_job_active) {
    if (download_plan_active(plan) || other_job_active) {
        return DownloadBegin::kBusy;
    }
    if (frame_size == 0U) {
        return DownloadBegin::kBadLayout;
    }
    plan = DownloadPlan{};
    plan.frame_count = frame_count;
    plan.frame_size = frame_size;
    plan.crc = kCrc32Init;
    plan.last_progress_us = now_us;
    plan.phase = DownloadPhase::kDrainLog;
    plan.end = DownloadEnd::kNone;
    return DownloadBegin::kOk;
}

bool download_plan_active(const DownloadPlan& plan) {
    return plan.phase != DownloadPhase::kIdle;
}

void download_plan_log_drained(DownloadPlan& plan, uint32_t now_us) {
    if (plan.phase != DownloadPhase::kDrainLog) {
        return;
    }
    plan.phase = DownloadPhase::kHeader;
    plan.last_progress_us = now_us;
}

void download_plan_header_sent(DownloadPlan& plan, uint32_t now_us) {
    if (plan.phase != DownloadPhase::kHeader) {
        return;
    }
    plan.last_progress_us = now_us;
    if (plan.frame_count == 0U) {
        plan.phase = DownloadPhase::kFooter;
        plan.end = DownloadEnd::kOk;
        return;
    }
    plan.phase = DownloadPhase::kPayload;
}

DownloadChunk download_plan_chunk(const DownloadPlan& plan, uint32_t room,
                                  uint32_t cap) {
    if (plan.phase != DownloadPhase::kPayload) {
        return DownloadChunk{0U, 0U};
    }
    const FlightLogRun run =
        flight_log_run_at(plan.frames_done, plan.frame_count, plan.frame_size);
    const uint32_t left = (run.frames * plan.frame_size) - plan.run_sent;
    uint32_t len = left;
    if (room < len) {
        len = room;
    }
    if (cap < len) {
        len = cap;
    }
    return DownloadChunk{run.offset + plan.run_sent, len};
}

void download_plan_sent(DownloadPlan& plan, const uint8_t* bytes,
                        uint32_t n, uint32_t now_us) {
    if (plan.phase != DownloadPhase::kPayload || n == 0U || bytes == nullptr) {
        return;
    }
    plan.crc = crc32_update(plan.crc, bytes, n);
    plan.bytes_sent += n;
    plan.run_sent += n;
    plan.last_progress_us = now_us;
    const FlightLogRun run =
        flight_log_run_at(plan.frames_done, plan.frame_count, plan.frame_size);
    if (plan.run_sent >= run.frames * plan.frame_size) {
        plan.frames_done += run.frames;
        plan.run_sent = 0U;
    }
    if (plan.frames_done >= plan.frame_count) {
        plan.phase = DownloadPhase::kFooter;
        plan.end = DownloadEnd::kOk;
    }
}

void download_plan_footer_sent(DownloadPlan& plan) {
    if (plan.phase == DownloadPhase::kFooter) {
        plan.phase = DownloadPhase::kIdle;
    }
}

bool download_plan_stalled(const DownloadPlan& plan, uint32_t now_us,
                           uint32_t timeout_us) {
    return download_plan_active(plan) &&
           (now_us - plan.last_progress_us >= timeout_us);
}

void download_plan_fail(DownloadPlan& plan, DownloadEnd why) {
    plan.phase = DownloadPhase::kIdle;
    plan.end = why;
}

uint32_t download_plan_crc(const DownloadPlan& plan) {
    return plan.crc ^ kCrc32Init;
}

uint32_t download_plan_total_bytes(const DownloadPlan& plan) {
    return plan.frame_count * plan.frame_size;
}

} // namespace rc
