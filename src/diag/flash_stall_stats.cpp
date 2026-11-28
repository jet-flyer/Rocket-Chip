// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Flash stall and poll-gap statistics. Pure code (host-tested).

#include "diag/flash_stall_stats.h"

namespace rc {

namespace {

// Datasheet values (W25Q32JV/W25Q64JV/W25Q128JV rev. tables tPP and tSE;
// GD25Q64C tPP and tSE). The JEDEC ID does not show the GD25Q64C
// temperature grade; the 105 C grade values are used.
constexpr FlashPart kFlashParts[] = {
    {0xEFU, 0x40U, 0x16U, "W25Q32JV",  400U, 3000U, 45000U, 400000U},  // Tiny 2350
    {0xEFU, 0x40U, 0x17U, "W25Q64JV",  400U, 3000U, 45000U, 400000U},  // Feather RP2350
    {0xEFU, 0x40U, 0x18U, "W25Q128JV", 400U, 3000U, 45000U, 400000U},  // Fruit Jam
    {0xC8U, 0x40U, 0x17U, "GD25Q64C",  600U, 4000U, 50000U, 400000U},
};

constexpr size_t kFlashPartCount = sizeof(kFlashParts) / sizeof(kFlashParts[0]);

} // namespace

size_t flash_stall_bin(uint32_t dur_us) {
    size_t bin = kStallBinCount - 1U;
    for (size_t i = 0; i < (kStallBinCount - 1U); ++i) {
        if (dur_us < kStallBinEdgeUs[i]) {
            bin = i;
            break;
        }
    }
    return bin;
}

void flash_stall_note(FlashStallStats& stats, FlashOpType type, uint32_t dur_us, bool ok) {
    const auto idx = static_cast<size_t>(type);
    if (idx >= kFlashOpTypeCount) {
        return;
    }
    FlashOpStats& op = stats.op[idx];
    op.count++;
    if (!ok) {
        op.fail++;
    }
    if (dur_us > op.max_us) {
        op.max_us = dur_us;
    }
    op.total_us += dur_us;
    op.bins[flash_stall_bin(dur_us)]++;
}

uint32_t flash_stall_span_us(uint64_t start_us, uint64_t end_us) {
    if (end_us < start_us) {
        return 0U;
    }
    const uint64_t span = end_us - start_us;
    if (span > UINT32_MAX) {
        return UINT32_MAX;
    }
    return static_cast<uint32_t>(span);
}

const char* flash_op_type_name(FlashOpType type) {
    switch (type) {
    case FlashOpType::kErase:   return "erase";
    case FlashOpType::kProgram: return "program";
    case FlashOpType::kCommand: return "cmd";
    case FlashOpType::kCount:   return "?";
    default:                    return "?";
    }
}

void poll_gap_note(PollGapStats& stats, uint64_t now_us) {
    stats.calls++;
    if (!stats.started) {
        stats.started = true;
        stats.last_us = now_us;
        return;
    }
    const uint32_t gap = flash_stall_span_us(stats.last_us, now_us);
    stats.last_us = now_us;
    if (gap > stats.max_gap_us) {
        stats.max_gap_us = gap;
    }
    if (gap > kPollGapWarnUs) {
        stats.over_warn++;
    }
    if (gap > kPollGapLongUs) {
        stats.over_long++;
    }
}

const FlashPart* flash_part_from_jedec(uint8_t mfr, uint8_t mem_type, uint8_t capacity) {
    const FlashPart* found = nullptr;
    for (size_t i = 0; i < kFlashPartCount; ++i) {
        const FlashPart& p = kFlashParts[i];
        if ((p.mfr == mfr) && (p.mem_type == mem_type) && (p.capacity == capacity)) {
            found = &p;
            break;
        }
    }
    return found;
}

} // namespace rc
