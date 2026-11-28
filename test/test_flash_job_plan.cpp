// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Flash job plans: CLI 'x' erase order (one operation each tick), busy
// rules, progress, and the 256-byte program split.
#include <gtest/gtest.h>

#include "logging/flash_job_plan.h"
#include "logging/crc32.h"
#include "logging/flight_log_layout.h"

#include <stdint.h>

#include <algorithm>
#include <vector>

using rc::EraseBegin;
using rc::EraseLayout;
using rc::EraseOp;
using rc::EraseOpKind;
using rc::ErasePlan;

namespace {

constexpr uint32_t kSector = 4096U;
constexpr uint32_t kTableA = 0x00100000U;
constexpr uint32_t kTableB = 0x00101000U;
constexpr uint32_t kLogStart = 0x00102000U;

EraseLayout layout(uint32_t log_sectors) {
    return EraseLayout{kTableA, kTableB, kLogStart, kSector, log_sectors};
}

// Run the plan to its end with every operation OK. Returns the steps.
uint32_t run_all(ErasePlan& p) {
    uint32_t steps = 0U;
    for (uint32_t guard = 0U; guard < 1000U; ++guard) {
        if (erase_plan_op(p).kind == EraseOpKind::kNone) {
            break;
        }
        ++steps;
        if (erase_plan_advance(p, true)) {
            break;
        }
    }
    return steps;
}

} // namespace

TEST(EraseLogCount, UsedSectorsFromFirstToNextFree) {
    EXPECT_EQ(rc::erase_log_sector_count(258U, 270U), 12U);
    EXPECT_EQ(rc::erase_log_sector_count(258U, 258U), 0U);
    EXPECT_EQ(rc::erase_log_sector_count(258U, 100U), 0U);
}

TEST(ErasePlan, TableFirstThenEachLogSectorInOrder) {
    ErasePlan p{};
    ASSERT_EQ(rc::erase_plan_begin(p, layout(3U), false), EraseBegin::kOk);
    EXPECT_TRUE(p.active);
    EXPECT_EQ(p.total, 6U);

    const EraseOp expect[] = {
        {EraseOpKind::kTableSector, kTableA},
        {EraseOpKind::kTableSector, kTableB},
        {EraseOpKind::kTableReset, 0U},
        {EraseOpKind::kLogSector, kLogStart},
        {EraseOpKind::kLogSector, kLogStart + kSector},
        {EraseOpKind::kLogSector, kLogStart + (2U * kSector)},
    };
    for (uint32_t i = 0U; i < 6U; ++i) {
        const EraseOp op = rc::erase_plan_op(p);
        EXPECT_EQ(op.kind, expect[i].kind) << "step " << i;
        EXPECT_EQ(op.offset, expect[i].offset) << "step " << i;
        const bool ended = rc::erase_plan_advance(p, true);
        EXPECT_EQ(ended, i == 5U) << "step " << i;
    }
    EXPECT_FALSE(p.active);
    EXPECT_FALSE(p.failed);
    EXPECT_EQ(rc::erase_plan_op(p).kind, EraseOpKind::kNone);
    EXPECT_EQ(rc::erase_plan_done(p), 6U);
}

TEST(ErasePlan, NoUsedLogSectorsStillClearsTheTable) {
    ErasePlan p{};
    ASSERT_EQ(rc::erase_plan_begin(p, layout(0U), false), EraseBegin::kOk);
    EXPECT_EQ(run_all(p), rc::kEraseFixedSteps);
    EXPECT_FALSE(p.active);
}

TEST(ErasePlan, OneOperationForEachStep) {
    ErasePlan p{};
    ASSERT_EQ(rc::erase_plan_begin(p, layout(100U), false), EraseBegin::kOk);
    EXPECT_EQ(run_all(p), 103U);
}

TEST(ErasePlan, SecondEraseIsBusyAndDoesNotRestart) {
    ErasePlan p{};
    ASSERT_EQ(rc::erase_plan_begin(p, layout(5U), false), EraseBegin::kOk);
    (void)rc::erase_plan_advance(p, true);
    (void)rc::erase_plan_advance(p, true);
    EXPECT_EQ(rc::erase_plan_begin(p, layout(9U), false), EraseBegin::kBusy);
    EXPECT_EQ(p.step, 2U);
    EXPECT_EQ(p.total, 8U);
    EXPECT_TRUE(p.active);
}

TEST(ErasePlan, ActiveFlushMakesEraseBusy) {
    ErasePlan p{};
    EXPECT_EQ(rc::erase_plan_begin(p, layout(5U), true), EraseBegin::kBusy);
    EXPECT_FALSE(p.active);
    EXPECT_EQ(rc::erase_plan_op(p).kind, EraseOpKind::kNone);
    // After the flush ends, the erase can start.
    EXPECT_EQ(rc::erase_plan_begin(p, layout(5U), false), EraseBegin::kOk);
}

TEST(ErasePlan, FailedOperationEndsTheJob) {
    ErasePlan p{};
    ASSERT_EQ(rc::erase_plan_begin(p, layout(5U), false), EraseBegin::kOk);
    (void)rc::erase_plan_advance(p, true);
    EXPECT_TRUE(rc::erase_plan_advance(p, false));
    EXPECT_FALSE(p.active);
    EXPECT_TRUE(p.failed);
    EXPECT_EQ(rc::erase_plan_op(p).kind, EraseOpKind::kNone);
    // A new job can start after a failure.
    EXPECT_EQ(rc::erase_plan_begin(p, layout(5U), false), EraseBegin::kOk);
    EXPECT_FALSE(p.failed);
    EXPECT_EQ(p.step, 0U);
}

TEST(ErasePlan, AbortStopsBeforeTheNextOperation) {
    ErasePlan p{};
    ASSERT_EQ(rc::erase_plan_begin(p, layout(5U), false), EraseBegin::kOk);
    rc::erase_plan_abort(p);
    EXPECT_FALSE(p.active);
    EXPECT_EQ(rc::erase_plan_op(p).kind, EraseOpKind::kNone);
    EXPECT_TRUE(rc::erase_plan_advance(p, true));
}

TEST(ErasePlan, ProgressOneTimeForEachTenth) {
    ErasePlan p{};
    ASSERT_EQ(rc::erase_plan_begin(p, layout(97U), false), EraseBegin::kOk);  // 100 steps
    uint32_t reports = 0U;
    for (uint32_t i = 0U; i < 100U; ++i) {
        (void)rc::erase_plan_advance(p, true);
        if (rc::erase_plan_progress_due(p)) {
            ++reports;
        }
    }
    EXPECT_EQ(reports, 10U);
    EXPECT_FALSE(rc::erase_plan_progress_due(p));
}

TEST(ErasePlan, ProgressShortJobReportsEnd) {
    ErasePlan p{};
    ASSERT_EQ(rc::erase_plan_begin(p, layout(0U), false), EraseBegin::kOk);  // 3 steps
    uint32_t reports = 0U;
    for (uint32_t i = 0U; i < 3U; ++i) {
        (void)rc::erase_plan_advance(p, true);
        if (rc::erase_plan_progress_due(p)) {
            ++reports;
        }
    }
    EXPECT_EQ(reports, 3U);
    EXPECT_EQ(p.decile, 10U);
}

TEST(ProgramChunks, SectorIsSixteenPages) {
    EXPECT_EQ(rc::flash_program_chunk_count(4096U, 256U), 16U);
}

TEST(ProgramChunks, RoundsUpAndHandlesEdges) {
    EXPECT_EQ(rc::flash_program_chunk_count(2816U, 256U), 11U);
    EXPECT_EQ(rc::flash_program_chunk_count(2817U, 256U), 12U);
    EXPECT_EQ(rc::flash_program_chunk_count(1U, 256U), 1U);
    EXPECT_EQ(rc::flash_program_chunk_count(0U, 256U), 0U);
    EXPECT_EQ(rc::flash_program_chunk_count(4096U, 0U), 0U);
}

// 'l' / 'x' jobs: the CLI reads keys, so a key during a job gets "Busy".
TEST(CalUiOwnsKeys, IdleLeavesKeysToCli) {
    EXPECT_FALSE(rc::cal_ui_owns_keys(false, false));
}

TEST(CalUiOwnsKeys, PromptOrCalReadsKeys) {
    EXPECT_TRUE(rc::cal_ui_owns_keys(true, false));
}

TEST(CalUiOwnsKeys, FlashJobLeavesKeysToCli) {
    EXPECT_FALSE(rc::cal_ui_owns_keys(true, true));
}

// ---- Download plan ('d'): one bounded USB step each tick ----
using rc::DownloadBegin;
using rc::DownloadChunk;
using rc::DownloadEnd;
using rc::DownloadPhase;
using rc::DownloadPlan;

namespace {

constexpr uint32_t kFrame = 61U;  // rc::kPcmFrameStandardSize
constexpr uint32_t kT0 = 1000U;
constexpr uint32_t kStallUs = 2000000U;

uint8_t frame_byte(uint32_t frame, uint32_t k) {
    return static_cast<uint8_t>((frame * 7U) + k);
}

// Flash image of one flight: each frame at its layout offset. The header
// and the gap bytes are 0xEE, so a chunk that reads them fails the test.
std::vector<uint8_t> flight_image(uint32_t frames) {
    std::vector<uint8_t> img(
        rc::flight_log_sectors_needed(frames, kFrame) * rc::kFlashSectorSize,
        0xEEU);
    for (uint32_t i = 0; i < frames; ++i) {
        const uint32_t off = rc::flight_log_frame_offset(i, kFrame);
        for (uint32_t k = 0; k < kFrame; ++k) {
            img[off + k] = frame_byte(i, k);
        }
    }
    return img;
}

std::vector<uint8_t> frames_in_order(uint32_t frames) {
    std::vector<uint8_t> out;
    for (uint32_t i = 0; i < frames; ++i) {
        for (uint32_t k = 0; k < kFrame; ++k) {
            out.push_back(frame_byte(i, k));
        }
    }
    return out;
}

DownloadPlan in_payload(uint32_t frames) {
    DownloadPlan p{};
    EXPECT_EQ(rc::download_plan_begin(p, frames, kFrame, kT0, false),
              DownloadBegin::kOk);
    rc::download_plan_log_drained(p, kT0);
    rc::download_plan_header_sent(p, kT0);
    return p;
}

// Send the payload. room/cap limit each chunk; the "USB" takes at most
// take bytes of each chunk. Returns the bytes in wire order.
std::vector<uint8_t> send_all(DownloadPlan& p, const std::vector<uint8_t>& img,
                              uint32_t room, uint32_t cap, uint32_t take) {
    std::vector<uint8_t> wire;
    for (int guard = 0; guard < 200000 && p.phase == DownloadPhase::kPayload;
         ++guard) {
        const DownloadChunk c = rc::download_plan_chunk(p, room, cap);
        EXPECT_GT(c.len, 0U);
        EXPECT_LE(c.len, std::min(room, cap));
        if (c.len == 0U) {
            break;
        }
        const uint32_t n = std::min(c.len, take);
        wire.insert(wire.end(), img.begin() + c.offset,
                    img.begin() + c.offset + n);
        rc::download_plan_sent(p, &img[c.offset], n, kT0);
    }
    return wire;
}

} // namespace

TEST(DownloadPlan, PhasesGoInWireOrder) {
    DownloadPlan p{};
    EXPECT_FALSE(rc::download_plan_active(p));
    ASSERT_EQ(rc::download_plan_begin(p, 10U, kFrame, kT0, false),
              DownloadBegin::kOk);
    EXPECT_EQ(p.phase, DownloadPhase::kDrainLog);
    EXPECT_EQ(rc::download_plan_chunk(p, 1024U, 512U).len, 0U);
    rc::download_plan_header_sent(p, kT0);  // Not yet: log first
    EXPECT_EQ(p.phase, DownloadPhase::kDrainLog);
    rc::download_plan_log_drained(p, kT0);
    EXPECT_EQ(p.phase, DownloadPhase::kHeader);
    EXPECT_EQ(rc::download_plan_chunk(p, 1024U, 512U).len, 0U);
    rc::download_plan_header_sent(p, kT0);
    EXPECT_EQ(p.phase, DownloadPhase::kPayload);
}

TEST(DownloadPlan, PayloadIsTheFramesInOrderWithNoGapBytes) {
    const uint32_t frames = 200U;  // More than two sectors
    const std::vector<uint8_t> img = flight_image(frames);
    DownloadPlan p = in_payload(frames);
    const std::vector<uint8_t> wire = send_all(p, img, 700U, 512U, 512U);
    const std::vector<uint8_t> expect = frames_in_order(frames);
    EXPECT_EQ(wire, expect);
    EXPECT_EQ(p.phase, DownloadPhase::kFooter);
    EXPECT_EQ(p.end, DownloadEnd::kOk);
    EXPECT_EQ(p.bytes_sent, frames * kFrame);
    EXPECT_EQ(rc::download_plan_total_bytes(p), frames * kFrame);
    EXPECT_EQ(rc::download_plan_crc(p),
              rc::crc32(expect.data(), static_cast<uint32_t>(expect.size())));
}

TEST(DownloadPlan, ShortWritesStillSendAllBytesOnce) {
    const uint32_t frames = 150U;
    const std::vector<uint8_t> img = flight_image(frames);
    DownloadPlan p = in_payload(frames);
    const std::vector<uint8_t> wire = send_all(p, img, 1024U, 512U, 37U);
    const std::vector<uint8_t> expect = frames_in_order(frames);
    EXPECT_EQ(wire, expect);
    EXPECT_EQ(rc::download_plan_crc(p),
              rc::crc32(expect.data(), static_cast<uint32_t>(expect.size())));
}

TEST(DownloadPlan, ChunkObeysRoomAndCap) {
    DownloadPlan p = in_payload(100U);
    EXPECT_EQ(rc::download_plan_chunk(p, 0U, 512U).len, 0U);
    EXPECT_EQ(rc::download_plan_chunk(p, 100U, 512U).len, 100U);
    EXPECT_EQ(rc::download_plan_chunk(p, 1024U, 64U).len, 64U);
    EXPECT_EQ(rc::download_plan_chunk(p, 1024U, 512U).offset,
              rc::flight_log_frame_offset(0U, kFrame));
}

TEST(DownloadPlan, BusyWhenAJobRuns) {
    DownloadPlan p{};
    EXPECT_EQ(rc::download_plan_begin(p, 10U, kFrame, kT0, true),
              DownloadBegin::kBusy);
    EXPECT_FALSE(rc::download_plan_active(p));
    ASSERT_EQ(rc::download_plan_begin(p, 10U, kFrame, kT0, false),
              DownloadBegin::kOk);
    EXPECT_EQ(rc::download_plan_begin(p, 20U, kFrame, kT0, false),
              DownloadBegin::kBusy);
    EXPECT_EQ(p.frame_count, 10U);
}

TEST(DownloadPlan, ZeroFrameSizeIsBadLayout) {
    DownloadPlan p{};
    EXPECT_EQ(rc::download_plan_begin(p, 10U, 0U, kT0, false),
              DownloadBegin::kBadLayout);
    EXPECT_FALSE(rc::download_plan_active(p));
}

TEST(DownloadPlan, NoFramesGoesFromHeaderToFooter) {
    DownloadPlan p = in_payload(0U);
    EXPECT_EQ(p.phase, DownloadPhase::kFooter);
    EXPECT_EQ(p.end, DownloadEnd::kOk);
    EXPECT_EQ(rc::download_plan_crc(p), rc::crc32(nullptr, 0U));
}

TEST(DownloadPlan, FooterSentEndsTheJob) {
    DownloadPlan p = in_payload(0U);
    rc::download_plan_footer_sent(p);
    EXPECT_FALSE(rc::download_plan_active(p));
    EXPECT_EQ(rc::download_plan_begin(p, 5U, kFrame, kT0, false),
              DownloadBegin::kOk);
}

TEST(DownloadPlan, StallOnlyAfterTimeoutWithNoBytes) {
    const std::vector<uint8_t> img = flight_image(50U);
    DownloadPlan p = in_payload(50U);
    EXPECT_FALSE(rc::download_plan_stalled(p, kT0 + kStallUs - 1U, kStallUs));
    EXPECT_TRUE(rc::download_plan_stalled(p, kT0 + kStallUs, kStallUs));
    const DownloadChunk c = rc::download_plan_chunk(p, 100U, 100U);
    rc::download_plan_sent(p, &img[c.offset], c.len, kT0 + kStallUs);
    EXPECT_FALSE(rc::download_plan_stalled(p, kT0 + kStallUs + 10U, kStallUs));
}

TEST(DownloadPlan, StallTimerSurvivesClockWrap) {
    DownloadPlan p{};
    ASSERT_EQ(rc::download_plan_begin(p, 10U, kFrame, 0xFFFFFF00U, false),
              DownloadBegin::kOk);
    EXPECT_FALSE(rc::download_plan_stalled(p, 0x00000100U, kStallUs));
}

TEST(DownloadPlan, FailEndsTheJobWithTheReason) {
    DownloadPlan p = in_payload(50U);
    rc::download_plan_fail(p, DownloadEnd::kStalled);
    EXPECT_FALSE(rc::download_plan_active(p));
    EXPECT_EQ(p.end, DownloadEnd::kStalled);
    EXPECT_FALSE(rc::download_plan_stalled(p, kT0 + 10U * kStallUs, kStallUs));
    EXPECT_EQ(rc::download_plan_chunk(p, 1024U, 512U).len, 0U);
}

