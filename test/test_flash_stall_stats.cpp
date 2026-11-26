// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Flash stall histogram, poll-gap counters, and JEDEC decode.

#include <gtest/gtest.h>

#include "diag/flash_stall_stats.h"

#include <cstring>

namespace {

using rc::FlashOpType;
using rc::FlashStallStats;
using rc::PollGapStats;

TEST(FlashStallStats, BinEdgesAreLowInclusive) {
    EXPECT_EQ(rc::flash_stall_bin(0U), 0U);
    EXPECT_EQ(rc::flash_stall_bin(999U), 0U);
    EXPECT_EQ(rc::flash_stall_bin(1000U), 1U);
    EXPECT_EQ(rc::flash_stall_bin(2999U), 1U);
    EXPECT_EQ(rc::flash_stall_bin(3000U), 2U);
    EXPECT_EQ(rc::flash_stall_bin(9999U), 2U);
    EXPECT_EQ(rc::flash_stall_bin(10000U), 3U);
    EXPECT_EQ(rc::flash_stall_bin(49999U), 3U);
    EXPECT_EQ(rc::flash_stall_bin(50000U), 4U);
    EXPECT_EQ(rc::flash_stall_bin(399999U), 4U);
    EXPECT_EQ(rc::flash_stall_bin(400000U), 5U);
    EXPECT_EQ(rc::flash_stall_bin(UINT32_MAX), 5U);
}

TEST(FlashStallStats, NoteKeepsTypesSeparate) {
    FlashStallStats s{};
    rc::flash_stall_note(s, FlashOpType::kErase, 45000U, true);
    rc::flash_stall_note(s, FlashOpType::kErase, 420000U, false);
    rc::flash_stall_note(s, FlashOpType::kProgram, 400U, true);

    const auto& erase = s.op[static_cast<size_t>(FlashOpType::kErase)];
    EXPECT_EQ(erase.count, 2U);
    EXPECT_EQ(erase.fail, 1U);
    EXPECT_EQ(erase.max_us, 420000U);
    EXPECT_EQ(erase.total_us, 465000U);
    EXPECT_EQ(erase.bins[3], 1U);
    EXPECT_EQ(erase.bins[5], 1U);

    const auto& prog = s.op[static_cast<size_t>(FlashOpType::kProgram)];
    EXPECT_EQ(prog.count, 1U);
    EXPECT_EQ(prog.fail, 0U);
    EXPECT_EQ(prog.max_us, 400U);
    EXPECT_EQ(prog.bins[0], 1U);

    EXPECT_EQ(s.op[static_cast<size_t>(FlashOpType::kCommand)].count, 0U);
}

TEST(FlashStallStats, OutOfRangeTypeIsIgnored) {
    FlashStallStats s{};
    rc::flash_stall_note(s, FlashOpType::kCount, 5000U, true);
    FlashStallStats zero{};
    EXPECT_EQ(std::memcmp(&s, &zero, sizeof(s)), 0);
}

TEST(FlashStallStats, TotalDoesNotWrapAt32Bits) {
    FlashStallStats s{};
    for (int i = 0; i < 3; ++i) {
        rc::flash_stall_note(s, FlashOpType::kErase, UINT32_MAX, true);
    }
    EXPECT_EQ(s.op[0].total_us, 3ULL * UINT32_MAX);
}

TEST(FlashStallStats, SpanClampsAndRejectsBackwards) {
    EXPECT_EQ(rc::flash_stall_span_us(100U, 350U), 250U);
    EXPECT_EQ(rc::flash_stall_span_us(350U, 100U), 0U);
    EXPECT_EQ(rc::flash_stall_span_us(0U, 0x1'0000'0000ULL), UINT32_MAX);
}

TEST(FlashStallStats, TypeNames) {
    EXPECT_STREQ(rc::flash_op_type_name(FlashOpType::kErase), "erase");
    EXPECT_STREQ(rc::flash_op_type_name(FlashOpType::kProgram), "program");
    EXPECT_STREQ(rc::flash_op_type_name(FlashOpType::kCommand), "cmd");
    EXPECT_STREQ(rc::flash_op_type_name(FlashOpType::kCount), "?");
}

TEST(PollGapStats, FirstCallSetsStartOnly) {
    PollGapStats g{};
    rc::poll_gap_note(g, 5'000'000U);
    EXPECT_EQ(g.calls, 1U);
    EXPECT_EQ(g.max_gap_us, 0U);
    EXPECT_EQ(g.over_warn, 0U);
    EXPECT_EQ(g.over_long, 0U);
}

TEST(PollGapStats, ThresholdsAreStrict) {
    PollGapStats g{};
    uint64_t t = 1000U;
    rc::poll_gap_note(g, t);
    t += rc::kPollGapWarnUs;  // Exactly 10 ms: not counted
    rc::poll_gap_note(g, t);
    EXPECT_EQ(g.over_warn, 0U);
    t += rc::kPollGapWarnUs + 1U;
    rc::poll_gap_note(g, t);
    EXPECT_EQ(g.over_warn, 1U);
    EXPECT_EQ(g.over_long, 0U);
    t += rc::kPollGapLongUs;  // Exactly 100 ms: warn only
    rc::poll_gap_note(g, t);
    EXPECT_EQ(g.over_warn, 2U);
    EXPECT_EQ(g.over_long, 0U);
    t += rc::kPollGapLongUs + 1U;
    rc::poll_gap_note(g, t);
    EXPECT_EQ(g.over_warn, 3U);
    EXPECT_EQ(g.over_long, 1U);
    EXPECT_EQ(g.max_gap_us, rc::kPollGapLongUs + 1U);
    EXPECT_EQ(g.calls, 5U);
}

TEST(PollGapStats, BackwardsTimeIsZeroGap) {
    PollGapStats g{};
    rc::poll_gap_note(g, 2000U);
    rc::poll_gap_note(g, 1000U);
    EXPECT_EQ(g.max_gap_us, 0U);
    rc::poll_gap_note(g, 1500U);
    EXPECT_EQ(g.max_gap_us, 500U);
}

TEST(FlashPart, KnownIds) {
    const rc::FlashPart* p = rc::flash_part_from_jedec(0xEFU, 0x40U, 0x17U);
    ASSERT_NE(p, nullptr);
    EXPECT_STREQ(p->name, "W25Q64JV");
    EXPECT_EQ(p->prog_typ_us, 400U);
    EXPECT_EQ(p->prog_max_us, 3000U);
    EXPECT_EQ(p->erase_typ_us, 45000U);
    EXPECT_EQ(p->erase_max_us, 400000U);

    p = rc::flash_part_from_jedec(0xEFU, 0x40U, 0x18U);
    ASSERT_NE(p, nullptr);
    EXPECT_STREQ(p->name, "W25Q128JV");

    p = rc::flash_part_from_jedec(0xC8U, 0x40U, 0x17U);
    ASSERT_NE(p, nullptr);
    EXPECT_STREQ(p->name, "GD25Q64C");
    EXPECT_EQ(p->prog_typ_us, 600U);
    EXPECT_EQ(p->prog_max_us, 4000U);
    EXPECT_EQ(p->erase_typ_us, 50000U);
    EXPECT_EQ(p->erase_max_us, 400000U);

    p = rc::flash_part_from_jedec(0xEFU, 0x40U, 0x16U);
    ASSERT_NE(p, nullptr);
    EXPECT_STREQ(p->name, "W25Q32JV");
}

TEST(FlashPart, UnknownIdIsNull) {
    EXPECT_EQ(rc::flash_part_from_jedec(0xC8U, 0x40U, 0x18U), nullptr);
    EXPECT_EQ(rc::flash_part_from_jedec(0xFFU, 0xFFU, 0xFFU), nullptr);
    EXPECT_EQ(rc::flash_part_from_jedec(0x00U, 0x00U, 0x00U), nullptr);
}

} // namespace
