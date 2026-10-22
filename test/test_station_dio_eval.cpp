// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include <gtest/gtest.h>

#include "diag/station_dio_eval.h"

using rc::dio_bench::DioSample;
using rc::dio_bench::eval_dio_toggle;
using rc::dio_bench::eval_fifo_depth;
using rc::dio_bench::kIrqFifoEmpty;
using rc::dio_bench::kIrqFifoFull;
using rc::dio_bench::kIrqFifoLevel;
using rc::dio_bench::kIrqFifoOverrun;

TEST(DioEval, TogglePassWhenPinsMatchAndBothEdges) {
    const DioSample s[] = {
        {0, false, false, kIrqFifoEmpty},
        {1, false, false, 0x00},
        {5, true, false, kIrqFifoLevel},
        {66, true, true, static_cast<uint8_t>(kIrqFifoFull | kIrqFifoLevel)},
    };
    const auto v = eval_dio_toggle(s, 4);
    EXPECT_TRUE(v.pass);
    EXPECT_FALSE(v.mismatch);
}

TEST(DioEval, ToggleFailFirstMismatchIsDio1) {
    const DioSample s[] = {
        {0, false, false, kIrqFifoEmpty},
        {5, false, false, kIrqFifoLevel},
    };
    const auto v = eval_dio_toggle(s, 2);
    EXPECT_FALSE(v.pass);
    EXPECT_TRUE(v.mismatch);
    EXPECT_EQ(v.mismatch_n, 5);
    EXPECT_TRUE(v.mismatch_dio1);
    EXPECT_FALSE(v.gpio_seen);
    EXPECT_TRUE(v.flag_seen);
}

TEST(DioEval, ToggleFailIfDio2NeverHigh) {
    const DioSample s[] = {
        {0, false, false, kIrqFifoEmpty},
        {5, true, false, kIrqFifoLevel},
    };
    const auto v = eval_dio_toggle(s, 2);
    EXPECT_FALSE(v.pass);
    EXPECT_FALSE(v.mismatch);
    EXPECT_TRUE(v.saw_dio1_low);
    EXPECT_TRUE(v.saw_dio1_high);
    EXPECT_FALSE(v.saw_dio2_high);
}

TEST(DioEval, DepthReportsFullThenOverrun) {
    const DioSample s[] = {
        {0, false, false, kIrqFifoEmpty},
        {63, true, false, kIrqFifoLevel},
        {64, true, true, static_cast<uint8_t>(kIrqFifoFull | kIrqFifoLevel)},
        {65, true, true, static_cast<uint8_t>(kIrqFifoFull | kIrqFifoLevel
                                              | kIrqFifoOverrun)},
    };
    const auto d = eval_fifo_depth(s, 4);
    EXPECT_FALSE(d.fail);
    EXPECT_TRUE(d.full_seen);
    EXPECT_EQ(d.full_at, 64);
    EXPECT_TRUE(d.dio2_at_full);
    EXPECT_TRUE(d.overrun_seen);
    EXPECT_EQ(d.overrun_at, 65);
}

TEST(DioEval, DepthFailWhenFullNeverSets) {
    const DioSample s[] = {
        {1, false, false, kIrqFifoLevel},
        {10, false, false, kIrqFifoLevel},
    };
    const auto d = eval_fifo_depth(s, 2);
    EXPECT_TRUE(d.fail);
    EXPECT_FALSE(d.full_seen);
    EXPECT_FALSE(d.overrun_seen);
}

TEST(DioEval, DepthIgnoresBeforeSample) {
    const DioSample s[] = {
        {0, false, true, static_cast<uint8_t>(kIrqFifoFull | kIrqFifoOverrun)},
        {66, false, true, kIrqFifoFull},
    };
    const auto d = eval_fifo_depth(s, 2);
    EXPECT_FALSE(d.fail);
    EXPECT_EQ(d.full_at, 66);
    EXPECT_TRUE(d.dio2_at_full);
    EXPECT_FALSE(d.overrun_seen);
}
