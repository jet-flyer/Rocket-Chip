// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Pad RX Hz window. 10 Hz is one CRC-ok count each 100 ms.

#include <gtest/gtest.h>

#include "diag/radio_rate_counters.h"

namespace {

uint32_t feed(uint32_t t_ms, uint32_t count) {
    g_radioRateCounters.rx_crc_ok_n = count;
    return radio_rate_rx_window_hz10(t_ms);
}

void expect_about_10(uint32_t hz10, uint32_t t_ms) {
    EXPECT_NE(hz10, kRadioRxWindowBlankHz10) << t_ms;
    EXPECT_GE(hz10, 95U) << t_ms;
    EXPECT_LE(hz10, 105U) << t_ms;
}

class RxWindowTest : public ::testing::Test {
protected:
    void SetUp() override {
        g_radioRateCounters = {};
        radio_rate_rx_window_reset(0U);
    }
};

}  // namespace

TEST_F(RxWindowTest, Steady10HzAcrossEdges) {
    for (uint32_t t = 0; t <= 4000U; t += 100U) {
        const uint32_t hz10 = feed(t, t / 100U);
        if (t < 1000U) {
            EXPECT_EQ(hz10, kRadioRxWindowBlankHz10) << t;
            continue;
        }
        expect_about_10(hz10, t);
        if (t == 2000U || t == 4000U) {
            const uint32_t across = feed(t + 1U, t / 100U);
            expect_about_10(across, t + 1U);
        }
    }
}

TEST_F(RxWindowTest, StopDropsToZeroWithin2s) {
    for (uint32_t t = 0; t <= 3000U; t += 100U) {
        feed(t, t / 100U);
    }
    const uint32_t early = feed(3100U, 30U);
    expect_about_10(early, 3100U);

    const uint32_t mid = feed(4000U, 30U);
    EXPECT_NE(mid, kRadioRxWindowBlankHz10);
    EXPECT_GE(mid, 45U);
    EXPECT_LE(mid, 55U);

    EXPECT_EQ(feed(5000U, 30U), 0U);
    EXPECT_EQ(feed(5100U, 30U), 0U);
}

TEST_F(RxWindowTest, ResumeRecoversWithin2s) {
    for (uint32_t t = 0; t <= 3000U; t += 100U) {
        feed(t, t / 100U);
    }
    EXPECT_EQ(feed(5000U, 30U), 0U);

    uint32_t count = 30U;
    for (uint32_t t = 5100U; t <= 7000U; t += 100U) {
        count += 1U;
        const uint32_t hz10 = feed(t, count);
        EXPECT_NE(hz10, kRadioRxWindowBlankHz10) << t;
        if (t == 7000U) {
            expect_about_10(hz10, t);
        }
    }
    expect_about_10(feed(7001U, count), 7001U);
}

TEST_F(RxWindowTest, NavPeriodResetStartsANewSpan) {
    for (uint32_t t = 0; t <= 2500U; t += 100U) {
        feed(t, t / 100U);
    }
    expect_about_10(feed(2500U, 25U), 2500U);

    g_radioRateCounters.rx_crc_ok_n = 25U;
    radio_rate_rx_window_reset(2500U);
    EXPECT_EQ(feed(2500U, 25U), kRadioRxWindowBlankHz10);
    EXPECT_EQ(feed(3400U, 34U), kRadioRxWindowBlankHz10);
    expect_about_10(feed(3500U, 35U), 3500U);
}
