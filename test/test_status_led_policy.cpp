// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Rocket Chip Project
// Host tests for status LED backend selection (board-pack constants).

#include <gtest/gtest.h>
#include "rocketchip/status_led_policy.h"

using rc::StatusLedKind;
using rc::status_led_kind;

TEST(StatusLedPolicy, Ws2812WhenCountPositive) {
    EXPECT_EQ(status_led_kind(1, false), StatusLedKind::kWs2812);
    EXPECT_EQ(status_led_kind(5, false), StatusLedKind::kWs2812);
}

TEST(StatusLedPolicy, Ws2812WinsIfBothAdvertised) {
    EXPECT_EQ(status_led_kind(1, true), StatusLedKind::kWs2812);
}

TEST(StatusLedPolicy, DiscreteRgbWhenNoNeoPixel) {
    EXPECT_EQ(status_led_kind(0, true), StatusLedKind::kDiscreteRgb);
}

TEST(StatusLedPolicy, NoneWhenNeither) {
    EXPECT_EQ(status_led_kind(0, false), StatusLedKind::kNone);
}

// Feather pack values (board_feather_rp2350.h).
TEST(StatusLedPolicy, FeatherLikeWs2812) {
    EXPECT_EQ(status_led_kind(1, false), StatusLedKind::kWs2812);
}

// Tiny 2350 pack values (board_tiny_2350.h).
TEST(StatusLedPolicy, TinyLikeDiscreteRgb) {
    EXPECT_EQ(status_led_kind(0, true), StatusLedKind::kDiscreteRgb);
}

// Pico 2 pack values: no NeoPixel, no RGB.
TEST(StatusLedPolicy, Pico2LikeNone) {
    EXPECT_EQ(status_led_kind(0, false), StatusLedKind::kNone);
}
