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

// Onboard LED vs RGB channel. Tiny 2350: kLedPin is green GP19, RGB is
// R=18 G=19 B=20, so the onboard LED is not a separate GPIO LED.
TEST(StatusLedPolicy, TinyOnboardLedIsRgbGreenChannel) {
    EXPECT_TRUE(rc::onboard_led_is_rgb_channel(true, 19, 18, 19, 20));
}

// Feather: kLedPin 7, no RGB LED (pack pins 0) -> a real GPIO LED.
TEST(StatusLedPolicy, FeatherOnboardLedIsGpio) {
    EXPECT_FALSE(rc::onboard_led_is_rgb_channel(false, 7, 0, 0, 0));
}

TEST(StatusLedPolicy, RgbPackWithSeparateOnboardLedKeepsGpio) {
    EXPECT_FALSE(rc::onboard_led_is_rgb_channel(true, 25, 18, 19, 20));
}

TEST(StatusLedPolicy, BootCheckCountsOnlyAFittedStatusLed) {
    EXPECT_TRUE(rc::status_led_counted(StatusLedKind::kWs2812));
    EXPECT_TRUE(rc::status_led_counted(StatusLedKind::kDiscreteRgb));
    EXPECT_FALSE(rc::status_led_counted(StatusLedKind::kNone));
}

TEST(StatusLedPolicy, BootCheckLabelNamesWhatIsFitted) {
    EXPECT_STREQ(rc::status_led_label(StatusLedKind::kWs2812), "NeoPixel");
    EXPECT_STREQ(rc::status_led_label(StatusLedKind::kDiscreteRgb), "RGB status LED");
    EXPECT_STREQ(rc::status_led_label(StatusLedKind::kNone), "none");
}
