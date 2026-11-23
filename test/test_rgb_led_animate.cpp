// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Rocket Chip Project
// Host tests for the discrete-RGB status LED animation (same timings
// as the WS2812 driver).

#include <gtest/gtest.h>
#include "rocketchip/rgb_led_animate.h"

using rc::Rgb8;
using rc::RgbAnimMode;
using rc::rgb_led_color;
using rc::rgb_hsv_to_rgb;

static const Rgb8 kRed{0x40, 0x00, 0x00};
static const Rgb8 kWhite{0x40, 0x40, 0x40};
static const Rgb8 kOff{0, 0, 0};

static void ExpectRgbEq(Rgb8 a, Rgb8 b) {
    EXPECT_EQ(a.r, b.r);
    EXPECT_EQ(a.g, b.g);
    EXPECT_EQ(a.b, b.b);
}

TEST(RgbLedAnimate, OffIsBlack) {
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kOff, kRed, kWhite, 0), kOff);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kOff, kRed, kWhite, 1234), kOff);
}

TEST(RgbLedAnimate, SolidIsBase) {
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kSolid, kRed, kWhite, 0), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kSolid, kRed, kWhite, 9999), kRed);
}

TEST(RgbLedAnimate, BlinkOnOffAtCanonicalTimings) {
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlink, kRed, kWhite, 0), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlink, kRed, kWhite, 499), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlink, kRed, kWhite, 500), kOff);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlink, kRed, kWhite, 999), kOff);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlink, kRed, kWhite, 1000), kRed);
}

TEST(RgbLedAnimate, BlinkFastIs100Ms) {
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlinkFast, kRed, kWhite, 0), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlinkFast, kRed, kWhite, 99), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlinkFast, kRed, kWhite, 100), kOff);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kBlinkFast, kRed, kWhite, 199), kOff);
}

TEST(RgbLedAnimate, AlternateHalfPeriod) {
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 0), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 249), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 250), kWhite);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 499), kWhite);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 500), kRed);
}

TEST(RgbLedAnimate, AlternateHonorsCustomHalfPeriod) {
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 99, 100), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 100, 100), kWhite);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 200, 100), kRed);
}

TEST(RgbLedAnimate, AlternateZeroHalfPeriodUsesDefault) {
    // Same 0 -> 250 ms remap as ws2812_set_mode_alternate; no modulo by zero.
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 249, 0), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kAlternate, kRed, kWhite, 250, 0), kWhite);
}

TEST(RgbLedAnimate, DoubleFlashShape) {
    // 100 on, 100 off, 100 on, 700 off.
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 0), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 99), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 100), kOff);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 199), kOff);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 200), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 299), kRed);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 300), kOff);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 999), kOff);
    ExpectRgbEq(rgb_led_color(RgbAnimMode::kDoubleFlash, kRed, kWhite, 1000), kRed);
}

TEST(RgbLedAnimate, BreatheVariesAndStaysInRange) {
    const Rgb8 a = rgb_led_color(RgbAnimMode::kBreathe, kRed, kWhite, 0);
    const Rgb8 b = rgb_led_color(RgbAnimMode::kBreathe, kRed, kWhite, 500);
    EXPECT_LE(a.r, 0x40);
    EXPECT_LE(b.r, 0x40);
    EXPECT_EQ(a.g, 0);
    EXPECT_EQ(a.b, 0);
    // A quarter-period later the sine is not at the same point.
    EXPECT_NE(a.r, b.r);
}

TEST(RgbLedAnimate, RainbowChangesHue) {
    const Rgb8 a = rgb_led_color(RgbAnimMode::kRainbow, kRed, kWhite, 0);
    const Rgb8 b = rgb_led_color(RgbAnimMode::kRainbow, kRed, kWhite, 1000);
    EXPECT_TRUE(a.r != b.r || a.g != b.g || a.b != b.b);
}

TEST(RgbLedAnimate, HsvRedAtZeroHue) {
    const Rgb8 c = rgb_hsv_to_rgb(0.0F, 100.0F, 25.0F);
    EXPECT_GT(c.r, c.g);
    EXPECT_GT(c.r, c.b);
}

// Tiny 2350 pack (board_tiny_2350.h; SDK TINY2350_LED_R/G/B_PIN):
// R=GP18, G=GP19, B=GP20, all active low.
static constexpr rc::RgbLedPins kTinyPins{18, 19, 20};

TEST(RgbPwmLevels, ColorChannelsMapToRgbPinsInOrder) {
    const rc::RgbPwmLevels lv =
        rc::rgb_pwm_levels(Rgb8{0x11, 0x22, 0x33}, kTinyPins, false);
    EXPECT_EQ(lv.r.pin, 18);
    EXPECT_EQ(lv.g.pin, 19);
    EXPECT_EQ(lv.b.pin, 20);
    EXPECT_EQ(lv.r.level, 0x11);
    EXPECT_EQ(lv.g.level, 0x22);
    EXPECT_EQ(lv.b.level, 0x33);
}

TEST(RgbPwmLevels, ActiveLowRedLightsOnlyGp18) {
    // Red only: GP18 low most of the period (lit), GP19/GP20 held high (off).
    const rc::RgbPwmLevels lv =
        rc::rgb_pwm_levels(Rgb8{255, 0, 0}, kTinyPins, true);
    EXPECT_EQ(lv.r.pin, 18);
    EXPECT_EQ(lv.r.level, 0);
    EXPECT_EQ(lv.g.level, rc::kRgbPwmWrap);
    EXPECT_EQ(lv.b.level, rc::kRgbPwmWrap);
}

TEST(RgbPwmLevels, ActiveLowGreenLightsOnlyGp19) {
    const rc::RgbPwmLevels lv =
        rc::rgb_pwm_levels(Rgb8{0, 255, 0}, kTinyPins, true);
    EXPECT_EQ(lv.g.pin, 19);
    EXPECT_EQ(lv.g.level, 0);
    EXPECT_EQ(lv.r.level, rc::kRgbPwmWrap);
    EXPECT_EQ(lv.b.level, rc::kRgbPwmWrap);
}

TEST(RgbPwmLevels, ActiveLowOffHoldsAllPinsHigh) {
    const rc::RgbPwmLevels lv = rc::rgb_pwm_levels(Rgb8{0, 0, 0}, kTinyPins, true);
    EXPECT_EQ(lv.r.level, rc::kRgbPwmWrap);
    EXPECT_EQ(lv.g.level, rc::kRgbPwmWrap);
    EXPECT_EQ(lv.b.level, rc::kRgbPwmWrap);
}

TEST(RgbPwmLevels, ActiveLowInvertsMidDuty) {
    EXPECT_EQ(rc::rgb_pwm_level(0x40, true), rc::kRgbPwmWrap - 0x40);
    EXPECT_EQ(rc::rgb_pwm_level(0x40, false), 0x40);
}
