// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Host tests for rc_snprintf — mirrors rc_log spec coverage plus
// buffer-bound concerns (NUL termination, return value, edge cases).

#include <gtest/gtest.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <string>

#include "rocketchip/rc_log.h"

// Helper: format reference output via libc snprintf for byte-on-wire comparison.
static std::string libc_format(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n < 0) { return {}; }
    if (static_cast<size_t>(n) >= sizeof(buf)) { n = sizeof(buf) - 1; }
    return std::string(buf, static_cast<size_t>(n));
}

// ============================================================================
// Spec coverage: each of the format-spec patterns Unit A enumerated
// ============================================================================

TEST(RcSnprintf, StringPlain) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "hello %s\n", "world");
    EXPECT_EQ(std::string(buf, n), libc_format("hello %s\n", "world"));
}

TEST(RcSnprintf, StringWidthLeftAlign) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "[%-8s]", "hi");
    EXPECT_EQ(std::string(buf, n), libc_format("[%-8s]", "hi"));
}

TEST(RcSnprintf, StringWidthRightAlign) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "[%6s]", "hi");
    EXPECT_EQ(std::string(buf, n), libc_format("[%6s]", "hi"));
}

TEST(RcSnprintf, IntSigned) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "count=%d\n", 42);
    EXPECT_EQ(std::string(buf, n), libc_format("count=%d\n", 42));
}

TEST(RcSnprintf, IntNegative) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "delta=%d\n", -17);
    EXPECT_EQ(std::string(buf, n), libc_format("delta=%d\n", -17));
}

TEST(RcSnprintf, UnsignedInt) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "ticks=%u\n", 12345U);
    EXPECT_EQ(std::string(buf, n), libc_format("ticks=%u\n", 12345U));
}

TEST(RcSnprintf, UnsignedLong) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "ms=%lu\n", 987654321UL);
    EXPECT_EQ(std::string(buf, n), libc_format("ms=%lu\n", 987654321UL));
}

TEST(RcSnprintf, UnsignedLongWidth) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "[%6lu]", 42UL);
    EXPECT_EQ(std::string(buf, n), libc_format("[%6lu]", 42UL));
}

TEST(RcSnprintf, UnsignedLongLeftAlign) {
    // %-6lu used by rc_os_dashboard's "Pkts: %-6lu" row.
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "Pkts: %-6lu Lost: %-4lu", 1234UL, 5UL);
    EXPECT_EQ(std::string(buf, n), libc_format("Pkts: %-6lu Lost: %-4lu", 1234UL, 5UL));
}

TEST(RcSnprintf, CharSpec) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "key=%c\n", 'X');
    EXPECT_EQ(std::string(buf, n), libc_format("key=%c\n", 'X'));
}

TEST(RcSnprintf, LiteralPercent) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "%d%% complete\n", 75);
    EXPECT_EQ(std::string(buf, n), libc_format("%d%% complete\n", 75));
}

TEST(RcSnprintf, HexLowerZeroPad) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "crc=%02x\n", 0xABU);
    EXPECT_EQ(std::string(buf, n), libc_format("crc=%02x\n", 0xABU));
}

TEST(RcSnprintf, HexUpperZeroPad) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "status=%02X\n", 0xCDU);
    EXPECT_EQ(std::string(buf, n), libc_format("status=%02X\n", 0xCDU));
}

// ============================================================================
// Float formatting — verifies the same float pipeline rc_log uses
// ============================================================================

TEST(RcSnprintf, Float1Decimal) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "temp=%.1f\n", 23.45);
    EXPECT_EQ(std::string(buf, n), libc_format("temp=%.1f\n", 23.45));
}

TEST(RcSnprintf, Float2Decimal) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "vbat=%.2f\n", 3.78);
    EXPECT_EQ(std::string(buf, n), libc_format("vbat=%.2f\n", 3.78));
}

TEST(RcSnprintf, FloatWidthPrecision) {
    // %7.1f used by rc_os_dashboard's "Alt: %7.1f m" row.
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "Alt:  %7.1f m", -2.2);
    EXPECT_EQ(std::string(buf, n), libc_format("Alt:  %7.1f m", -2.2));
}

TEST(RcSnprintf, FloatForceSign) {
    // %+6.1f used by rc_os_dashboard's "Vvel: %+6.1f m/s" row.
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "Vvel: %+6.1f m/s", 0.0);
    EXPECT_EQ(std::string(buf, n), libc_format("Vvel: %+6.1f m/s", 0.0));
}

TEST(RcSnprintf, FloatHighPrecision) {
    // %.7f used by rc_os_dashboard's "Lat: %.7f" / "Lon: %.7f" rows.
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "Lat: %.7f", 30.1724500);
    EXPECT_EQ(std::string(buf, n), libc_format("Lat: %.7f", 30.1724500));
}

TEST(RcSnprintf, FloatZeroDecimal) {
    // %.0f used by ao_flight_director's "[PIO] Backup timers armed: drogue=%.0fs".
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "drogue=%.0fs main=%.0fs", 15.0, 45.0);
    EXPECT_EQ(std::string(buf, n), libc_format("drogue=%.0fs main=%.0fs", 15.0, 45.0));
}

// ============================================================================
// Buffer-bound behavior — distinct from rc_log
// ============================================================================

TEST(RcSnprintf, NulTerminatesOnSuccess) {
    char buf[64];
    memset(buf, 0xFF, sizeof(buf));  // poison
    rc::rc_snprintf(buf, sizeof(buf), "hi");
    EXPECT_EQ(buf[2], '\0');  // NUL at the right offset
}

TEST(RcSnprintf, NulTerminatesOnTruncation) {
    char buf[8];
    memset(buf, 0xFF, sizeof(buf));  // poison
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "hello world this is too long");
    // buf[n] must be NUL
    EXPECT_EQ(buf[n], '\0');
    EXPECT_LT(n, sizeof(buf));  // written less than buf size
}

TEST(RcSnprintf, ReturnsBytesWritten) {
    char buf[64];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "abc%d", 42);
    EXPECT_EQ(n, 5U);  // "abc42" = 5 chars
}

TEST(RcSnprintf, ZeroSizeIsSafeNoOp) {
    char buf[1] = {'X'};
    size_t n = rc::rc_snprintf(buf, 0U, "anything");
    EXPECT_EQ(n, 0U);
    EXPECT_EQ(buf[0], 'X');  // untouched
}

TEST(RcSnprintf, NullBufIsSafeNoOp) {
    size_t n = rc::rc_snprintf(nullptr, 64U, "anything");
    EXPECT_EQ(n, 0U);
}

TEST(RcSnprintf, OneByteBufWritesEmptyString) {
    // n=1 means we have room only for the NUL.
    char buf[1] = {'X'};
    size_t n = rc::rc_snprintf(buf, 1U, "hi");
    EXPECT_EQ(n, 0U);
    EXPECT_EQ(buf[0], '\0');
}

TEST(RcSnprintf, MultipleSpecsInOneCall) {
    // Dashboard-like multi-spec composition.
    char buf[128];
    size_t n = rc::rc_snprintf(buf, sizeof(buf),
        "Radio: BW%u %uHz SF%u CR%u  |  Vehicle: BW%u",
        125U, 5U, 7U, 5U, 125U);
    EXPECT_EQ(std::string(buf, n),
              libc_format("Radio: BW%u %uHz SF%u CR%u  |  Vehicle: BW%u",
                          125U, 5U, 7U, 5U, 125U));
}

TEST(RcSnprintf, EmptyFormatString) {
    char buf[16] = {'X','Y','Z'};
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "");
    EXPECT_EQ(n, 0U);
    EXPECT_EQ(buf[0], '\0');
}

TEST(RcSnprintf, LiteralOnly) {
    char buf[16];
    size_t n = rc::rc_snprintf(buf, sizeof(buf), "literal text");
    EXPECT_EQ(std::string(buf, n), "literal text");
}
