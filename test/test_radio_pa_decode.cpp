// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Host decode of radio HW PaConfig and PaDac. No SPI.

#include <gtest/gtest.h>

#include "drivers/rfm95w.h"

TEST(RadioPaDecode, DefaultDacFullByteIs2Dbm) {
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(0xF0, 0x84), 2);
}

TEST(RadioPaDecode, DefaultDacMaskedFieldIs2Dbm) {
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(0xF0, 0x04), 2);
}

TEST(RadioPaDecode, Plus20EncodingIs20Dbm) {
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(0xFF, 0x87), 20);
}

TEST(RadioPaDecode, Plus20DacOtherOutputIsUnverified) {
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(0xFA, 0x87), -2);
}

TEST(RadioPaDecode, BoostClearIsUnread) {
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(0x70, 0x84), -1);
}
