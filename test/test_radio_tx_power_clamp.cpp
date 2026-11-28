// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Host plan of radio HW TX power. No SPI.
// 2 dBm bytes match the pre-clamp PA_BOOST normal write.

#include <gtest/gtest.h>

#include "drivers/rfm95w.h"
#include "rocketchip/radio_config_table.h"

TEST(RadioTxPowerClamp, CompileDefaultCapIs17) {
    EXPECT_FALSE(kRfm95Allow20DbmDefault);
    EXPECT_EQ(kRfm95TxPowerContinuousMaxDbm, 17);
    EXPECT_EQ(kRfm95TxPowerPeakMaxDbm, 20);
}

TEST(RadioTxPowerClamp, Request20WithoutOverrideClampsTo17) {
    const rfm95w_tx_power_prog p = rfm95w_plan_tx_power(20, false);
    EXPECT_TRUE(p.clamped);
    EXPECT_EQ(p.requested_dbm, 20);
    EXPECT_EQ(p.applied_dbm, 17);
    EXPECT_EQ(p.pa_dac, 0x84);
    EXPECT_EQ(p.pa_config, 0xFF);
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(p.pa_config, p.pa_dac), 17);
}

TEST(RadioTxPowerClamp, RequestAbove20WithoutOverrideClampsTo17) {
    const rfm95w_tx_power_prog p = rfm95w_plan_tx_power(21, false);
    EXPECT_TRUE(p.clamped);
    EXPECT_EQ(p.applied_dbm, 17);
    EXPECT_EQ(p.pa_dac, 0x84);
    EXPECT_EQ(p.pa_config, 0xFF);
}

TEST(RadioTxPowerClamp, Request18WithoutOverrideClampsTo17) {
    const rfm95w_tx_power_prog p = rfm95w_plan_tx_power(18, false);
    EXPECT_TRUE(p.clamped);
    EXPECT_EQ(p.applied_dbm, 17);
    EXPECT_EQ(p.pa_dac, 0x84);
}

TEST(RadioTxPowerClamp, Request20WithOverrideWritesHighPower) {
    const rfm95w_tx_power_prog p = rfm95w_plan_tx_power(20, true);
    EXPECT_FALSE(p.clamped);
    EXPECT_EQ(p.applied_dbm, 20);
    EXPECT_EQ(p.pa_dac, 0x87);
    EXPECT_EQ(p.pa_config, 0xFF);
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(p.pa_config, p.pa_dac), 20);
}

TEST(RadioTxPowerClamp, RequestAbove20WithOverrideClampsTo20) {
    const rfm95w_tx_power_prog p = rfm95w_plan_tx_power(25, true);
    EXPECT_TRUE(p.clamped);
    EXPECT_EQ(p.requested_dbm, 25);
    EXPECT_EQ(p.applied_dbm, 20);
    EXPECT_EQ(p.pa_dac, 0x87);
    EXPECT_EQ(p.pa_config, 0xFF);
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(p.pa_config, p.pa_dac), 20);
}

TEST(RadioTxPowerClamp, TwoDbmPassthroughIsNormalPa) {
    const rfm95w_tx_power_prog off = rfm95w_plan_tx_power(2, false);
    const rfm95w_tx_power_prog on = rfm95w_plan_tx_power(2, true);
    EXPECT_FALSE(off.clamped);
    EXPECT_FALSE(on.clamped);
    EXPECT_EQ(off.applied_dbm, 2);
    EXPECT_EQ(on.applied_dbm, 2);
    EXPECT_EQ(off.pa_dac, 0x84);
    EXPECT_EQ(off.pa_config, 0xF0);
    EXPECT_EQ(on.pa_dac, off.pa_dac);
    EXPECT_EQ(on.pa_config, off.pa_config);
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(off.pa_config, off.pa_dac), 2);
}

TEST(RadioTxPowerClamp, ContinuousValuesStayOnNormalPa) {
    const rfm95w_tx_power_prog p15 = rfm95w_plan_tx_power(15, false);
    const rfm95w_tx_power_prog p17 = rfm95w_plan_tx_power(17, true);
    EXPECT_FALSE(p15.clamped);
    EXPECT_EQ(p15.applied_dbm, 15);
    EXPECT_EQ(p15.pa_dac, 0x84);
    EXPECT_EQ(p15.pa_config, 0xFD);
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(p15.pa_config, p15.pa_dac), 15);
    EXPECT_FALSE(p17.clamped);
    EXPECT_EQ(p17.applied_dbm, 17);
    EXPECT_EQ(p17.pa_dac, 0x84);
    EXPECT_EQ(p17.pa_config, 0xFF);
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(p17.pa_config, p17.pa_dac), 17);
}

TEST(RadioTxPowerClamp, BelowTwoIsRaisedQuietly) {
    const rfm95w_tx_power_prog zero = rfm95w_plan_tx_power(0, false);
    const rfm95w_tx_power_prog neg = rfm95w_plan_tx_power(-1, true);
    EXPECT_FALSE(zero.clamped);
    EXPECT_FALSE(neg.clamped);
    EXPECT_EQ(zero.applied_dbm, 2);
    EXPECT_EQ(neg.applied_dbm, 2);
    EXPECT_EQ(neg.pa_dac, 0x84);
    EXPECT_EQ(neg.pa_config, 0xF0);
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(neg.pa_config, neg.pa_dac), 2);
}

TEST(RadioTxPowerClamp, EighteenWithOptInStepsTo17) {
    const rfm95w_tx_power_prog p18 = rfm95w_plan_tx_power(18, true);
    const rfm95w_tx_power_prog p19 = rfm95w_plan_tx_power(19, true);
    EXPECT_TRUE(p18.clamped);
    EXPECT_TRUE(p19.clamped);
    EXPECT_EQ(p18.applied_dbm, 17);
    EXPECT_EQ(p19.applied_dbm, 17);
    EXPECT_EQ(p18.pa_dac, 0x84);
    EXPECT_EQ(p19.pa_dac, 0x84);
    EXPECT_EQ(p18.pa_config, 0xFF);
    EXPECT_EQ(rfm95w_decode_tx_power_dbm(p18.pa_config, p18.pa_dac), 17);
}

TEST(RadioTxPowerClamp, AppliedValueReplacesRequested) {
    uint8_t stored = 20;
    const int8_t applied = rfm95w_plan_tx_power(
        static_cast<int8_t>(stored), false).applied_dbm;
    stored = static_cast<uint8_t>(applied);
    EXPECT_EQ(stored, 17);
    const int8_t peak = rfm95w_plan_tx_power(20, true).applied_dbm;
    EXPECT_EQ(peak, 20);
}

TEST(RadioTxPowerClamp, RejectLeavesStoredPower) {
    uint8_t stored = 5;
    const rc::RadioPowerReject why18 =
        rc::radio_power_reject(18, true);
    if (why18 == rc::RadioPowerReject::kOk) {
        stored = 18;
    }
    EXPECT_EQ(why18, rc::RadioPowerReject::kNoRegisterCode);
    EXPECT_EQ(static_cast<uint8_t>(why18), 2);
    EXPECT_EQ(stored, 5);

    const rc::RadioPowerReject why19 = rc::radio_power_reject(19, true);
    EXPECT_EQ(why19, rc::RadioPowerReject::kNoRegisterCode);
    EXPECT_EQ(stored, 5);

    const rc::RadioPowerReject why20 = rc::radio_power_reject(20, false);
    if (why20 == rc::RadioPowerReject::kOk) {
        stored = 20;
    }
    EXPECT_EQ(why20, rc::RadioPowerReject::kOptInRequired);
    EXPECT_EQ(static_cast<uint8_t>(why20), 3);
    EXPECT_EQ(stored, 5);

    const rc::RadioPowerReject low = rc::radio_power_reject(0, true);
    EXPECT_EQ(low, rc::RadioPowerReject::kOutOfRange);
    EXPECT_EQ(static_cast<uint8_t>(low), 1);
    EXPECT_EQ(stored, 5);

    const rc::RadioPowerReject ok = rc::radio_power_reject(17, false);
    if (ok == rc::RadioPowerReject::kOk) {
        stored = 17;
    }
    EXPECT_EQ(stored, 17);
    EXPECT_EQ(rc::radio_power_reject(20, true), rc::RadioPowerReject::kOk);
}
