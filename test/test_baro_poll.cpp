// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Rocket Chip Project
// Host tests for the barometer new-sample gate:
//   - dps310_poll_plan (drivers/dps310_poll.h): which DPS310 result
//     registers one poll reads from MEAS_CFG
//   - BaroSampleWatch (drivers/baro_sample.h): publish / skip / fault per
//     poll, so a no-new-sample poll is never a fault or a sample
//   - a simulated DPS310 (32 Hz pressure, 2 Hz temperature, ready bits that
//     clear on read) polled with jitter: every published sample is a new
//     conversion, which the old unconditional re-read did not guarantee

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "drivers/baro_sample.h"
#include "drivers/dps310_poll.h"

namespace {

constexpr uint32_t kGapUs = 1000000U;

// ---------------------------------------------------------------------------
// dps310_poll_plan
// ---------------------------------------------------------------------------

TEST(Dps310PollPlan, ReadyBitsMatchDatasheet) {
    // DPS310 datasheet §8.5 MEAS_CFG: TMP_RDY bit 5, PRS_RDY bit 4.
    EXPECT_EQ(kDps310MeasCfgTmpReady, 1U << 5);
    EXPECT_EQ(kDps310MeasCfgPrsReady, 1U << 4);
}

TEST(Dps310PollPlan, NothingReadyReadsNothing) {
    for (bool have_tmp : {false, true}) {
        const Dps310PollPlan p = dps310_poll_plan(0x00U, have_tmp);
        EXPECT_FALSE(p.read_tmp);
        EXPECT_FALSE(p.read_psr);
    }
}

TEST(Dps310PollPlan, PressureReadyWithCachedTemperature) {
    const Dps310PollPlan p = dps310_poll_plan(kDps310MeasCfgPrsReady, true);
    EXPECT_FALSE(p.read_tmp);
    EXPECT_TRUE(p.read_psr);
}

TEST(Dps310PollPlan, PressureReadyBeforeAnyTemperatureIsLeft) {
    const Dps310PollPlan p = dps310_poll_plan(kDps310MeasCfgPrsReady, false);
    EXPECT_FALSE(p.read_tmp);
    EXPECT_FALSE(p.read_psr);
}

TEST(Dps310PollPlan, TemperatureOnlyRefreshesCache) {
    for (bool have_tmp : {false, true}) {
        const Dps310PollPlan p = dps310_poll_plan(kDps310MeasCfgTmpReady, have_tmp);
        EXPECT_TRUE(p.read_tmp);
        EXPECT_FALSE(p.read_psr);
    }
}

TEST(Dps310PollPlan, BothReadyReadsBothEvenWithoutCache) {
    const uint8_t both = kDps310MeasCfgTmpReady | kDps310MeasCfgPrsReady;
    for (bool have_tmp : {false, true}) {
        const Dps310PollPlan p = dps310_poll_plan(both, have_tmp);
        EXPECT_TRUE(p.read_tmp);
        EXPECT_TRUE(p.read_psr);
    }
}

TEST(Dps310PollPlan, OtherMeasCfgBitsIgnored) {
    // COEF_RDY, SENSOR_RDY and the continuous-both mode bits.
    const Dps310PollPlan p = dps310_poll_plan(0xC7U, true);
    EXPECT_FALSE(p.read_tmp);
    EXPECT_FALSE(p.read_psr);
}

// ---------------------------------------------------------------------------
// BaroSampleWatch
// ---------------------------------------------------------------------------

TEST(BaroSampleWatch, NewSamplePublishes) {
    BaroSampleWatch w{kGapUs};
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNewSample, 1000U), BaroSampleAction::kPublish);
}

TEST(BaroSampleWatch, NoNewSampleWithinGapIsSkippedNotFault) {
    BaroSampleWatch w{kGapUs};
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNewSample, 0U), BaroSampleAction::kPublish);
    for (uint32_t t = 32000U; t <= kGapUs; t += 32000U) {
        EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, t), BaroSampleAction::kSkip) << t;
    }
}

TEST(BaroSampleWatch, FirstPollWithoutSampleIsSkipped) {
    BaroSampleWatch w{kGapUs};
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, 5000000U), BaroSampleAction::kSkip);
}

TEST(BaroSampleWatch, NoNewSamplePastGapIsFault) {
    BaroSampleWatch w{kGapUs};
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNewSample, 0U), BaroSampleAction::kPublish);
    uint32_t t = 0U;
    while (t + 32000U <= kGapUs) {
        t += 32000U;
        EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, t), BaroSampleAction::kSkip);
    }
    t += 32000U;
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, t), BaroSampleAction::kFault);
    // Stays a fault every poll until a sample arrives.
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, t + 32000U), BaroSampleAction::kFault);
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNewSample, t + 64000U), BaroSampleAction::kPublish);
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, t + 96000U), BaroSampleAction::kSkip);
}

TEST(BaroSampleWatch, BusErrorIsFault) {
    BaroSampleWatch w{kGapUs};
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kError, 100U), BaroSampleAction::kFault);
}

TEST(BaroSampleWatch, PausedPollingDoesNotCountAsGap) {
    // Core 1 held off the bus (flash erase) for 3 s: no polls, so the part
    // can't be blamed. The first poll after resume restarts the clock.
    BaroSampleWatch w{kGapUs};
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNewSample, 0U), BaroSampleAction::kPublish);
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, 3000000U), BaroSampleAction::kSkip);
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, 3032000U), BaroSampleAction::kSkip);
}

TEST(BaroSampleWatch, RestartGivesAFullGap) {
    BaroSampleWatch w{kGapUs};
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNewSample, 0U), BaroSampleAction::kPublish);
    uint32_t t = 0U;
    for (int i = 0; i < 40; ++i) {
        t += 32000U;
        (void)w.on_poll(BaroSampleStatus::kNoNewSample, t);
    }
    ASSERT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, t + 32000U), BaroSampleAction::kFault);
    t += 32000U;
    w.restart(t);
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, t + 500000U), BaroSampleAction::kSkip);
}

TEST(BaroSampleWatch, TimerWrapAround) {
    BaroSampleWatch w{kGapUs};
    const uint32_t before_wrap = 0xFFFFFFFFU - 10000U;
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNewSample, before_wrap), BaroSampleAction::kPublish);
    EXPECT_EQ(w.on_poll(BaroSampleStatus::kNoNewSample, 20000U), BaroSampleAction::kSkip);
}

// ---------------------------------------------------------------------------
// Simulated DPS310 in continuous mode
// ---------------------------------------------------------------------------

// Result registers and ready bits per datasheet §4.7 / §8.5 (FIFO off):
// a conversion overwrites the register and sets its bit; reading the
// register clears the bit but keeps the value.
struct FakeDps310 {
    uint32_t prs_period_us;
    uint32_t tmp_period_us;
    uint32_t prs_count = 0;   // conversions finished; register holds prs_count
    uint32_t tmp_count = 0;
    bool prs_rdy = false;
    bool tmp_rdy = false;

    void advance_to(uint32_t t_us) {
        const uint32_t prs = t_us / prs_period_us;
        const uint32_t tmp = t_us / tmp_period_us;
        if (prs != prs_count) { prs_count = prs; prs_rdy = true; }
        if (tmp != tmp_count) { tmp_count = tmp; tmp_rdy = true; }
    }
    uint8_t meas_cfg() const {
        return static_cast<uint8_t>((prs_rdy ? kDps310MeasCfgPrsReady : 0U) |
                                    (tmp_rdy ? kDps310MeasCfgTmpReady : 0U));
    }
    uint32_t read_psr() { prs_rdy = false; return prs_count; }
    uint32_t read_tmp() { tmp_rdy = false; return tmp_count; }
};

// Poll the fake the way baro_dps310_poll() + core1_read_baro() do. Returns
// the pressure conversion index of each published sample.
std::vector<uint32_t> run_gated(uint32_t duration_us, uint32_t poll_us, uint32_t jitter_us,
                                uint32_t* faults) {
    FakeDps310 dev{31250U, 500000U};
    BaroSampleWatch watch{kGapUs};
    bool have_tmp = false;
    std::vector<uint32_t> published;
    uint32_t lcg = 12345U;
    *faults = 0U;
    for (uint32_t t = 0U; t < duration_us;) {
        dev.advance_to(t);
        const Dps310PollPlan plan = dps310_poll_plan(dev.meas_cfg(), have_tmp);
        if (plan.read_tmp) { (void)dev.read_tmp(); have_tmp = true; }
        BaroSampleStatus st = BaroSampleStatus::kNoNewSample;
        uint32_t idx = 0U;
        if (plan.read_psr) { idx = dev.read_psr(); st = BaroSampleStatus::kNewSample; }
        switch (watch.on_poll(st, t)) {
            case BaroSampleAction::kPublish: published.push_back(idx); break;
            case BaroSampleAction::kFault: ++*faults; break;
            case BaroSampleAction::kSkip: break;
        }
        lcg = lcg * 1664525U + 1013904223U;
        t += poll_us - jitter_us + (lcg >> 8) % (2U * jitter_us + 1U);
    }
    return published;
}

TEST(BaroPollSimulation, GatedPollNeverPublishesARepeat) {
    // Core 1 polls every 32 loops of >= 1 ms: about the conversion rate,
    // with jitter, so polls regularly land before the next result.
    uint32_t faults = 0U;
    const std::vector<uint32_t> pub = run_gated(10000000U, 32000U, 3000U, &faults);
    ASSERT_GT(pub.size(), 250U);
    for (size_t i = 1; i < pub.size(); ++i) {
        ASSERT_GT(pub[i], pub[i - 1]) << "repeat or reorder at sample " << i;
    }
    EXPECT_EQ(faults, 0U) << "a no-new poll was treated as a fault";
}

TEST(BaroPollSimulation, FastPollingStillOnlyNewSamples) {
    uint32_t faults = 0U;
    const std::vector<uint32_t> pub = run_gated(5000000U, 5000U, 1000U, &faults);
    // First temperature at 0.5 s, then every 31.25 ms conversion is seen.
    ASSERT_GT(pub.size(), 130U);
    for (size_t i = 1; i < pub.size(); ++i) {
        ASSERT_EQ(pub[i], pub[i - 1] + 1U) << "missed or repeated conversion at " << i;
    }
    EXPECT_EQ(faults, 0U);
}

TEST(BaroPollSimulation, UngatedReReadRepeats) {
    // What the old unconditional burst read did: publish the register every
    // poll. Same schedule, so this shows the gate is what removes repeats.
    FakeDps310 dev{31250U, 500000U};
    uint32_t repeats = 0U;
    uint32_t last = 0U;
    uint32_t lcg = 12345U;
    for (uint32_t t = 0U; t < 10000000U;) {
        dev.advance_to(t);
        const uint32_t idx = dev.read_psr();
        if (t > 0U && idx == last) { ++repeats; }
        last = idx;
        lcg = lcg * 1664525U + 1013904223U;
        t += 32000U - 3000U + (lcg >> 8) % (2U * 3000U + 1U);
    }
    EXPECT_GT(repeats, 0U);
}

// ---------------------------------------------------------------------------
// Scratch logger BaroReadyGate cases, run against this one gate
// ---------------------------------------------------------------------------
// The Tiny scratch logger had its own gate (Rocket-Chip-tiny2350,
// src/drivers/baro_dps310_limits.h BaroReadyGate; cases in
// test/test_sensor_logger.cpp BaroReady.*). Same rule as dps310_poll_plan:
// use a pressure only when PRS_RDY is set and a temperature has been seen
// since (re)start. Its three outcomes map onto the plan as below.

enum class GateOutcome : uint8_t { kNoNewResult, kDiscard, kUse };

struct GateHarness {
    bool have_tmp = false;  // the driver's g_haveTmpRaw
    void reset() { have_tmp = false; }  // baro_dps310_start_continuous/stop
    GateOutcome on_meas_cfg(uint8_t meas_cfg) {
        const Dps310PollPlan p = dps310_poll_plan(meas_cfg, have_tmp);
        have_tmp = have_tmp || p.read_tmp;
        if (p.read_psr) {
            return GateOutcome::kUse;
        }
        return ((meas_cfg & kDps310MeasCfgPrsReady) != 0U) ? GateOutcome::kDiscard
                                                           : GateOutcome::kNoNewResult;
    }
};

TEST(BaroReadyGateMerged, RequiresPressureAndInitialTemperature) {
    GateHarness g;
    EXPECT_EQ(g.on_meas_cfg(0xC7U), GateOutcome::kNoNewResult);
    EXPECT_EQ(g.on_meas_cfg(kDps310MeasCfgPrsReady), GateOutcome::kDiscard);
    EXPECT_EQ(g.on_meas_cfg(kDps310MeasCfgTmpReady), GateOutcome::kNoNewResult);
    EXPECT_EQ(g.on_meas_cfg(kDps310MeasCfgPrsReady), GateOutcome::kUse);
}

TEST(BaroReadyGateMerged, FreshPressureUsesCachedTemperatureBetweenConversions) {
    GateHarness g;
    EXPECT_EQ(g.on_meas_cfg(0xF7U), GateOutcome::kUse);
    for (int i = 0; i < 16; ++i) {
        EXPECT_EQ(g.on_meas_cfg(0xC7U), GateOutcome::kNoNewResult);
        EXPECT_EQ(g.on_meas_cfg(0xD7U), GateOutcome::kUse);
    }
    EXPECT_EQ(g.on_meas_cfg(0xE7U), GateOutcome::kNoNewResult);
}

TEST(BaroReadyGateMerged, ResetRequiresNewTemperatureAgain) {
    GateHarness g;
    EXPECT_EQ(g.on_meas_cfg(0x30U), GateOutcome::kUse);
    g.reset();
    EXPECT_EQ(g.on_meas_cfg(0x10U), GateOutcome::kDiscard);
    EXPECT_EQ(g.on_meas_cfg(0x30U), GateOutcome::kUse);
}

}  // namespace
