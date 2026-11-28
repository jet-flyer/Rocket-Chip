// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// WMM GPS-fix persist gate — host tests (cal_persist_gate.h, wmm_persist.h)
//
// eskf_runner applies the GPS-derived WMM field in RAM whenever
// accept_fix() returns true, and writes cal storage only when
// take_persist() returns true. These tests pin that contract:
//   - only kIdle allows a calibration flash write
//   - a fix on the pad (kIdle) persists at once
//   - a fix after ARM is accepted (RAM) but never persists while armed or
//     flying; it persists after DISARM or landing + RESET returns to kIdle
//   - one fix and one persist per boot
//============================================================================

#include <gtest/gtest.h>
#include "fusion/wmm_persist.h"

namespace {

constexpr float kLat = 40.5F;
constexpr float kLon = -105.25F;

// Every phase in which the vehicle is armed, flying, or not yet RESET.
constexpr rc::FlightPhase kNoPersistPhases[] = {
    rc::FlightPhase::kArmed,
    rc::FlightPhase::kBoost,
    rc::FlightPhase::kCoast,
    rc::FlightPhase::kDrogueDescent,
    rc::FlightPhase::kMainDescent,
    rc::FlightPhase::kLanded,
    rc::FlightPhase::kAbort,
    rc::FlightPhase::kFault,
};

} // namespace

TEST(CalPersistGate, OnlyIdleAllowsFlashWrite) {
    EXPECT_TRUE(rc::cal_persist_allowed(rc::FlightPhase::kIdle));
    for (rc::FlightPhase phase : kNoPersistPhases) {
        EXPECT_FALSE(rc::cal_persist_allowed(phase))
            << "phase " << static_cast<int>(phase);
    }
}

TEST(CalPersistGate, RefusesOutOfRangePhase) {
    EXPECT_FALSE(rc::cal_persist_allowed(rc::FlightPhase::kCount));
}

TEST(WmmGpsPersist, FixInIdlePersistsImmediately) {
    rc::WmmGpsPersist gate;
    EXPECT_TRUE(gate.accept_fix(kLat, kLon));  // RAM field applied
    rc::WmmPosition pos{};
    ASSERT_TRUE(gate.take_persist(rc::FlightPhase::kIdle, &pos));
    EXPECT_FLOAT_EQ(pos.lat_deg, kLat);
    EXPECT_FLOAT_EQ(pos.lon_deg, kLon);
    EXPECT_FALSE(gate.persist_pending());
}

TEST(WmmGpsPersist, FixAfterArmAppliesButDoesNotPersistInFlight) {
    rc::WmmGpsPersist gate;
    EXPECT_TRUE(gate.accept_fix(kLat, kLon));  // RAM field applied while armed
    rc::WmmPosition pos{};
    for (rc::FlightPhase phase : kNoPersistPhases) {
        EXPECT_FALSE(gate.take_persist(phase, &pos))
            << "phase " << static_cast<int>(phase);
        EXPECT_TRUE(gate.persist_pending());
    }
}

TEST(WmmGpsPersist, FixAfterArmPersistsAfterLandingReset) {
    rc::WmmGpsPersist gate;
    rc::WmmPosition pos{};
    EXPECT_FALSE(gate.take_persist(rc::FlightPhase::kArmed, &pos));
    EXPECT_TRUE(gate.accept_fix(kLat, kLon));
    const rc::FlightPhase flight[] = {
        rc::FlightPhase::kArmed, rc::FlightPhase::kBoost,
        rc::FlightPhase::kCoast, rc::FlightPhase::kDrogueDescent,
        rc::FlightPhase::kMainDescent, rc::FlightPhase::kLanded,
    };
    for (rc::FlightPhase phase : flight) {
        EXPECT_FALSE(gate.take_persist(phase, &pos));
    }
    ASSERT_TRUE(gate.take_persist(rc::FlightPhase::kIdle, &pos));  // RESET
    EXPECT_FLOAT_EQ(pos.lat_deg, kLat);
    EXPECT_FLOAT_EQ(pos.lon_deg, kLon);
}

TEST(WmmGpsPersist, FixAfterArmPersistsAfterDisarm) {
    rc::WmmGpsPersist gate;
    rc::WmmPosition pos{};
    EXPECT_TRUE(gate.accept_fix(kLat, kLon));
    EXPECT_FALSE(gate.take_persist(rc::FlightPhase::kArmed, &pos));
    EXPECT_TRUE(gate.take_persist(rc::FlightPhase::kIdle, &pos));  // DISARM
}

TEST(WmmGpsPersist, OneFixAndOnePersistPerBoot) {
    rc::WmmGpsPersist gate;
    rc::WmmPosition pos{};
    EXPECT_TRUE(gate.accept_fix(kLat, kLon));
    EXPECT_FALSE(gate.accept_fix(kLat + 1.0F, kLon + 1.0F));
    ASSERT_TRUE(gate.take_persist(rc::FlightPhase::kIdle, &pos));
    EXPECT_FLOAT_EQ(pos.lat_deg, kLat);  // first fix, not the later one
    EXPECT_FALSE(gate.take_persist(rc::FlightPhase::kIdle, &pos));
    EXPECT_FALSE(gate.accept_fix(kLat, kLon));
    EXPECT_FALSE(gate.take_persist(rc::FlightPhase::kIdle, &pos));
}

TEST(WmmGpsPersist, NothingToPersistWithoutFix) {
    rc::WmmGpsPersist gate;
    rc::WmmPosition pos{};
    EXPECT_FALSE(gate.persist_pending());
    EXPECT_FALSE(gate.take_persist(rc::FlightPhase::kIdle, &pos));
}

TEST(WmmGpsPersist, NullOutputKeepsRequestPending) {
    rc::WmmGpsPersist gate;
    EXPECT_TRUE(gate.accept_fix(kLat, kLon));
    EXPECT_FALSE(gate.take_persist(rc::FlightPhase::kIdle, nullptr));
    EXPECT_TRUE(gate.persist_pending());
}
