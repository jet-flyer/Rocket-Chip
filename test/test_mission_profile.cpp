// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// IVP-74: Mission Profile Configuration Tests
//
// Validates:
//   - Rocket profile: correct values, has pyro, correct ID
//   - HAB profile: no pyro, emergency deploy, lower thresholds
//   - Struct properties: no pointers, name fits, serialization-ready
//   - Both profiles: all timeouts positive
//============================================================================

#include <gtest/gtest.h>
#include "mission_profile.h"

// HAB profile generated from profiles/hab.cfg (test-only)
#include "test_hab_profile_data.h"

// ============================================================================
// Rocket Profile Tests
// ============================================================================

TEST(MissionProfileRocket, IdIsRocket) {
    EXPECT_EQ(rc::kDefaultRocketProfile.id, rc::ProfileId::kRocket);
}

TEST(MissionProfileRocket, NameIsRocket) {
    EXPECT_STREQ(rc::kDefaultRocketProfile.name, "Rocket");
}

TEST(MissionProfileRocket, HasPyro) {
    EXPECT_TRUE(rc::kDefaultRocketProfile.has_pyro);
}

TEST(MissionProfileRocket, AbortDoesNotFireDrogueByDefault) {
    // ABORT is a safety command, not a deployment trigger.
    // User can enable via .cfg if desired.
    EXPECT_FALSE(rc::kDefaultRocketProfile.abort_fires_drogue_from_boost);
    EXPECT_FALSE(rc::kDefaultRocketProfile.abort_fires_drogue_from_coast);
}

TEST(MissionProfileRocket, RespectsLockouts) {
    EXPECT_FALSE(rc::kDefaultRocketProfile.emergency_deploy_anytime);
}

TEST(MissionProfileRocket, ApogeeRequiresBoth) {
    EXPECT_TRUE(rc::kDefaultRocketProfile.apogee_require_both);
}

TEST(MissionProfileRocket, ThresholdsMatchCfg) {
    // These values come from profiles/rocket.cfg — if someone changes the
    // .cfg and regenerates, these tests will catch the discrepancy.
    EXPECT_FLOAT_EQ(rc::kDefaultRocketProfile.launch_accel_threshold, 20.0f);
    EXPECT_EQ(rc::kDefaultRocketProfile.launch_sustain_ms, 50u);
    EXPECT_FLOAT_EQ(rc::kDefaultRocketProfile.burnout_accel_threshold, 5.0f);
    EXPECT_FLOAT_EQ(rc::kDefaultRocketProfile.apogee_velocity_threshold, 0.5f);
    EXPECT_FLOAT_EQ(rc::kDefaultRocketProfile.main_deploy_altitude_m, 150.0f);
    EXPECT_FLOAT_EQ(rc::kDefaultRocketProfile.landing_velocity_threshold, 0.5f);
    EXPECT_EQ(rc::kDefaultRocketProfile.landing_sustain_ms, 2000u);
}

TEST(MissionProfileRocket, TimeoutsMatchCfg) {
    EXPECT_EQ(rc::kDefaultRocketProfile.armed_timeout_ms, 300000u);
    EXPECT_EQ(rc::kDefaultRocketProfile.abort_timeout_ms, 300000u);
    EXPECT_EQ(rc::kDefaultRocketProfile.coast_timeout_ms, 15000u);
}

TEST(MissionProfileRocket, SafetyLockoutsMatchCfg) {
    EXPECT_FLOAT_EQ(rc::kDefaultRocketProfile.deploy_lockout_mps, 80.0f);
    EXPECT_EQ(rc::kDefaultRocketProfile.apogee_lockout_ms, 3000u);
}

TEST(MissionProfileRocket, PreArmAllOptional) {
    EXPECT_FALSE(rc::kDefaultRocketProfile.require_gps_lock);
    EXPECT_FALSE(rc::kDefaultRocketProfile.require_mag_cal);
    EXPECT_FALSE(rc::kDefaultRocketProfile.require_radio);
}

// ============================================================================
// HAB Profile Tests
// ============================================================================

TEST(MissionProfileHab, IdIsHab) {
    EXPECT_EQ(rc::kHabProfile.id, rc::ProfileId::kHab);
}

TEST(MissionProfileHab, NameIsHab) {
    EXPECT_STREQ(rc::kHabProfile.name, "HAB");
}

TEST(MissionProfileHab, NoPyro) {
    EXPECT_FALSE(rc::kHabProfile.has_pyro);
}

TEST(MissionProfileHab, EmergencyDeployEnabled) {
    EXPECT_TRUE(rc::kHabProfile.emergency_deploy_anytime);
}

TEST(MissionProfileHab, NoAbortPyro) {
    EXPECT_FALSE(rc::kHabProfile.abort_fires_drogue_from_boost);
    EXPECT_FALSE(rc::kHabProfile.abort_fires_drogue_from_coast);
}

TEST(MissionProfileHab, LowerLaunchAccelThanRocket) {
    EXPECT_LT(rc::kHabProfile.launch_accel_threshold,
              rc::kDefaultRocketProfile.launch_accel_threshold);
}

TEST(MissionProfileHab, ApogeeUsesOr) {
    EXPECT_FALSE(rc::kHabProfile.apogee_require_both);
}

TEST(MissionProfileHab, RequiresGpsAndRadio) {
    EXPECT_TRUE(rc::kHabProfile.require_gps_lock);
    EXPECT_TRUE(rc::kHabProfile.require_radio);
}

TEST(MissionProfileHab, LongerArmedTimeout) {
    EXPECT_GT(rc::kHabProfile.armed_timeout_ms,
              rc::kDefaultRocketProfile.armed_timeout_ms);
}

// ============================================================================
// Struct Property Tests
// ============================================================================

TEST(MissionProfileStruct, NameFitsInBuffer) {
    // Profile name must fit in char[16] (15 chars + null)
    EXPECT_LE(strlen(rc::kDefaultRocketProfile.name), 15u);
    EXPECT_LE(strlen(rc::kHabProfile.name), 15u);
}

TEST(MissionProfileStruct, NoPointers) {
    // Struct must be position-independent for future serialization.
    // A struct with pointers would be larger than the sum of its fields.
    // This is a heuristic check — if sizeof grows unexpectedly, investigate.
    // IVP-83: PhaseQRTable adds ~257 bytes (8 phases × 32 bytes + ramp_steps)
    EXPECT_LE(sizeof(rc::MissionProfile), 384u);
}

TEST(MissionProfileStruct, AllTimeoutsPositive) {
    // Both profiles
    EXPECT_GT(rc::kDefaultRocketProfile.armed_timeout_ms, 0u);
    EXPECT_GT(rc::kDefaultRocketProfile.abort_timeout_ms, 0u);
    EXPECT_GT(rc::kDefaultRocketProfile.coast_timeout_ms, 0u);
    EXPECT_GT(rc::kDefaultRocketProfile.burnout_backup_ms, 0u);
    EXPECT_GT(rc::kDefaultRocketProfile.main_backup_ms, 0u);

    EXPECT_GT(rc::kHabProfile.armed_timeout_ms, 0u);
    EXPECT_GT(rc::kHabProfile.abort_timeout_ms, 0u);
    EXPECT_GT(rc::kHabProfile.coast_timeout_ms, 0u);
    EXPECT_GT(rc::kHabProfile.burnout_backup_ms, 0u);
    EXPECT_GT(rc::kHabProfile.main_backup_ms, 0u);
}

TEST(MissionProfileStruct, ProfileIdCount) {
    EXPECT_EQ(static_cast<uint8_t>(rc::ProfileId::kCount), 3u);
}
