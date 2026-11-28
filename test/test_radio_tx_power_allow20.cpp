// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Host check of RC_RFM95_ALLOW_20DBM. This target defines the flag.

#include <gtest/gtest.h>

#include "drivers/rfm95w.h"

TEST(RadioTxPowerAllow20, CompileFlagRaisesInitOptIn) {
    EXPECT_TRUE(kRfm95Allow20DbmDefault);
}
