// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include <gtest/gtest.h>

#include "cli/cli_catalog.h"

using rc::cli::CatalogId;
using rc::cli::WriteClass;
using rc::cli::kCatalog;
using rc::cli::kCatalogCount;

TEST(CliCatalog, V0RowsAndWriteClass) {
    EXPECT_GE(kCatalogCount, 6u);
    bool saw_preset = false;
    bool saw_power = false;
    bool saw_usb = false;
    for (size_t i = 0; i < kCatalogCount; ++i) {
        const auto& r = kCatalog[i];
        if (r.id == CatalogId::kNavPreset) {
            saw_preset = true;
            EXPECT_EQ(r.wr, WriteClass::kBasic);
            EXPECT_STREQ(r.starcom, "radio");
            EXPECT_FALSE(r.persist);
        }
        if (r.id == CatalogId::kTxPower) {
            saw_power = true;
            EXPECT_EQ(r.wr, WriteClass::kLocked);
        }
        if (r.id == CatalogId::kUsbArmInh) {
            saw_usb = true;
            EXPECT_EQ(r.wr, WriteClass::kLocked);
            EXPECT_TRUE(r.vehicle);
            EXPECT_FALSE(r.station);
        }
    }
    EXPECT_TRUE(saw_preset);
    EXPECT_TRUE(saw_power);
    EXPECT_TRUE(saw_usb);
}
