// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Sitting 0: freeze live CLI keys. Does not link firmware dispatch.
#include <gtest/gtest.h>

#include "cli/cli_keymap.h"

using rc::cli::Action;
using rc::cli::find;
using rc::cli::Gate;
using rc::cli::kEsc;
using rc::cli::kKeyCount;
using rc::cli::kKeys;
using rc::cli::kStation;
using rc::cli::kVehicle;
using rc::cli::Menu;

TEST(CliKeymap, CountIsFrozenInventory) {
    EXPECT_EQ(kKeyCount, 63u);
}

TEST(CliKeymap, MainHelpAndMenus) {
    EXPECT_EQ(find(Menu::kMain, 'h', kVehicle)->action, Action::kHelp);
    EXPECT_EQ(find(Menu::kMain, 'H', kVehicle)->action, Action::kHelp);
    EXPECT_EQ(find(Menu::kMain, '?', kVehicle)->action, Action::kHelp);
    EXPECT_EQ(find(Menu::kMain, 'c', kVehicle)->action, Action::kEnterCal);
    EXPECT_EQ(find(Menu::kMain, 'f', kStation)->action, Action::kEnterFlight);
    EXPECT_EQ(find(Menu::kMain, 'q', kVehicle)->action, Action::kEnterDebug);
    EXPECT_EQ(find(Menu::kMain, 'b', kVehicle)->action, Action::kBeacon);
}

TEST(CliKeymap, MainPreflightBothRolesNotGpsPush) {
    // Live handle_main_menu eats 'p' as preflight. Station GPS-push in
    // cli_handle_unhandled_key is unreachable from main.
    EXPECT_EQ(find(Menu::kMain, 'p', kVehicle)->action, Action::kPreflight);
    EXPECT_EQ(find(Menu::kMain, 'P', kStation)->action, Action::kPreflight);
}

TEST(CliKeymap, MainXCaseSplit) {
    EXPECT_EQ(find(Menu::kMain, 'x', kVehicle)->action, Action::kEraseFlights);
    EXPECT_EQ(find(Menu::kMain, 'x', kStation)->action, Action::kEraseFlights);
    EXPECT_EQ(find(Menu::kMain, 'X', kStation)->action, Action::kStationDisarm);
    EXPECT_EQ(find(Menu::kMain, 'X', kVehicle), nullptr);
}

TEST(CliKeymap, MainRLowercaseOnly) {
    EXPECT_EQ(find(Menu::kMain, 'r', kVehicle)->action, Action::kCycleTxRate);
    EXPECT_EQ(find(Menu::kMain, 'r', kStation)->action,
              Action::kStationRadioConfigCycle);
    EXPECT_EQ(find(Menu::kMain, 'R', kVehicle), nullptr);
    EXPECT_EQ(find(Menu::kMain, 'R', kStation), nullptr);
}

TEST(CliKeymap, MainRoleColumns) {
    EXPECT_EQ(find(Menu::kMain, 'g', kVehicle)->action, Action::kListFlights);
    EXPECT_EQ(find(Menu::kMain, 'g', kStation)->action, Action::kStationGps);
    EXPECT_EQ(find(Menu::kMain, 'd', kVehicle)->action, Action::kDownloadFlight);
    EXPECT_EQ(find(Menu::kMain, 'd', kStation)->action, Action::kStationDistance);
    EXPECT_EQ(find(Menu::kMain, 'a', kStation)->action, Action::kStationArmConfirm);
    EXPECT_EQ(find(Menu::kMain, 'a', kVehicle), nullptr);
    EXPECT_EQ(find(Menu::kMain, 'A', kStation), nullptr);
}

TEST(CliKeymap, CalAndFlightBack) {
    EXPECT_EQ(find(Menu::kCal, 'g', kVehicle)->action, Action::kCalGyro);
    EXPECT_EQ(find(Menu::kCal, 'x', kVehicle)->action, Action::kBackMain);
    EXPECT_EQ(find(Menu::kCal, kEsc, kVehicle)->action, Action::kBackMain);
    EXPECT_EQ(find(Menu::kFlight, 'a', kVehicle)->action, Action::kFlightArm);
    EXPECT_EQ(find(Menu::kFlight, 'l', kVehicle)->action, Action::kInjectLaunch);
    EXPECT_EQ(find(Menu::kFlight, 'z', kVehicle)->action, Action::kBackMain);
}

TEST(CliKeymap, DebugGatesAndLowercaseE) {
    EXPECT_EQ(find(Menu::kDebug, 's', kVehicle)->action, Action::kDebugSensors);
    EXPECT_EQ(find(Menu::kDebug, 'e', kVehicle)->action, Action::kDebugEskfLive);
    EXPECT_EQ(find(Menu::kDebug, 'E', kVehicle), nullptr);
    EXPECT_EQ(find(Menu::kDebug, 'l', kVehicle)->gate, Gate::kTestMode);
    EXPECT_EQ(find(Menu::kDebug, '0', kVehicle)->action, Action::kDebugRadioCfg0);
    EXPECT_EQ(find(Menu::kDebug, '0', kVehicle)->gate, Gate::kTestMode);
}

TEST(CliKeymap, HelpPrintedMainKeysExist) {
    // print_help_menu: h p c f / g d l x / t r m b q
    const char help[] = {'h', 'p', 'c', 'f', 'g', 'd', 'l', 'x',
                         't', 'r', 'm', 'b', 'q'};
    for (char c : help) {
        EXPECT_NE(find(Menu::kMain, c, kVehicle), nullptr) << c;
    }
}

TEST(CliKeymap, NoDuplicateExactBinding) {
    for (size_t i = 0; i < kKeyCount; ++i) {
        for (size_t j = i + 1; j < kKeyCount; ++j) {
            if (kKeys[i].menu != kKeys[j].menu) {
                continue;
            }
            if ((kKeys[i].roles & kKeys[j].roles) == 0) {
                continue;
            }
            const bool same = kKeys[i].fold_case && kKeys[j].fold_case
                ? (rc::cli::fold_ascii(kKeys[i].key) ==
                   rc::cli::fold_ascii(kKeys[j].key))
                : (kKeys[i].key == kKeys[j].key &&
                   kKeys[i].fold_case == kKeys[j].fold_case);
            EXPECT_FALSE(same)
                << "duplicate " << kKeys[i].key << " menu "
                << static_cast<int>(kKeys[i].menu);
        }
    }
}
