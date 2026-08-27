// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Host test: menu engine (stack + ActionId). No Pico/QP.
#include <gtest/gtest.h>

#include "cli/cli_engine.h"
#include "cli/cli_menus.h"

using rc::cli::ActionId;
using rc::cli::Engine;
using rc::cli::Event;
using rc::cli::MenuId;
using rc::cli::kEsc;
using rc::cli::kStationItemCount;
using rc::cli::kStationItems;
using rc::cli::kVehicleItemCount;
using rc::cli::kVehicleItems;
using rc::cli::init;
using rc::cli::on_key;
using rc::cli::top;

TEST(CliEngine, VehicleMainPreflightAndCalPush) {
    Engine e{};
    init(e);
    auto r = on_key(e, kVehicleItems, kVehicleItemCount, 'p');
    EXPECT_EQ(r.ev, Event::kAction);
    EXPECT_EQ(r.act, ActionId::kPreflight);
    EXPECT_EQ(top(e), MenuId::kMain);

    r = on_key(e, kVehicleItems, kVehicleItemCount, 'c');
    EXPECT_EQ(r.ev, Event::kPushed);
    EXPECT_EQ(top(e), MenuId::kCal);

    r = on_key(e, kVehicleItems, kVehicleItemCount, 'g');
    EXPECT_EQ(r.ev, Event::kAction);
    EXPECT_EQ(r.act, ActionId::kCalGyro);

    r = on_key(e, kVehicleItems, kVehicleItemCount, 'x');
    EXPECT_EQ(r.ev, Event::kPopped);
    EXPECT_EQ(top(e), MenuId::kMain);
}

#if defined(ROCKETCHIP_DEV_MODE)
TEST(CliEngine, VehicleFlightHasInjectWhenDevCompiled) {
    Engine e{};
    init(e);
    auto r = on_key(e, kVehicleItems, kVehicleItemCount, 'v');
    EXPECT_EQ(r.act, ActionId::kDevModeToggle);
    on_key(e, kVehicleItems, kVehicleItemCount, 'f');
    EXPECT_EQ(top(e), MenuId::kFlight);
    r = on_key(e, kVehicleItems, kVehicleItemCount, 'l');
    EXPECT_EQ(r.act, ActionId::kInjectLaunch);
    r = on_key(e, kVehicleItems, kVehicleItemCount, 'n');
    EXPECT_EQ(r.act, ActionId::kInjectLanding);
    r = on_key(e, kVehicleItems, kVehicleItemCount, 'a');
    EXPECT_EQ(r.act, ActionId::kFlightArm);
    on_key(e, kVehicleItems, kVehicleItemCount, 'z');
    on_key(e, kVehicleItems, kVehicleItemCount, 'c');
    r = on_key(e, kVehicleItems, kVehicleItemCount, 'r');
    EXPECT_EQ(r.act, ActionId::kCalReset);
}
#else
TEST(CliEngine, VehicleFlightOmitsInjectWhenField) {
    Engine e{};
    init(e);
    auto r = on_key(e, kVehicleItems, kVehicleItemCount, 'v');
    EXPECT_EQ(r.ev, Event::kUnknown);
    on_key(e, kVehicleItems, kVehicleItemCount, 'f');
    EXPECT_EQ(top(e), MenuId::kFlight);
    r = on_key(e, kVehicleItems, kVehicleItemCount, 'l');
    EXPECT_EQ(r.ev, Event::kUnknown);
    r = on_key(e, kVehicleItems, kVehicleItemCount, 'a');
    EXPECT_EQ(r.act, ActionId::kFlightArm);
}
#endif

TEST(CliEngine, VehicleSettingsStubAndEsc) {
    Engine e{};
    init(e);
    on_key(e, kVehicleItems, kVehicleItemCount, 's');
    EXPECT_EQ(top(e), MenuId::kSettings);
    auto r = on_key(e, kVehicleItems, kVehicleItemCount, 'h');
    EXPECT_EQ(r.act, ActionId::kSettingsStub);
    r = on_key(e, kVehicleItems, kVehicleItemCount, kEsc);
    EXPECT_EQ(r.ev, Event::kPopped);
    EXPECT_EQ(top(e), MenuId::kMain);
}

TEST(CliEngine, StationHasNoCalAndZReturnsPadAction) {
    Engine e{};
    init(e);
    auto r = on_key(e, kStationItems, kStationItemCount, 'c');
    EXPECT_EQ(r.ev, Event::kUnknown);
    r = on_key(e, kStationItems, kStationItemCount, 'a');
    EXPECT_EQ(r.act, ActionId::kStationArmConfirm);
    r = on_key(e, kStationItems, kStationItemCount, 'X');
    EXPECT_EQ(r.act, ActionId::kStationDisarm);
    r = on_key(e, kStationItems, kStationItemCount, 'z');
    EXPECT_EQ(r.act, ActionId::kReturnPad);
}

TEST(CliEngine, HelpIsNotAPush) {
    Engine e{};
    init(e);
    auto r = on_key(e, kVehicleItems, kVehicleItemCount, '?');
    EXPECT_EQ(r.ev, Event::kHelp);
    EXPECT_EQ(top(e), MenuId::kMain);
}
