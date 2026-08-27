// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Shallow ops trees. Vehicle and station are different products.
// Inject / cal-reset exist only when ROCKETCHIP_DEV_MODE is compiled in;
// they still need the runtime `v` toggle (USB + idle to enable).
// Digit-RF / LED test stay omitted (probe test_mode, later sitting).
#ifndef ROCKETCHIP_CLI_MENUS_H
#define ROCKETCHIP_CLI_MENUS_H

#include "cli/cli_engine.h"

namespace rc {
namespace cli {

// Vehicle operator console.
inline constexpr Item kVehicleItems[] = {
    // main
    {MenuId::kMain, 'h', true,  "help",           MenuId::kNone,     ActionId::kHelp},
    {MenuId::kMain, '?', false, "help",           MenuId::kNone,     ActionId::kHelp},
    {MenuId::kMain, 'p', true,  "preflight",      MenuId::kNone,     ActionId::kPreflight},
    {MenuId::kMain, 'c', true,  "calibration",    MenuId::kCal,      ActionId::kNone},
    {MenuId::kMain, 'f', true,  "flight",         MenuId::kFlight,   ActionId::kNone},
    {MenuId::kMain, 'q', true,  "debug",          MenuId::kDebug,    ActionId::kNone},
    {MenuId::kMain, 's', true,  "settings",       MenuId::kSettings, ActionId::kNone},
    {MenuId::kMain, 'b', true,  "beacon",         MenuId::kNone,     ActionId::kBeacon},
    {MenuId::kMain, 't', true,  "radio status",   MenuId::kNone,     ActionId::kRadioStatus},
    {MenuId::kMain, 'g', true,  "list flights",   MenuId::kNone,     ActionId::kListFlights},
    {MenuId::kMain, 'd', true,  "download",       MenuId::kNone,     ActionId::kDownloadFlight},
    {MenuId::kMain, 'l', true,  "flush log",      MenuId::kNone,     ActionId::kFlushLog},
    {MenuId::kMain, 'x', false, "erase flights",  MenuId::kNone,     ActionId::kEraseFlights},
#if defined(ROCKETCHIP_DEV_MODE)
    {MenuId::kMain, 'v', true,  "DEV MODE",       MenuId::kNone,     ActionId::kDevModeToggle},
#endif

    // cal
    {MenuId::kCal, 'g', true,  "gyro cal",       MenuId::kNone, ActionId::kCalGyro},
    {MenuId::kCal, 'l', true,  "level cal",      MenuId::kNone, ActionId::kCalLevel},
    {MenuId::kCal, 'b', true,  "baro cal",       MenuId::kNone, ActionId::kCalBaro},
    {MenuId::kCal, 'a', true,  "accel 6-pos",    MenuId::kNone, ActionId::kCalAccel6pos},
    {MenuId::kCal, 'm', true,  "mag cal",        MenuId::kNone, ActionId::kCalMag},
    {MenuId::kCal, 'w', true,  "wizard",         MenuId::kNone, ActionId::kCalWizard},
    {MenuId::kCal, 'v', true,  "save cal",       MenuId::kNone, ActionId::kCalSave},
#if defined(ROCKETCHIP_DEV_MODE)
    {MenuId::kCal, 'r', true,  "reset cal",      MenuId::kNone, ActionId::kCalReset, Gate::kDevRuntime},
#endif
    {MenuId::kCal, 'h', true,  "help",           MenuId::kNone, ActionId::kHelp},
    {MenuId::kCal, '?', false, "help",           MenuId::kNone, ActionId::kHelp},
    {MenuId::kCal, 'x', true,  "back",           MenuId::kNone, ActionId::kBack},
    {MenuId::kCal, static_cast<char>(kEsc), false, "back", MenuId::kNone, ActionId::kBack},

    {MenuId::kFlight, 'a', true,  "ARM",            MenuId::kNone, ActionId::kFlightArm},
    {MenuId::kFlight, 'd', true,  "DISARM",         MenuId::kNone, ActionId::kFlightDisarm},
    {MenuId::kFlight, 'x', true,  "ABORT",          MenuId::kNone, ActionId::kFlightAbort},
    {MenuId::kFlight, 'r', true,  "RESET",          MenuId::kNone, ActionId::kFlightReset},
#if defined(ROCKETCHIP_DEV_MODE)
    {MenuId::kFlight, 'l', true,  "inject LAUNCH",  MenuId::kNone, ActionId::kInjectLaunch,  Gate::kDevRuntime},
    {MenuId::kFlight, 'b', true,  "inject BURNOUT", MenuId::kNone, ActionId::kInjectBurnout, Gate::kDevRuntime},
    {MenuId::kFlight, 'p', true,  "inject APOGEE",  MenuId::kNone, ActionId::kInjectApogee,  Gate::kDevRuntime},
    {MenuId::kFlight, 'm', true,  "inject MAIN",    MenuId::kNone, ActionId::kInjectMain,    Gate::kDevRuntime},
    {MenuId::kFlight, 'n', true,  "inject LANDING", MenuId::kNone, ActionId::kInjectLanding, Gate::kDevRuntime},
#endif
    {MenuId::kFlight, 's', true,  "status",         MenuId::kNone, ActionId::kFlightStatus},
    {MenuId::kFlight, 'h', true,  "help",    MenuId::kNone, ActionId::kHelp},
    {MenuId::kFlight, '?', false, "help",    MenuId::kNone, ActionId::kHelp},
    {MenuId::kFlight, 'z', true,  "back",    MenuId::kNone, ActionId::kBack},
    {MenuId::kFlight, static_cast<char>(kEsc), false, "back", MenuId::kNone, ActionId::kBack},

    // debug — mutators omitted
    {MenuId::kDebug, 's', true,  "sensors",    MenuId::kNone, ActionId::kDebugSensors},
    {MenuId::kDebug, 'i', true,  "I2C scan",   MenuId::kNone, ActionId::kDebugI2cScan},
    {MenuId::kDebug, 'b', true,  "boot/HW",    MenuId::kNone, ActionId::kDebugBootHw},
    {MenuId::kDebug, 'e', false, "ESKF live",  MenuId::kNone, ActionId::kDebugEskfLive},
    {MenuId::kDebug, 'y', true,  "pyro log",   MenuId::kNone, ActionId::kDebugPyroLog},
    {MenuId::kDebug, 'd', true,  "diag",       MenuId::kNone, ActionId::kDebugDiag},
    {MenuId::kDebug, 'h', true,  "help",       MenuId::kNone, ActionId::kHelp},
    {MenuId::kDebug, '?', false, "help",       MenuId::kNone, ActionId::kHelp},
    {MenuId::kDebug, 'z', true,  "back",       MenuId::kNone, ActionId::kBack},
    {MenuId::kDebug, static_cast<char>(kEsc), false, "back", MenuId::kNone, ActionId::kBack},

    {MenuId::kSettings, 'h', true,  "help", MenuId::kNone, ActionId::kSettingsStub},
    {MenuId::kSettings, '?', false, "help", MenuId::kNone, ActionId::kSettingsStub},
    {MenuId::kSettings, 'z', true,  "back", MenuId::kNone, ActionId::kBack},
    {MenuId::kSettings, static_cast<char>(kEsc), false, "back", MenuId::kNone, ActionId::kBack},
};

inline constexpr size_t kVehicleItemCount =
    sizeof(kVehicleItems) / sizeof(kVehicleItems[0]);

// Station console (after pad 'x'). No cal/flight inject.
inline constexpr Item kStationItems[] = {
    {MenuId::kMain, 'h', true,  "help",         MenuId::kNone,     ActionId::kHelp},
    {MenuId::kMain, '?', false, "help",         MenuId::kNone,     ActionId::kHelp},
    {MenuId::kMain, 'p', true,  "preflight",    MenuId::kNone,     ActionId::kPreflight},
    {MenuId::kMain, 't', true,  "radio status", MenuId::kNone,     ActionId::kRadioStatus},
    {MenuId::kMain, 'g', true,  "station GPS",  MenuId::kNone,     ActionId::kStationGps},
    {MenuId::kMain, 'd', true,  "distance",     MenuId::kNone,     ActionId::kStationDistance},
    {MenuId::kMain, 'b', true,  "beacon",       MenuId::kNone,     ActionId::kBeacon},
    {MenuId::kMain, 'a', false, "ARM confirm",  MenuId::kNone,     ActionId::kStationArmConfirm},
    {MenuId::kMain, 'X', false, "DISARM",       MenuId::kNone,     ActionId::kStationDisarm},
    {MenuId::kMain, 'm', true,  "output mode",  MenuId::kNone,     ActionId::kCycleOutputMode},
    {MenuId::kMain, 'q', true,  "debug",        MenuId::kDebug,    ActionId::kNone},
    {MenuId::kMain, 's', true,  "settings",     MenuId::kSettings, ActionId::kNone},
    {MenuId::kMain, 'z', true,  "pad",          MenuId::kNone,     ActionId::kReturnPad},

    {MenuId::kDebug, 's', true,  "RX status",  MenuId::kNone, ActionId::kDebugSensors},
    {MenuId::kDebug, 'b', true,  "boot/HW",    MenuId::kNone, ActionId::kDebugBootHw},
    {MenuId::kDebug, 'd', true,  "diag",       MenuId::kNone, ActionId::kDebugDiag},
    {MenuId::kDebug, 'h', true,  "help",       MenuId::kNone, ActionId::kHelp},
    {MenuId::kDebug, '?', false, "help",       MenuId::kNone, ActionId::kHelp},
    {MenuId::kDebug, 'z', true,  "back",       MenuId::kNone, ActionId::kBack},
    {MenuId::kDebug, static_cast<char>(kEsc), false, "back", MenuId::kNone, ActionId::kBack},

    {MenuId::kSettings, 'h', true,  "help", MenuId::kNone, ActionId::kSettingsStub},
    {MenuId::kSettings, '?', false, "help", MenuId::kNone, ActionId::kSettingsStub},
    {MenuId::kSettings, 'z', true,  "back", MenuId::kNone, ActionId::kBack},
    {MenuId::kSettings, static_cast<char>(kEsc), false, "back", MenuId::kNone, ActionId::kBack},
};

inline constexpr size_t kStationItemCount =
    sizeof(kStationItems) / sizeof(kStationItems[0]);

}  // namespace cli
}  // namespace rc

#endif
