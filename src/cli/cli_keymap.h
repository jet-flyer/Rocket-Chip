// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Live CLI key inventory. Host-testable; Pico/QP-free.
// Sitting 0 freeze of src/cli/rc_os.cpp + rc_os_commands.cpp + rc_os_debug.cpp.
// Runtime dispatch still uses the switches until sitting 1 wires this table.
#ifndef ROCKETCHIP_CLI_KEYMAP_H
#define ROCKETCHIP_CLI_KEYMAP_H

#include <stddef.h>
#include <stdint.h>

namespace rc {
namespace cli {

enum class Menu : uint8_t {
    kMain = 0,
    kCal,
    kFlight,
    kDebug,
};

enum Role : uint8_t {
    kVehicle = 1u << 0,
    kStation = 1u << 1,
    kBoth    = static_cast<uint8_t>(kVehicle | kStation),
};

enum class Gate : uint8_t {
    kNone = 0,
    kTestMode,
};

enum class Action : uint8_t {
    kHelp = 0,
    kEnterCal,
    kEnterFlight,
    kEnterDebug,
    kPreflight,
    kBeacon,
    kFlushLog,
    kEraseFlights,
    kListFlights,
    kDownloadFlight,
    kStationGps,
    kStationDistance,
    kRadioStatus,
    kCycleTxRate,
    kStationRadioConfigCycle,
    kCycleOutputMode,
    kStationArmConfirm,
    kStationDisarm,
    kCalGyro,
    kCalLevel,
    kCalBaro,
    kCalAccel6pos,
    kCalMag,
    kCalWizard,
    kCalReset,
    kCalSave,
    kBackMain,
    kFlightArm,
    kFlightDisarm,
    kFlightAbort,
    kFlightReset,
    kInjectLaunch,
    kInjectBurnout,
    kInjectApogee,
    kInjectMain,
    kInjectLanding,
    kFlightStatus,
    kDebugSensors,
    kDebugI2cScan,
    kDebugBootHw,
    kDebugEskfLive,
    kDebugPyroLog,
    kDebugReplayRetired,
    kDebugDiag,
    kDebugLedTest,
    kDebugRadioCfg0,
    kDebugRadioCfg1,
    kDebugRadioCfg2,
    kDebugRadioCfg3,
    kDebugRadioCfg4,
    kDebugRadioCfg5,
};

struct Key {
    char        key;       // exact char stored; fold_case compares ignoring case
    bool        fold_case;
    Menu        menu;
    uint8_t     roles;
    Gate        gate;
    Action      action;
    const char* help;
};

// ESC is 27. Stored as the exact key with fold_case false.
inline constexpr int kEsc = 27;

// Frozen 2026-08-26 from live switches. See docs/plans/RCOS_REWORK.md.
inline constexpr Key kKeys[] = {
    // Main — handle_main_menu
    {'h', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kHelp, "help"},
    {'?', false, Menu::kMain, kBoth,    Gate::kNone, Action::kHelp, "help"},
    {'c', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kEnterCal, "calibration menu"},
    {'f', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kEnterFlight, "flight director menu"},
    {'p', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kPreflight, "preflight go/no-go"},
    {'q', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kEnterDebug, "debug menu"},
    {'b', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kBeacon, "find-me beacon"},
    // Main — cli_handle_unhandled_key (reachable from handle_main_menu default)
    {'l', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kFlushLog, "flush log to flash"},
    {'x', false, Menu::kMain, kBoth,    Gate::kNone, Action::kEraseFlights, "erase all flights"},
    {'d', true,  Menu::kMain, kVehicle, Gate::kNone, Action::kDownloadFlight, "download flight"},
    {'d', true,  Menu::kMain, kStation, Gate::kNone, Action::kStationDistance, "station distance"},
    {'g', true,  Menu::kMain, kVehicle, Gate::kNone, Action::kListFlights, "list flights"},
    {'g', true,  Menu::kMain, kStation, Gate::kNone, Action::kStationGps, "station GPS"},
    {'t', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kRadioStatus, "radio status"},
    {'r', false, Menu::kMain, kVehicle, Gate::kNone, Action::kCycleTxRate, "cycle TX rate"},
    {'r', false, Menu::kMain, kStation, Gate::kNone, Action::kStationRadioConfigCycle, "cycle radio config"},
    {'m', true,  Menu::kMain, kBoth,    Gate::kNone, Action::kCycleOutputMode, "cycle output mode"},
    {'a', false, Menu::kMain, kStation, Gate::kNone, Action::kStationArmConfirm, "station ARM confirm"},
    {'X', false, Menu::kMain, kStation, Gate::kNone, Action::kStationDisarm, "station DISARM"},

    // Cal
    {'g', true,  Menu::kCal, kBoth, Gate::kNone, Action::kCalGyro, "gyro cal"},
    {'l', true,  Menu::kCal, kBoth, Gate::kNone, Action::kCalLevel, "level cal"},
    {'b', true,  Menu::kCal, kBoth, Gate::kNone, Action::kCalBaro, "baro cal"},
    {'a', true,  Menu::kCal, kBoth, Gate::kNone, Action::kCalAccel6pos, "accel 6-position"},
    {'m', true,  Menu::kCal, kBoth, Gate::kNone, Action::kCalMag, "mag cal"},
    {'w', true,  Menu::kCal, kBoth, Gate::kNone, Action::kCalWizard, "cal wizard"},
    {'r', true,  Menu::kCal, kBoth, Gate::kNone, Action::kCalReset, "reset cal"},
    {'v', true,  Menu::kCal, kBoth, Gate::kNone, Action::kCalSave, "save cal"},
    {'h', true,  Menu::kCal, kBoth, Gate::kNone, Action::kHelp, "help"},
    {'?', false, Menu::kCal, kBoth, Gate::kNone, Action::kHelp, "help"},
    {'x', true,  Menu::kCal, kBoth, Gate::kNone, Action::kBackMain, "back"},
    {static_cast<char>(kEsc), false, Menu::kCal, kBoth, Gate::kNone, Action::kBackMain, "back"},

    // Flight
    {'a', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kFlightArm, "ARM"},
    {'d', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kFlightDisarm, "DISARM"},
    {'x', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kFlightAbort, "ABORT"},
    {'r', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kFlightReset, "RESET"},
    {'l', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kInjectLaunch, "inject LAUNCH"},
    {'b', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kInjectBurnout, "inject BURNOUT"},
    {'p', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kInjectApogee, "inject APOGEE"},
    {'m', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kInjectMain, "inject MAIN"},
    {'n', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kInjectLanding, "inject LANDING"},
    {'s', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kFlightStatus, "flight status"},
    {'h', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kHelp, "help"},
    {'?', false, Menu::kFlight, kBoth, Gate::kNone, Action::kHelp, "help"},
    {'z', true,  Menu::kFlight, kBoth, Gate::kNone, Action::kBackMain, "back"},
    {static_cast<char>(kEsc), false, Menu::kFlight, kBoth, Gate::kNone, Action::kBackMain, "back"},

    // Debug
    {'s', true,  Menu::kDebug, kBoth, Gate::kNone,     Action::kDebugSensors, "sensors / station RX"},
    {'i', true,  Menu::kDebug, kBoth, Gate::kNone,     Action::kDebugI2cScan, "I2C scan"},
    {'b', true,  Menu::kDebug, kBoth, Gate::kNone,     Action::kDebugBootHw, "boot / HW status"},
    {'e', false, Menu::kDebug, kBoth, Gate::kNone,     Action::kDebugEskfLive, "ESKF live"},
    {'y', true,  Menu::kDebug, kBoth, Gate::kNone,     Action::kDebugPyroLog, "pyro edge log"},
    {'r', true,  Menu::kDebug, kBoth, Gate::kNone,     Action::kDebugReplayRetired, "replay retired"},
    {'d', true,  Menu::kDebug, kBoth, Gate::kNone,     Action::kDebugDiag, "diag stats"},
    {'l', true,  Menu::kDebug, kBoth, Gate::kTestMode, Action::kDebugLedTest, "LED test"},
    {'0', false, Menu::kDebug, kBoth, Gate::kTestMode, Action::kDebugRadioCfg0, "radio cfg 0"},
    {'1', false, Menu::kDebug, kBoth, Gate::kTestMode, Action::kDebugRadioCfg1, "radio cfg 1"},
    {'2', false, Menu::kDebug, kBoth, Gate::kTestMode, Action::kDebugRadioCfg2, "radio cfg 2"},
    {'3', false, Menu::kDebug, kBoth, Gate::kTestMode, Action::kDebugRadioCfg3, "radio cfg 3"},
    {'4', false, Menu::kDebug, kBoth, Gate::kTestMode, Action::kDebugRadioCfg4, "radio cfg 4"},
    {'5', false, Menu::kDebug, kBoth, Gate::kTestMode, Action::kDebugRadioCfg5, "radio cfg 5"},
    {'h', true,  Menu::kDebug, kBoth, Gate::kNone,     Action::kHelp, "help"},
    {'?', false, Menu::kDebug, kBoth, Gate::kNone,     Action::kHelp, "help"},
    {'z', true,  Menu::kDebug, kBoth, Gate::kNone,     Action::kBackMain, "back"},
    {static_cast<char>(kEsc), false, Menu::kDebug, kBoth, Gate::kNone, Action::kBackMain, "back"},
};

inline constexpr size_t kKeyCount = sizeof(kKeys) / sizeof(kKeys[0]);

inline constexpr char fold_ascii(int c) {
    if (c >= 'A' && c <= 'Z') {
        return static_cast<char>(c - 'A' + 'a');
    }
    return static_cast<char>(c);
}

inline constexpr const Key* find(Menu menu, int c, uint8_t role) {
    const Key* found = nullptr;
    for (size_t i = 0; i < kKeyCount; ++i) {
        const Key& k = kKeys[i];
        if (k.menu != menu) {
            continue;
        }
        if ((k.roles & role) == 0) {
            continue;
        }
        const bool match = k.fold_case ? (fold_ascii(c) == fold_ascii(k.key))
                                       : (c == static_cast<int>(k.key));
        if (!match) {
            continue;
        }
        found = &k;
        break;
    }
    return found;
}

}  // namespace cli
}  // namespace rc

#endif
