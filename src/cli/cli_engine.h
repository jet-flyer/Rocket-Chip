// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Console engine: menu-as-data + stack. Pico/QP-free.
// Modes (pad / line / catalog / mavlock) stay in the caller. This is
// kKey only: lookup → push submenu | ActionId | help | back.
// P10-9: items carry ActionId, not function pointers (direct switch in
// cli_actions.cpp). AO_RCOS ticks USB; it is not one QP state per menu.
// JSF-190: find() has no continue. P10-2: scan bound is table length n.
#ifndef ROCKETCHIP_CLI_ENGINE_H
#define ROCKETCHIP_CLI_ENGINE_H

#include <stddef.h>
#include <stdint.h>

namespace rc {
namespace cli {

inline constexpr int kEsc = 27;
inline constexpr uint8_t kStackMax = 4;

enum class MenuId : uint8_t {
    kMain = 0,
    kCal,
    kFlight,
    kDebug,
    kSettings,
    kNone = 255
};

enum class ActionId : uint8_t {
    kNone = 0,
    kHelp,
    kBack,
    kReturnPad,
    kPreflight,
    kBeacon,
    kRadioStatus,
    kFlushLog,
    kEraseFlights,
    kListFlights,
    kDownloadFlight,
    kCycleOutputMode,
    kStationGps,
    kStationDistance,
    kStationArmConfirm,
    kStationDisarm,
    kCalGyro,
    kCalLevel,
    kCalBaro,
    kCalAccel6pos,
    kCalMag,
    kCalWizard,
    kCalSave,
    kCalReset,
    kDevModeToggle,
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
    kDebugDiag,
    kSettingsStub,
};

enum class Gate : uint8_t {
    kNone = 0,
    kDevRuntime = 1,  // compile-time ROCKETCHIP_DEV_MODE + runtime toggle
};

struct Item {
    MenuId      menu;
    char        key;
    bool        fold_case;
    const char* label;
    MenuId      push;  // kNone = not a submenu
    ActionId    act;
    Gate        gate = Gate::kNone;
};

struct Engine {
    MenuId  stack[kStackMax];
    uint8_t depth;
};

inline constexpr char fold_ascii(int c) {
    if (c >= 'A' && c <= 'Z') {
        return static_cast<char>(c - 'A' + 'a');
    }
    return static_cast<char>(c);
}

inline void init(Engine& e) {
    e.depth = 1;
    e.stack[0] = MenuId::kMain;
}

inline MenuId top(const Engine& e) {
    return (e.depth == 0) ? MenuId::kMain : e.stack[e.depth - 1];
}

inline bool push_menu(Engine& e, MenuId m) {
    if (m == MenuId::kNone || e.depth >= kStackMax) {
        return false;
    }
    e.stack[e.depth++] = m;
    return true;
}

inline bool pop_menu(Engine& e) {
    if (e.depth <= 1) {
        e.depth = 1;
        e.stack[0] = MenuId::kMain;
        return false;
    }
    --e.depth;
    return true;
}

inline constexpr bool key_match(const Item& it, int c) {
    if (it.fold_case) {
        return fold_ascii(c) == fold_ascii(static_cast<int>(it.key));
    }
    return c == static_cast<int>(it.key);
}

inline constexpr const Item* find(const Item* table, size_t n, MenuId menu,
                                  int c) {
    for (size_t i = 0; i < n; ++i) {
        if (table[i].menu == menu && key_match(table[i], c)) {
            return &table[i];
        }
    }
    return nullptr;
}

enum class Event : uint8_t {
    kUnknown = 0,
    kHelp,
    kPushed,
    kPopped,
    kAction,
};

struct Result {
    Event    ev;
    ActionId act;
    MenuId   menu;
};

inline Result on_key(Engine& e, const Item* table, size_t n, int c) {
    Result r{};
    r.ev   = Event::kUnknown;
    r.act  = ActionId::kNone;
    r.menu = top(e);

    const Item* it = find(table, n, r.menu, c);
    if (it == nullptr) {
        return r;
    }

    if (it->act == ActionId::kHelp) {
        r.ev  = Event::kHelp;
        r.act = ActionId::kHelp;
        return r;
    }
    if (it->act == ActionId::kBack) {
        pop_menu(e);
        r.ev   = Event::kPopped;
        r.act  = ActionId::kBack;
        r.menu = top(e);
        return r;
    }
    if (it->push != MenuId::kNone) {
        push_menu(e, it->push);
        r.ev   = Event::kPushed;
        r.menu = top(e);
        r.act  = it->act;
        return r;
    }
    r.ev  = Event::kAction;
    r.act = it->act;
    return r;
}

}  // namespace cli
}  // namespace rc

#endif
