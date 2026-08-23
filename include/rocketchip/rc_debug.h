// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Debug print helpers. Callers use DBG_PRINT / DBG_ERROR / DBG_STATE.
// kDebugEnabled is the single #ifdef DEBUG bridge; if constexpr elides
// the body when DEBUG is off.

#ifndef ROCKETCHIP_RC_DEBUG_H
#define ROCKETCHIP_RC_DEBUG_H

#include "pico/time.h"
#include "rocketchip/rc_log.h"

#ifdef DEBUG
inline constexpr bool kDebugEnabled = true;
#else
inline constexpr bool kDebugEnabled = false;
#endif

template<typename... Args>
inline void dbg_print(const char* fmt, Args... args) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] ", (unsigned long)time_us_32());
        rc::rc_log(fmt, args...);
        rc::rc_log("\n");
    }
}

template<typename... Args>
inline void dbg_error(const char* fmt, Args... args) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] ERROR: ", (unsigned long)time_us_32());
        rc::rc_log(fmt, args...);
        rc::rc_log("\n");
    }
}

inline void dbg_state(const char* from, const char* to) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] State: %s -> %s\n", (unsigned long)time_us_32(), from, to);
    }
}

#define DBG_PRINT(fmt, ...) dbg_print(fmt, ##__VA_ARGS__)
#define DBG_STATE(from, to) dbg_state(from, to)
#define DBG_ERROR(fmt, ...) dbg_error(fmt, ##__VA_ARGS__)

#endif // ROCKETCHIP_RC_DEBUG_H
