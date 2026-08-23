// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Call surface is DBG_PRINT / DBG_ERROR / DBG_STATE only.
// Templates in rc::dbg_impl are the if-constexpr bodies — not a second API.
// kDebugEnabled is the single #ifdef DEBUG bridge.

#ifndef ROCKETCHIP_RC_DEBUG_H
#define ROCKETCHIP_RC_DEBUG_H

#include "pico/time.h"
#include "rocketchip/rc_log.h"

#ifdef DEBUG
inline constexpr bool kDebugEnabled = true;
#else
inline constexpr bool kDebugEnabled = false;
#endif

namespace rc {
namespace dbg_impl {

template<typename... Args>
inline void print(const char* fmt, Args... args) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] ", (unsigned long)time_us_32());
        rc::rc_log(fmt, args...);
        rc::rc_log("\n");
    }
}

template<typename... Args>
inline void error(const char* fmt, Args... args) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] ERROR: ", (unsigned long)time_us_32());
        rc::rc_log(fmt, args...);
        rc::rc_log("\n");
    }
}

inline void state(const char* from, const char* to) {
    if constexpr (kDebugEnabled) {
        rc::rc_log("[%lu] State: %s -> %s\n", (unsigned long)time_us_32(), from, to);
    }
}

} // namespace dbg_impl
} // namespace rc

#define DBG_PRINT(fmt, ...) ::rc::dbg_impl::print(fmt, ##__VA_ARGS__)
#define DBG_STATE(from, to) ::rc::dbg_impl::state(from, to)
#define DBG_ERROR(fmt, ...) ::rc::dbg_impl::error(fmt, ##__VA_ARGS__)

#endif // ROCKETCHIP_RC_DEBUG_H
