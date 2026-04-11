// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Notification Priority Resolver — Internal Header (Stage 14, IVP-115)
//
// Pure function mapping NotifyState to a single resolved LED pattern code.
// No hardware dependencies, no QP symbols — testable directly from host.
//
// Iterates categories in priority order (Fault > Cal > Flight > Radio >
// Sensor > Idle) and returns the rc::led::k* pattern code for the first
// non-kNone category. Idle fallback returns kSensorNoGps (blue blink).
//
// This is an internal header used by notify_backend_led.cpp and by
// tests/test_notify.cpp. Not part of the public notify_backend.h API.
//============================================================================
#ifndef ROCKETCHIP_NOTIFY_RESOLVER_H
#define ROCKETCHIP_NOTIFY_RESOLVER_H

#include <stdint.h>
#include "rocketchip/notify_intents.h"

namespace rc {
namespace notify {

// Resolve the highest-priority active intent in NotifyState and return
// its corresponding rc::led::k* pattern code.
uint8_t resolve_led_pattern(const NotifyState& state);

} // namespace notify
} // namespace rc

#endif // ROCKETCHIP_NOTIFY_RESOLVER_H
