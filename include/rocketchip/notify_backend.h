// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Notification Backend Interface (Stage 14, IVP-115)
//
// Direct function calls — no function pointer vtable (JSF AV Rule 170).
// Each backend is a free function that receives the resolved NotifyState
// and updates its hardware output.
//
// Backend selection is compile-time. The audio backend is always compiled
// but is a no-op on platforms without I2S DAC (Feather). Future OLED and
// other backends follow the same pattern.
//
// Called from AO_Notify's 33Hz tick handler after priority resolution.
//============================================================================
#ifndef ROCKETCHIP_NOTIFY_BACKEND_H
#define ROCKETCHIP_NOTIFY_BACKEND_H

#include "rocketchip/notify_intents.h"

namespace rc {
namespace notify {

// LED backend — maps NotifyState to AO_LedEngine pattern commands.
// Implemented in src/notify/notify_backend_led.cpp.
void notify_backend_led_update(const NotifyState& state);

// Audio backend — future I2S DAC tone generation.
// Stub on all platforms until the audio stage. Defined in notify_backend_audio.cpp.
void notify_backend_audio_update(const NotifyState& state);

} // namespace notify
} // namespace rc

#endif // ROCKETCHIP_NOTIFY_BACKEND_H
