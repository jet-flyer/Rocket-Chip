// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Notification Audio Backend (Stage 14, IVP-115)
//
// No-op stub until the audio stage implements the TLV320DAC3100 I2S
// codec driver + AP tone string parser on the Fruit Jam ground station.
//
// AP tone string constants are defined here as data so the notification
// engine can reference them now. Parser is deferred to the audio stage.
// Format: RTTTL-like strings matching ArduPilot AP_Notify/ToneAlarm.
//============================================================================

#include "rocketchip/notify_backend.h"

namespace rc {
namespace notify {

// ============================================================================
// AP_Notify tone string constants (data only — parser deferred)
// ============================================================================
// Format reference: ArduPilot libraries/AP_Notify/ToneAlarm.cpp
// MFT<tempo> L<length> O<octave> <notes>
[[maybe_unused]] static const char* const kToneBoot     = "MFT240L8 O4AB>CE";
[[maybe_unused]] static const char* const kToneArmed    = "MFT200L8 O4CEG";
[[maybe_unused]] static const char* const kToneDisarmed = "MFT200L8 O4GEC";
[[maybe_unused]] static const char* const kToneError    = "MFT200L4 O4C C C";
[[maybe_unused]] static const char* const kToneLanded   = "MFT200L4 O4G E C";

// ============================================================================
// Backend update function — stub
// ============================================================================
void notify_backend_audio_update(const NotifyState& state) {
    (void)state;  // No audio hardware present until the audio stage.
}

} // namespace notify
} // namespace rc
