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
#include "safety/health_monitor.h"

namespace rc {
namespace notify {

// Resolve the highest-priority active intent in NotifyState and return
// its corresponding rc::led::k* pattern code.
uint8_t resolve_led_pattern(const NotifyState& state);

// Decode a HealthStatusEvt primary + secondary byte pair into the
// highest-priority FaultIntent. Ascending FaultIntent values = ascending
// priority. Returns FaultIntent::kNone if no faults are active.
//
// Inline so both AO_Notify and host tests can use it without pulling QP.
inline FaultIntent decode_health_faults(uint8_t primary, uint8_t secondary) {
    FaultIntent max_fault = FaultIntent::kNone;

    if (rc::health_imu(primary) == rc::kHealthFault) {
        max_fault = FaultIntent::kImuFail;
    }
    if (rc::health_eskf(primary) == rc::kHealthFault &&
        FaultIntent::kEskfFail > max_fault) {
        max_fault = FaultIntent::kEskfFail;
    }
    if (rc::health_baro(primary) == rc::kHealthFault &&
        FaultIntent::kBaroFail > max_fault) {
        max_fault = FaultIntent::kBaroFail;
    }
    if ((secondary & rc::kHealthPioOk) == 0 &&
        FaultIntent::kPioWdt > max_fault) {
        max_fault = FaultIntent::kPioWdt;
    }
    if ((secondary & rc::kHealthWatchdogOk) == 0 &&
        FaultIntent::kSafeMode > max_fault) {
        max_fault = FaultIntent::kSafeMode;
    }
    if ((secondary & rc::kHealthCore1Ok) == 0 &&
        FaultIntent::kCore1Stall > max_fault) {
        max_fault = FaultIntent::kCore1Stall;
    }
    return max_fault;
}

} // namespace notify
} // namespace rc

#endif // ROCKETCHIP_NOTIFY_RESOLVER_H
