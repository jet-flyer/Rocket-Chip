// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Notify module — internal header
//
// Production API for the notify implementation TUs and AO_Notify:
//   - resolve_led_pattern: LED priority resolver (Fault > Cal > Flight >
//     Radio > Sensor > Idle). Called from notify_backend_led_update on
//     the 33 Hz tick. Not a test double.
//   - decode_health_faults: used by AO_Notify to map health bytes to
//     FaultIntent.
//   - notify_backend_*_update: LED and audio backends. One production
//     caller (AO_Notify tick). Decls live here so they are not a
//     one-off public header (WN-035).
//
// Host tests include this same header to call those production functions
// without QP or hardware. The functions exist for firmware; tests are
// a second consumer, not the reason the code exists.
//============================================================================
#ifndef ROCKETCHIP_NOTIFY_RESOLVER_H
#define ROCKETCHIP_NOTIFY_RESOLVER_H

#include <stdint.h>
#include "rocketchip/notify_intents.h"
#include "safety/health_monitor.h"

namespace rc {
namespace notify {

// Resolve the highest-priority active intent in NotifyState and return
// its corresponding rc::led::k* pattern code. Beacon overlay
// (beacon_manual / beacon_auto) is applied after that resolution —
// see apply_beacon_overlay in notify_backend_led.cpp.
uint8_t resolve_led_pattern(const NotifyState& state);

void notify_backend_led_update(const NotifyState& state);
void notify_backend_audio_update(const NotifyState& state);

// Decode a HealthStatusEvt primary + secondary byte pair into the
// highest-priority FaultIntent. Ascending FaultIntent values = ascending
// priority. Returns FaultIntent::kNone if no faults are active.
//
// Inline so AO_Notify and host tests share one definition without QP.
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
