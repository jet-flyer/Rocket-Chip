// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Notify — Notification Hub Active Object (Stage 14, IVP-114)
//
// AP_Notify-style intent layer. Sits between state producers and output
// consumers. Maintains per-category intent state, resolves priority,
// dispatches to output backends at 33Hz.
//
// Council-reviewed: 4 personas, unanimous GO. See NOTIFY_CONTRACT.md.
//============================================================================
#ifndef ROCKETCHIP_AO_NOTIFY_H
#define ROCKETCHIP_AO_NOTIFY_H

#include "rocketchip/notify_intents.h"

extern "C" {
#include "qp_port.h"
}

extern QActive * const AO_Notify;

void AO_Notify_start(uint8_t prio);

// Post a calibration intent (replaces AO_LedEngine_post_override).
// Called by AO_RCOS when calibration wizards activate/deactivate overlays.
void AO_Notify_post_cal_intent(rc::notify::CalIntent intent);

// Stage L — post pre-arm-fail visual (yellow double-flash, ~3s auto-clear).
// Called by AO_RCOS after command_handler rejects an ARM. Each call
// resets the counter to full (per JPL council — rapid-fire rejections
// refresh the window rather than decrement the existing count).
void AO_Notify_post_prearm_fail();

// Stage T Batch B IVP-T14 Round 2 #10 — unmissable "VEHICLE NOT HEARD"
// indicator. Station-side only. Called by AO_RfManager on the
// link-lost / re-acquired edge. Latches NotifyState::vehicle_lost so
// backends (LED, audio once wired) surface the condition until cleared.
void AO_Notify_post_vehicle_lost();
void AO_Notify_post_vehicle_found();

#endif // ROCKETCHIP_AO_NOTIFY_H
