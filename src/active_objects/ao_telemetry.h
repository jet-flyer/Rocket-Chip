// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Telemetry — Telemetry Protocol Active Object (IVP-94)
//
// Protocol-only: CCSDS/MAVLink encoding, APID mux, rate dividers.
// No radio hardware references. Posts SIG_RADIO_TX to AO_Radio.
// Receives SIG_RADIO_RX for decode + output.
//============================================================================
#ifndef ROCKETCHIP_AO_TELEMETRY_H
#define ROCKETCHIP_AO_TELEMETRY_H

extern "C" {
#include "qp_port.h"
}

#include "rocketchip/telemetry_state.h"

extern QActive * const AO_Telemetry;

void AO_Telemetry_start(uint8_t prio);

// CLI access — safe under QV cooperative scheduling
void AO_Telemetry_set_telem_snapshot(const rc::TelemetryState& telem);
bool AO_Telemetry_get_mavlink_output();
void AO_Telemetry_toggle_mavlink();
uint8_t AO_Telemetry_cycle_rate();

#endif // ROCKETCHIP_AO_TELEMETRY_H
