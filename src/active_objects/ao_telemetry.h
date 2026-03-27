// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Telemetry — Telemetry Active Object (IVP-80)
//
// Wraps telemetry_radio_tick() and mavlink_direct_tick() in a QF Active
// Object. Owns LoRa TX/RX scheduling and USB MAVLink encoding.
//============================================================================
#ifndef ROCKETCHIP_AO_TELEMETRY_H
#define ROCKETCHIP_AO_TELEMETRY_H

extern "C" {
#include "qp_port.h"
}

extern QActive * const AO_Telemetry;

void AO_Telemetry_start(uint8_t prio);

#endif // ROCKETCHIP_AO_TELEMETRY_H
