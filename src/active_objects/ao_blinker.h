// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Blinker — Demo Active Object: Heartbeat LED (IVP-76)
//
// Trivial AO that toggles the board LED on a 1Hz QF time event.
// Replaces heartbeat_tick() as the first AO migration proof.
// Validates: QF time events, QV dispatch, AO lifecycle.
//============================================================================
#ifndef ROCKETCHIP_AO_BLINKER_H
#define ROCKETCHIP_AO_BLINKER_H

extern "C" {
#include "qp_port.h"
}

// Opaque — internal state defined in ao_blinker.cpp
extern QActive * const AO_Blinker;

// Start the blinker AO at the given priority
void AO_Blinker_start(uint8_t prio);

#endif // ROCKETCHIP_AO_BLINKER_H
