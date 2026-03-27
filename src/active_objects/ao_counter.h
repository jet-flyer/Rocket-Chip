// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Counter — Demo Active Object: Jitter-Measuring Event Counter (IVP-76)
//
// 10Hz time event counter. Every 5 seconds prints: event count, min/max/avg
// inter-dispatch jitter in microseconds. Jitter > 1ms indicates QV_onIdle
// processing time is too large (Council Amendment A5).
//
// This AO serves as a standing regression test for the Stage 9 migration.
// Run it after every IVP to verify timing is not degrading.
//============================================================================
#ifndef ROCKETCHIP_AO_COUNTER_H
#define ROCKETCHIP_AO_COUNTER_H

extern "C" {
#include "qp_port.h"
}

// Opaque — internal state defined in ao_counter.cpp
extern QActive * const AO_Counter;

// Start the counter AO at the given priority
void AO_Counter_start(uint8_t prio);

#endif // ROCKETCHIP_AO_COUNTER_H
