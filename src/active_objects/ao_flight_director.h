// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_FlightDirector — Flight Director Active Object (IVP-78)
//
// Wraps the Stage 8 QEP Flight Director HSM in a QF Active Object.
// State handler code is unchanged — the migration is purely structural.
// Receives SIG_FD_TICK time events instead of manual dispatch from superloop.
//============================================================================
#ifndef ROCKETCHIP_AO_FLIGHT_DIRECTOR_H
#define ROCKETCHIP_AO_FLIGHT_DIRECTOR_H

extern "C" {
#include "qp_port.h"
}

extern QActive * const AO_FlightDirector;

// Initialize and start the Flight Director AO.
// Must be called after flight_director_ctor() and flight_director_init().
void AO_FlightDirector_start(uint8_t prio);

#endif // ROCKETCHIP_AO_FLIGHT_DIRECTOR_H
