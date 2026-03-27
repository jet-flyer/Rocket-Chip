// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Logger — Flight Data Logger Active Object (IVP-79)
//
// Wraps logging_tick() in a QF Active Object. Owns PCM frame encoding,
// ring buffer writes, and flash flush coordination.
//============================================================================
#ifndef ROCKETCHIP_AO_LOGGER_H
#define ROCKETCHIP_AO_LOGGER_H

extern "C" {
#include "qp_port.h"
}

extern QActive * const AO_Logger;

void AO_Logger_start(uint8_t prio);

#endif // ROCKETCHIP_AO_LOGGER_H
