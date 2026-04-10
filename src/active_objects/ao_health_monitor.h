// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_HealthMonitor — System Health Active Object (Stage 13, IVP-105)
//
// Standalone AO that evaluates system health at 10Hz, publishes
// SIG_HEALTH_STATUS to LED/Logger/Telemetry. Decoupled from FD so
// health reporting survives FD handler overruns.
//
// Council-reviewed: priority between FD and Logger, queue depth 8.
//============================================================================
#ifndef ROCKETCHIP_AO_HEALTH_MONITOR_H
#define ROCKETCHIP_AO_HEALTH_MONITOR_H

extern "C" {
#include "qp_port.h"
}

// Opaque — internal state defined in ao_health_monitor.cpp
extern QActive * const AO_HealthMonitor;

// Start the health monitor AO at the given priority
void AO_HealthMonitor_start(uint8_t prio);

#endif // ROCKETCHIP_AO_HEALTH_MONITOR_H
