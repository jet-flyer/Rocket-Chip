// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_HealthMonitor — 10 Hz: test_mode_evaluate() then health tick;
// publishes SIG_HEALTH_STATUS. Pub/sub: docs/AO_ARCHITECTURE.md.
//============================================================================
#ifndef ROCKETCHIP_AO_HEALTH_MONITOR_H
#define ROCKETCHIP_AO_HEALTH_MONITOR_H

extern "C" {
#include "qp_port.h"
}

// Opaque — internal state defined in ao_health_monitor.cpp
extern QActive * const AO_HealthMonitor;

// Start at prio.
void AO_HealthMonitor_start(uint8_t prio);

#endif // ROCKETCHIP_AO_HEALTH_MONITOR_H
