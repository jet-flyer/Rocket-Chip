// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station-side synthetic RX injection (IVP-132a.3). Bench-only.
// Exercises station's decode pipeline + dashboard + ACK paths without a
// live vehicle radio. Useful for 30-min soak and ACK protocol stress.

#ifndef BUILD_FOR_FLIGHT

#include "dev/station_replay.h"
#include "active_objects/ao_telemetry.h"
#include "rocketchip/ao_signals.h"
#include <string.h>

extern "C" {
#include "qp_port.h"
}

static uint32_t s_injectCount = 0;
static bool s_replayActive = false;

// File-scope static event — safe under QV cooperative scheduling (LL Entry 35).
// Only written by dev CLI / GDB calls which run in Core 0 handler context,
// so there's no race with the RX path that would cause overwrite.
static rc::RadioRxEvt s_injectEvt;

extern "C" __attribute__((used))
void station_replay_inject_bytes(const uint8_t* buf, uint8_t len) {
    // Stage T IVP-T5: RadioRxEvt.buf bumped to 256; uint8_t len can never
    // exceed it. Keep the null/zero guards.
    if (buf == nullptr || len == 0) { return; }

    s_injectEvt.super.sig = rc::SIG_RADIO_RX;
    s_injectEvt.super.refCtr_ = 0;
    memcpy(s_injectEvt.buf, buf, len);
    s_injectEvt.len = len;
    s_injectEvt.rssi = -80;  // synthetic signal strength
    s_injectEvt.snr  = 10;

    extern QActive * const AO_Telemetry;
    QACTIVE_POST(AO_Telemetry, &s_injectEvt.super, (void *)0);
    ++s_injectCount;
}

extern "C" __attribute__((used))
uint32_t station_replay_get_inject_count() {
    return s_injectCount;
}

extern "C" __attribute__((used))
void station_replay_start() {
    s_replayActive = true;
    s_injectCount = 0;
}

extern "C" __attribute__((used))
void station_replay_stop() {
    s_replayActive = false;
}

extern "C" __attribute__((used))
bool station_replay_active() {
    return s_replayActive;
}

#endif // BUILD_FOR_FLIGHT
