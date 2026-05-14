// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station-side synthetic RX injection for bench testing (IVP-132a.3).
// Accepts pre-encoded packet bytes (CCSDS nav, CCSDS cmd ACK, MAVLink, etc.)
// and posts them to AO_Telemetry as if they came from the radio.
// Excluded from flight builds.
//
// Usage (CLI dev menu or GDB):
//   station_replay_inject_bytes(buf, len)
//
// Python harness at scripts/station_replay_harness.py generates hex-encoded
// CCSDS nav packets matching IVP-131 scenario profiles and streams them over
// the station dev CLI.
#ifndef ROCKETCHIP_DEV_STATION_REPLAY_H
#define ROCKETCHIP_DEV_STATION_REPLAY_H

#include <stdint.h>

#ifndef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS

// No-op stubs for flight tier (R-25-exec step 2 migration: rc_os_debug.cpp
// now lives in the flight binary and references these symbols at compile
// time; runtime-gated by test_mode_active() before reaching them).
// Step 6 deletes this module entirely and replaces with host-side harness
// per council amendment #4.
static inline void station_replay_inject_bytes(const uint8_t*, uint8_t) {}
static inline uint32_t station_replay_get_inject_count() { return 0; }
static inline void station_replay_start() {}
static inline void station_replay_stop()  {}
static inline bool station_replay_active() { return false; }

#else

extern "C" {
void station_replay_inject_bytes(const uint8_t* buf, uint8_t len);
uint32_t station_replay_get_inject_count();

// Serial-input replay mode (enabled by station dev CLI):
//   Python sends "R,AABBCC...\n" where hex is a full CCSDS (or other) packet.
//   Python sends "REPLAY_END\n" to exit.
void station_replay_start();
void station_replay_stop();
bool station_replay_active();
}

#endif

#endif // ROCKETCHIP_DEV_STATION_REPLAY_H
