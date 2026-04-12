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
// Legacy compat for vehicle mode (TX) — reads/writes AO_RCOS output mode
bool AO_Telemetry_get_mavlink_output();
void AO_Telemetry_toggle_mavlink();
uint8_t AO_Telemetry_cycle_rate();

// Station RX telemetry access (IVP-99: station CLI display)
struct RxTelemSnapshot {
    rc::TelemetryState telem;
    uint32_t met_ms;        // MET from CCSDS secondary header
    uint16_t seq;           // CCSDS sequence counter
    bool     valid;         // At least one packet successfully decoded
};
const RxTelemSnapshot* AO_Telemetry_get_rx_state();

// IVP-62a: notify GCS heartbeat received (transitions to full telemetry output)
void AO_Telemetry_notify_gcs_heartbeat();

// IVP-62b: feed USB input byte to MAVLink RX parser (called from rc_os.cpp)
void AO_Telemetry_feed_usb_byte(uint8_t byte);

// IVP-62d: check if a GCS is currently connected (suppress CLI routing)
bool AO_Telemetry_is_gcs_connected();

// IVP-62c: send MAVLink COMMAND_LONG to vehicle over LoRa (station only)
void AO_Telemetry_send_command(uint16_t command, float p1 = 0, float p2 = 0,
                               float p3 = 0, float p4 = 0, float p5 = 0,
                               float p6 = 0, float p7 = 0);

// IVP-122: send a tracked command — populates pending-cmd state for ACK
// tracking. Uses the MAVLink COMMAND_LONG `confirmation` field to carry the
// station's command sequence number. 3 retries at 3-second intervals.
void AO_Telemetry_send_tracked_command(uint16_t command, float p1 = 0);

// IVP-122: check if a tracked command ACK is still pending
bool AO_Telemetry_is_cmd_pending();

// IVP-122: tick the pending command retry timer (called from station tick loop)
void AO_Telemetry_cmd_retry_tick(uint32_t now_ms);

#endif // ROCKETCHIP_AO_TELEMETRY_H
