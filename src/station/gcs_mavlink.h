// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Station USB GCS bridge for QGC / Mission Planner.
// Air is Starcom COP-P. This module only emits MAVLink v2 on USB.

#ifndef ROCKETCHIP_GCS_MAVLINK_H
#define ROCKETCHIP_GCS_MAVLINK_H

#include <stdint.h>
#include "rocketchip/telemetry_encoder.h"
#include "rocketchip/telemetry_state.h"

namespace rc {

// 1 Hz HEARTBEAT is the MAVLink connection service:
// https://mavlink.io/en/services/heartbeat.html
constexpr uint32_t kGcsHeartbeatPeriodMs = 1000U;

struct GcsMavlinkSink {
    void (*write)(const uint8_t* data, uint16_t len, void* ctx);
    void* ctx;
};

struct GcsMavlink {
    MavlinkEncoder encoder;
    TelemetryState last_telem;
    bool telem_valid;
    bool heartbeat_sent;
    uint32_t last_heartbeat_ms;
    int16_t param_send_idx;  // -1 idle; else next table index
    uint8_t gcs_sysid;
    uint8_t gcs_compid;

    // Pad GPS as HOME_POSITION (USB only).
    int32_t home_lat_1e7;
    int32_t home_lon_1e7;
    int32_t home_alt_mm;
    bool home_valid;
    bool home_sent;
    int32_t home_sent_lat_1e7;
    int32_t home_sent_lon_1e7;
    uint32_t home_sent_ms;
    bool home_moving;
    uint32_t home_speed_since_ms;
};

struct GcsStationGps {
    int32_t lat_1e7;
    int32_t lon_1e7;
    int32_t alt_mm;
    float speed_mps;
    uint8_t fix_type;
    bool valid;
};

void gcs_mavlink_init(GcsMavlink* s);

void gcs_mavlink_request_params(GcsMavlink* s);

// MISSION_COUNT count=0. QGC times out ~20s without this.
bool gcs_mavlink_emit_empty_mission(GcsMavlink* s, uint8_t mission_type,
                                    GcsMavlinkSink sink);

// Parsed USB MAVLink frame (mavlink_message_t*). HEARTBEAT / params / mission.
bool gcs_mavlink_handle_usb_frame(GcsMavlink* s, const void* mav_msg,
                                  GcsMavlinkSink sink);

// HEARTBEAT + SYS_STATUS at 1 Hz, even with no nav yet.
// One PARAM_VALUE per call while a list request is in flight.
bool gcs_mavlink_tick(GcsMavlink* s, uint32_t now_ms, GcsMavlinkSink sink);

// ATTITUDE + GLOBAL_POSITION_INT for one Starcom nav SDU.
bool gcs_mavlink_on_nav(GcsMavlink* s, const TelemetryState& telem,
                        uint32_t now_ms, GcsMavlinkSink sink);

// Fruit Jam GPS → HOME_POSITION. First fix always. Then only if the pad
// moved ~10 m or speed stayed up ~3 s (walk/drive) at 1 Hz.
bool gcs_mavlink_on_station_gps(GcsMavlink* s, const GcsStationGps& gps,
                                uint32_t now_ms, GcsMavlinkSink sink);
bool gcs_mavlink_emit_home(GcsMavlink* s, uint32_t now_ms, GcsMavlinkSink sink);

// Pico SDK stdio_usb_connected() is tud_cdc_connected() (DTR) unless
// PICO_STDIO_USB_CONNECTION_WITHOUT_DTR (then tud_ready()).
// QGC 5.x config-task CDC reopen drops DTR for tens-hundreds of ms.
// 1500 ms sits above typical Windows CDC re-enum (<1 s) and below QGC
// InitialConnect COMMAND_LONG retry (~3 s, handle_gcs_command).
constexpr uint32_t kGcsUsbDisconnectDebounceMs = 1500U;

// *since_ms: 0 = last seen connected. Nonzero = first disconnected now_ms.
// Returns true when exclusive MAVLink mode should drop back to the pad.
inline bool gcs_usb_disconnect_expired(bool connected, uint32_t now_ms,
                                       uint32_t* since_ms) {
    if (since_ms == nullptr) {
        return !connected;
    }
    if (connected) {
        *since_ms = 0U;
        return false;
    }
    if (*since_ms == 0U) {
        *since_ms = (now_ms == 0U) ? 1U : now_ms;
        return false;
    }
    return (now_ms - *since_ms) >= kGcsUsbDisconnectDebounceMs;
}

}  // namespace rc

#endif
