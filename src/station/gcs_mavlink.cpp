// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Station USB GCS MAVLink stream. Packers live in MavlinkEncoder.

#include "station/gcs_mavlink.h"
#include "rocketchip/mavlink_rx.h"

#include <math.h>
#include <string.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wpedantic"
extern "C" {
#include "common/mavlink.h"
}
#pragma GCC diagnostic pop

namespace rc {

static constexpr uint16_t kFrameCap = 128;  // COMPONENT_METADATA v2 is 108 B payload
// USB TX. USB RX parse is COMM_0 — do not share it.
static constexpr uint8_t kMavUsbTxChan = MAVLINK_COMM_3;

// HOME_POSITION is 60 B payload (~72 B on USB). 1 Hz is negligible on
// USB FS (12 Mbit/s). Gates below are to keep the QGC home pin still.
// 10 m: ~4× PA1010D 2.5 m CEP so static wander does not look like a move.
// 0.7 m/s for 3 s: below typical walk (1.4 m/s) but above GPS speed noise.
static constexpr float kHomeMoveM = 10.0F;
static constexpr float kHomeWalkMps = 0.7F;
static constexpr uint32_t kHomeWalkHoldMs = 3000U;
static constexpr uint32_t kHomeMovingPeriodMs = 1000U;
static constexpr float kMetersPerDegE7 = 0.011132F;  // 111320 m/deg / 1e7

static void sink_write(GcsMavlinkSink sink, const uint8_t* data, uint16_t len) {
    if ((sink.write == nullptr) || (len == 0U)) {
        return;
    }
    sink.write(data, len, sink.ctx);
}

static bool emit_frame(GcsMavlinkSink sink, const uint8_t* frame, uint16_t len) {
    if (len == 0U) {
        return false;
    }
    sink_write(sink, frame, len);
    return true;
}

void gcs_mavlink_init(GcsMavlink* s) {
    if (s == nullptr) {
        return;
    }
    s->encoder.init();
    s->last_telem = TelemetryState{};
    s->telem_valid = false;
    s->heartbeat_sent = false;
    s->last_heartbeat_ms = 0;
    s->param_send_idx = -1;
    s->gcs_sysid = 255;
    s->gcs_compid = 0;
    s->home_lat_1e7 = 0;
    s->home_lon_1e7 = 0;
    s->home_alt_mm = 0;
    s->home_valid = false;
    s->home_sent = false;
    s->home_sent_lat_1e7 = 0;
    s->home_sent_lon_1e7 = 0;
    s->home_sent_ms = 0;
    s->home_moving = false;
    s->home_speed_since_ms = 0;
}

void gcs_mavlink_request_params(GcsMavlink* s) {
    if (s == nullptr) {
        return;
    }
    s->param_send_idx = 0;
}

static bool heartbeat_due(const GcsMavlink* s, uint32_t now_ms) {
    if (!s->heartbeat_sent) {
        return true;
    }
    return (now_ms - s->last_heartbeat_ms) >= kGcsHeartbeatPeriodMs;
}

static bool emit_heartbeat(GcsMavlink* s, uint32_t now_ms, GcsMavlinkSink sink) {
    uint8_t frame[kFrameCap];
    const uint8_t fs = s->telem_valid ? s->last_telem.flight_state : 0U;
    const uint16_t n = s->encoder.encode_heartbeat(fs, frame);
    if (!emit_frame(sink, frame, n)) {
        return false;
    }
    const TelemetryState& telem = s->telem_valid ? s->last_telem
                                                 : TelemetryState{};
    const uint16_t n2 = s->encoder.encode_sys_status(telem, frame);
    (void)emit_frame(sink, frame, n2);
    s->last_heartbeat_ms = now_ms;
    s->heartbeat_sent = true;
    return true;
}

static bool emit_one_param(GcsMavlink* s, GcsMavlinkSink sink) {
    if (s->param_send_idx < 0) {
        return false;
    }
    const uint16_t count = mavlink_rx_param_count();
    if (s->param_send_idx >= static_cast<int16_t>(count)) {
        s->param_send_idx = -1;
        return false;
    }
    const MavParam& p = mavlink_rx_param_table()[s->param_send_idx];
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(msg));
    mavlink_msg_param_value_pack_chan(
        s->encoder.system_id, s->encoder.component_id, kMavUsbTxChan, &msg,
        p.name, p.value, MAV_PARAM_TYPE_REAL32, count,
        static_cast<uint16_t>(s->param_send_idx));
    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(frame, &msg);
    s->param_send_idx++;
    if (s->param_send_idx >= static_cast<int16_t>(count)) {
        s->param_send_idx = -1;
    }
    return emit_frame(sink, frame, n);
}

bool gcs_mavlink_tick(GcsMavlink* s, uint32_t now_ms, GcsMavlinkSink sink) {
    if (s == nullptr) {
        return false;
    }
    bool wrote = false;
    if (heartbeat_due(s, now_ms)) {
        wrote = emit_heartbeat(s, now_ms, sink) || wrote;
    }
    wrote = emit_one_param(s, sink) || wrote;
    return wrote;
}

bool gcs_mavlink_emit_empty_mission(GcsMavlink* s, uint8_t mission_type,
                                    GcsMavlinkSink sink) {
    if (s == nullptr) {
        return false;
    }
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(msg));
    mavlink_msg_mission_count_pack_chan(
        s->encoder.system_id, s->encoder.component_id, kMavUsbTxChan, &msg,
        s->gcs_sysid, s->gcs_compid, 0, mission_type, 0);
    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(frame, &msg);
    return emit_frame(sink, frame, n);
}

static bool emit_packed(GcsMavlinkSink sink, mavlink_message_t* msg) {
    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(frame, msg);
    return emit_frame(sink, frame, n);
}

static bool emit_command_ack(GcsMavlink* s, uint16_t command, uint8_t result,
                             GcsMavlinkSink sink) {
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(msg));
    mavlink_msg_command_ack_pack_chan(
        s->encoder.system_id, s->encoder.component_id, kMavUsbTxChan, &msg,
        command, result, 0, 0, s->gcs_sysid, s->gcs_compid);
    return emit_packed(sink, &msg);
}

static bool emit_autopilot_version(GcsMavlink* s, GcsMavlinkSink sink) {
    const uint8_t zeros8[8] = {};
    const uint8_t zeros18[18] = {};
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(msg));
    mavlink_msg_autopilot_version_pack_chan(
        s->encoder.system_id, s->encoder.component_id, kMavUsbTxChan, &msg,
        MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MAVLINK2,
        (0U << 24) | (16U << 16) | (3U << 8),
        0, 0, 2350,
        zeros8, zeros8, zeros8,
        0, 0, 0, zeros18);
    return emit_packed(sink, &msg);
}

static bool handle_gcs_command(GcsMavlink* s, const mavlink_command_long_t& cmd,
                               GcsMavlinkSink sink) {
    const int32_t req_id = lroundf(cmd.param1);
    const bool want_ver =
        (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) ||
        ((cmd.command == MAV_CMD_REQUEST_MESSAGE) &&
         (req_id == MAVLINK_MSG_ID_AUTOPILOT_VERSION));
    if (want_ver) {
        const bool wrote = emit_autopilot_version(s, sink);
        (void)emit_command_ack(s, cmd.command, MAV_RESULT_ACCEPTED, sink);
        return wrote;
    }
    const bool want_home =
        (cmd.command == MAV_CMD_GET_HOME_POSITION) ||
        ((cmd.command == MAV_CMD_REQUEST_MESSAGE) &&
         (req_id == MAVLINK_MSG_ID_HOME_POSITION));
    if (want_home) {
        if (!s->home_valid) {
            (void)emit_command_ack(s, cmd.command, MAV_RESULT_FAILED, sink);
            return true;
        }
        const bool wrote = gcs_mavlink_emit_home(s, 0, sink);
        (void)emit_command_ack(s, cmd.command, MAV_RESULT_ACCEPTED, sink);
        return wrote;
    }
    // QGC 5.1.4 InitialConnect StandardModes::request() needs ACCEPTED plus
    // AVAILABLE_MODES (mode_index from 1). One MANUAL row ends the list.
    if ((cmd.command == MAV_CMD_REQUEST_MESSAGE) &&
        (req_id == MAVLINK_MSG_ID_AVAILABLE_MODES)) {
        (void)emit_command_ack(s, cmd.command, MAV_RESULT_ACCEPTED, sink);
        mavlink_message_t msg;
        memset(&msg, 0, sizeof(msg));
        mavlink_msg_available_modes_pack_chan(
            s->encoder.system_id, s->encoder.component_id, kMavUsbTxChan, &msg,
            1, 1, MAV_STANDARD_MODE_NON_STANDARD, 0, 0, "MANUAL");
        return emit_packed(sink, &msg);
    }
    // CompInfo: ACCEPTED + empty URI skips HTTP/FTP (uri empty in
    // RequestMetaDataTypeStateMachine::_requestFile).
    if ((cmd.command == MAV_CMD_REQUEST_MESSAGE) &&
        (req_id == MAVLINK_MSG_ID_COMPONENT_METADATA)) {
        (void)emit_command_ack(s, cmd.command, MAV_RESULT_ACCEPTED, sink);
        mavlink_message_t msg;
        memset(&msg, 0, sizeof(msg));
        mavlink_msg_component_metadata_pack_chan(
            s->encoder.system_id, s->encoder.component_id, kMavUsbTxChan, &msg,
            0, 0, "");
        return emit_packed(sink, &msg);
    }
    if (cmd.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
        (void)emit_command_ack(s, cmd.command, MAV_RESULT_ACCEPTED, sink);
        return true;
    }
    (void)emit_command_ack(s, cmd.command, MAV_RESULT_UNSUPPORTED, sink);
    return true;
}

static bool handle_param_request_read(GcsMavlink* s,
                                      const mavlink_message_t* msg,
                                      GcsMavlinkSink sink) {
    mavlink_param_request_read_t req{};
    mavlink_msg_param_request_read_decode(msg, &req);
    const uint16_t npar = mavlink_rx_param_count();
    int16_t idx = req.param_index;
    if (idx < 0) {
        const MavParam* table = mavlink_rx_param_table();
        for (uint16_t i = 0; i < npar; ++i) {
            if (strncmp(table[i].name, req.param_id, 16) == 0) {
                idx = static_cast<int16_t>(i);
                break;
            }
        }
    }
    if ((idx < 0) || (idx >= static_cast<int16_t>(npar))) {
        return false;
    }
    s->param_send_idx = idx;
    const bool wrote = emit_one_param(s, sink);
    s->param_send_idx = -1;
    return wrote;
}

static bool handle_mission_usb(GcsMavlink* s, const mavlink_message_t* msg,
                               GcsMavlinkSink sink) {
    uint8_t mtype = MAV_MISSION_TYPE_MISSION;
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
        mavlink_mission_request_list_t req{};
        mavlink_msg_mission_request_list_decode(msg, &req);
        mtype = req.mission_type;
    }
    return gcs_mavlink_emit_empty_mission(s, mtype, sink);
}

bool gcs_mavlink_handle_usb_frame(GcsMavlink* s, const void* mav_msg,
                                  GcsMavlinkSink sink) {
    if ((s == nullptr) || (mav_msg == nullptr)) {
        return false;
    }
    const auto* msg = static_cast<const mavlink_message_t*>(mav_msg);
    if ((msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) && (msg->sysid != 0)) {
        s->gcs_sysid = msg->sysid;
        s->gcs_compid = msg->compid;
        return false;
    }
    if (msg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
        gcs_mavlink_request_params(s);
        bool wrote = false;
        while (s->param_send_idx >= 0) {
            wrote = emit_one_param(s, sink) || wrote;
        }
        return wrote;
    }
    if (msg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
        return handle_param_request_read(s, msg, sink);
    }
    if ((msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) ||
        (msg->msgid == MAVLINK_MSG_ID_MISSION_CLEAR_ALL)) {
        return handle_mission_usb(s, msg, sink);
    }
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t cmd{};
        mavlink_msg_command_long_decode(msg, &cmd);
        return handle_gcs_command(s, cmd, sink);
    }
    return false;
}

static float home_shift_m(int32_t lat0, int32_t lon0,
                          int32_t lat1, int32_t lon1) {
    const float dlat_m = static_cast<float>(lat1 - lat0) * kMetersPerDegE7;
    const float lat_rad = static_cast<float>(lat0) * 1.0e-7F *
                          (3.14159265F / 180.0F);
    const float dlon_m = static_cast<float>(lon1 - lon0) * kMetersPerDegE7 *
                         cosf(lat_rad);
    return hypotf(dlat_m, dlon_m);
}

bool gcs_mavlink_emit_home(GcsMavlink* s, uint32_t now_ms, GcsMavlinkSink sink) {
    if ((s == nullptr) || !s->home_valid) {
        return false;
    }
    const float qnan[4] = { NAN, NAN, NAN, NAN };
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(msg));
    mavlink_msg_home_position_pack_chan(
        s->encoder.system_id, s->encoder.component_id, kMavUsbTxChan, &msg,
        s->home_lat_1e7, s->home_lon_1e7, s->home_alt_mm,
        0.0F, 0.0F, 0.0F, qnan, 0.0F, 0.0F, 0.0F,
        static_cast<uint64_t>(now_ms) * 1000ULL);
    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(frame, &msg);
    if (!emit_frame(sink, frame, n)) {
        return false;
    }
    s->home_sent = true;
    s->home_sent_lat_1e7 = s->home_lat_1e7;
    s->home_sent_lon_1e7 = s->home_lon_1e7;
    s->home_sent_ms = now_ms;
    return true;
}

bool gcs_mavlink_on_station_gps(GcsMavlink* s, const GcsStationGps& gps,
                                uint32_t now_ms, GcsMavlinkSink sink) {
    if (s == nullptr) {
        return false;
    }
    if (!gps.valid || (gps.fix_type < 2U)) {
        s->home_moving = false;
        return false;
    }
    s->home_lat_1e7 = gps.lat_1e7;
    s->home_lon_1e7 = gps.lon_1e7;
    s->home_alt_mm = gps.alt_mm;
    s->home_valid = true;

    if (gps.speed_mps >= kHomeWalkMps) {
        if (!s->home_moving) {
            s->home_moving = true;
            s->home_speed_since_ms = now_ms;
        }
    } else {
        s->home_moving = false;
    }

    if (!s->home_sent) {
        return gcs_mavlink_emit_home(s, now_ms, sink);
    }
    const float shift = home_shift_m(s->home_sent_lat_1e7, s->home_sent_lon_1e7,
                                     gps.lat_1e7, gps.lon_1e7);
    const bool moved = shift >= kHomeMoveM;
    const bool walking =
        s->home_moving &&
        ((now_ms - s->home_speed_since_ms) >= kHomeWalkHoldMs) &&
        ((now_ms - s->home_sent_ms) >= kHomeMovingPeriodMs);
    if (!moved && !walking) {
        return false;
    }
    return gcs_mavlink_emit_home(s, now_ms, sink);
}

bool gcs_mavlink_on_nav(GcsMavlink* s, const TelemetryState& telem,
                        uint32_t now_ms, GcsMavlinkSink sink) {
    if (s == nullptr) {
        return false;
    }
    s->last_telem = telem;
    s->telem_valid = true;
    // QGC ATTITUDE time_boot_ms is the USB device clock (Fruit Jam).
    // telem.met_ms is vehicle to_ms_since_boot stuffed in the nav SDU;
    // launch T+ MET is not implemented. Last-night dispatch used now_ms.
    const uint32_t t_ms = now_ms;
    uint8_t frame[kFrameCap];
    bool wrote = emit_frame(sink, frame,
                            s->encoder.encode_attitude(telem, t_ms, frame));
    wrote = emit_frame(sink, frame,
                       s->encoder.encode_global_pos(telem, t_ms, frame)) || wrote;
    wrote = emit_frame(sink, frame,
                       s->encoder.encode_gps_raw(telem, t_ms, frame)) || wrote;
    return wrote;
}

}  // namespace rc
