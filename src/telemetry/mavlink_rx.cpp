// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file mavlink_rx.cpp
 * @brief MAVLink v2 GCS command receiver — secondary interface
 *
 * Parses incoming MAVLink v2 frames and generates protocol responses.
 * This is MAVLink GCS compatibility for QGC/Mission Planner, NOT the
 * primary telemetry protocol (CCSDS over LoRa).
 *
 * IVP-62: Bidirectional MAVLink Commands (Stage 7)
 */

#include "rocketchip/mavlink_rx.h"
#include "rocketchip/mavlink_rx.h"  // double-include guard test
#include "rocketchip/telemetry_encoder.h"
#include "flight_director/flight_state.h"

#include <string.h>

// c_library_v2 — official MAVLink C library (header-only)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wpedantic"
extern "C" {
#include "common/mavlink.h"
}
#pragma GCC diagnostic pop

namespace rc {

// ============================================================================
// Parser channel — use MAVLINK_COMM_1 to avoid conflicts with tests on COMM_0
// ============================================================================

static constexpr uint8_t kRxChannel = MAVLINK_COMM_1;

// ============================================================================
// Static parameter table (read-only, informational + tunable)
// ============================================================================

static MavParam g_paramTable[] = {
    {"RC_FW_VER",       0.2f,    false},
    {"RC_BOARD_ID",     2350.0f, false},
    {"RC_MAV_RATE",     10.0f,   false},
    {"RC_TELEM_PRI",    0.0f,    false},  // 0=CCSDS primary
    {"RC_IMU_RATE",     1000.0f, false},
    {"RC_ESKF_RATE",    200.0f,  false},
    {"RC_GPS_RATE",     10.0f,   false},
    {"RC_BARO_RATE",    50.0f,   false},
    {"RC_RADIO_PWR",    20.0f,   false},
    {"RC_RADIO_RATE",   2.0f,    false},
    {"RC_GPS_HDOP",     2.0f,    false},
    {"RC_ZUPT_SIG",     0.5f,    false},
    {"RC_MAG_INH",      1.0f,    false},
    {"RC_WIND_INH",     1.0f,    false},
    {"RC_ESKF_OK",      1.0f,    false},
};

static constexpr uint16_t kParamCount =
    sizeof(g_paramTable) / sizeof(g_paramTable[0]);

// ============================================================================
// Response helpers — append encoded frame to result buffer
// ============================================================================

static void append_frame(MavlinkRxResult* result, const uint8_t* frame,
                          uint16_t len) {
    if (result->len + len > sizeof(result->buf)) { return; }  // overflow guard
    memcpy(result->buf + result->len, frame, len);
    result->len += len;
}

static void emit_param_value(MavlinkRxState* state, MavlinkRxResult* result,
                              uint16_t index) {
    if (index >= kParamCount) { return; }
    const MavParam& p = g_paramTable[index];

    mavlink_message_t msg;
    mavlink_msg_param_value_pack(
        state->encoder->system_id, state->encoder->component_id, &msg,
        p.name, p.value,
        MAV_PARAM_TYPE_REAL32,
        kParamCount, index);
    msg.seq = state->encoder->seq++;

    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(frame, &msg);
    append_frame(result, frame, len);
}

static void emit_command_ack(MavlinkRxState* state, MavlinkRxResult* result,
                              uint16_t command, uint8_t mav_result) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(
        state->encoder->system_id, state->encoder->component_id, &msg,
        command, mav_result,
        0,    // progress
        0,    // result_param2
        state->gcs_sysid, state->gcs_compid);
    msg.seq = state->encoder->seq++;

    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(frame, &msg);
    append_frame(result, frame, len);
}

// ============================================================================
// Message handlers
// ============================================================================

static void handle_heartbeat(MavlinkRxState* state,
                              const mavlink_message_t* msg) {
    // Capture GCS identity from first heartbeat received
    if (!state->gcs_seen) {
        state->gcs_sysid  = msg->sysid;
        state->gcs_compid = msg->compid;
        state->gcs_seen   = true;
    }
    // No response — our own heartbeat is already sent at 1 Hz
}

static void handle_param_request_list(MavlinkRxState* state,
                                       MavlinkRxResult* result) {
    for (uint16_t i = 0; i < kParamCount; i++) {
        emit_param_value(state, result, i);
    }
}

static void handle_param_request_read(MavlinkRxState* state,
                                       const mavlink_message_t* msg,
                                       MavlinkRxResult* result) {
    mavlink_param_request_read_t req;
    mavlink_msg_param_request_read_decode(msg, &req);

    if (req.param_index >= 0 && req.param_index < static_cast<int16_t>(kParamCount)) {
        emit_param_value(state, result, static_cast<uint16_t>(req.param_index));
        return;
    }

    // Search by name
    for (uint16_t i = 0; i < kParamCount; i++) {
        if (strncmp(g_paramTable[i].name, req.param_id, 16) == 0) {
            emit_param_value(state, result, i);
            return;
        }
    }
    // Not found — no response (QGC retries then gives up)
}

static void handle_param_set(MavlinkRxState* state,
                              const mavlink_message_t* msg,
                              MavlinkRxResult* result) {
    mavlink_param_set_t req;
    mavlink_msg_param_set_decode(msg, &req);

    // Find param by name — echo current value (read-only for now)
    for (uint16_t i = 0; i < kParamCount; i++) {
        if (strncmp(g_paramTable[i].name, req.param_id, 16) == 0) {
            emit_param_value(state, result, i);
            return;
        }
    }
    // Not found — no response
}

static void handle_command_long(MavlinkRxState* state,
                                 const mavlink_message_t* msg,
                                 uint8_t flight_state,
                                 MavlinkRxResult* result) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);

    switch (cmd.command) {
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
        // Respond with AUTOPILOT_VERSION then ACK
        mavlink_message_t ver_msg;
        uint8_t uid[8] = {};
        mavlink_msg_autopilot_version_pack(
            state->encoder->system_id, state->encoder->component_id, &ver_msg,
            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
            (0U << 24) | (2U << 16),  // flight_sw_version v0.2
            0,                         // middleware_sw_version
            0,                         // os_sw_version
            2350,                      // board_version
            nullptr, nullptr, nullptr, // custom versions (unused)
            0, 0,                      // vendor/product IDs
            0,                         // uid
            uid);
        ver_msg.seq = state->encoder->seq++;

        uint8_t frame[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(frame, &ver_msg);
        append_frame(result, frame, len);

        emit_command_ack(state, result, cmd.command, MAV_RESULT_ACCEPTED);
        break;
    }

    case MAV_CMD_DO_SET_MODE: {
        // Pre-Flight Director: accept only if currently IDLE
        uint8_t res = (flight_state == static_cast<uint8_t>(rc::FlightPhase::kIdle))
                      ? MAV_RESULT_ACCEPTED
                      : MAV_RESULT_DENIED;
        emit_command_ack(state, result, cmd.command, res);
        break;
    }

    case MAV_CMD_COMPONENT_ARM_DISARM:
        // Pre-Flight Director: ACK but no-op. IVP-67 wires to real ARM.
        emit_command_ack(state, result, cmd.command, MAV_RESULT_ACCEPTED);
        break;

    case MAV_CMD_PREFLIGHT_CALIBRATION:
        emit_command_ack(state, result, cmd.command, MAV_RESULT_UNSUPPORTED);
        break;

    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        // Safety: never allow remote reboot
        emit_command_ack(state, result, cmd.command, MAV_RESULT_DENIED);
        break;

    default:
        emit_command_ack(state, result, cmd.command, MAV_RESULT_UNSUPPORTED);
        break;
    }
}

static void handle_set_mode(MavlinkRxState* state,
                             const mavlink_message_t* msg,
                             uint8_t flight_state,
                             MavlinkRxResult* result) {
    // Legacy SET_MODE message (#11) — same logic as DO_SET_MODE
    uint8_t res = (flight_state == static_cast<uint8_t>(rc::FlightPhase::kIdle))
                  ? MAV_RESULT_ACCEPTED
                  : MAV_RESULT_DENIED;
    emit_command_ack(state, result, MAV_CMD_DO_SET_MODE, res);
}

static void handle_mission_request_list(MavlinkRxState* state,
                                         MavlinkRxResult* result) {
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(
        state->encoder->system_id, state->encoder->component_id, &msg,
        state->gcs_sysid, state->gcs_compid,
        0,                          // count = 0 (no missions)
        MAV_MISSION_TYPE_MISSION,
        0);
    msg.seq = state->encoder->seq++;

    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(frame, &msg);
    append_frame(result, frame, len);
}

// ============================================================================
// Main dispatcher
// ============================================================================

static void dispatch_message(MavlinkRxState* state,
                              const mavlink_message_t* msg,
                              uint8_t flight_state,
                              MavlinkRxResult* result) {
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_heartbeat(state, msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        handle_param_request_list(state, result);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_param_request_read(state, msg, result);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        handle_param_set(state, msg, result);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_command_long(state, msg, flight_state, result);
        break;
    case MAVLINK_MSG_ID_SET_MODE:
        handle_set_mode(state, msg, flight_state, result);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        handle_mission_request_list(state, result);
        break;
    default:
        // Silently ignore unknown message types
        break;
    }
}

// ============================================================================
// Public API
// ============================================================================

void mavlink_rx_init(MavlinkRxState* state, MavlinkEncoder* encoder) {
    memset(state, 0, sizeof(*state));
    state->encoder = encoder;
}

bool mavlink_rx_feed_byte(MavlinkRxState* state, uint8_t byte,
                          uint8_t flight_state, uint32_t /*now_ms*/,
                          MavlinkRxResult* result) {
    // Access parser state from the opaque buffer
    mavlink_message_t* rx_msg =
        reinterpret_cast<mavlink_message_t*>(state->parser_buf);
    mavlink_status_t* rx_status =
        reinterpret_cast<mavlink_status_t*>(state->parser_buf + sizeof(mavlink_message_t));

    uint8_t parsed = mavlink_parse_char(kRxChannel, byte, rx_msg, rx_status);
    if (!parsed) { return false; }

    // Complete frame parsed — dispatch
    dispatch_message(state, rx_msg, flight_state, result);
    return true;
}

const MavParam* mavlink_rx_param_table() {
    return g_paramTable;
}

uint16_t mavlink_rx_param_count() {
    return kParamCount;
}

} // namespace rc
