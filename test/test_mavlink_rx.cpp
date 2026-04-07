// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_mavlink_rx.cpp
 * @brief Host-side tests for MAVLink v2 GCS command receiver
 *
 * Tests the RX handler's parsing and response generation without
 * any Pico SDK dependencies. Uses c_library_v2 directly to craft
 * incoming frames and decode response frames.
 *
 * IVP-62: Bidirectional MAVLink Commands (Stage 7)
 */

#include <gtest/gtest.h>
#include "rocketchip/mavlink_rx.h"
#include "rocketchip/telemetry_encoder.h"
#include "flight_director/flight_state.h"

#include <cstring>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wpedantic"
extern "C" {
#include "common/mavlink.h"
}
#pragma GCC diagnostic pop

using namespace rc;

// ============================================================================
// Test fixture
// ============================================================================

class MavlinkRxTest : public ::testing::Test {
protected:
    MavlinkRxState  state;
    MavlinkEncoder  encoder;
    MavlinkRxResult result;

    // GCS identity for crafting messages
    static constexpr uint8_t kGcsSysid  = 255;
    static constexpr uint8_t kGcsCompid = 190;
    static constexpr uint8_t kVehSysid  = 1;
    static constexpr uint8_t kVehCompid = 1;

    void SetUp() override {
        encoder.init(kVehSysid, kVehCompid);
        mavlink_rx_init(&state, &encoder);
        memset(&result, 0, sizeof(result));
    }

    // Feed a complete MAVLink frame into the RX handler byte-by-byte
    bool feed_frame(const uint8_t* frame, uint16_t len, uint8_t flight_state = 0) {
        bool parsed = false;
        for (uint16_t i = 0; i < len; i++) {
            if (mavlink_rx_feed_byte(&state, frame[i], flight_state, 0, &result)) {
                parsed = true;
            }
        }
        return parsed;
    }

    // Encode a MAVLink message into a wire frame and feed it
    bool feed_msg(const mavlink_message_t& msg, uint8_t flight_state = 0) {
        uint8_t frame[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(frame, &msg);
        return feed_frame(frame, len, flight_state);
    }

    // Build and feed a GCS heartbeat
    void send_gcs_heartbeat() {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(kGcsSysid, kGcsCompid, &msg,
                                   MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                                   0, 0, 0);
        feed_msg(msg);
    }

    // Decode a PARAM_VALUE from the response buffer at a given offset
    bool decode_param_value(uint16_t& offset, mavlink_param_value_t& pv) {
        // Parse response bytes from offset
        mavlink_message_t rx_msg;
        mavlink_status_t rx_status;
        memset(&rx_status, 0, sizeof(rx_status));

        while (offset < result.len) {
            if (mavlink_parse_char(MAVLINK_COMM_0, result.buf[offset], &rx_msg, &rx_status)) {
                offset++;
                if (rx_msg.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
                    mavlink_msg_param_value_decode(&rx_msg, &pv);
                    return true;
                }
            } else {
                offset++;
            }
        }
        return false;
    }

    // Decode a COMMAND_ACK from the response buffer at a given offset
    bool decode_command_ack(uint16_t& offset, mavlink_command_ack_t& ack) {
        mavlink_message_t rx_msg;
        mavlink_status_t rx_status;
        memset(&rx_status, 0, sizeof(rx_status));

        while (offset < result.len) {
            if (mavlink_parse_char(MAVLINK_COMM_0, result.buf[offset], &rx_msg, &rx_status)) {
                offset++;
                if (rx_msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    mavlink_msg_command_ack_decode(&rx_msg, &ack);
                    return true;
                }
            } else {
                offset++;
            }
        }
        return false;
    }

    // Count messages of a given type in the response buffer
    uint16_t count_messages(uint32_t msgid) {
        mavlink_message_t rx_msg;
        mavlink_status_t rx_status;
        memset(&rx_status, 0, sizeof(rx_status));
        uint16_t count = 0;

        for (uint16_t i = 0; i < result.len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, result.buf[i], &rx_msg, &rx_status)) {
                if (rx_msg.msgid == msgid) { count++; }
            }
        }
        return count;
    }
};

// ============================================================================
// Test 1: ParseHeartbeat — GCS heartbeat sets gcs_seen, no response
// ============================================================================

TEST_F(MavlinkRxTest, ParseHeartbeat) {
    EXPECT_FALSE(state.gcs_seen);

    send_gcs_heartbeat();

    EXPECT_TRUE(state.gcs_seen);
    EXPECT_EQ(state.gcs_sysid, kGcsSysid);
    EXPECT_EQ(state.gcs_compid, kGcsCompid);
    EXPECT_EQ(result.len, 0);  // No response to heartbeat
}

// ============================================================================
// Test 2: ParamRequestList — all params returned with correct indices/count
// ============================================================================

TEST_F(MavlinkRxTest, ParamRequestList) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_param_request_list_pack(kGcsSysid, kGcsCompid, &msg,
                                        kVehSysid, kVehCompid);
    feed_msg(msg);

    uint16_t param_count = mavlink_rx_param_count();
    ASSERT_GT(param_count, 0);
    EXPECT_EQ(count_messages(MAVLINK_MSG_ID_PARAM_VALUE), param_count);

    // Verify first and last param indices
    uint16_t offset = 0;
    mavlink_param_value_t pv;
    ASSERT_TRUE(decode_param_value(offset, pv));
    EXPECT_EQ(pv.param_index, 0);
    EXPECT_EQ(pv.param_count, param_count);
}

// ============================================================================
// Test 3: ParamRequestReadByIndex — single PARAM_VALUE by index
// ============================================================================

TEST_F(MavlinkRxTest, ParamRequestReadByIndex) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_param_request_read_pack(kGcsSysid, kGcsCompid, &msg,
                                        kVehSysid, kVehCompid,
                                        "", 2);  // index=2
    feed_msg(msg);

    EXPECT_EQ(count_messages(MAVLINK_MSG_ID_PARAM_VALUE), 1);

    uint16_t offset = 0;
    mavlink_param_value_t pv;
    ASSERT_TRUE(decode_param_value(offset, pv));
    EXPECT_EQ(pv.param_index, 2);

    // Verify against param table
    const MavParam* table = mavlink_rx_param_table();
    EXPECT_FLOAT_EQ(pv.param_value, table[2].value);
}

// ============================================================================
// Test 4: ParamRequestReadByName — single PARAM_VALUE by name lookup
// ============================================================================

TEST_F(MavlinkRxTest, ParamRequestReadByName) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    const MavParam* table = mavlink_rx_param_table();
    const char* target_name = table[5].name;  // RC_ESKF_RATE

    mavlink_message_t msg;
    mavlink_msg_param_request_read_pack(kGcsSysid, kGcsCompid, &msg,
                                        kVehSysid, kVehCompid,
                                        target_name, -1);  // by name
    feed_msg(msg);

    EXPECT_EQ(count_messages(MAVLINK_MSG_ID_PARAM_VALUE), 1);

    uint16_t offset = 0;
    mavlink_param_value_t pv;
    ASSERT_TRUE(decode_param_value(offset, pv));
    EXPECT_FLOAT_EQ(pv.param_value, table[5].value);
    EXPECT_EQ(pv.param_index, 5);
}

// ============================================================================
// Test 5: ParamSetReadOnly — echoes current value unchanged
// ============================================================================

TEST_F(MavlinkRxTest, ParamSetReadOnly) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    const MavParam* table = mavlink_rx_param_table();

    mavlink_message_t msg;
    mavlink_msg_param_set_pack(kGcsSysid, kGcsCompid, &msg,
                               kVehSysid, kVehCompid,
                               table[0].name,
                               999.0f,  // attempt to change
                               MAV_PARAM_TYPE_REAL32);
    feed_msg(msg);

    EXPECT_EQ(count_messages(MAVLINK_MSG_ID_PARAM_VALUE), 1);

    uint16_t offset = 0;
    mavlink_param_value_t pv;
    ASSERT_TRUE(decode_param_value(offset, pv));
    // Value should be unchanged (read-only)
    EXPECT_FLOAT_EQ(pv.param_value, table[0].value);
}

// ============================================================================
// Test 6: CommandAutoVersion — AUTOPILOT_CAPABILITIES → ACCEPTED + AUTOPILOT_VERSION
// ============================================================================

TEST_F(MavlinkRxTest, CommandAutoVersion) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(kGcsSysid, kGcsCompid, &msg,
                                  kVehSysid, kVehCompid,
                                  MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                                  0, 1, 0, 0, 0, 0, 0, 0);
    feed_msg(msg);

    // Should have AUTOPILOT_VERSION + COMMAND_ACK
    EXPECT_EQ(count_messages(MAVLINK_MSG_ID_AUTOPILOT_VERSION), 1);
    EXPECT_EQ(count_messages(MAVLINK_MSG_ID_COMMAND_ACK), 1);

    uint16_t offset = 0;
    mavlink_command_ack_t ack;
    ASSERT_TRUE(decode_command_ack(offset, ack));
    EXPECT_EQ(ack.command, MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES);
    EXPECT_EQ(ack.result, MAV_RESULT_ACCEPTED);
}

// ============================================================================
// Test 7: CommandUnsupported — unknown cmd → UNSUPPORTED ACK
// ============================================================================

TEST_F(MavlinkRxTest, CommandUnsupported) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(kGcsSysid, kGcsCompid, &msg,
                                  kVehSysid, kVehCompid,
                                  9999,  // unknown command
                                  0, 0, 0, 0, 0, 0, 0, 0);
    feed_msg(msg);

    uint16_t offset = 0;
    mavlink_command_ack_t ack;
    ASSERT_TRUE(decode_command_ack(offset, ack));
    EXPECT_EQ(ack.command, 9999);
    EXPECT_EQ(ack.result, MAV_RESULT_UNSUPPORTED);
}

// ============================================================================
// Test 8: CommandSetMode — DO_SET_MODE → ACCEPTED when IDLE
// ============================================================================

TEST_F(MavlinkRxTest, CommandSetModeIdle) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(kGcsSysid, kGcsCompid, &msg,
                                  kVehSysid, kVehCompid,
                                  MAV_CMD_DO_SET_MODE,
                                  0, 1, 0, 0, 0, 0, 0, 0);
    uint8_t idle = static_cast<uint8_t>(rc::FlightPhase::kIdle);
    feed_msg(msg, idle);

    uint16_t offset = 0;
    mavlink_command_ack_t ack;
    ASSERT_TRUE(decode_command_ack(offset, ack));
    EXPECT_EQ(ack.result, MAV_RESULT_ACCEPTED);

    // Now try when NOT idle — should be DENIED
    memset(&result, 0, sizeof(result));
    mavlink_msg_command_long_pack(kGcsSysid, kGcsCompid, &msg,
                                  kVehSysid, kVehCompid,
                                  MAV_CMD_DO_SET_MODE,
                                  0, 1, 0, 0, 0, 0, 0, 0);
    uint8_t armed = static_cast<uint8_t>(rc::FlightPhase::kArmed);
    feed_msg(msg, armed);

    offset = 0;
    ASSERT_TRUE(decode_command_ack(offset, ack));
    EXPECT_EQ(ack.result, MAV_RESULT_DENIED);
}

// ============================================================================
// Test 9: CommandArmDisarm — COMPONENT_ARM_DISARM → ACCEPTED (no-op)
// ============================================================================

TEST_F(MavlinkRxTest, CommandArmDisarm) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(kGcsSysid, kGcsCompid, &msg,
                                  kVehSysid, kVehCompid,
                                  MAV_CMD_COMPONENT_ARM_DISARM,
                                  0, 1, 0, 0, 0, 0, 0, 0);
    feed_msg(msg);

    uint16_t offset = 0;
    mavlink_command_ack_t ack;
    ASSERT_TRUE(decode_command_ack(offset, ack));
    EXPECT_EQ(ack.result, MAV_RESULT_ACCEPTED);
}

// ============================================================================
// Test 10: CommandRebootDenied — PREFLIGHT_REBOOT → DENIED
// ============================================================================

TEST_F(MavlinkRxTest, CommandRebootDenied) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(kGcsSysid, kGcsCompid, &msg,
                                  kVehSysid, kVehCompid,
                                  MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                  0, 0, 0, 0, 0, 0, 0, 0);
    feed_msg(msg);

    uint16_t offset = 0;
    mavlink_command_ack_t ack;
    ASSERT_TRUE(decode_command_ack(offset, ack));
    EXPECT_EQ(ack.result, MAV_RESULT_DENIED);
}

// ============================================================================
// Test 11: MissionRequestList — MISSION_COUNT(0) response
// ============================================================================

TEST_F(MavlinkRxTest, MissionRequestList) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_mission_request_list_pack(kGcsSysid, kGcsCompid, &msg,
                                           kVehSysid, kVehCompid,
                                           MAV_MISSION_TYPE_MISSION);
    feed_msg(msg);

    EXPECT_EQ(count_messages(MAVLINK_MSG_ID_MISSION_COUNT), 1);

    // Decode MISSION_COUNT and verify count=0
    mavlink_message_t rx_msg;
    mavlink_status_t rx_status;
    memset(&rx_status, 0, sizeof(rx_status));
    for (uint16_t i = 0; i < result.len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, result.buf[i], &rx_msg, &rx_status)) {
            if (rx_msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
                mavlink_mission_count_t mc;
                mavlink_msg_mission_count_decode(&rx_msg, &mc);
                EXPECT_EQ(mc.count, 0);
            }
        }
    }
}

// ============================================================================
// Test 12: SetModeMessage — legacy msg #11 → ACK
// ============================================================================

TEST_F(MavlinkRxTest, SetModeMessage) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(kGcsSysid, kGcsCompid, &msg,
                               kVehSysid, 0, 0);
    uint8_t idle = static_cast<uint8_t>(rc::FlightPhase::kIdle);
    feed_msg(msg, idle);

    uint16_t offset = 0;
    mavlink_command_ack_t ack;
    ASSERT_TRUE(decode_command_ack(offset, ack));
    EXPECT_EQ(ack.command, MAV_CMD_DO_SET_MODE);
    EXPECT_EQ(ack.result, MAV_RESULT_ACCEPTED);
}

// ============================================================================
// Test 13: GarbageRecovery — non-MAVLink bytes don't break parser
// ============================================================================

TEST_F(MavlinkRxTest, GarbageRecovery) {
    // Feed random garbage (avoid 0xFD which is MAVLink v2 start byte)
    uint8_t garbage[] = {0x01, 0x02, 0xFF, 0xFE, 0x00, 0xAB, 0xCD};
    for (auto b : garbage) {
        EXPECT_FALSE(mavlink_rx_feed_byte(&state, b, 0, 0, &result));
    }
    EXPECT_EQ(result.len, 0);

    // Feed a valid heartbeat — parser should recover and parse it.
    // Send twice: first may be consumed as noise if parser was mid-frame
    // from 0xFE (MAVLink v1 start byte) in the garbage.
    send_gcs_heartbeat();
    if (!state.gcs_seen) {
        send_gcs_heartbeat();
    }
    EXPECT_TRUE(state.gcs_seen);
}

// ============================================================================
// Test 14: BackToBackFrames — two commands in sequence both handled
// ============================================================================

TEST_F(MavlinkRxTest, BackToBackFrames) {
    send_gcs_heartbeat();
    memset(&result, 0, sizeof(result));

    // First command: ARM/DISARM
    mavlink_message_t msg1;
    mavlink_msg_command_long_pack(kGcsSysid, kGcsCompid, &msg1,
                                  kVehSysid, kVehCompid,
                                  MAV_CMD_COMPONENT_ARM_DISARM,
                                  0, 1, 0, 0, 0, 0, 0, 0);
    feed_msg(msg1);

    // Second command: PREFLIGHT_CALIBRATION
    mavlink_message_t msg2;
    mavlink_msg_command_long_pack(kGcsSysid, kGcsCompid, &msg2,
                                  kVehSysid, kVehCompid,
                                  MAV_CMD_PREFLIGHT_CALIBRATION,
                                  0, 0, 0, 0, 0, 0, 0, 0);
    feed_msg(msg2);

    // Should have 2 COMMAND_ACK responses
    EXPECT_EQ(count_messages(MAVLINK_MSG_ID_COMMAND_ACK), 2);

    // Decode both ACKs
    uint16_t offset = 0;
    mavlink_command_ack_t ack1, ack2;
    ASSERT_TRUE(decode_command_ack(offset, ack1));
    ASSERT_TRUE(decode_command_ack(offset, ack2));

    EXPECT_EQ(ack1.command, MAV_CMD_COMPONENT_ARM_DISARM);
    EXPECT_EQ(ack1.result, MAV_RESULT_ACCEPTED);

    EXPECT_EQ(ack2.command, MAV_CMD_PREFLIGHT_CALIBRATION);
    EXPECT_EQ(ack2.result, MAV_RESULT_UNSUPPORTED);
}
