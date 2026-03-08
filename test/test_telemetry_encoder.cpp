// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_telemetry_encoder.cpp
 * @brief Host-side tests for CCSDS and MAVLink telemetry encoders
 *
 * IVP-58: Telemetry Encoder (Stage 7: Radio & Telemetry)
 */

#include <gtest/gtest.h>
#include "rocketchip/telemetry_encoder.h"
#include "crc16_ccitt.h"
#include <cstring>
#include <cmath>

extern "C" {
#include "common/mavlink.h"
}

using namespace rc;

// Helper: create a populated TelemetryState for testing
static TelemetryState make_test_telem() {
    TelemetryState t{};
    // Quaternion: identity (1,0,0,0) in Q15
    t.q_w = 32767;
    t.q_x = 0;
    t.q_y = 0;
    t.q_z = 0;
    // GPS: roughly Seattle
    t.lat_1e7 = 474000000;    // 47.4 deg N
    t.lon_1e7 = -1222000000;  // -122.2 deg W
    t.alt_mm  = 100000;       // 100m MSL
    // Velocity: 1 m/s north
    t.vel_n_cms = 100;
    t.vel_e_cms = 0;
    t.vel_d_cms = 0;
    // Baro
    t.baro_alt_mm   = 50000;  // 50m AGL
    t.baro_vvel_cms = 0;
    // GPS status
    t.gps_speed_cms = 100;    // 1 m/s ground speed
    t.gps_fix_sats  = 0x3C;  // fix=3, sats=12
    t.flight_state  = 0;      // IDLE
    t.health        = 0x01;   // ESKF healthy
    t.temperature_c = 22;
    t.battery_mv    = 3700;
    t.met_ms        = 12345;
    t._reserved     = 0;
    return t;
}

// ============================================================================
// CCSDS Encoder Tests
// ============================================================================

class CcsdsEncoderTest : public ::testing::Test {
protected:
    CcsdsEncoder enc;
    TelemetryState telem;
    EncodeResult result;

    void SetUp() override {
        enc.init();
        telem = make_test_telem();
        memset(&result, 0, sizeof(result));
    }
};

TEST_F(CcsdsEncoderTest, PacketSize) {
    enc.encode_nav(telem, 12345, result);
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(result.len, 54);
}

TEST_F(CcsdsEncoderTest, MaxPacketSize) {
    EXPECT_EQ(CcsdsEncoder::max_packet_size(), 54);
}

TEST_F(CcsdsEncoderTest, PrimaryHeaderVersion) {
    enc.encode_nav(telem, 12345, result);
    // Bits [2:0] of byte 0 upper nibble = version 000
    // Byte 0 upper 3 bits should be 000
    EXPECT_EQ(result.buf[0] & 0xE0, 0x00) << "Version should be 000";
}

TEST_F(CcsdsEncoderTest, PrimaryHeaderType) {
    enc.encode_nav(telem, 12345, result);
    // Bit 3 (0x10 in byte 0) = Type = 0 (telemetry)
    EXPECT_EQ(result.buf[0] & 0x10, 0x00) << "Type should be 0 (telemetry)";
}

TEST_F(CcsdsEncoderTest, PrimaryHeaderSecHdrFlag) {
    enc.encode_nav(telem, 12345, result);
    // Bit 4 (0x08 in byte 0) = Secondary Header Flag = 1
    EXPECT_EQ(result.buf[0] & 0x08, 0x08) << "SecHdrFlag should be 1";
}

TEST_F(CcsdsEncoderTest, PrimaryHeaderApid) {
    enc.encode_nav(telem, 12345, result);
    // APID in bits [15:5] = lower 3 bits of byte 0 + all of byte 1
    uint16_t apid = static_cast<uint16_t>(
        ((result.buf[0] & 0x07) << 8) | result.buf[1]);
    EXPECT_EQ(apid, ccsds::kApidNav) << "APID should be 0x001 (nav)";
}

TEST_F(CcsdsEncoderTest, PrimaryHeaderSeqFlags) {
    enc.encode_nav(telem, 12345, result);
    // SeqFlags in bits [17:16] = upper 2 bits of byte 2 = 11 (unsegmented)
    EXPECT_EQ(result.buf[2] & 0xC0, 0xC0) << "SeqFlags should be 11 (unsegmented)";
}

TEST_F(CcsdsEncoderTest, SequenceCounterIncrements) {
    enc.encode_nav(telem, 100, result);
    // Extract 14-bit seq count from bytes 2-3
    uint16_t seq0 = static_cast<uint16_t>(
        ((result.buf[2] & 0x3F) << 8) | result.buf[3]);
    EXPECT_EQ(seq0, 0);

    enc.encode_nav(telem, 200, result);
    uint16_t seq1 = static_cast<uint16_t>(
        ((result.buf[2] & 0x3F) << 8) | result.buf[3]);
    EXPECT_EQ(seq1, 1);

    enc.encode_nav(telem, 300, result);
    uint16_t seq2 = static_cast<uint16_t>(
        ((result.buf[2] & 0x3F) << 8) | result.buf[3]);
    EXPECT_EQ(seq2, 2);
}

TEST_F(CcsdsEncoderTest, SequenceCounterWraps14Bit) {
    // Set counter near wrap point
    enc.seq_count = 0x3FFD;  // 16381

    enc.encode_nav(telem, 100, result);
    uint16_t seq0 = static_cast<uint16_t>(
        ((result.buf[2] & 0x3F) << 8) | result.buf[3]);
    EXPECT_EQ(seq0, 0x3FFD);

    enc.encode_nav(telem, 200, result);
    uint16_t seq1 = static_cast<uint16_t>(
        ((result.buf[2] & 0x3F) << 8) | result.buf[3]);
    EXPECT_EQ(seq1, 0x3FFE);

    enc.encode_nav(telem, 300, result);
    uint16_t seq2 = static_cast<uint16_t>(
        ((result.buf[2] & 0x3F) << 8) | result.buf[3]);
    EXPECT_EQ(seq2, 0x3FFF);

    // Should wrap to 0
    enc.encode_nav(telem, 400, result);
    uint16_t seq3 = static_cast<uint16_t>(
        ((result.buf[2] & 0x3F) << 8) | result.buf[3]);
    EXPECT_EQ(seq3, 0x0000);
}

TEST_F(CcsdsEncoderTest, DataLengthField) {
    enc.encode_nav(telem, 12345, result);
    // Data Length = bytes after primary header - 1
    //            = (4 + 42 + 2) - 1 = 47
    uint16_t data_len = static_cast<uint16_t>(
        (result.buf[4] << 8) | result.buf[5]);
    EXPECT_EQ(data_len, 47);
}

TEST_F(CcsdsEncoderTest, SecondaryHeaderMet) {
    uint32_t met = 0x12345678;
    enc.encode_nav(telem, met, result);
    // MET in bytes 6-9 (big-endian)
    EXPECT_EQ(result.buf[6], 0x12);
    EXPECT_EQ(result.buf[7], 0x34);
    EXPECT_EQ(result.buf[8], 0x56);
    EXPECT_EQ(result.buf[9], 0x78);
}

TEST_F(CcsdsEncoderTest, CrcValidates) {
    enc.encode_nav(telem, 12345, result);
    ASSERT_EQ(result.len, 54);

    // CRC is over bytes 0-51, stored big-endian in bytes 52-53
    uint16_t computed_crc = crc16_ccitt(result.buf, 52);
    uint16_t stored_crc = static_cast<uint16_t>(
        (result.buf[52] << 8) | result.buf[53]);
    EXPECT_EQ(computed_crc, stored_crc);
}

TEST_F(CcsdsEncoderTest, CrcDetectsCorruption) {
    enc.encode_nav(telem, 12345, result);

    // Flip a bit in the payload
    result.buf[20] ^= 0x01;

    uint16_t computed_crc = crc16_ccitt(result.buf, 52);
    uint16_t stored_crc = static_cast<uint16_t>(
        (result.buf[52] << 8) | result.buf[53]);
    EXPECT_NE(computed_crc, stored_crc);
}

TEST_F(CcsdsEncoderTest, PayloadContainsQuaternion) {
    enc.encode_nav(telem, 12345, result);
    // Payload starts at byte 10 (after 6B primary + 4B secondary)
    // First field: q_w as int16 (little-endian, same as TelemetryState)
    int16_t q_w;
    memcpy(&q_w, &result.buf[10], sizeof(q_w));
    EXPECT_EQ(q_w, 32767) << "q_w should be identity quaternion (Q15)";
}

TEST_F(CcsdsEncoderTest, PayloadContainsGps) {
    enc.encode_nav(telem, 12345, result);
    // lat_1e7 starts at payload offset 8 (after 4x int16 quaternion)
    // = byte 10 + 8 = byte 18
    int32_t lat;
    memcpy(&lat, &result.buf[18], sizeof(lat));
    EXPECT_EQ(lat, 474000000);
}

TEST_F(CcsdsEncoderTest, FitsInLoRaPayload) {
    EXPECT_LE(CcsdsEncoder::max_packet_size(), 128)
        << "Must fit in SX1276 max payload";
}

// ============================================================================
// MAVLink Encoder Tests (IVP-61: c_library_v2)
// ============================================================================

class MavlinkEncoderTest : public ::testing::Test {
protected:
    MavlinkEncoder enc;
    TelemetryState telem;
    uint8_t frame[MAVLINK_MAX_PACKET_LEN];

    void SetUp() override {
        enc.init();
        telem = make_test_telem();
        memset(frame, 0, sizeof(frame));
    }
};

TEST_F(MavlinkEncoderTest, HeartbeatWireFormat) {
    uint16_t len = enc.encode_heartbeat(0, frame);
    EXPECT_GT(len, 0);
    EXPECT_EQ(frame[0], 0xFD) << "MAVLink v2 magic byte";
    EXPECT_EQ(frame[1], 9) << "HEARTBEAT payload length";
    // msgid = 0 (HEARTBEAT) in bytes 7-9
    EXPECT_EQ(frame[7], 0);
    EXPECT_EQ(frame[8], 0);
    EXPECT_EQ(frame[9], 0);
}

TEST_F(MavlinkEncoderTest, HeartbeatFlightStateMapping) {
    // IDLE → STANDBY (3)
    EXPECT_EQ(flight_state_to_mav_state(0), MAV_STATE_STANDBY);
    // ARMED → ACTIVE (4)
    EXPECT_EQ(flight_state_to_mav_state(1), MAV_STATE_ACTIVE);
    // BOOST → ACTIVE (4)
    EXPECT_EQ(flight_state_to_mav_state(2), MAV_STATE_ACTIVE);
    // COAST → ACTIVE (4)
    EXPECT_EQ(flight_state_to_mav_state(3), MAV_STATE_ACTIVE);
    // DESCENT → ACTIVE (4)
    EXPECT_EQ(flight_state_to_mav_state(4), MAV_STATE_ACTIVE);
    // LANDED → STANDBY (3)
    EXPECT_EQ(flight_state_to_mav_state(5), MAV_STATE_STANDBY);
    // ERROR → CRITICAL (5)
    EXPECT_EQ(flight_state_to_mav_state(6), MAV_STATE_CRITICAL);
    // Unknown → BOOT (1)
    EXPECT_EQ(flight_state_to_mav_state(255), MAV_STATE_BOOT);
}

TEST_F(MavlinkEncoderTest, SysStatusWireFormat) {
    uint16_t len = enc.encode_sys_status(telem, frame);
    EXPECT_GT(len, 0);
    EXPECT_EQ(frame[0], 0xFD);
    // msgid = 1 (SYS_STATUS)
    EXPECT_EQ(frame[7], 1);
    EXPECT_EQ(frame[8], 0);
    EXPECT_EQ(frame[9], 0);
}

TEST_F(MavlinkEncoderTest, AttitudeIdentityQuaternion) {
    // Identity quaternion (1,0,0,0) → roll=0, pitch=0, yaw=0
    telem.q_w = 32767;
    telem.q_x = 0;
    telem.q_y = 0;
    telem.q_z = 0;
    uint16_t len = enc.encode_attitude(telem, 1000, frame);
    EXPECT_GT(len, 0);

    // Decode the MAVLink frame to extract payload
    mavlink_message_t msg;
    mavlink_status_t status;
    for (uint16_t i = 0; i < len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, frame[i], &msg, &status)) {
            EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_ATTITUDE);
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);
            EXPECT_NEAR(att.roll, 0.0F, 0.001F);
            EXPECT_NEAR(att.pitch, 0.0F, 0.001F);
            EXPECT_NEAR(att.yaw, 0.0F, 0.001F);
        }
    }
}

TEST_F(MavlinkEncoderTest, Attitude45DegRoll) {
    // 45-degree roll: q = cos(22.5°) + sin(22.5°)*i
    float half = 22.5F * 3.14159265F / 180.0F;
    telem.q_w = static_cast<int16_t>(cosf(half) * 32767.0F);
    telem.q_x = static_cast<int16_t>(sinf(half) * 32767.0F);
    telem.q_y = 0;
    telem.q_z = 0;
    uint16_t len = enc.encode_attitude(telem, 1000, frame);

    mavlink_message_t msg;
    mavlink_status_t status;
    for (uint16_t i = 0; i < len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_1, frame[i], &msg, &status)) {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);
            EXPECT_NEAR(att.roll, 3.14159265F / 4.0F, 0.01F)
                << "Roll should be ~pi/4 radians";
        }
    }
}

TEST_F(MavlinkEncoderTest, AttitudeWireFormat) {
    uint16_t len = enc.encode_attitude(telem, 1000, frame);
    EXPECT_GT(len, 0);
    EXPECT_EQ(frame[0], 0xFD);
    // msgid = 30 (ATTITUDE)
    EXPECT_EQ(frame[7], 30);
}

TEST_F(MavlinkEncoderTest, GlobalPosIntWireFormat) {
    uint16_t len = enc.encode_global_pos(telem, 1000, frame);
    EXPECT_GT(len, 0);
    EXPECT_EQ(frame[0], 0xFD);
    // msgid = 33 (GLOBAL_POSITION_INT)
    EXPECT_EQ(frame[7], 33);
}

TEST_F(MavlinkEncoderTest, GlobalPosNoGps) {
    // No GPS fix → hdg=UINT16_MAX, lat/lon=0
    telem.gps_fix_sats = 0x0C;  // fix=0, sats=12
    uint16_t len = enc.encode_global_pos(telem, 1000, frame);

    mavlink_message_t msg;
    mavlink_status_t status;
    for (uint16_t i = 0; i < len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_2, frame[i], &msg, &status)) {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);
            EXPECT_EQ(pos.lat, 0);
            EXPECT_EQ(pos.lon, 0);
            EXPECT_EQ(pos.hdg, UINT16_MAX);
        }
    }
}

TEST_F(MavlinkEncoderTest, SequenceMonotonic) {
    // 4 messages: HB, SYS, ATT, POS → seq 0, 1, 2, 3
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    enc.encode_heartbeat(0, buf);
    EXPECT_EQ(enc.seq, 1);
    enc.encode_sys_status(telem, buf);
    EXPECT_EQ(enc.seq, 2);
    enc.encode_attitude(telem, 1000, buf);
    EXPECT_EQ(enc.seq, 3);
    enc.encode_global_pos(telem, 1000, buf);
    EXPECT_EQ(enc.seq, 4);
}

TEST_F(MavlinkEncoderTest, EncodeNavFourFrames) {
    EncodeResult result{};
    enc.encode_nav(telem, 12345, result);
    EXPECT_TRUE(result.ok);
    // Walk MAVLink v2 frame structure: header(10) + payload(N) + CRC(2)
    int frame_count = 0;
    uint16_t pos = 0;
    while (pos < result.len) {
        ASSERT_EQ(result.buf[pos], 0xFD) << "Frame " << frame_count << " should start with 0xFD";
        uint8_t payload_len = result.buf[pos + 1];
        pos += 10 + payload_len + 2;  // header + payload + CRC
        frame_count++;
    }
    EXPECT_EQ(frame_count, 4) << "encode_nav should produce 4 MAVLink frames";
    EXPECT_EQ(pos, result.len) << "Total frame bytes should match result.len";
}

TEST_F(MavlinkEncoderTest, BinaryIntegrity0x0A) {
    // Verify 0x0A (newline) bytes in payload don't break MAVLink framing.
    // The frame is raw binary — no CR/LF translation should occur.
    telem.vel_n_cms = 0x0A0A;  // Contains 0x0A bytes

    MavlinkEncoder fresh;
    fresh.init();
    EncodeResult result{};
    fresh.encode_nav(telem, 1000, result);
    EXPECT_TRUE(result.ok);
    EXPECT_GT(result.len, 0);

    // Walk MAVLink v2 frame structure to count frames
    int frame_count = 0;
    uint16_t pos = 0;
    while (pos < result.len) {
        ASSERT_EQ(result.buf[pos], 0xFD) << "Frame " << frame_count << " missing magic";
        uint8_t payload_len = result.buf[pos + 1];
        pos += 10 + payload_len + 2;
        frame_count++;
    }
    EXPECT_EQ(frame_count, 4)
        << "All 4 MAVLink frames should encode with 0x0A bytes in payload";
    EXPECT_EQ(pos, result.len);

    // Verify the 0x0A byte exists in the raw buffer (not stripped/translated)
    bool found_0a = false;
    for (uint16_t i = 0; i < result.len; i++) {
        if (result.buf[i] == 0x0A) {
            found_0a = true;
            break;
        }
    }
    EXPECT_TRUE(found_0a) << "0x0A byte should be preserved in binary output";
}

// ============================================================================
// Strategy Wrapper Tests
// ============================================================================

TEST(TelemetryEncoderStateTest, CcsdsSelection) {
    TelemetryEncoderState state;
    state.init(EncoderType::kCcsds);
    EncodeResult result{};
    TelemetryState telem = make_test_telem();
    state.encode_nav(telem, 12345, result);
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(result.len, 54);
}

TEST(TelemetryEncoderStateTest, MavlinkSelection) {
    TelemetryEncoderState state;
    state.init(EncoderType::kMavlink);
    EncodeResult result{};
    TelemetryState telem = make_test_telem();
    state.encode_nav(telem, 12345, result);
    EXPECT_TRUE(result.ok);
    EXPECT_GT(result.len, 0);
}

TEST(TelemetryEncoderStateTest, MaxPacketSize) {
    TelemetryEncoderState state;
    state.init(EncoderType::kCcsds);
    EXPECT_EQ(state.max_packet_size(), 54);
    state.init(EncoderType::kMavlink);
    EXPECT_EQ(state.max_packet_size(), 144);
}

// ============================================================================
// CCSDS Decoder Tests (IVP-60)
// ============================================================================

class CcsdsDecoderTest : public ::testing::Test {
protected:
    CcsdsEncoder enc;
    TelemetryState telem_in;
    EncodeResult encoded;

    void SetUp() override {
        enc.init();
        telem_in = make_test_telem();
        memset(&encoded, 0, sizeof(encoded));
        enc.encode_nav(telem_in, telem_in.met_ms, encoded);
    }
};

TEST_F(CcsdsDecoderTest, RoundTripIdentity) {
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    ASSERT_TRUE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
    EXPECT_EQ(seq, 0);
    EXPECT_EQ(met, 12345u);
}

TEST_F(CcsdsDecoderTest, RoundTripQuaternion) {
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    ASSERT_TRUE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
    EXPECT_EQ(decoded.q_w, telem_in.q_w);
    EXPECT_EQ(decoded.q_x, telem_in.q_x);
    EXPECT_EQ(decoded.q_y, telem_in.q_y);
    EXPECT_EQ(decoded.q_z, telem_in.q_z);
}

TEST_F(CcsdsDecoderTest, RoundTripGps) {
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    ASSERT_TRUE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
    EXPECT_EQ(decoded.lat_1e7, telem_in.lat_1e7);
    EXPECT_EQ(decoded.lon_1e7, telem_in.lon_1e7);
    EXPECT_EQ(decoded.alt_mm, telem_in.alt_mm);
}

TEST_F(CcsdsDecoderTest, RoundTripVelocity) {
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    ASSERT_TRUE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
    EXPECT_EQ(decoded.vel_n_cms, telem_in.vel_n_cms);
    EXPECT_EQ(decoded.vel_e_cms, telem_in.vel_e_cms);
    EXPECT_EQ(decoded.vel_d_cms, telem_in.vel_d_cms);
}

TEST_F(CcsdsDecoderTest, RoundTripBaro) {
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    ASSERT_TRUE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
    EXPECT_EQ(decoded.baro_alt_mm, telem_in.baro_alt_mm);
    EXPECT_EQ(decoded.baro_vvel_cms, telem_in.baro_vvel_cms);
}

TEST_F(CcsdsDecoderTest, RoundTripStatus) {
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    ASSERT_TRUE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
    EXPECT_EQ(decoded.gps_speed_cms, telem_in.gps_speed_cms);
    EXPECT_EQ(decoded.gps_fix_sats, telem_in.gps_fix_sats);
    EXPECT_EQ(decoded.flight_state, telem_in.flight_state);
    EXPECT_EQ(decoded.health, telem_in.health);
    EXPECT_EQ(decoded.temperature_c, telem_in.temperature_c);
    EXPECT_EQ(decoded.battery_mv, telem_in.battery_mv);
}

TEST_F(CcsdsDecoderTest, RoundTripMet) {
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    ASSERT_TRUE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
    EXPECT_EQ(met, telem_in.met_ms);
    EXPECT_EQ(decoded.met_ms, telem_in.met_ms);
}

TEST_F(CcsdsDecoderTest, SequenceCounterPreserved) {
    // Encode 3 packets, decode each, verify seq
    for (uint16_t i = 0; i < 3; i++) {
        EncodeResult r{};
        enc.encode_nav(telem_in, 1000 + i, r);
        TelemetryState decoded{};
        uint16_t seq = 0;
        uint32_t met = 0;
        ASSERT_TRUE(ccsds_decode_nav(r.buf, r.len, decoded, seq, met));
        EXPECT_EQ(seq, i + 1);  // SetUp already consumed seq 0
    }
}

TEST_F(CcsdsDecoderTest, RejectWrongLength) {
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    EXPECT_FALSE(ccsds_decode_nav(encoded.buf, 53, decoded, seq, met));
    EXPECT_FALSE(ccsds_decode_nav(encoded.buf, 55, decoded, seq, met));
    EXPECT_FALSE(ccsds_decode_nav(encoded.buf, 0, decoded, seq, met));
}

TEST_F(CcsdsDecoderTest, RejectCorruptedCrc) {
    encoded.buf[20] ^= 0x01;  // Flip a payload bit
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    EXPECT_FALSE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
}

TEST_F(CcsdsDecoderTest, RejectWrongApid) {
    // Corrupt APID (byte 1)
    encoded.buf[1] = 0xFF;
    // Recompute CRC so only APID check triggers
    uint16_t crc = crc16_ccitt(encoded.buf, 52);
    encoded.buf[52] = static_cast<uint8_t>(crc >> 8);
    encoded.buf[53] = static_cast<uint8_t>(crc & 0xFF);
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    EXPECT_FALSE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
}

TEST_F(CcsdsDecoderTest, RejectWrongVersion) {
    // Set version bits to non-zero
    encoded.buf[0] |= 0x20;
    uint16_t crc = crc16_ccitt(encoded.buf, 52);
    encoded.buf[52] = static_cast<uint8_t>(crc >> 8);
    encoded.buf[53] = static_cast<uint8_t>(crc & 0xFF);
    TelemetryState decoded{};
    uint16_t seq = 0;
    uint32_t met = 0;
    EXPECT_FALSE(ccsds_decode_nav(encoded.buf, encoded.len, decoded, seq, met));
}
