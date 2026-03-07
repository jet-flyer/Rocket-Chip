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
// MAVLink Encoder Stub Tests
// ============================================================================

TEST(MavlinkEncoderTest, StubProducesCorrectLength) {
    MavlinkEncoder enc;
    enc.init();
    EncodeResult result{};
    TelemetryState telem = make_test_telem();
    enc.encode_nav(telem, 12345, result);
    EXPECT_EQ(result.len, 105);
    // Stub returns ok=false until fastmavlink is integrated
    EXPECT_FALSE(result.ok);
}

TEST(MavlinkEncoderTest, MaxPacketSize) {
    EXPECT_EQ(MavlinkEncoder::max_packet_size(), 105);
}

TEST(MavlinkEncoderTest, FitsInLoRaPayload) {
    EXPECT_LE(MavlinkEncoder::max_packet_size(), 128)
        << "Must fit in SX1276 max payload";
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
    EXPECT_EQ(result.len, 105);
}

TEST(TelemetryEncoderStateTest, MaxPacketSize) {
    TelemetryEncoderState state;
    state.init(EncoderType::kCcsds);
    EXPECT_EQ(state.max_packet_size(), 54);
    state.init(EncoderType::kMavlink);
    EXPECT_EQ(state.max_packet_size(), 105);
}
