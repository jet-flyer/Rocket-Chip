// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file telemetry_encoder.cpp
 * @brief CCSDS and MAVLink telemetry encoders
 *
 * CCSDS primary header is big-endian per CCSDS 133.0-B-2 Section 4.1.1.
 * Secondary header contains MET in big-endian uint32.
 * Nav payload is the TelemetryState struct minus met_ms and _reserved
 * (those are in the secondary header or dropped).
 *
 * CRC-16-CCITT covers primary header + secondary header + payload
 * (same polynomial as PCM frames).
 *
 * IVP-58: Telemetry Encoder (Stage 7: Radio & Telemetry)
 */

#include "rocketchip/telemetry_encoder.h"
#include "crc16_ccitt.h"
#include <string.h>
#include <math.h>

// c_library_v2 — official MAVLink C library (header-only)
// common dialect includes standard (GLOBAL_POSITION_INT) and minimal (HEARTBEAT)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wpedantic"
extern "C" {
#include "common/mavlink.h"
}
#pragma GCC diagnostic pop

namespace rc {

// ============================================================================
// CCSDS Encoder
// ============================================================================

void CcsdsEncoder::init() {
    seq_count = 0;
}

// Build 6-byte CCSDS primary header (big-endian)
static void build_primary_header(uint8_t* buf, uint16_t apid,
                                  uint16_t seq_count, uint16_t data_len) {
    // Word 0 (bytes 0-1): Version(3) | Type(1) | SecHdrFlag(1) | APID(11)
    //   Version = 000, Type = 0 (telemetry), SecHdrFlag = 1
    //   Bits: 000 0 1 AAAAAAAAAAA
    uint16_t word0 = static_cast<uint16_t>(0x0800U | (apid & 0x07FFU));
    buf[0] = static_cast<uint8_t>(word0 >> 8);
    buf[1] = static_cast<uint8_t>(word0 & 0xFF);

    // Word 1 (bytes 2-3): SeqFlags(2) | SeqCount(14)
    //   SeqFlags = 11 (unsegmented)
    uint16_t word1 = static_cast<uint16_t>(0xC000U | (seq_count & 0x3FFFU));
    buf[2] = static_cast<uint8_t>(word1 >> 8);
    buf[3] = static_cast<uint8_t>(word1 & 0xFF);

    // Word 2 (bytes 4-5): Data Length = (total octets after primary header) - 1
    buf[4] = static_cast<uint8_t>(data_len >> 8);
    buf[5] = static_cast<uint8_t>(data_len & 0xFF);
}

// Build 4-byte secondary header (MET ms, big-endian)
static void build_secondary_header(uint8_t* buf, uint32_t met_ms) {
    buf[0] = static_cast<uint8_t>((met_ms >> 24) & 0xFF);
    buf[1] = static_cast<uint8_t>((met_ms >> 16) & 0xFF);
    buf[2] = static_cast<uint8_t>((met_ms >>  8) & 0xFF);
    buf[3] = static_cast<uint8_t>( met_ms        & 0xFF);
}

void CcsdsEncoder::encode_nav(const TelemetryState& telem, uint32_t met_ms,
                               EncodeResult& result) {
    // Data length field = secondary_header + payload + CRC - 1
    uint16_t data_len = static_cast<uint16_t>(
        ccsds::kSecondaryHeaderLen + ccsds::kNavPayloadLen + ccsds::kCrcLen - 1);

    uint8_t* p = result.buf;

    // Primary header (6 bytes)
    build_primary_header(p, ccsds::kApidNav, seq_count, data_len);
    p += ccsds::kPrimaryHeaderLen;

    // Secondary header (4 bytes — MET)
    build_secondary_header(p, met_ms);
    p += ccsds::kSecondaryHeaderLen;

    // Nav payload: TelemetryState excluding met_ms (already in secondary header)
    // and _reserved. Copy the first 40 bytes (up to and including battery_mv),
    // which is everything before met_ms.
    //
    // TelemetryState layout:
    //   bytes  0-39: q_w through battery_mv  (40 bytes)
    //   bytes 40-43: met_ms                  (4 bytes — in secondary header)
    //   byte  44:    _reserved               (1 byte — dropped)
    //
    // We send 42 bytes: the first 40 bytes of TelemetryState + 2 bytes padding
    // to maintain fixed payload size. The padding bytes are zeroed.
    const uint8_t* telem_bytes = reinterpret_cast<const uint8_t*>(&telem);
    memcpy(p, telem_bytes, 40);
    p[40] = 0;    // Padding byte 1
    p[41] = 0;    // Padding byte 2
    p += ccsds::kNavPayloadLen;

    // CRC-16-CCITT over everything before the CRC (primary + secondary + payload)
    uint32_t crc_len = static_cast<uint32_t>(p - result.buf);
    uint16_t crc = crc16_ccitt(result.buf, crc_len);
    p[0] = static_cast<uint8_t>(crc >> 8);     // Big-endian CRC
    p[1] = static_cast<uint8_t>(crc & 0xFF);
    p += ccsds::kCrcLen;

    result.len = static_cast<uint16_t>(p - result.buf);
    result.ok  = true;

    // Advance 14-bit sequence counter (wraps at 16383)
    seq_count = static_cast<uint16_t>((seq_count + 1) & 0x3FFFU);
}

// ============================================================================
// MAVLink Encoder (c_library_v2 — IVP-61)
// ============================================================================

// Flight state → MAV_STATE mapping
// IDLE/LANDED=STANDBY, ARMED/BOOST/COAST/DESCENT=ACTIVE, ERROR=CRITICAL, other=BOOT
uint8_t flight_state_to_mav_state(uint8_t flight_state) {
    switch (flight_state) {
        case 0: return MAV_STATE_STANDBY;   // IDLE
        case 1: return MAV_STATE_ACTIVE;    // ARMED
        case 2: return MAV_STATE_ACTIVE;    // BOOST
        case 3: return MAV_STATE_ACTIVE;    // COAST
        case 4: return MAV_STATE_ACTIVE;    // DESCENT
        case 5: return MAV_STATE_STANDBY;   // LANDED
        case 6: return MAV_STATE_CRITICAL;  // ERROR
        default: return MAV_STATE_BOOT;     // pre-convergence
    }
}

void MavlinkEncoder::init(uint8_t sysid, uint8_t compid) {
    system_id    = sysid;
    component_id = compid;
    seq          = 0;
}

uint16_t MavlinkEncoder::encode_heartbeat(uint8_t flight_state, uint8_t* buf) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        system_id, component_id, &msg,
        MAV_TYPE_ROCKET,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        static_cast<uint32_t>(flight_state),
        flight_state_to_mav_state(flight_state));
    msg.seq = seq++;
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t MavlinkEncoder::encode_sys_status(const TelemetryState& telem,
                                            uint8_t* buf) {
    // Sensor present/enabled/health bitmasks
    // Report 3D_ACCEL + 3D_GYRO + 3D_MAG + ABSOLUTE_PRESSURE
    uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_ACCEL
                     | MAV_SYS_STATUS_SENSOR_3D_GYRO
                     | MAV_SYS_STATUS_SENSOR_3D_MAG
                     | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    uint32_t health = sensors;  // All healthy (simplified)
    if ((telem.health & kHealthEskfHealthy) == 0) {
        health = 0;  // ESKF unhealthy — report no sensors healthy
    }

    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(
        system_id, component_id, &msg,
        sensors,        // onboard_control_sensors_present
        sensors,        // onboard_control_sensors_enabled
        health,         // onboard_control_sensors_health
        0,              // load (0 = unknown)
        static_cast<uint16_t>(telem.battery_mv),  // voltage_battery (mV)
        -1,             // current_battery (-1 = unknown)
        -1,             // battery_remaining (-1 = unknown)
        0, 0, 0, 0, 0, 0,  // drop/error rates (unused)
        0, 0, 0);       // extended sensor fields (unused)
    msg.seq = seq++;
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t MavlinkEncoder::encode_attitude(const TelemetryState& telem,
                                          uint32_t boot_ms, uint8_t* buf) {
    // Q15 quaternion → Euler angles
    float qw = static_cast<float>(telem.q_w) / 32767.0F;
    float qx = static_cast<float>(telem.q_x) / 32767.0F;
    float qy = static_cast<float>(telem.q_y) / 32767.0F;
    float qz = static_cast<float>(telem.q_z) / 32767.0F;

    float roll  = atan2f(2.0F * (qw * qx + qy * qz),
                         1.0F - 2.0F * (qx * qx + qy * qy));
    float pitch = asinf(2.0F * (qw * qy - qz * qx));
    float yaw   = atan2f(2.0F * (qw * qz + qx * qy),
                         1.0F - 2.0F * (qy * qy + qz * qz));

    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        system_id, component_id, &msg,
        boot_ms,    // time_boot_ms
        roll,       // roll (rad)
        pitch,      // pitch (rad)
        yaw,        // yaw (rad)
        0.0F,       // rollspeed (not available from TelemetryState)
        0.0F,       // pitchspeed
        0.0F);      // yawspeed
    msg.seq = seq++;
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t MavlinkEncoder::encode_global_pos(const TelemetryState& telem,
                                            uint32_t boot_ms, uint8_t* buf) {
    // GPS fix type from upper nibble of gps_fix_sats
    uint8_t fix_type = (telem.gps_fix_sats >> 4) & 0x0F;

    int32_t lat = telem.lat_1e7;
    int32_t lon = telem.lon_1e7;
    int32_t alt = telem.alt_mm;
    int32_t relative_alt = telem.baro_alt_mm;
    int16_t vx = telem.vel_n_cms;   // cm/s — matches MAVLink units
    int16_t vy = telem.vel_e_cms;
    int16_t vz = telem.vel_d_cms;
    // Heading: compute from velocity if GPS has fix, else UINT16_MAX
    uint16_t hdg;
    if (fix_type == 0) {
        // No GPS fix — send invalid heading, zero position
        lat = 0;
        lon = 0;
        alt = 0;
        relative_alt = 0;
        vx = 0;
        vy = 0;
        vz = 0;
        hdg = UINT16_MAX;
    } else {
        // Heading from yaw (same Euler conversion as ATTITUDE)
        float qw = static_cast<float>(telem.q_w) / 32767.0F;
        float qx = static_cast<float>(telem.q_x) / 32767.0F;
        float qy = static_cast<float>(telem.q_y) / 32767.0F;
        float qz = static_cast<float>(telem.q_z) / 32767.0F;
        float yaw = atan2f(2.0F * (qw * qz + qx * qy),
                           1.0F - 2.0F * (qy * qy + qz * qz));
        // Convert rad to centidegrees (0-36000), wrap to positive
        float yaw_deg = yaw * (180.0F / 3.14159265F);
        if (yaw_deg < 0.0F) { yaw_deg += 360.0F; }
        hdg = static_cast<uint16_t>(yaw_deg * 100.0F);
    }

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(
        system_id, component_id, &msg,
        boot_ms,        // time_boot_ms
        lat,            // lat (degE7)
        lon,            // lon (degE7)
        alt,            // alt (mm MSL)
        relative_alt,   // relative_alt (mm AGL)
        vx,             // vx (cm/s north)
        vy,             // vy (cm/s east)
        vz,             // vz (cm/s down)
        hdg);           // hdg (cdeg, UINT16_MAX if unknown)
    msg.seq = seq++;
    return mavlink_msg_to_send_buffer(buf, &msg);
}

void MavlinkEncoder::encode_nav(const TelemetryState& telem, uint32_t met_ms,
                                 EncodeResult& result) {
    uint8_t* p = result.buf;
    p += encode_heartbeat(telem.flight_state, p);
    p += encode_sys_status(telem, p);
    p += encode_attitude(telem, met_ms, p);
    p += encode_global_pos(telem, met_ms, p);
    result.len = static_cast<uint16_t>(p - result.buf);
    result.ok  = true;
}

// ============================================================================
// Strategy Wrapper
// ============================================================================

void TelemetryEncoderState::init(EncoderType encoder_type) {
    type = encoder_type;
    ccsds.init();
    mavlink.init();
}

void TelemetryEncoderState::encode_nav(const TelemetryState& telem,
                                        uint32_t met_ms,
                                        EncodeResult& result) {
    switch (type) {
        case EncoderType::kCcsds:
            ccsds.encode_nav(telem, met_ms, result);
            break;
        case EncoderType::kMavlink:
            mavlink.encode_nav(telem, met_ms, result);
            break;
        default:
            result.ok = false;
            result.len = 0;
            break;
    }
}

uint8_t TelemetryEncoderState::max_packet_size() const {
    switch (type) {
        case EncoderType::kCcsds:   return CcsdsEncoder::max_packet_size();
        case EncoderType::kMavlink: return MavlinkEncoder::max_packet_size();
        default:                    return 0;
    }
}

// ============================================================================
// CCSDS Decoder (IVP-60: RX mode)
// ============================================================================

bool ccsds_decode_nav(const uint8_t* buf, uint8_t len,
                      TelemetryState& telem, uint16_t& seq_out,
                      uint32_t& met_ms_out) {
    // Length check
    if (len != ccsds::kNavPacketLen) {
        return false;
    }

    // Validate primary header: Version=000, Type=0, SecHdrFlag=1
    // Byte 0 upper 5 bits: VVV T S = 000 0 1 = 0x08
    if ((buf[0] & 0xF8) != 0x08) {
        return false;
    }

    // APID check
    uint16_t apid = static_cast<uint16_t>(
        ((buf[0] & 0x07) << 8) | buf[1]);
    if (apid != ccsds::kApidNav) {
        return false;
    }

    // CRC-16 verification: CRC covers bytes 0..51, stored big-endian in 52..53
    uint16_t computed_crc = crc16_ccitt(buf, 52);
    uint16_t stored_crc = static_cast<uint16_t>(
        (buf[52] << 8) | buf[53]);
    if (computed_crc != stored_crc) {
        return false;
    }

    // Extract 14-bit sequence counter from bytes 2-3
    seq_out = static_cast<uint16_t>(
        ((buf[2] & 0x3F) << 8) | buf[3]);

    // Extract MET from secondary header (bytes 6-9, big-endian)
    met_ms_out = (static_cast<uint32_t>(buf[6]) << 24) |
                 (static_cast<uint32_t>(buf[7]) << 16) |
                 (static_cast<uint32_t>(buf[8]) <<  8) |
                  static_cast<uint32_t>(buf[9]);

    // Copy 40-byte nav payload (bytes 10-49) into TelemetryState
    memset(&telem, 0, sizeof(telem));
    memcpy(&telem, &buf[10], 40);

    // Restore met_ms from secondary header
    telem.met_ms = met_ms_out;

    return true;
}

} // namespace rc
