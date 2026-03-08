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

    result.len = static_cast<uint8_t>(p - result.buf);
    result.ok  = true;

    // Advance 14-bit sequence counter (wraps at 16383)
    seq_count = static_cast<uint16_t>((seq_count + 1) & 0x3FFFU);
}

// ============================================================================
// MAVLink Encoder (stub — produces placeholder packets until fastmavlink integrated)
// ============================================================================

void MavlinkEncoder::init(uint8_t sysid, uint8_t compid) {
    system_id    = sysid;
    component_id = compid;
    seq          = 0;
}

void MavlinkEncoder::encode_nav(const TelemetryState& telem, uint32_t met_ms,
                                 EncodeResult& result) {
    // TODO(IVP-58): Replace with fastmavlink encode calls
    // For now, produce a zero-filled buffer at the expected size
    // so the telemetry service can test packet flow and duty cycle.
    memset(result.buf, 0, sizeof(result.buf));
    result.len = 105;
    result.ok  = false;    // false = not a real MAVLink encode yet
    (void)telem;
    (void)met_ms;
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
