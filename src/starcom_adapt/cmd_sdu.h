// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Command / ACK user-data for a Starcom Space Packet SDU (IVP 22).
// APID 0x003 is RC kApidCmdAck (telemetry_encoder.h), not a Starcom MIB.

#ifndef ROCKETCHIP_CMD_SDU_H
#define ROCKETCHIP_CMD_SDU_H

#include <stdint.h>
#include <array>
#include "rocketchip/telemetry_encoder.h"

namespace rc {

// cmd_id u16, seq u8, pad u8, p1..p5 float. MAVLink COMMAND_LONG fields
// already used on LoRa; same numbers, not a new air dialect.
constexpr uint8_t kCmdSduUserBytes = 24;
constexpr uint8_t kAckSduUserBytes = ccsds::kCmdAckPayloadLen;  // 10

struct CmdSduFields {
    uint16_t cmd_id;
    uint8_t seq;
    std::array<float, 5> p;
};

uint8_t pack_cmd_sdu_user(uint8_t* out, uint8_t out_len, const CmdSduFields& f);

bool unpack_cmd_sdu_user(const uint8_t* in, uint8_t in_len, CmdSduFields& f);

uint8_t pack_ack_sdu_user(uint8_t* out, uint8_t out_len,
                          const ccsds::CommandAckPayload& ack);

bool unpack_ack_sdu_user(const uint8_t* in, uint8_t in_len,
                         ccsds::CommandAckPayload* ack);

}  // namespace rc

#endif  // ROCKETCHIP_CMD_SDU_H
