// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include "starcom_adapt/cmd_sdu.h"
#include <string.h>

namespace rc {

uint8_t pack_cmd_sdu_user(uint8_t* out, uint8_t out_len, uint16_t cmd_id,
                          uint8_t seq, float p1, float p2, float p3, float p4,
                          float p5) {
    if ((out == nullptr) || (out_len < kCmdSduUserBytes)) {
        return 0;
    }
    out[0] = static_cast<uint8_t>(cmd_id & 0xFFu);
    out[1] = static_cast<uint8_t>(cmd_id >> 8);
    out[2] = seq;
    out[3] = 0;
    memcpy(out + 4, &p1, sizeof(p1));
    memcpy(out + 8, &p2, sizeof(p2));
    memcpy(out + 12, &p3, sizeof(p3));
    memcpy(out + 16, &p4, sizeof(p4));
    memcpy(out + 20, &p5, sizeof(p5));
    return kCmdSduUserBytes;
}

bool unpack_cmd_sdu_user(const uint8_t* in, uint8_t in_len, uint16_t* cmd_id,
                         uint8_t* seq, float* p1, float* p2, float* p3,
                         float* p4, float* p5) {
    if ((in == nullptr) || (cmd_id == nullptr) || (seq == nullptr) ||
        (p1 == nullptr) || (p2 == nullptr) || (p3 == nullptr) ||
        (p4 == nullptr) || (p5 == nullptr) || (in_len != kCmdSduUserBytes)) {
        return false;
    }
    *cmd_id = static_cast<uint16_t>(
        static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8));
    *seq = in[2];
    memcpy(p1, in + 4, sizeof(*p1));
    memcpy(p2, in + 8, sizeof(*p2));
    memcpy(p3, in + 12, sizeof(*p3));
    memcpy(p4, in + 16, sizeof(*p4));
    memcpy(p5, in + 20, sizeof(*p5));
    return true;
}

uint8_t pack_ack_sdu_user(uint8_t* out, uint8_t out_len,
                          const ccsds::CommandAckPayload& ack) {
    if ((out == nullptr) || (out_len < kAckSduUserBytes)) {
        return 0;
    }
    memcpy(out, &ack, kAckSduUserBytes);
    return kAckSduUserBytes;
}

bool unpack_ack_sdu_user(const uint8_t* in, uint8_t in_len,
                         ccsds::CommandAckPayload* ack) {
    if ((in == nullptr) || (ack == nullptr) || (in_len != kAckSduUserBytes)) {
        return false;
    }
    memcpy(ack, in, kAckSduUserBytes);
    return true;
}

}  // namespace rc
