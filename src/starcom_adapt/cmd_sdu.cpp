// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include "starcom_adapt/cmd_sdu.h"
#include <string.h>

namespace rc {

uint8_t pack_cmd_sdu_user(uint8_t* out, uint8_t out_len,
                          const CmdSduFields& f) {
    if ((out == nullptr) || (out_len < kCmdSduUserBytes)) {
        return 0;
    }
    out[0] = static_cast<uint8_t>(f.cmd_id & 0xFFU);
    out[1] = static_cast<uint8_t>(f.cmd_id >> 8);
    out[2] = f.seq;
    out[3] = 0;
    memcpy(out + 4, &f.p[0], sizeof(f.p[0]));
    memcpy(out + 8, &f.p[1], sizeof(f.p[1]));
    memcpy(out + kCmdSduP2Offset, &f.p[2], sizeof(f.p[2]));
    memcpy(out + 16, &f.p[3], sizeof(f.p[3]));
    memcpy(out + kCmdSduP4Offset, &f.p[4], sizeof(f.p[4]));
    return kCmdSduUserBytes;
}

bool unpack_cmd_sdu_user(const uint8_t* in, uint8_t in_len, CmdSduFields& f) {
    if ((in == nullptr) || (in_len != kCmdSduUserBytes)) {
        return false;
    }
    f.cmd_id = static_cast<uint16_t>(
        static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8));
    f.seq = in[2];
    memcpy(&f.p[0], in + 4, sizeof(f.p[0]));
    memcpy(&f.p[1], in + 8, sizeof(f.p[1]));
    memcpy(&f.p[2], in + kCmdSduP2Offset, sizeof(f.p[2]));
    memcpy(&f.p[3], in + 16, sizeof(f.p[3]));
    memcpy(&f.p[4], in + kCmdSduP4Offset, sizeof(f.p[4]));
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
