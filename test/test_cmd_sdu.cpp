// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Command / ACK SDU packer (IVP 22). No radio.

#include <gtest/gtest.h>
#include "starcom_adapt/cmd_sdu.h"

TEST(CmdSdu, PinsKnownCommandBytes) {
    // cmd_id 400, seq 7, p = 1.0, 2.0, 3.0, -4.5, 1e6. Little-endian floats.
    constexpr uint8_t kPinned[rc::kCmdSduUserBytes] = {
        0x90, 0x01, 0x07, 0x00,
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x00, 0x40,
        0x00, 0x00, 0x40, 0x40,
        0x00, 0x00, 0x90, 0xC0,
        0x00, 0x24, 0x74, 0x49,
    };
    uint8_t buf[rc::kCmdSduUserBytes] = {};
    const rc::CmdSduFields fields{400, 7, {1.0F, 2.0F, 3.0F, -4.5F, 1.0e6F}};
    ASSERT_EQ(rc::pack_cmd_sdu_user(buf, sizeof(buf), fields),
              rc::kCmdSduUserBytes);
    for (uint8_t i = 0; i < rc::kCmdSduUserBytes; ++i) {
        EXPECT_EQ(buf[i], kPinned[i]) << "index " << static_cast<unsigned>(i);
    }
}

TEST(CmdSdu, PackUnpackCommand) {
    uint8_t buf[rc::kCmdSduUserBytes] = {};
    const rc::CmdSduFields fields{400, 7, {1.0F, 2.0F, 3.0F, 4.0F, 5.0F}};
    ASSERT_EQ(rc::pack_cmd_sdu_user(buf, sizeof(buf), fields),
              rc::kCmdSduUserBytes);
    rc::CmdSduFields out{};
    ASSERT_TRUE(rc::unpack_cmd_sdu_user(buf, sizeof(buf), out));
    EXPECT_EQ(out.cmd_id, 400);
    EXPECT_EQ(out.seq, 7);
    EXPECT_FLOAT_EQ(out.p[0], 1.0F);
    EXPECT_FLOAT_EQ(out.p[4], 5.0F);
}

TEST(CmdSdu, PackUnpackAck) {
    rc::ccsds::CommandAckPayload ack{};
    ack.cmd_seq = 3;
    ack.cmd_id = 400;
    ack.result = 0;
    ack.reserved = 2;
    uint8_t buf[rc::kAckSduUserBytes] = {};
    ASSERT_EQ(rc::pack_ack_sdu_user(buf, sizeof(buf), ack), rc::kAckSduUserBytes);
    rc::ccsds::CommandAckPayload out{};
    ASSERT_TRUE(rc::unpack_ack_sdu_user(buf, sizeof(buf), &out));
    EXPECT_EQ(out.cmd_seq, 3);
    EXPECT_EQ(out.cmd_id, 400);
    EXPECT_EQ(out.result, 0);
    EXPECT_EQ(out.reserved, 2);
}

TEST(CmdSdu, RejectsShortBuffer) {
    uint8_t tiny[4] = {};
    const rc::CmdSduFields fields{400, 0, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F}};
    EXPECT_EQ(rc::pack_cmd_sdu_user(tiny, sizeof(tiny), fields), 0);
}
