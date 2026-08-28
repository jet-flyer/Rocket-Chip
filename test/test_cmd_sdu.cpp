// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Command / ACK SDU packer (IVP 22). No radio.

#include <gtest/gtest.h>
#include "starcom_adapt/cmd_sdu.h"

TEST(CmdSdu, PackUnpackCommand) {
    uint8_t buf[rc::kCmdSduUserBytes] = {};
    ASSERT_EQ(rc::pack_cmd_sdu_user(buf, sizeof(buf), 400, 7, 1.0F, 2.0F, 3.0F,
                                    4.0F, 5.0F),
              rc::kCmdSduUserBytes);
    uint16_t cmd_id = 0;
    uint8_t seq = 0;
    float p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0;
    ASSERT_TRUE(rc::unpack_cmd_sdu_user(buf, sizeof(buf), &cmd_id, &seq, &p1,
                                        &p2, &p3, &p4, &p5));
    EXPECT_EQ(cmd_id, 400);
    EXPECT_EQ(seq, 7);
    EXPECT_FLOAT_EQ(p1, 1.0F);
    EXPECT_FLOAT_EQ(p5, 5.0F);
}

TEST(CmdSdu, PackUnpackAck) {
    rc::ccsds::CommandAckPayload ack{};
    ack.cmd_seq = 3;
    ack.cmd_id = 400;
    ack.result = 0;
    uint8_t buf[rc::kAckSduUserBytes] = {};
    ASSERT_EQ(rc::pack_ack_sdu_user(buf, sizeof(buf), ack), rc::kAckSduUserBytes);
    rc::ccsds::CommandAckPayload out{};
    ASSERT_TRUE(rc::unpack_ack_sdu_user(buf, sizeof(buf), &out));
    EXPECT_EQ(out.cmd_seq, 3);
    EXPECT_EQ(out.cmd_id, 400);
    EXPECT_EQ(out.result, 0);
}

TEST(CmdSdu, RejectsShortBuffer) {
    uint8_t tiny[4] = {};
    EXPECT_EQ(rc::pack_cmd_sdu_user(tiny, sizeof(tiny), 400, 0, 0, 0, 0, 0, 0),
              0);
}
