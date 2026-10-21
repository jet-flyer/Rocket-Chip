// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Baseline link counters. The bearer calls these. LoRa does that today.
// A later FSK bearer calls the same functions.
// CLI `t` prints one line. scripts/baseline_lora.py parses that line.
//
// 133.0-B-2 §4.1.3.4.3.4: Packet Sequence Count is continuous modulo 16384.
#ifndef ROCKETCHIP_DIAG_LINK_BASELINE_H
#define ROCKETCHIP_DIAG_LINK_BASELINE_H

#include <stddef.h>
#include <stdint.h>

// 14-bit mask. Same modulus as byte_pump.h kSpaceSeqMask.
constexpr uint16_t kLinkSeqMask = 0x3FFFu;

// Two air APIDs: nav 0x001 and command 0x003 (byte_pump.h).
constexpr uint8_t kLinkSeqLanes = 2;

// Missed counts strictly between two accepted sequence counts.
// A repeat misses 0. A step of 1 misses 0.
// 16383 then 0 misses 0. 16383 then 1 misses 1.
uint16_t link_seq_missing(uint16_t prev, uint16_t cur);

struct LinkBaselineCounters {
    uint32_t delivered;
    uint32_t missing;
    uint32_t crc_fail;
    uint32_t air_ms;
    uint32_t air_bytes;
};

extern LinkBaselineCounters g_linkBaseline;

void link_baseline_reset(void);
void link_baseline_note_delivered(void);
void link_baseline_note_crc_fail(void);
// Per APID. The first count on a lane opens it and adds no gap.
void link_baseline_note_seq(uint16_t apid, uint16_t seq);
// add_us is on-air time of one finished transmission. 1000 us = 1 ms.
// bytes is the modem payload length of that transmission.
void link_baseline_note_air(uint32_t add_us, uint32_t bytes);

// One line, no newline. copp_rtx is the COP-P resend count.
size_t link_baseline_format(char* buf, size_t n, uint32_t copp_rtx);
void link_baseline_dump(void);

#endif  // ROCKETCHIP_DIAG_LINK_BASELINE_H
