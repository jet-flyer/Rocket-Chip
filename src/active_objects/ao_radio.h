// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Radio — Radio Hardware Active Object (IVP-93)
//
// Owns RFM95W radio driver + RadioScheduler half-duplex state machine.
// Protocol-agnostic: receives encoded packets via SIG_RADIO_TX, posts
// raw received bytes via SIG_RADIO_RX. Never inspects packet contents.
//
// Council 1 (universality) + Council 2 (scheduler) + Council 3 (final).
//============================================================================
#ifndef ROCKETCHIP_AO_RADIO_H
#define ROCKETCHIP_AO_RADIO_H

extern "C" {
#include "qp_port.h"
}

#include "rocketchip/radio_scheduler.h"
#include "drivers/rfm95w.h"

extern QActive * const AO_Radio;

void AO_Radio_start(uint8_t prio);

// CLI access — safe under QV cooperative scheduling (no preemption on Core 0)
struct RadioAoState {
    rfm95w_t           radio;
    rc::RadioScheduler scheduler;
    bool               initialized;
    uint8_t            tx_consec_fail;   // TX failure escalation counter
    uint8_t            tx_bw_mode;       // 0=BW125, 1=BW250, 2=BW500
    int16_t            last_rx_rssi;     // RSSI of last received packet (dBm)
    uint32_t           last_rx_ms;       // Timestamp of last valid RX
    uint32_t           rx_count;         // Total valid packets received
};

const RadioAoState* AO_Radio_get_state();

#endif // ROCKETCHIP_AO_RADIO_H
