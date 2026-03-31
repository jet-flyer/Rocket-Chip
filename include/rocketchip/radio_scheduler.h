// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RadioScheduler — Half-duplex TX-priority state machine (IVP-93)
//
// Protocol-agnostic: never inspects packet contents [C2-4].
// Council 2 + Council 3 unanimous.
//
// Vehicle: TX at scheduled rate, RX between TX slots.
// Station: RX continuous from init.
// Relay:   RX continuous, TX on packet receipt.
//============================================================================
#ifndef ROCKETCHIP_RADIO_SCHEDULER_H
#define ROCKETCHIP_RADIO_SCHEDULER_H

#include <stdint.h>

namespace rc {

enum class RadioPhase : uint8_t {
    kIdle         = 0,   // Standby or init failure [C3-A5]
    kTxActive     = 1,   // send_start() called, polling send_poll()
    kRxWindow     = 2,   // Listening between TX slots (Vehicle)
    kRxContinuous = 3,   // Always listening (Station/Relay)
};

struct RadioScheduler {
    RadioPhase phase;
    uint32_t   tx_interval_ms;      // 1000 / rate_hz
    uint32_t   next_tx_deadline_ms;  // When next TX slot opens

    void init(uint32_t interval_ms, bool rx_continuous) {
        phase = rx_continuous ? RadioPhase::kRxContinuous : RadioPhase::kRxWindow;
        tx_interval_ms = interval_ms;
        next_tx_deadline_ms = 0;
    }

    // Should we start a TX now? (Vehicle mode: checks deadline)
    bool tx_slot_open(uint32_t now_ms) const {
        if (phase == RadioPhase::kTxActive) { return false; }
        if (phase == RadioPhase::kRxContinuous) { return false; }
        return (now_ms >= next_tx_deadline_ms);
    }

    // Should we poll for RX? (Any mode except kTxActive and kIdle)
    bool rx_active() const {
        return (phase == RadioPhase::kRxWindow ||
                phase == RadioPhase::kRxContinuous);
    }

    // Called when TX starts
    void on_tx_start(uint32_t now_ms) {
        phase = RadioPhase::kTxActive;
        (void)now_ms;
    }

    // Called when TX completes (kDone or kTimeout)
    // TX timeout transitions to kRxWindow (not kIdle) [C3-A3]
    void on_tx_complete(uint32_t now_ms) {
        next_tx_deadline_ms = now_ms + tx_interval_ms;
        // Return to the appropriate RX mode based on initial config
        // For relay/station: kRxContinuous is sticky (set at init)
        // For vehicle: kRxWindow between TX slots
        phase = RadioPhase::kRxWindow;
    }

    // Set TX rate (called when config changes)
    void set_rate(uint8_t rate_hz) {
        if (rate_hz == 0) { rate_hz = 2; }
        tx_interval_ms = 1000 / rate_hz;
    }
};

} // namespace rc

#endif // ROCKETCHIP_RADIO_SCHEDULER_H
