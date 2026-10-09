// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Pure latch for one forced radio HW TX timeout. No SPI.
#ifndef ROCKETCHIP_RADIO_TX_FORCE_H
#define ROCKETCHIP_RADIO_TX_FORCE_H

#include <stdint.h>

struct RadioTxForceLatch {
    bool armed;   // command waiting for a successful send start
    bool marked;  // the in-flight send is the forced one
};

inline void radio_tx_force_arm(RadioTxForceLatch& lat) {
    lat.armed = true;
}

// started false: latch stays armed, device flag stays clear.
// started true and armed: clear the latch, mark the send, set the device flag.
inline bool radio_tx_force_on_start(RadioTxForceLatch& lat, bool started) {
    if (!started || !lat.armed) {
        return false;
    }
    lat.armed = false;
    lat.marked = true;
    return true;
}

// Forced send: clear the mark and leave consec unchanged. Returns false.
// Real timeout: increment consec. Returns true so the caller escalates.
inline bool radio_tx_timeout_counts(RadioTxForceLatch& lat, uint8_t& consec) {
    if (lat.marked) {
        lat.marked = false;
        return false;
    }
    consec = static_cast<uint8_t>(consec + 1U);
    return true;
}

#endif
