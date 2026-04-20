// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Radio — Radio Hardware Active Object
//
// Owns RFM95W radio driver + RadioScheduler half-duplex state machine.
// Protocol-agnostic: receives encoded packets via SIG_RADIO_TX, posts
// raw received bytes via SIG_RADIO_RX. Never inspects packet contents.
//============================================================================
#ifndef ROCKETCHIP_AO_RADIO_H
#define ROCKETCHIP_AO_RADIO_H

extern "C" {
#include "qp_port.h"
}

#include "rocketchip/radio_scheduler.h"
#include "rocketchip/radio_config.h"
#include "drivers/rfm95w.h"

extern QActive * const AO_Radio;

/// Start AO_Radio. spi_ok: whether SPI bus was initialized successfully.
void AO_Radio_start(uint8_t prio, bool spi_ok);

// CLI access — safe under QV cooperative scheduling (no preemption on Core 0)
struct RadioAoState {
    rfm95w_t           radio;
    rc::RadioScheduler scheduler;
    bool               initialized;
    uint8_t            tx_consec_fail;   // TX failure escalation counter
    uint8_t            tx_bw_mode;       // 0=BW125, 1=BW250, 2=BW500
    int16_t            last_rx_rssi;     // RSSI of last received packet (dBm)
    int8_t             last_rx_snr;      // SNR of last received packet (dB)
    uint32_t           last_rx_ms;       // Timestamp of last valid RX
    uint32_t           rx_count;         // Total valid packets received
    uint32_t           rx_crc_errors;    // CRC failures
    uint16_t           last_rx_seq;      // Last received CCSDS seq counter
    uint32_t           tx_count;         // Total packets sent (vehicle mode)
    uint32_t           relay_count;      // Packets relayed (relay mode)
    uint8_t            link_quality;     // 0=no radio, 1=lost, 2=gap, 3=receiving
    // Stage T IVP-T5.5 prereq #1: mutable current config. Initialized to
    // kDefaultRocketRadioConfig at boot. Survives rfm95w_init() reinit-recovery
    // via ao_radio_apply_runtime_config() — without this, the recovery path at
    // :148-152 silently snapped radio back to compile-time defaults.
    rc::RadioConfig    runtime_config;

    // Stage T IVP-T5.5 sub 2d: symmetric-revert support. When a config apply
    // happens, `runtime_config` before apply is cached here. `tx_since_apply`
    // counts our own TXes on the new config; if it reaches the revert
    // threshold with no RX from the station, we self-revert to prev_config.
    // Threshold = max(15, ceil(3 * nav_hz)). At nav_hz=10 -> 30 TXes ≈ 3 s.
    // Station's own revert window is SHORTER (max(6, ceil(1.5 * nav_hz)))
    // so the station gives up first visually; vehicle is the final fallback
    // (smell-test A.2 asymmetric revert).
    rc::RadioConfig    prev_config;
    uint32_t           tx_since_apply;   // our-TX count since last apply
    uint32_t           rx_at_apply;      // rx_count snapshot at apply time
    bool               apply_in_progress;// true while waiting for NEW-config RX
};

const RadioAoState* AO_Radio_get_state();

/// Stage T IVP-T5.5: queue a pending radio config. Applied by AO_Radio
/// after the next TX-poll reports kDone (TxDone IRQ equivalent) — i.e.
/// immediately after the ACK packet has physically left the antenna.
/// If no TX is outstanding, applied on the next tick.
/// A ~200 ms backstop timer guards against TxDone never firing.
/// Caller is responsible for validating the config BEFORE calling this.
void AO_Radio_set_pending_config(const rc::RadioConfig& cfg);

/// Stage T IVP-T5.5: read-only access to the AO's current runtime config.
/// Used by SET dispatch to compute power-delta for the +/-6 dB gate.
const rc::RadioConfig* AO_Radio_get_runtime_config();

#endif // ROCKETCHIP_AO_RADIO_H
