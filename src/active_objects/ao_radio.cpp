// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Radio — Radio Hardware Active Object (IVP-93)
//
// Owns radio hardware (RFM95W) and RadioScheduler half-duplex SM.
// Protocol-agnostic: posts SIG_RADIO_RX with raw bytes, receives
// SIG_RADIO_TX with encoded packets.
//
// Tick rate: 100Hz fixed [C3-R1]. No-op when idle (~10µs).
// TX-busy policy: drop and log [C3-A2].
// TX timeout → kRxWindow (not kIdle) [C3-A3].
// IRQ register for TX completion, not GPIO DIO0 [C3-R3].
//============================================================================

#include "ao_radio.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/config.h"
#include "rocketchip/job.h"
#include "drivers/spi_bus.h"
#include "drivers/ws2812_status.h"
#include "crc16_ccitt.h"
#include <string.h>

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#endif

// Pin configuration (rocketchip::pins::kRadioCs/Rst/Irq)

// ============================================================================
// Internal signal (private to this AO)
// ============================================================================

enum : uint16_t {
    SIG_RADIO_TICK = rc::SIG_AO_MAX + 10
};

// ============================================================================
// TX failure escalation thresholds (IVP-92 plan)
// ============================================================================

static constexpr uint8_t kTxFailLogThresh   = 1;   // Log after 1 consecutive timeout
static constexpr uint8_t kTxFailReinitThresh = 3;  // Reinit radio after 3
static constexpr uint8_t kTxFailErrorThresh  = 5;  // Set error flag after 5

// ============================================================================
// AO State
// ============================================================================

struct RadioAo {
    QActive         super;
    QTimeEvt        tick_timer;     // 100Hz (every 1 tick at 100Hz base)
    RadioAoState    state;
};

static RadioAo l_radioAo;
static QEvtPtr  l_radioAoQueue[32]; // Match other AOs — 100Hz tick needs headroom

// Forward declarations
static QState RadioAo_initial(RadioAo * const me, QEvt const * const e);
static QState RadioAo_running(RadioAo * const me, QEvt const * const e);

// ============================================================================
// Helpers
// ============================================================================

static uint32_t now_ms() {
#ifndef ROCKETCHIP_HOST_TEST
    return to_ms_since_boot(get_absolute_time());
#else
    return 0;
#endif
}

static void handle_tx_event(RadioAo* me, const rc::RadioTxEvt* txEvt) {
    RadioAoState& s = me->state;

    // TX-busy: drop and log [C3-A2]
    if (s.scheduler.phase == rc::RadioPhase::kTxActive) {
        DBG_PRINT("RADIO: TX busy, dropping packet (%u bytes)", txEvt->len);
        return;
    }

    if (!s.initialized || txEvt->len == 0) {
        return;
    }

    if (rfm95w_send_start(&s.radio, txEvt->buf, txEvt->len)) {
        s.scheduler.on_tx_start(now_ms());
    }
}

static void handle_tx_poll(RadioAo* me) {
    RadioAoState& s = me->state;

    TxPollResult result = rfm95w_send_poll(&s.radio);

    if (result == TxPollResult::kDone) {
        s.tx_consec_fail = 0;
        s.tx_count++;
        s.scheduler.on_tx_complete(now_ms());
    } else if (result == TxPollResult::kTimeout) {
        s.tx_consec_fail++;

        if (s.tx_consec_fail >= kTxFailErrorThresh) {
            DBG_ERROR("RADIO: %u consecutive TX failures — error flag set",
                      static_cast<unsigned>(s.tx_consec_fail));
        } else if (s.tx_consec_fail >= kTxFailReinitThresh) {
            DBG_ERROR("RADIO: %u consecutive TX failures — reinit attempt",
                      static_cast<unsigned>(s.tx_consec_fail));
            rfm95w_init(&s.radio,
                        s.radio.cs_pin, s.radio.rst_pin, s.radio.irq_pin);
        } else if (s.tx_consec_fail >= kTxFailLogThresh) {
            DBG_ERROR("RADIO: TX timeout (%u consecutive)",
                      static_cast<unsigned>(s.tx_consec_fail));
        }

        // TX timeout → kRxWindow (not kIdle) [C3-A3]
        s.scheduler.on_tx_complete(now_ms());
    }
    // kBusy — continue polling next tick
}

// CCSDS packet constants for relay CRC validation (IVP-98)
static constexpr uint8_t  kCcsdsMinLen  = 54;   // Nav packet size
static constexpr uint8_t  kCcsdsCrcOff  = 52;   // CRC starts at byte 52

// Relay dedup: last relayed sequence counter
static uint16_t g_lastRelaySeq = 0xFFFF;

// Validate CCSDS packet integrity without decoding payload [C3-R2]
static bool validate_ccsds_crc(const uint8_t* buf, uint8_t len) {
    if (len < kCcsdsMinLen) { return false; }
    uint16_t computed = rc::crc16_ccitt(buf, kCcsdsCrcOff);
    uint16_t stored = static_cast<uint16_t>(
        (buf[kCcsdsCrcOff] << 8) | buf[kCcsdsCrcOff + 1]);
    return computed == stored;
}

// Extract 14-bit CCSDS sequence counter from primary header
static uint16_t extract_ccsds_seq(const uint8_t* buf) {
    return static_cast<uint16_t>(
        ((buf[2] & 0x3F) << 8) | buf[3]);
}

// Validate and track a received packet. Returns false if CRC fails.
static bool validate_rx_packet(RadioAoState& s, const uint8_t* buf, uint8_t len) {
    s.last_rx_rssi = rfm95w_rssi(&s.radio);
    s.last_rx_snr = s.radio.last_snr;
    s.last_rx_ms = now_ms();
    s.rx_count++;

    if (len >= kCcsdsMinLen) {
        s.last_rx_seq = extract_ccsds_seq(buf);
        if (!validate_ccsds_crc(buf, len)) {
            s.rx_crc_errors++;
            return false;
        }
    }
    return true;
}

// Relay: dedup + re-TX without decoding payload
static void handle_relay_forward(RadioAo* me, const uint8_t* buf, uint8_t len) {
    RadioAoState& s = me->state;
    uint16_t seq = extract_ccsds_seq(buf);
    if (seq == g_lastRelaySeq) { return; }
    g_lastRelaySeq = seq;

    if (s.scheduler.phase != rc::RadioPhase::kTxActive) {
        rfm95w_send_start(&s.radio, buf, len);
        s.scheduler.on_tx_start(now_ms());
        s.relay_count++;
    }
}

static void handle_rx_poll(RadioAo* me) {
    RadioAoState& s = me->state;
    if (!rfm95w_available(&s.radio)) { return; }

    uint8_t buf[128];
    uint8_t len = rfm95w_recv(&s.radio, buf, sizeof(buf));
    if (len == 0) { return; }

    if (!validate_rx_packet(s, buf, len)) { return; }

    if constexpr (job::kRole == job::DeviceRole::kRelay) {
        handle_relay_forward(me, buf, len);
        return;
    }

    // Station/Vehicle: post to AO_Telemetry for decode
    // File-scope static — safe under QV cooperative scheduling (one at a time)
    static rc::RadioRxEvt rxEvt;
    rxEvt.super.sig = rc::SIG_RADIO_RX;
    rxEvt.super.refCtr_ = 0;
    memcpy(rxEvt.buf, buf, len);
    rxEvt.len = len;
    rxEvt.rssi = s.last_rx_rssi;
    rxEvt.snr = s.last_rx_snr;

    extern QActive * const AO_Telemetry;
    QACTIVE_POST(AO_Telemetry, &rxEvt.super, me);
}

// ============================================================================
// State Handlers
// ============================================================================

// Extern references to main.cpp radio state (IVP-93 transitional — removed in IVP-94)
extern bool g_radioInitialized;
extern rfm95w_t g_radio;

static QState RadioAo_initial(RadioAo * const me, QEvt const * const e) {
    (void)e;
    RadioAoState& s = me->state;
    s.tx_consec_fail = 0;
    s.tx_bw_mode = 0;

    // IVP-93 transitional: radio is initialized by init_peripherals() in main.cpp.
    // AO_Radio borrows the existing g_radio handle. Full ownership transfer in IVP-94.
    if (g_radioInitialized) {
        s.radio = g_radio;  // Copy device handle (just pin config + flags)
        s.initialized = true;

        // Mode selection based on device role (IVP-95/98)
        bool rx_continuous = (job::kRole == job::DeviceRole::kStation ||
                              job::kRole == job::DeviceRole::kRelay);
        s.scheduler.init(500, rx_continuous);  // 2Hz for vehicle, RX continuous for station/relay

        if (rx_continuous) {
            rfm95w_start_rx(&s.radio);
            DBG_PRINT("RADIO: AO_Radio RX continuous (%s)",
                       job::kRole == job::DeviceRole::kRelay ? "relay" : "station");
        } else {
            DBG_PRINT("RADIO: AO_Radio TX+RX (vehicle)");
        }
    } else {
        s.initialized = false;
        s.scheduler.phase = rc::RadioPhase::kIdle;
        DBG_PRINT("RADIO: not detected — AO_Radio in kIdle");
    }

    // Subscribe to SIG_RADIO_TX from AO_Telemetry
    QActive_subscribe(&me->super, rc::SIG_RADIO_TX);

    // 100Hz tick (every 1 tick at 100Hz base) [C3-R1]
    QTimeEvt_armX(&me->tick_timer, 1U, 1U);

    return Q_TRAN(&RadioAo_running);
}

static QState RadioAo_running(RadioAo * const me, QEvt const * const e) {
    RadioAoState& s = me->state;

    switch (e->sig) {
    case SIG_RADIO_TICK: {
        if (!s.initialized) { return Q_HANDLED(); }

        // kTxActive: poll for TX completion
        if (s.scheduler.phase == rc::RadioPhase::kTxActive) {
            handle_tx_poll(me);
        }

        // RX poll (kRxWindow or kRxContinuous)
        if (s.scheduler.rx_active()) {
            handle_rx_poll(me);
        }

        // RSSI bar — station/relay only (AO_LedEngine disabled for these roles)
        if constexpr (job::kRole != job::DeviceRole::kVehicle) {
            static uint8_t rssi_div = 0;
            if (++rssi_div >= 50) {  // ~2Hz update
                rssi_div = 0;
                uint32_t gap = now_ms() - s.last_rx_ms;
                bool no_signal = (s.rx_count == 0 || gap >= 5000);
                ws2812_set_rssi_bar(s.last_rx_rssi, no_signal);
            }
        }

        return Q_HANDLED();
    }

    case rc::SIG_RADIO_TX: {
        const auto* txEvt = reinterpret_cast<const rc::RadioTxEvt*>(e);
        handle_tx_event(me, txEvt);
        return Q_HANDLED();
    }

    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Public API
// ============================================================================

QActive * const AO_Radio = &l_radioAo.super;

const RadioAoState* AO_Radio_get_state() {
    return &l_radioAo.state;
}

void AO_Radio_start(uint8_t prio) {
    QActive_ctor(&l_radioAo.super,
                 Q_STATE_CAST(&RadioAo_initial));

    QTimeEvt_ctorX(&l_radioAo.tick_timer, &l_radioAo.super,
                   SIG_RADIO_TICK, 0U);

    // Zero-init state
    memset(&l_radioAo.state, 0, sizeof(l_radioAo.state));

    QActive_start(&l_radioAo.super,
                  Q_PRIO(prio, 0U),
                  l_radioAoQueue,
                  Q_DIM(l_radioAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}
