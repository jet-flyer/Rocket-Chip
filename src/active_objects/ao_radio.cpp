// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Radio — Radio Hardware Active Object
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
#include "rocketchip/config.h"  // rocketchip::pins::kRadioCs/Rst/Irq
#include "rocketchip/radio_config.h"
#include "flight_director/mission_profile_data.h"  // kDefaultRocketRadioConfig
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
// TX failure escalation thresholds
// ============================================================================

static constexpr uint8_t kTxFailLogThresh   = 1;   // Log after 1 consecutive timeout
static constexpr uint8_t kTxFailReinitThresh = 3;  // Reinit radio after 3
static constexpr uint8_t kTxFailErrorThresh  = 5;  // Set error flag after 5

// SPI bus status — set by AO_Radio_start(), read by RadioAo_initial()
static bool g_spiOk = false;

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

// Stage T (IVP-T1) — RadioScheduler timing diagnostics.
// Off unless built with -DROCKETCHIP_STAGE_T_LOGGING=ON. Never in flight builds.
#if defined(ROCKETCHIP_STAGE_T_LOGGING) && !defined(ROCKETCHIP_HOST_TEST)
static uint32_t stage_t_now_us() { return time_us_32(); }
static const char* phase_name(rc::RadioPhase p) {
    switch (p) {
        case rc::RadioPhase::kIdle:         return "IDLE";
        case rc::RadioPhase::kTxActive:     return "TX";
        case rc::RadioPhase::kRxWindow:     return "RXW";
        case rc::RadioPhase::kRxContinuous: return "RXC";
    }
    return "?";
}
static void stage_t_log_state(rc::RadioPhase old_phase, rc::RadioPhase new_phase) {
    if (old_phase == new_phase) { return; }
    printf("[STAGE_T] state %s->%s t=%lu\n",
           phase_name(old_phase), phase_name(new_phase),
           static_cast<unsigned long>(stage_t_now_us()));
}
static void stage_t_log_rx(rc::RadioPhase state_at_rx, int rssi, int snr,
                           bool crc_ok, uint8_t len, uint16_t seq) {
    printf("[STAGE_T] rx state=%s rssi=%d snr=%d crc=%s len=%u seq=%u t=%lu\n",
           phase_name(state_at_rx), rssi, snr, crc_ok ? "ok" : "err",
           static_cast<unsigned>(len), static_cast<unsigned>(seq),
           static_cast<unsigned long>(stage_t_now_us()));
}
#else
static inline void stage_t_log_state(rc::RadioPhase, rc::RadioPhase) {}
static inline void stage_t_log_rx(rc::RadioPhase, int, int, bool,
                                  uint8_t, uint16_t) {}
#endif

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
        // Enter RX mode between TX slots to receive commands from station
        rfm95w_start_rx(&s.radio);
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

// CCSDS packet constants for relay CRC validation
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

// Track received packet metadata. CCSDS CRC checked if packet looks like CCSDS.
// Non-CCSDS packets (MAVLink commands) are passed through — AO_Telemetry decides.
static bool validate_rx_packet(RadioAoState& s, const uint8_t* buf, uint8_t len) {
    s.last_rx_rssi = rfm95w_rssi(&s.radio);
    s.last_rx_snr = s.radio.last_snr;
    s.last_rx_ms = now_ms();
    s.rx_count++;

    // CCSDS validation: check CRC only if it looks like a CCSDS packet
    // (version bits 000 in first byte). MAVLink starts with 0xFD (v2).
    if (len >= kCcsdsMinLen && (buf[0] & 0xE0) == 0x00) {
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

    // Stage T — capture state at arrival BEFORE validate (we want CRC errors).
    rc::RadioPhase state_at_rx = s.scheduler.phase;
    bool crc_ok = validate_rx_packet(s, buf, len);
    uint16_t seq = 0;
    if (len >= kCcsdsMinLen && (buf[0] & 0xE0) == 0x00) {
        seq = extract_ccsds_seq(buf);
    }
    stage_t_log_rx(state_at_rx, rfm95w_rssi(&s.radio), s.radio.last_snr,
                   crc_ok, len, seq);
    if (!crc_ok) { return; }

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

static QState RadioAo_initial(RadioAo * const me, QEvt const * const e) {
    (void)e;
    RadioAoState& s = me->state;
    s.tx_consec_fail = 0;
    s.tx_bw_mode = 0;
    s.link_quality = 0;

    // Initialize radio hardware (owned by this AO)
    if (g_spiOk && rfm95w_init(&s.radio,
            rocketchip::pins::kRadioCs,
            rocketchip::pins::kRadioRst,
            rocketchip::pins::kRadioIrq)) {
        s.initialized = true;

        // Apply RadioConfig from Mission Profile (IVP-64)
        const auto& rc = rc::kDefaultRocketRadioConfig;
        rfm95w_set_spreading_factor(&s.radio, rc.spreading_factor);
        rfm95w_set_coding_rate(&s.radio, rc.coding_rate);
        rfm95w_set_tx_power(&s.radio, rc.power_dbm);
        // BW: SX1276 register encoding (7=125kHz, 8=250kHz, 9=500kHz)
        uint8_t bw_reg = (rc.bandwidth_khz >= 500) ? rfm95w::kBw500 :
                         (rc.bandwidth_khz >= 250) ? rfm95w::kBw250 :
                                                     rfm95w::kBw125;
        rfm95w_set_bandwidth(&s.radio, bw_reg);

        // Mode selection based on device role
        bool rx_continuous = (job::kRole == job::DeviceRole::kStation ||
                              job::kRole == job::DeviceRole::kRelay);
        uint32_t interval = 1000 / rc.nav_rate_hz;
        s.scheduler.init(interval, rx_continuous);

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

static void handle_link_quality(RadioAo* me) {
    RadioAoState& s = me->state;
    static constexpr uint32_t kLinkLostMs  = 5000;  // 5s = lost
    static constexpr uint32_t kLinkGapMs   = 2000;  // 2s = gap
    uint32_t age = now_ms() - s.last_rx_ms;
    uint8_t lq;
    if (s.rx_count == 0)         { lq = 0; }
    else if (age >= kLinkLostMs) { lq = 1; }
    else if (age >= kLinkGapMs)  { lq = 2; }
    else                         { lq = 3; }
    if (lq != s.link_quality) {
        s.link_quality = lq;
        static rc::RadioStatusEvt statusEvt;
        statusEvt.super.sig = rc::SIG_RADIO_STATUS;
        statusEvt.link_quality = lq;
        QActive_publish_(&statusEvt.super, &me->super, me->super.prio);
    }
}

static void handle_rssi_bar(RadioAo* me) {
    // Station/relay only — vehicle's AO_LedEngine owns the NeoPixel.
    if constexpr (job::kRole != job::DeviceRole::kVehicle) {
        RadioAoState& s = me->state;
        static uint8_t rssi_div = 0;
        if (++rssi_div >= 50) {  // ~2Hz update
            rssi_div = 0;
            uint32_t gap = now_ms() - s.last_rx_ms;
            bool no_signal = (s.rx_count == 0 || gap >= 5000);
            ws2812_set_rssi_bar(s.last_rx_rssi, no_signal);
        }
    } else {
        (void)me;
    }
}

static void handle_radio_tick(RadioAo* me) {
    RadioAoState& s = me->state;
    if (!s.initialized) { return; }

    // Stage T (IVP-T1) — log any phase change since last tick.
    // Captures transitions triggered by SIG_RADIO_TX between ticks.
    static rc::RadioPhase s_stage_t_prev_phase = rc::RadioPhase::kIdle;
    stage_t_log_state(s_stage_t_prev_phase, s.scheduler.phase);

    if (s.scheduler.phase == rc::RadioPhase::kTxActive) {
        handle_tx_poll(me);
    }
    if (s.scheduler.rx_active()) {
        handle_rx_poll(me);
    }

    // Stage T — capture any transition that happened during this tick's work
    stage_t_log_state(s_stage_t_prev_phase, s.scheduler.phase);
    s_stage_t_prev_phase = s.scheduler.phase;

    handle_link_quality(me);
    handle_rssi_bar(me);
}

static QState RadioAo_running(RadioAo * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_RADIO_TICK: {
        handle_radio_tick(me);
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

void AO_Radio_start(uint8_t prio, bool spi_ok) {
    g_spiOk = spi_ok;
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
