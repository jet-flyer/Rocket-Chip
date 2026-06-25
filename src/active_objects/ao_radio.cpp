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
#include "ao_telemetry.h"         // AO_Telemetry_set_rate / _set_ack_retry_timeout_ms (Batch B prelim)
#include "ao_flight_director.h"  // AO_FlightDirector_is_ground_state (T5.5)
#include "ao_rf_manager.h"       // AO_RfManager_next_tx_window_us (Batch B IVP-T14)
#include "rocketchip/ao_signals.h"
#include "rocketchip/config.h"  // rocketchip::pins::kRadioCs/Rst/Irq
#include "rocketchip/radio_config.h"
#include "rocketchip/rc_log.h"  // R-5 Unit F.5: STAGE_T diagnostics use rc::rc_log
#include "logging/radio_config_storage.h"  // T5.5 sub persist: boot read + debounced write
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

// Stage T IVP-T5.5: pending radio config (set by AO_Radio_set_pending_config,
// applied after next TX-poll kDone). File-scope static — safe under QV
// cooperative scheduling (no preemption on Core 0).
static rc::RadioConfig s_pending_radio_config = {};
static bool            s_pending_radio_config_valid = false;
// Backstop: tick count at which a pending apply must fire even if no
// TxDone arrives. Guards against stale pending state if TX was never
// actually in progress when set_pending_config was called.
// ~20 ticks at 100 Hz = 200 ms. Per smell-test A.1 + correctness council.
static constexpr uint32_t kPendingApplyBackstopTicks = 20;
static uint32_t s_pending_apply_backstop_count = 0;

// Stage T IVP-T5.5 sub 2f: "config just changed" latch. Set by commit and
// revert, consumed once by the telemetry encoder — gives the station explicit
// UX confirmation that THIS packet is from the intentional transition.
static bool s_config_just_changed = false;

// Sub-persist (Option C): debounced flash-write trigger. Set to a countdown
// on successful apply; decrements each tick; when it hits 0 AND the apply
// watchdog has cleared (apply_in_progress == false), we flush the current
// runtime_config to flash. Skipped if the debounce starts while another
// apply is still in flight (any new SET resets the counter).
// 5 s at 100 Hz = 500 ticks.
static constexpr uint32_t kPersistDebounceTicks = 500;
static uint32_t s_persist_debounce_count = 0;
static bool     s_persist_requested = false;

// Forward declarations
static QState RadioAo_initial(RadioAo * const me, QEvt const * const e);
static QState RadioAo_running(RadioAo * const me, QEvt const * const e);
static void ao_radio_apply_runtime_config(RadioAoState& s);       // T5.5 prereq #1
static void ao_radio_commit_pending_config(RadioAoState& s);      // T5.5 sub 2b
#if defined(ROCKETCHIP_RADIO_PERSIST)
static void ao_radio_revert_to_prev_config(RadioAoState& s);      // T5.5 sub 2d
#endif

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

// Microsecond-resolution counter used by AO_RfManager TX-window anchoring
// (Stage T Batch B IVP-T14). Always compiled (unlike stage_t_now_us which
// is gated behind ROCKETCHIP_STAGE_T_LOGGING).
static uint32_t now_us_rf() {
#ifndef ROCKETCHIP_HOST_TEST
    return time_us_32();
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
    rc::rc_log("[STAGE_T] state %s->%s t=%lu\n",
               phase_name(old_phase), phase_name(new_phase),
               static_cast<unsigned long>(stage_t_now_us()));
}
static void stage_t_log_rx(rc::RadioPhase state_at_rx, int rssi, int snr,
                           bool crc_ok, uint8_t len, uint16_t seq) {
    rc::rc_log("[STAGE_T] rx state=%s rssi=%d snr=%d crc=%s len=%u seq=%u t=%lu\n",
               phase_name(state_at_rx), rssi, snr, crc_ok ? "ok" : "err",
               static_cast<unsigned>(len), static_cast<unsigned>(seq),
               static_cast<unsigned long>(stage_t_now_us()));
}
// T14 pre-Batch-B: TX-start + TxDone instrumentation for ACK-path timing.
// Plan consensus #2: measure RxDone-to-TX-start jitter + ACK drain.
static void stage_t_log_tx_start(uint8_t len) {
    rc::rc_log("[STAGE_T] tx_start len=%u t=%lu\n",
               static_cast<unsigned>(len),
               static_cast<unsigned long>(stage_t_now_us()));
}
static void stage_t_log_tx_done(TxPollResult result) {
    const char* r = (result == TxPollResult::kDone) ? "done"
                  : (result == TxPollResult::kTimeout) ? "timeout"
                  : "busy";
    rc::rc_log("[STAGE_T] tx_poll result=%s t=%lu\n",
               r, static_cast<unsigned long>(stage_t_now_us()));
}
#else
static inline void stage_t_log_state(rc::RadioPhase, rc::RadioPhase) {}
static inline void stage_t_log_rx(rc::RadioPhase, int, int, bool,
                                  uint8_t, uint16_t) {}
static inline void stage_t_log_tx_start(uint8_t) {}
static inline void stage_t_log_tx_done(TxPollResult) {}
#endif

static void handle_tx_event(RadioAo* me, const rc::RadioTxEvt* tx_evt) {
    RadioAoState& s = me->state;

    // TX-busy: drop and log [C3-A2]
    if (s.scheduler.phase == rc::RadioPhase::kTxActive) {
        DBG_PRINT("RADIO: TX busy, dropping packet (%u bytes)", tx_evt->len);
        return;
    }

    if (!s.initialized || tx_evt->len == 0) {
        return;
    }

    // Stage T Batch B IVP-T14: station-side TX is anchored to vehicle RxDone
    // via AO_RfManager. If the link isn't in a TX-safe state (kAcq or stale
    // anchor), drop the TX event. AO_Telemetry's airtime-scaled retry timer
    // will re-fire; on first successful RxDone (post ACQ→TENTATIVE), the
    // retry lands in a real window instead of firing blind.
    // Vehicle-role keeps free-running TX (it's the anchor source, not the
    // follower).
    if constexpr (kRadioModeRx) {
        uint32_t window = rc::AO_RfManager_next_tx_window_us(now_us_rf());
        if (window == 0) {
            DBG_PRINT("RADIO: station TX held — RfManager window=0 "
                      "(link ACQ or stale anchor), dropping %u bytes",
                      tx_evt->len);
            return;
        }
    }

    if (rfm95w_send_start(&s.radio, tx_evt->buf, tx_evt->len)) {
        s.scheduler.on_tx_start(now_ms());
        stage_t_log_tx_start(tx_evt->len);
    }
}

static void handle_tx_poll(RadioAo* me) {
    RadioAoState& s = me->state;

    TxPollResult result = rfm95w_send_poll(&s.radio);

    if (result != TxPollResult::kBusy) {
        stage_t_log_tx_done(result);
    }

    if (result == TxPollResult::kDone) {
        s.tx_consec_fail = 0;
        s.tx_count++;
        s.scheduler.on_tx_complete(now_ms());
        // Stage T IVP-T5.5 sub 2b: if a config change is pending, the ACK
        // we just finished transmitting was its last carrier on the OLD
        // config — safe to reconfigure now.
        ao_radio_commit_pending_config(s);
        // Sub 2d: if we're in the post-apply "waiting for station RX" window,
        // count this TX toward the revert threshold. Threshold check itself
        // happens in handle_radio_tick so a revert is never triggered from
        // inside a TX-completion handler.
        if (s.apply_in_progress) {
            s.tx_since_apply++;
        }
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
            // Stage T IVP-T5.5 prereq #1: rfm95w_init() writes
            // configure_modem() compile-time defaults (SF7/BW125/CR5/+20dBm).
            // Reapply runtime_config so a runtime-SET config survives the
            // recovery. Before this fix, post-recovery radio silently snapped
            // back to defaults while station expected new config → zero RX.
            ao_radio_apply_runtime_config(s);
        } else if (s.tx_consec_fail >= kTxFailLogThresh) {
            DBG_ERROR("RADIO: TX timeout (%u consecutive)",
                      static_cast<unsigned>(s.tx_consec_fail));
        }

        // TX timeout → kRxWindow (not kIdle) [C3-A3]
        s.scheduler.on_tx_complete(now_ms());
    }
    // kBusy — continue polling next tick
}

// Stage T IVP-T5.5 prereq #1: apply runtime radio config to the SX1276.
// Called from BOTH RadioAo_initial (fresh boot) AND the reinit-recovery branch
// in handle_tx_poll() — without this, rfm95w_init() during TX-fail recovery
// silently snapped radio back to configure_modem() compile-time defaults
// (SF7/BW125/CR5/+20dBm) while RuntimeRadioConfig tracked something else.
// Setters only, no reinit. Does NOT enter RX mode — caller handles RX/TX mode.
//
// Per correctness council edit #3: power_dbm setter is called LAST so a
// power increase or decrease doesn't affect the ACK transmission we just
// finished. (Edit: matters most when called mid-stream, e.g. from the
// pending-apply path. At boot order doesn't matter since no ACK is pending.)
static void ao_radio_apply_runtime_config(RadioAoState& s) {
    const rc::RadioConfig& rc = s.runtime_config;
    // BW: SX1276 register encoding (7=125kHz, 8=250kHz, 9=500kHz)
    uint8_t bw_reg = (rc.bandwidth_khz >= 500) ? rfm95w::kBw500 :
                     (rc.bandwidth_khz >= 250) ? rfm95w::kBw250 :
                                                 rfm95w::kBw125;
    rfm95w_set_bandwidth(&s.radio, bw_reg);
    rfm95w_set_spreading_factor(&s.radio, rc.spreading_factor);
    rfm95w_set_coding_rate(&s.radio, rc.coding_rate);
    // Scheduler rate follows nav_rate_hz; caller owns RX-mode transition.
    if (rc.nav_rate_hz != 0) {
        s.scheduler.set_rate(rc.nav_rate_hz);
        // Stage T Batch B prelim: actually wire nav_rate_hz into the vehicle TX
        // cadence. AO_Telemetry owns the rate-limit check (interval_ms) that
        // gates encode_and_send(); without this call, SET_RADIO_CONFIG only
        // updated the RadioConfig struct + dashboard string, not the on-wire
        // cadence. Bug predates Stage T and made every prior test's nav_rate_hz
        // cosmetic — discovered 2026-04-21 during IVP-T14 instrumentation when
        // station inter-arrival PDF showed 5 Hz at "BW500 10Hz" config.
        AO_Telemetry_set_rate(rc.nav_rate_hz);
    }

    // Stage T Batch B prelim: airtime-scaled TX timeout. Replaces hardcoded
    // 150 ms kTxTimeoutUs (over-generous for BW500, too tight if SF ever
    // rises). Use worst-case payload (128 B max per rfm95w::kMaxPayload)
    // * 2 safety factor. At SF7/BW500/128B: ~70ms * 2 = 140ms; at
    // SF7/BW125/128B: ~270ms * 2 = 540ms.
    uint32_t airtime_worst_us = rfm95w_airtime_us(rc.spreading_factor,
                                                   rc.bandwidth_khz,
                                                   rfm95w::kMaxPayload);
    rfm95w_set_tx_timeout_us(&s.radio, airtime_worst_us * 2U);
    // Also inform AO_Telemetry so its ACK-retry timeout scales with airtime.
    // ACK-retry should be ≥ 2× round-trip worst case. With current 22 B ACK
    // the round trip is ~(tx airtime) + (RX decode) + (tx airtime) + margin.
    // Use 4× single-airtime + 50 ms guard as a coarse scaling, with a
    // 200 ms floor (for short airtimes) and 1000 ms ceiling (for SF12 edge).
    uint32_t single_airtime_us = rfm95w_airtime_us(rc.spreading_factor,
                                                    rc.bandwidth_khz,
                                                    rfm95w::kMaxPayload);
    uint32_t ack_retry_ms = (single_airtime_us * 4U) / 1000U + 50U;
    if (ack_retry_ms < 200U)  { ack_retry_ms = 200U; }
    if (ack_retry_ms > 1000U) { ack_retry_ms = 1000U; }
    AO_Telemetry_set_ack_retry_timeout_ms(ack_retry_ms);
    // Power LAST — per correctness council edit #3 item 3.
    rfm95w_set_tx_power(&s.radio, rc.power_dbm);
}

// Stage T IVP-T5.5 sub 2b/2d: commit a pending config to the radio.
// Called from handle_tx_poll() when TxDone fires (TX has physically left
// the antenna — safe to reconfigure) OR from the tick handler via the
// backstop timer if no TX was ever in progress.
//
// Per correctness council: re-validate ground-state at apply time (not
// just at receive time). If the vehicle transitioned out of IDLE during
// the apply window, abort the config change — leaves old config intact.
// Sub 2d: cache prev_config and arm the symmetric-revert watchdog.
static void ao_radio_commit_pending_config(RadioAoState& s) {
    if (!s_pending_radio_config_valid) { return; }
    s_pending_radio_config_valid = false;
    s_pending_apply_backstop_count = 0;

    // Re-validate ground state at apply (closes the ARM-during-pending race).
    if (!AO_FlightDirector_is_ground_state()) {
        DBG_ERROR("RADIO: pending config apply aborted — not in IDLE");
        return;
    }

    // Sub 2d: cache current config and arm symmetric-revert watchdog.
    s.prev_config = s.runtime_config;
    s.rx_at_apply = s.rx_count;
    s.tx_since_apply = 0;
    s.apply_in_progress = true;

    s.runtime_config = s_pending_radio_config;
    ao_radio_apply_runtime_config(s);
    s_config_just_changed = true;   // sub 2f: latch for next nav packet
    DBG_PRINT("RADIO: config applied — BW=%u nav=%u SF=%u CR=%u pwr=%u",
              static_cast<unsigned>(s.runtime_config.bandwidth_khz),
              static_cast<unsigned>(s.runtime_config.nav_rate_hz),
              static_cast<unsigned>(s.runtime_config.spreading_factor),
              static_cast<unsigned>(s.runtime_config.coding_rate),
              static_cast<unsigned>(s.runtime_config.power_dbm));
    // Sub-persist: arm (or rearm) the debounced-write timer. If a second
    // apply lands inside the window, the counter just resets here — only
    // the final settled config gets written to flash.
    s_persist_requested = true;
    s_persist_debounce_count = kPersistDebounceTicks;
}

#if defined(ROCKETCHIP_RADIO_PERSIST)
// Sub 2d: symmetric revert. Restore prev_config when the vehicle's
// post-apply TX count exceeds the threshold without receiving any
// station packet on the new config. Uses the setter-only apply path
// (same as a fresh SET) so recovery is clean.
// Gated by ROCKETCHIP_RADIO_PERSIST (Stage T IVP-T6): revert is a
// safety mechanism for operational config changes; during sweep tests
// with persistence off it would corrupt measurements by bouncing
// station back to prev_config after a swept BW doesn't link.
static void ao_radio_revert_to_prev_config(RadioAoState& s) {
    DBG_ERROR("RADIO: symmetric revert — no peer RX in %u TXes",
              static_cast<unsigned>(s.tx_since_apply));
    s.runtime_config = s.prev_config;
    ao_radio_apply_runtime_config(s);
    s_config_just_changed = true;   // sub 2f: latch for next nav packet
    s.apply_in_progress = false;
    s.tx_since_apply = 0;
    // Sub-persist: revert-to-prev is a stable state worth persisting. Arm
    // the debounce so the recovered config lands in flash (matches user
    // intent: "a glitch or bug triggers the change" should settle with
    // the known-working prev config persisted, not the broken one).
    s_persist_requested = true;
    s_persist_debounce_count = kPersistDebounceTicks;
    DBG_PRINT("RADIO: reverted to BW=%u nav=%u SF=%u CR=%u pwr=%u",
              static_cast<unsigned>(s.runtime_config.bandwidth_khz),
              static_cast<unsigned>(s.runtime_config.nav_rate_hz),
              static_cast<unsigned>(s.runtime_config.spreading_factor),
              static_cast<unsigned>(s.runtime_config.coding_rate),
              static_cast<unsigned>(s.runtime_config.power_dbm));
}
#endif  // ROCKETCHIP_RADIO_PERSIST

// CCSDS packet constants for relay CRC validation.
// Minimum size accepted = legacy 54-byte nav packet (APID 0x001).
// Stage T IVP-T5.5 sub 2f: nav-with-config is 58 bytes (APID 0x004). CRC
// offset is now (len - 2) — computed per-packet instead of hard-coded.
static constexpr uint8_t kCcsdsMinLen = 54;

// Relay dedup: last relayed sequence counter
static uint16_t g_lastRelaySeq = 0xFFFF;

// Validate CCSDS packet integrity without decoding payload [C3-R2]
static bool validate_ccsds_crc(const uint8_t* buf, uint8_t len) {
    if (len < kCcsdsMinLen) { return false; }
    // CRC always occupies the trailing 2 bytes of the packet.
    const uint8_t crc_off = static_cast<uint8_t>(len - 2);
    uint16_t computed = rc::crc16_ccitt(buf, crc_off);
    uint16_t stored = static_cast<uint16_t>(
        (buf[crc_off] << 8) | buf[crc_off + 1]);
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

    // Station/Vehicle: publish SIG_RADIO_RX for decode + link-health tracking.
    // Consumers: AO_Telemetry (decodes CCSDS/MAVLink), AO_RfManager (updates
    // link state, anchor estimate, LQ window). Stage T Batch B switched from
    // direct POST → publish so both AOs receive per AO Commandment X
    // (publish-subscribe pattern). File-scope static event is safe under QV
    // cooperative scheduling (Commandment VI).
    static rc::RadioRxEvt g_rxEvt;
    g_rxEvt.super.sig = rc::SIG_RADIO_RX;
    g_rxEvt.super.refCtr_ = 0;
    memcpy(g_rxEvt.buf, buf, len);
    g_rxEvt.len = len;
    g_rxEvt.rssi = s.last_rx_rssi;
    g_rxEvt.snr = s.last_rx_snr;

    QActive_publish_(&g_rxEvt.super, &me->super, me->super.prio);
}

// ============================================================================
// State Handlers
// ============================================================================

// Stage T IVP-T5.5 sub-persist + T6: boot-time runtime_config seed.
// Runtime default is kDefaultRocketRadioConfig (caller sets). If persistence
// is enabled and flash has a validated config, override to that value.
// When persistence is disabled (ROCKETCHIP_RADIO_PERSIST undef), every boot
// is deterministic on the mission-profile default.
static void ao_radio_boot_seed_runtime_config(RadioAoState& s) {
#if defined(ROCKETCHIP_RADIO_PERSIST)
    rc::RadioConfig persisted{};
    if (radio_config_storage_read(&persisted)) {
        s.runtime_config = persisted;
        DBG_PRINT("RADIO: boot config from flash — BW=%u nav=%u SF=%u",
                  static_cast<unsigned>(persisted.bandwidth_khz),
                  static_cast<unsigned>(persisted.nav_rate_hz),
                  static_cast<unsigned>(persisted.spreading_factor));
    }
#else
    (void)s;
    DBG_PRINT("RADIO: persistence disabled — boot on kDefaultRocketRadioConfig");
#endif
}

static QState RadioAo_initial(RadioAo * const me, QEvt const * const e) {
    (void)e;
    RadioAoState& s = me->state;
    s.tx_consec_fail = 0;
    s.tx_bw_mode = 0;
    s.link_quality = 0;
    s.boot_audit_valid = false;

    // T5.5 prereq #1: seed runtime_config. Default is the mission-profile
    // value; persistence-on may override from flash. SET_RADIO_CONFIG
    // later mutates the field at runtime.
    s.runtime_config = rc::kDefaultRocketRadioConfig;
    ao_radio_boot_seed_runtime_config(s);

    // Initialize radio hardware (owned by this AO)
    if (g_spiOk && rfm95w_init(&s.radio,
            rocketchip::pins::kRadioCs,
            rocketchip::pins::kRadioRst,
            rocketchip::pins::kRadioIrq)) {
        s.initialized = true;

        // Apply RadioConfig from Mission Profile (IVP-64 / T5.5 prereq #1)
        ao_radio_apply_runtime_config(s);

        // IVP-T11 boot audit: read registers that must match between vehicle
        // and station. Snapshot into state for cmd_radio_status() display
        // (`t` command) — DBG_PRINT at this point in boot isn't visible on
        // USB CDC because the terminal isn't yet connected.
        rfm95w_read_audit(&s.radio, &s.boot_audit);
        s.boot_audit_valid = true;

        // Mode selection based on device role
        bool rx_continuous = (job::kRole == job::DeviceRole::kStation ||
                              job::kRole == job::DeviceRole::kRelay);
        uint32_t interval = 1000 / s.runtime_config.nav_rate_hz;
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
    static constexpr uint32_t k_link_lost_ms  = 5000;  // 5s = lost
    static constexpr uint32_t k_link_gap_ms   = 2000;  // 2s = gap
    uint32_t age = now_ms() - s.last_rx_ms;
    uint8_t lq;
    if (s.rx_count == 0)         { lq = 0; }
    else if (age >= k_link_lost_ms) { lq = 1; }
    else if (age >= k_link_gap_ms)  { lq = 2; }
    else                         { lq = 3; }
    if (lq != s.link_quality) {
        s.link_quality = lq;
        static rc::RadioStatusEvt g_statusEvt;
        g_statusEvt.super.sig = rc::SIG_RADIO_STATUS;
        g_statusEvt.link_quality = lq;
        QActive_publish_(&g_statusEvt.super, &me->super, me->super.prio);
    }
}

static void handle_rssi_bar(RadioAo* me) {
    // Station/relay only — vehicle's AO_LedEngine owns the NeoPixel.
    if constexpr (job::kRole != job::DeviceRole::kVehicle) {
        RadioAoState& s = me->state;
        // Sub 2g: while LOS-watchdog is armed (apply_in_progress), show
        // a KITT sweep at ~20Hz in yellow so the operator sees that a
        // config change is in flight. Drop back to the normal RSSI bar
        // (~2Hz) once the watchdog clears.
        if (s.apply_in_progress) {
            static uint8_t g_sweepDiv = 0;
            if (++g_sweepDiv >= 5) {  // 100Hz tick / 5 = 20Hz sweep
                g_sweepDiv = 0;
                constexpr ws2812_rgb_t k_sweep_color = {0x20, 0x18, 0x00};  // dim yellow
                ws2812_set_sweep_bar(k_sweep_color);
            }
            return;
        }
        static uint8_t g_rssiDiv = 0;
        if (++g_rssiDiv >= 50) {  // ~2Hz update
            g_rssiDiv = 0;
            uint32_t gap = now_ms() - s.last_rx_ms;
            bool no_signal = (s.rx_count == 0 || gap >= 5000);
            ws2812_set_rssi_bar(s.last_rx_rssi, no_signal);
        }
    } else {
        (void)me;
    }
}

// Sub 2b: if a config change is queued but no TxDone fires in ~200 ms,
// apply anyway. Guards against stale pending state.
static void tick_apply_backstop(RadioAoState& s) {
    if (!s_pending_radio_config_valid) { return; }
    s_pending_apply_backstop_count++;
    if (s_pending_apply_backstop_count >= kPendingApplyBackstopTicks) {
        DBG_PRINT("RADIO: backstop fired — applying pending config");
        ao_radio_commit_pending_config(s);
    }
}

// Sub 2d/2f: symmetric-revert watchdog. Station uses shorter threshold
// than vehicle so operator sees failure first visually.
// Stage T IVP-T6 — gated by ROCKETCHIP_RADIO_PERSIST. During testing
// (persistence off) the sweep deliberately drives configs that may not
// link immediately, and revert would corrupt measurements. When the gate
// is off, clear any armed state so the post-apply latches still work and
// skip the revert check entirely.
static void tick_symmetric_revert(RadioAoState& s) {
#if !defined(ROCKETCHIP_RADIO_PERSIST)
    if (s.apply_in_progress) {
        s.apply_in_progress = false;
        s.tx_since_apply = 0;
    }
    return;
#else
    if (!s.apply_in_progress) { return; }
    // Any new RX since apply means link is alive on new config.
    if (s.rx_count > s.rx_at_apply) {
        s.apply_in_progress = false;
        s.tx_since_apply = 0;
        return;
    }
    uint8_t nav_hz = s.runtime_config.nav_rate_hz;
    uint32_t threshold;
    if constexpr (job::kRole == job::DeviceRole::kVehicle) {
        threshold = (3U * static_cast<uint32_t>(nav_hz));
        if (threshold < 15U) { threshold = 15U; }
    } else {
        threshold = (3U * static_cast<uint32_t>(nav_hz) + 1U) / 2U;
        if (threshold < 6U) { threshold = 6U; }
    }
    if (s.tx_since_apply >= threshold) {
        ao_radio_revert_to_prev_config(s);
    }
#endif  // ROCKETCHIP_RADIO_PERSIST
}

// Sub-persist: debounced flash write. flash_safe_execute() is blocking
// (~100 ms) but QV cooperative scheduling means it only delays the next
// tick — no AO queues race with us.
// Stage T IVP-T6 — gated by ROCKETCHIP_RADIO_PERSIST. When disabled, any
// set/apply simply clears the persist-requested state and never reaches
// flash_safe_execute. Keeps boards deterministic across reboots during
// testing.
static void tick_persist_debounce(RadioAoState& s) {
#if !defined(ROCKETCHIP_RADIO_PERSIST)
    // Swallow any pending persist request so the state machine can't linger.
    if (s_persist_requested || s_persist_debounce_count != 0) {
        s_persist_requested = false;
        s_persist_debounce_count = 0;
    }
    (void)s;
    return;
#else
    if (!s_persist_requested ||
        s.apply_in_progress ||
        s_pending_radio_config_valid ||
        s_persist_debounce_count == 0) {
        return;
    }
    s_persist_debounce_count--;
    if (s_persist_debounce_count != 0) { return; }
    s_persist_requested = false;
    bool ok = radio_config_storage_write(&s.runtime_config);
    if (ok) {
        DBG_PRINT("RADIO: persisted config to flash");
    } else {
        DBG_ERROR("RADIO: flash persist FAILED");
    }
#endif  // ROCKETCHIP_RADIO_PERSIST
}

static void handle_radio_tick(RadioAo* me) {
    RadioAoState& s = me->state;
    if (!s.initialized) { return; }

    // Stage T (IVP-T1) — log phase change since last tick.
    static rc::RadioPhase g_stageTPrevPhase = rc::RadioPhase::kIdle;
    stage_t_log_state(g_stageTPrevPhase, s.scheduler.phase);

    if (s.scheduler.phase == rc::RadioPhase::kTxActive) {
        handle_tx_poll(me);
    }
    if (s.scheduler.rx_active()) {
        handle_rx_poll(me);
    }

    tick_apply_backstop(s);
    tick_symmetric_revert(s);
    tick_persist_debounce(s);

    stage_t_log_state(g_stageTPrevPhase, s.scheduler.phase);
    g_stageTPrevPhase = s.scheduler.phase;

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
        const auto* tx_evt = rc::evt_cast<rc::RadioTxEvt>(e);
        handle_tx_event(me, tx_evt);
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

// Stage T IVP-T5.5 sub 2b: queue a runtime radio config change.
// The apply fires on the next TxDone (from handle_tx_poll) or via the
// backstop timer (~200 ms). Caller validates the config BEFORE calling.
// Safe under QV cooperative scheduling — file-scope statics, no preemption.
void AO_Radio_set_pending_config(const rc::RadioConfig& cfg) {
    s_pending_radio_config = cfg;
    s_pending_radio_config_valid = true;
    s_pending_apply_backstop_count = 0;
}

const rc::RadioConfig* AO_Radio_get_runtime_config() {
    return &l_radioAo.state.runtime_config;
}

bool AO_Radio_consume_just_changed() {
    bool was = s_config_just_changed;
    s_config_just_changed = false;
    return was;
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
                  nullptr, 0U,
                  nullptr);
}
