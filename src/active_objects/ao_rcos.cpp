// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_RCOS — CLI / Terminal Active Object (Stage 12B Phase 2)
//
// 20Hz tick handler: polls USB key input, dispatches to rc_os menu system,
// renders ANSI dashboard, manages output mode cycling.
//
// Blocking calibration wizards stay in qv_idle_bridge via the cal bridge.
//============================================================================

#include "ao_rcos.h"
#include "ao_telemetry.h"
#include "ao_radio.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/board.h"
#include "rocketchip/config.h"
#include "rocketchip/job.h"
#include "cli/ansi_dashboard.h"
#include "cli/rc_os.h"

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include <stdio.h>
#endif

// ============================================================================
// Internal tick signal
// ============================================================================

enum : uint16_t {
    SIG_RCOS_TICK = rc::SIG_AO_MAX + 20
};

// ============================================================================
// Output mode state (shared via station_output_mode.h)
// ============================================================================

static StationOutputMode s_output_mode = StationOutputMode::kAnsi;

StationOutputMode AO_RCOS_get_output_mode() {
    return s_output_mode;
}

void AO_RCOS_set_output_mode(StationOutputMode mode) {
    s_output_mode = mode;
}

void AO_RCOS_cycle_output_mode() {
    switch (s_output_mode) {
    case StationOutputMode::kAnsi:    s_output_mode = StationOutputMode::kCsv;     break;
    case StationOutputMode::kCsv:     s_output_mode = StationOutputMode::kMavlink;  break;
    case StationOutputMode::kMavlink: s_output_mode = StationOutputMode::kAnsi;     break;
    case StationOutputMode::kMenu:    s_output_mode = StationOutputMode::kAnsi;     break;
    }
}

// ============================================================================
// Calibration bridge globals (Council A2)
// ============================================================================

volatile PendingCalType g_pending_cal = PendingCalType::kNone;
volatile bool           g_cal_in_progress = false;

// ============================================================================
// AO State
// ============================================================================

struct RcosAo {
    QActive   super;
    QTimeEvt  tick_timer;     // 20Hz

    // ANSI dashboard state
    uint32_t last_ansi_rx_count;
    uint32_t last_ansi_render_ms;
};

static RcosAo l_rcosAo;
static QEvtPtr l_rcosAoQueue[16];

// Forward declarations
static QState RcosAo_initial(RcosAo * const me, QEvt const * const e);
static QState RcosAo_running(RcosAo * const me, QEvt const * const e);

// ============================================================================
// Helpers (moved from main.cpp)
// ============================================================================

#ifndef ROCKETCHIP_HOST_TEST

// Enter CLI menu from dashboard
static void enter_cli_menu() {
    AO_RCOS_set_output_mode(StationOutputMode::kMenu);
    printf("\033[2J\033[H");
    printf("========================================\n");
    printf("  RocketChip OS v0.4.0 — Station RX\n");
    printf("  Board: %s\n", board::kBoardName);
    printf("========================================\n\n");
    printf("Status:  h-Help  s-Sensor  b-Boot\n");
    printf("Radio:   t-Status  r-Rate\n");
    printf("Station: g-GPS  d-Distance\n");
    printf("Output:  m-Dashboard\n");
    printf("========================================\n");
    printf("[main] ");
}

// Handle 'm' key — cycle output mode
static void handle_mode_cycle() {
    if constexpr (kRadioModeRx) {
        AO_RCOS_cycle_output_mode();
        auto new_mode = AO_RCOS_get_output_mode();
        const char* name = (new_mode == StationOutputMode::kAnsi) ? "ANSI" :
                           (new_mode == StationOutputMode::kCsv)  ? "CSV" : "MAVLink";
        if (new_mode == StationOutputMode::kAnsi) {
            printf("\033[2J\033[H");
        } else {
            printf("\n[RX] Output: %s\n", name);
        }
    } else {
        // Vehicle: toggle CSV ↔ MAVLink
        AO_Telemetry_toggle_mavlink();
        printf("\n[USB] Output: CLI\n");
    }
}

// Poll keys in dashboard/MAVLink mode — only 'm' and 'x' accepted
static void poll_dashboard_keys() {
    if (!stdio_usb_connected()) { return; }
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == 'x' || ch == 'X') { enter_cli_menu(); return; }
        if (ch == 'm' || ch == 'M') { handle_mode_cycle(); return; }
    }
}

// CLI dispatch — handles keys in dashboard/MAVLink mode.
// rc_os_update() stays in qv_idle_bridge for Menu/CSV modes
// because blocking calibration wizards run inside it.
static void cli_dispatch() {
    auto mode = AO_RCOS_get_output_mode();

    if (mode == StationOutputMode::kMavlink || mode == StationOutputMode::kAnsi) {
        poll_dashboard_keys();
    }
    // Menu and CSV modes: rc_os_update() called from idle bridge
}

// ANSI dashboard render — event-driven on new RX packet or 1Hz idle
static void ansi_render_tick(RcosAo* me) {
    if constexpr (!kRadioModeRx) { return; }
    if (AO_RCOS_get_output_mode() != StationOutputMode::kAnsi) { return; }
    if (!stdio_usb_connected()) { return; }

    const auto* rs = AO_Radio_get_state();
    const auto* rx = AO_Telemetry_get_rx_state();
    uint32_t now = to_ms_since_boot(get_absolute_time());

    bool new_packet = (rs->rx_count != me->last_ansi_rx_count);
    bool timer_expired = (now - me->last_ansi_render_ms >= 1000);
    if (!new_packet && !timer_expired) { return; }

    me->last_ansi_rx_count = rs->rx_count;
    me->last_ansi_render_ms = now;

    ansi_dashboard_render(rx->telem, rs, rx->met_ms, rx->seq, rx->valid);
}

#endif // !ROCKETCHIP_HOST_TEST

// ============================================================================
// State Handlers
// ============================================================================

static QState RcosAo_initial(RcosAo * const me, QEvt const * const e) {
    (void)e;

    me->last_ansi_rx_count = 0;
    me->last_ansi_render_ms = 0;

    // 20Hz tick (every 5 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 5U, 5U);
    return Q_TRAN(&RcosAo_running);
}

static QState RcosAo_running(RcosAo * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_RCOS_TICK: {
#ifndef ROCKETCHIP_HOST_TEST
        cli_dispatch();
        ansi_render_tick(me);
#endif
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

QActive * const AO_RCOS = &l_rcosAo.super;

void AO_RCOS_start(uint8_t prio) {
    QActive_ctor(&l_rcosAo.super,
                 Q_STATE_CAST(&RcosAo_initial));

    QTimeEvt_ctorX(&l_rcosAo.tick_timer, &l_rcosAo.super,
                   SIG_RCOS_TICK, 0U);

    QActive_start(&l_rcosAo.super,
                  Q_PRIO(prio, 0U),
                  l_rcosAoQueue,
                  Q_DIM(l_rcosAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}

void AO_RCOS_resume_tick() {
    QTimeEvt_armX(&l_rcosAo.tick_timer, 5U, 5U);
}
