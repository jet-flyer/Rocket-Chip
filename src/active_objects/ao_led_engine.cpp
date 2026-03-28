// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_LedEngine — NeoPixel Status LED Active Object (IVP-77)
//
// Sole owner of the NeoPixel. Receives SIG_LED_PATTERN events for overlays
// (calibration, RX, flight phase) and evaluates base GPS/ESKF status on a
// periodic tick (~33Hz). Replaces core1_neopixel_update() + neo_apply_override().
//
// Runs on Core 0 via QV cooperative scheduler. ws2812_update() called at
// ~33Hz (every 3 ticks at 100Hz base) — adequate for all animation modes.
//============================================================================

#include "ao_led_engine.h"
#include "rocketchip/ao_signals.h"
#include "drivers/ws2812_status.h"

// Internal signal for animation tick (private to this AO)
enum : uint16_t {
    SIG_LED_TICK = rc::SIG_AO_MAX + 2  // After blinker and counter private signals
};

// Pattern value constants — must match main.cpp kCalNeo*/kRxNeo*/kFdNeo* values.
// These are the wire format between event publishers and this AO.
static constexpr uint8_t kPatternOff         = 0;
static constexpr uint8_t kPatternCalGyro     = 1;
static constexpr uint8_t kPatternCalLevel    = 2;
static constexpr uint8_t kPatternCalBaro     = 3;
static constexpr uint8_t kPatternCalAccelWait   = 4;
static constexpr uint8_t kPatternCalAccelSample = 5;
static constexpr uint8_t kPatternCalMag      = 6;
static constexpr uint8_t kPatternCalSuccess  = 7;
static constexpr uint8_t kPatternCalFail     = 8;
static constexpr uint8_t kPatternRxReceiving = 9;
static constexpr uint8_t kPatternRxGap       = 10;
static constexpr uint8_t kPatternRxLost      = 11;
static constexpr uint8_t kPatternFdArmed     = 20;
static constexpr uint8_t kPatternFdBoost     = 21;
static constexpr uint8_t kPatternFdCoast     = 22;
static constexpr uint8_t kPatternFdDrogue    = 23;
static constexpr uint8_t kPatternFdMain      = 24;
static constexpr uint8_t kPatternFdLanded    = 25;
static constexpr uint8_t kPatternFdAbort     = 26;
static constexpr uint8_t kPatternFdBeacon    = 27;

struct LedEngine {
    QActive super;
    QTimeEvt tick_timer;    // ~33Hz animation tick (every 3 ticks at 100Hz)
    uint8_t current_pattern;   // Active override pattern (0 = no override)
    ws2812_mode_t last_mode;
    ws2812_rgb_t last_color;
};

static LedEngine l_ledEngine;
static bool s_ledEngineStarted = false;

// Queue depth 8: pattern change events + tick events. Callers should
// deduplicate before posting (see AO_LedEngine_post_pattern). (A6)
static QEvtPtr l_ledEngineQueue[8];

// Forward declarations
static QState LedEngine_initial(LedEngine * const me, QEvt const * const e);
static QState LedEngine_running(LedEngine * const me, QEvt const * const e);

//----------------------------------------------------------------------------
// Helpers (moved from main.cpp)

// Only call ws2812_set_mode() on transitions to avoid resetting animation state
static void led_set_if_changed(LedEngine * const me,
                                ws2812_mode_t mode, ws2812_rgb_t color) {
    if (mode != me->last_mode ||
        color.r != me->last_color.r ||
        color.g != me->last_color.g ||
        color.b != me->last_color.b) {
        me->last_mode  = mode;
        me->last_color = color;
        ws2812_set_mode(mode, color);
    }
}

// Map pattern value → mode + color (the 19-case switch from neo_apply_override)
static void led_apply_pattern(LedEngine * const me, uint8_t val) {
    switch (val) {
        case kPatternCalGyro:
        case kPatternCalLevel:    led_set_if_changed(me, WS2812_MODE_BREATHE, kColorBlue); break;
        case kPatternCalBaro:     led_set_if_changed(me, WS2812_MODE_BREATHE, kColorCyan); break;
        case kPatternCalAccelWait: led_set_if_changed(me, WS2812_MODE_BLINK, kColorYellow); break;
        case kPatternCalAccelSample: led_set_if_changed(me, WS2812_MODE_SOLID, kColorYellow); break;
        case kPatternCalMag:      led_set_if_changed(me, WS2812_MODE_RAINBOW, kColorWhite); break;
        case kPatternCalSuccess:  led_set_if_changed(me, WS2812_MODE_SOLID, kColorGreen); break;
        case kPatternCalFail:     led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        case kPatternRxReceiving: led_set_if_changed(me, WS2812_MODE_SOLID, kColorGreen); break;
        case kPatternRxGap:       led_set_if_changed(me, WS2812_MODE_BLINK, kColorYellow); break;
        case kPatternRxLost:      led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        case kPatternFdArmed:     led_set_if_changed(me, WS2812_MODE_SOLID, kColorOrange); break;
        case kPatternFdBoost:     led_set_if_changed(me, WS2812_MODE_SOLID, kColorRed); break;
        case kPatternFdCoast:     led_set_if_changed(me, WS2812_MODE_SOLID, kColorYellow); break;
        case kPatternFdDrogue:    led_set_if_changed(me, WS2812_MODE_BLINK, kColorRed); break;
        case kPatternFdMain:      led_set_if_changed(me, WS2812_MODE_BLINK, kColorRed); break;
        case kPatternFdLanded:    led_set_if_changed(me, WS2812_MODE_BLINK, kColorGreen); break;
        case kPatternFdAbort:     led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        case kPatternFdBeacon:    led_set_if_changed(me, WS2812_MODE_BLINK, kColorWhite); break;
        default: break;
    }
}

//----------------------------------------------------------------------------
// State handlers

static QState LedEngine_initial(LedEngine * const me, QEvt const * const e) {
    (void)e;
    me->current_pattern = kPatternOff;
    me->last_mode = WS2812_MODE_OFF;
    me->last_color = kColorOff;

    // Subscribe to LED pattern events
    QActive_subscribe(&me->super, rc::SIG_LED_PATTERN);
    QActive_subscribe(&me->super, rc::SIG_LED_OVERRIDE);

    // ~33Hz animation tick (every 3 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 3U, 3U);
    return Q_TRAN(&LedEngine_running);
}

static QState LedEngine_running(LedEngine * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_LED_TICK:
        // Animation update — drive ws2812 animation frame
        if (me->current_pattern != kPatternOff) {
            led_apply_pattern(me, me->current_pattern);
        }
        ws2812_update();
        return Q_HANDLED();

    case rc::SIG_LED_PATTERN:
    case rc::SIG_LED_OVERRIDE: {
        // Pattern change event from FD, calibration, or RX overlay
        const rc::LedPatternEvt* pe =
            reinterpret_cast<const rc::LedPatternEvt*>(e);
        me->current_pattern = pe->pattern;
        if (pe->pattern != kPatternOff) {
            led_apply_pattern(me, pe->pattern);
        } else {
            // Override cleared — default to slow blue blink until next tick
            // re-evaluates base status. This is safe because the tick runs
            // at 33Hz and will correct within 30ms.
            led_set_if_changed(me, WS2812_MODE_BLINK, kColorBlue);
        }
        return Q_HANDLED();
    }

    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

//----------------------------------------------------------------------------
// Public interface

QActive * const AO_LedEngine = &l_ledEngine.super;

void AO_LedEngine_start(uint8_t prio) {
    s_ledEngineStarted = true;
    QActive_ctor(&l_ledEngine.super,
                 Q_STATE_CAST(&LedEngine_initial));

    QTimeEvt_ctorX(&l_ledEngine.tick_timer, &l_ledEngine.super,
                   SIG_LED_TICK, 0U);

    QActive_start(&l_ledEngine.super,
                  Q_PRIO(prio, 0U),
                  l_ledEngineQueue,
                  Q_DIM(l_ledEngineQueue),
                  (void *)0, 0U,
                  (void *)0);
}

static uint8_t s_lastPostedPattern = 0;

void AO_LedEngine_post_pattern(uint8_t pattern) {
    // Guard: no-op if AO hasn't been started yet (init-time LED callbacks
    // fire before QF_run — e.g., flight_director_init IDLE entry action).
    if (!s_ledEngineStarted) {
        return;
    }
    // Deduplicate: don't post if pattern hasn't changed. Callers like
    // telemetry_radio_tick post every cycle — without this, queue overflows.
    if (pattern == s_lastPostedPattern) {
        return;
    }
    s_lastPostedPattern = pattern;

    // Direct-post a pattern event to the LED engine.
    // Stack-local — QV copies event into queue. Static was unsafe due to
    // ISR preemption race (Council C5).
    rc::LedPatternEvt evt;
    evt.super = QEVT_INITIALIZER(rc::SIG_LED_PATTERN);
    evt.pattern = pattern;
    QACTIVE_POST(&l_ledEngine.super, &evt.super, (void *)0);
}
