// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_LedEngine — NeoPixel Status LED Active Object (IVP-77, Phase 5)
//
// Priority layer compositor. Each tick: iterate layers from highest priority
// (fault) to lowest (idle). First non-zero layer determines the LED pattern.
//
// Sensor status (GPS fix, ESKF health) evaluated on each tick via seqlock.
// Core 1 vitality monitored via core1_loop_count staleness.
//
// Runs on Core 0 via QV cooperative scheduler. ws2812_update() called at
// ~33Hz (every 3 ticks at 100Hz base) — adequate for all animation modes.
//============================================================================

#include "ao_led_engine.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/led_patterns.h"
#include "rocketchip/sensor_seqlock.h"
#include "drivers/ws2812_status.h"
#include "fusion/eskf_runner.h"
#include "core1/sensor_core1.h"
#include "pico/time.h"

// Internal signal for animation tick (private to this AO)
enum : uint16_t {
    SIG_LED_TICK = rc::SIG_AO_MAX + 2  // After blinker and counter private signals
};

// ============================================================================
// Priority Layer Indices (highest priority = lowest index)
// ============================================================================
enum LedLayerIdx : uint8_t {
    kLayerFault = 0,        // Highest — Core 1 stall, health errors
    kLayerFlightPhase,      // FD phase patterns (armed/boost/coast/etc)
    kLayerCalibration,      // CLI calibration overlays
    kLayerRadioStatus,      // RX link quality
    kLayerSensorStatus,     // GPS fix, ESKF health, idle status
    kLayerIdle,             // Default — slow blue blink
    kLayerCount
};

// ============================================================================
// Core 1 vitality check parameters (Council A4)
// ============================================================================
// At 33Hz ticks, 500ms = ~16 ticks without a core1_loop_count increment.
static constexpr uint8_t kCore1StallThreshold = 17;

// Sensor phase timeout — matches Core 1's former kSensorPhaseTimeoutMs
static constexpr uint32_t kSensorPhaseTimeoutMs = 300000;  // 5 min

// ============================================================================
// LedEngine struct
// ============================================================================
struct LedEngine {
    QActive super;
    QTimeEvt tick_timer;        // ~33Hz animation tick (every 3 ticks at 100Hz)
    uint8_t layers[kLayerCount]; // Pattern code per layer (0 = inactive)
    ws2812_mode_t last_mode;
    ws2812_rgb_t last_color;
    uint32_t sensor_phase_start_ms;  // For 5-minute timeout
    uint32_t last_core1_count;       // Last seen core1_loop_count
    uint8_t core1_stall_ticks;       // Consecutive ticks without core1 progress
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
// Helpers

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

// Map pattern value -> mode + color
static void led_apply_pattern(LedEngine * const me, uint8_t val) {
    switch (val) {
        // Calibration overlays
        case kCalNeoGyro:
        case kCalNeoLevel:       led_set_if_changed(me, WS2812_MODE_BREATHE, kColorBlue); break;
        case kCalNeoBaro:        led_set_if_changed(me, WS2812_MODE_BREATHE, kColorCyan); break;
        case kCalNeoAccelWait:   led_set_if_changed(me, WS2812_MODE_BLINK, kColorYellow); break;
        case kCalNeoAccelSample: led_set_if_changed(me, WS2812_MODE_SOLID, kColorYellow); break;
        case kCalNeoMag:         led_set_if_changed(me, WS2812_MODE_RAINBOW, kColorWhite); break;
        case kCalNeoSuccess:     led_set_if_changed(me, WS2812_MODE_SOLID, kColorGreen); break;
        case kCalNeoFail:        led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        // RX link quality overlays
        case kRxNeoReceiving:    led_set_if_changed(me, WS2812_MODE_SOLID, kColorGreen); break;
        case kRxNeoGap:          led_set_if_changed(me, WS2812_MODE_BLINK, kColorYellow); break;
        case kRxNeoLost:         led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        // Flight phase overlays
        case kFdNeoArmed:        led_set_if_changed(me, WS2812_MODE_SOLID, kColorOrange); break;
        case kFdNeoBoost:        led_set_if_changed(me, WS2812_MODE_SOLID, kColorRed); break;
        case kFdNeoCoast:        led_set_if_changed(me, WS2812_MODE_SOLID, kColorYellow); break;
        case kFdNeoDrogue:       led_set_if_changed(me, WS2812_MODE_BLINK, kColorRed); break;
        case kFdNeoMain:         led_set_if_changed(me, WS2812_MODE_BLINK, kColorRed); break;
        case kFdNeoLanded:       led_set_if_changed(me, WS2812_MODE_BLINK, kColorGreen); break;
        case kFdNeoAbort:        led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        case kFdNeoBeacon:       led_set_if_changed(me, WS2812_MODE_BLINK, kColorWhite); break;
        // Sensor status patterns (migrated from core1_neopixel_update)
        case kSensorNeoEskfInit: led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        case kSensorNeoGps3d:    led_set_if_changed(me, WS2812_MODE_SOLID, kColorGreen); break;
        case kSensorNeoGps2d:    led_set_if_changed(me, WS2812_MODE_BLINK, kColorGreen); break;
        case kSensorNeoGpsSearch: led_set_if_changed(me, WS2812_MODE_BLINK, kColorYellow); break;
        case kSensorNeoGpsNoNmea: led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorCyan); break;
        case kSensorNeoNoGps:    led_set_if_changed(me, WS2812_MODE_BLINK, kColorBlue); break;
        case kSensorNeoTimeout:  led_set_if_changed(me, WS2812_MODE_SOLID, kColorMagenta); break;
        // Fault patterns
        case kFaultNeoCore1Stall: led_set_if_changed(me, WS2812_MODE_SOLID, kColorMagenta); break;
        default: break;
    }
}

//----------------------------------------------------------------------------
// Sensor status evaluation (migrated from core1_neopixel_update)
//
// Reads seqlock snapshot and ESKF state to determine base sensor status.
// Sets layers[kLayerSensorStatus].

static void led_evaluate_sensor_status(LedEngine * const me,
                                        const shared_sensor_data_t* snap) {
    uint32_t nowMs = to_ms_since_boot(get_absolute_time());

    if ((nowMs - me->sensor_phase_start_ms) >= kSensorPhaseTimeoutMs) {
        me->layers[kLayerSensorStatus] = kSensorNeoTimeout;
    } else if (!eskf_runner_is_initialized()) {
        me->layers[kLayerSensorStatus] = kSensorNeoEskfInit;
    } else if (g_gpsInitialized) {
        if (snap->gps_fix_type >= 3) {
            me->layers[kLayerSensorStatus] = kSensorNeoGps3d;
        } else if (snap->gps_fix_type == 2) {
            me->layers[kLayerSensorStatus] = kSensorNeoGps2d;
        } else if (snap->gps_read_count > 0) {
            me->layers[kLayerSensorStatus] = kSensorNeoGpsSearch;
        } else {
            me->layers[kLayerSensorStatus] = kSensorNeoGpsNoNmea;
        }
    } else {
        me->layers[kLayerSensorStatus] = kSensorNeoNoGps;
    }
}

//----------------------------------------------------------------------------
// Core 1 vitality check (Council A4)
//
// Reads core1_loop_count from seqlock snapshot. If unchanged for
// kCore1StallThreshold consecutive ticks (~500ms), sets fault layer.

static void led_check_core1_vitality(LedEngine * const me,
                                      const shared_sensor_data_t* snap) {
    if (snap->core1_loop_count != me->last_core1_count) {
        me->last_core1_count = snap->core1_loop_count;
        me->core1_stall_ticks = 0;
        me->layers[kLayerFault] = 0;  // Clear fault
    } else {
        if (me->core1_stall_ticks < kCore1StallThreshold) {
            me->core1_stall_ticks++;
        }
        if (me->core1_stall_ticks >= kCore1StallThreshold) {
            me->layers[kLayerFault] = kFaultNeoCore1Stall;
        }
    }
}

//----------------------------------------------------------------------------
// Priority compositor: iterate layers from highest to lowest.
// First non-zero layer wins. If all zero, default blue blink (idle layer).

static void led_apply_compositor(LedEngine * const me) {
    // Idle layer is always set as fallback
    me->layers[kLayerIdle] = kSensorNeoNoGps;  // Blue blink default

    for (uint8_t i = 0; i < kLayerCount; i++) {
        if (me->layers[i] != 0) {
            led_apply_pattern(me, me->layers[i]);
            return;
        }
    }
    // Should not reach here (kLayerIdle is always non-zero), but just in case
    led_set_if_changed(me, WS2812_MODE_BLINK, kColorBlue);
}

//----------------------------------------------------------------------------
// State handlers

static QState LedEngine_initial(LedEngine * const me, QEvt const * const e) {
    (void)e;
    for (uint8_t i = 0; i < kLayerCount; i++) {
        me->layers[i] = 0;
    }
    me->last_mode = WS2812_MODE_OFF;
    me->last_color = kColorOff;
    me->sensor_phase_start_ms = to_ms_since_boot(get_absolute_time());
    me->last_core1_count = 0;
    me->core1_stall_ticks = 0;

    // Subscribe to LED events
    QActive_subscribe(&me->super, rc::SIG_LED_PATTERN);
    QActive_subscribe(&me->super, rc::SIG_LED_OVERRIDE);
    QActive_subscribe(&me->super, rc::SIG_RADIO_STATUS);

    // ~33Hz animation tick (every 3 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 3U, 3U);
    return Q_TRAN(&LedEngine_running);
}

static QState LedEngine_running(LedEngine * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_LED_TICK: {
        // Read sensor snapshot for status evaluation + Core 1 vitality
        shared_sensor_data_t snap{};
        if (seqlock_read(&g_sensorSeqlock, &snap)) {
            led_evaluate_sensor_status(me, &snap);
            led_check_core1_vitality(me, &snap);
        }

        // Apply priority compositor and drive animation frame
        led_apply_compositor(me);
        ws2812_update();
        return Q_HANDLED();
    }

    case rc::SIG_LED_PATTERN: {
        // Flight phase pattern from FD
        const rc::LedPatternEvt* pe =
            reinterpret_cast<const rc::LedPatternEvt*>(e);
        me->layers[kLayerFlightPhase] = pe->pattern;  // 0 clears the layer
        return Q_HANDLED();
    }

    case rc::SIG_LED_OVERRIDE: {
        // Calibration overlay from CLI
        const rc::LedPatternEvt* pe =
            reinterpret_cast<const rc::LedPatternEvt*>(e);
        me->layers[kLayerCalibration] = pe->pattern;  // 0 clears the layer
        return Q_HANDLED();
    }

    case rc::SIG_RADIO_STATUS: {
        // Radio link quality from AO_Radio
        const rc::RadioStatusEvt* re =
            reinterpret_cast<const rc::RadioStatusEvt*>(e);
        switch (re->link_quality) {
            case 1: me->layers[kLayerRadioStatus] = kRxNeoLost; break;
            case 2: me->layers[kLayerRadioStatus] = kRxNeoGap; break;
            case 3: me->layers[kLayerRadioStatus] = kRxNeoReceiving; break;
            default: me->layers[kLayerRadioStatus] = 0; break;  // Clear layer
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
    // fire before QF_run -- e.g., flight_director_init IDLE entry action).
    if (!s_ledEngineStarted) {
        return;
    }
    // Deduplicate: don't post if pattern hasn't changed. Callers like
    // telemetry_radio_tick post every cycle -- without this, queue overflows.
    if (pattern == s_lastPostedPattern) {
        return;
    }
    s_lastPostedPattern = pattern;

    // Direct-post a pattern event to the LED engine.
    // Stack-local -- QV copies event into queue. Static was unsafe due to
    // ISR preemption race (Council C5).
    rc::LedPatternEvt evt;
    evt.super = QEVT_INITIALIZER(rc::SIG_LED_PATTERN);
    evt.pattern = pattern;
    QACTIVE_POST(&l_ledEngine.super, &evt.super, (void *)0);
}

static uint8_t s_lastPostedOverride = 0;

void AO_LedEngine_post_override(uint8_t pattern) {
    if (!s_ledEngineStarted) {
        return;
    }
    if (pattern == s_lastPostedOverride) {
        return;
    }
    s_lastPostedOverride = pattern;

    rc::LedPatternEvt evt;
    evt.super = QEVT_INITIALIZER(rc::SIG_LED_OVERRIDE);
    evt.pattern = pattern;
    QACTIVE_POST(&l_ledEngine.super, &evt.super, (void *)0);
}
