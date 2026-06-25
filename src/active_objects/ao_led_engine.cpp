// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_LedEngine — NeoPixel Status LED Active Object
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
#include "pico/time.h"
// IVP-117: eskf_runner.h and core1/sensor_core1.h removed — sensor status
// evaluation migrated to AO_Notify. LedEngine only reads the seqlock for
// Core 1 vitality fallback (Council A1).

// Internal signal for animation tick (private to this AO)
enum : uint16_t {
    SIG_LED_TICK = rc::SIG_AO_MAX + 2  // After blinker and counter private signals
};

// ============================================================================
// Priority Layer Indices (highest priority = lowest index)
//
// IVP-117: simplified to 3 layers. Sensor status evaluation moved to
// AO_Notify. LedEngine is now a pure display driver with only the
// Core 1 vitality fallback (Council A1 defense-in-depth).
//
//   - kLayerFault: Core 1 vitality stall — defense-in-depth per A1.
//     Primary check lives in AO_HealthMonitor; this is the local
//     fallback in case AO_Notify or AO_HealthMonitor crash.
//   - kLayerNotify: resolved pattern from AO_Notify via SIG_LED_PATTERN
//   - kLayerIdle: blue-blink fallback
// ============================================================================
enum LedLayerIdx : uint8_t {
    kLayerFault = 0,        // Highest — Core 1 vitality stall (A1 fallback)
    kLayerNotify,           // Resolved pattern from AO_Notify
    kLayerIdle,             // Default — slow blue blink
    kLayerCount
};

// ============================================================================
// Core 1 vitality check parameters (Council A1 defense-in-depth fallback)
// ============================================================================
// At 33Hz ticks, 500ms = ~16 ticks without a core1_loop_count increment.
static constexpr uint8_t kCore1StallThreshold = 17;

// ============================================================================
// LedEngine struct
// ============================================================================
struct LedEngine {
    QActive super;
    QTimeEvt tick_timer;        // ~33Hz animation tick (every 3 ticks at 100Hz)
    uint8_t layers[kLayerCount]; // Pattern code per layer (0 = inactive)
    ws2812_mode_t last_mode;
    ws2812_rgb_t last_color;
    ws2812_rgb_t last_alt_color;   // Stage L: tracked for MODE_ALTERNATE dedup
    uint32_t last_core1_count;  // Core1 vitality defense-in-depth (A1)
    uint8_t core1_stall_ticks;  // Consecutive ticks without core1 progress
    // IVP-117: sensor_phase_start_ms removed — sensor evaluation migrated
    //          to AO_Notify.
    // IVP-116: health_fault_code removed — AO_Notify owns health faults.
};

static LedEngine g_ledEngine;
static bool g_ledEngineStarted = false;

// Stage L IVP-L1 dev override: non-zero value wins over all layers.
// Set via AO_LedEngine_dev_force_fault_layer() from the LED-test debug CLI.
// 0 = inactive (normal layer compositor applies).
static uint8_t g_devOverridePattern = 0;

// Queue depth 8: pattern change events + tick events. Callers should
// deduplicate before posting (see AO_LedEngine_post_pattern). (A6)
static QEvtPtr g_ledEngineQueue[8];

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

// Stage L: dedup helper for two-color alternating mode.
static bool rgb_eq(ws2812_rgb_t a, ws2812_rgb_t b) {
    return a.r == b.r && a.g == b.g && a.b == b.b;
}

static void led_set_alternate_if_changed(LedEngine * const me,
                                          ws2812_rgb_t a, ws2812_rgb_t b) {
    if (me->last_mode != WS2812_MODE_ALTERNATE ||
        !rgb_eq(a, me->last_color) ||
        !rgb_eq(b, me->last_alt_color)) {
        me->last_mode = WS2812_MODE_ALTERNATE;
        me->last_color = a;
        me->last_alt_color = b;
        ws2812_set_mode_alternate(a, b, 250U);  // 250ms each = 2Hz toggle
    }
}

// Map pattern value -> mode + color
static void led_apply_pattern(LedEngine * const me, uint8_t val) {
    switch (val) {
        // Calibration overlays
        case kCalNeoGyro:
        case kCalNeoLevel:       led_set_if_changed(me, WS2812_MODE_BLINK, kColorYellow); break;  // Stage L AP parity (was BREATHE+Blue)
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
        // Stage L: beacon-overlay composed patterns (state color + white alt 2Hz)
        case kFdNeoLandedBeacon:         led_set_alternate_if_changed(me, kColorGreen,   kColorWhite); break;
        case kFdNeoAbortBeacon:          led_set_alternate_if_changed(me, kColorRed,     kColorWhite); break;
        case kFaultNeoImuFailBeacon:     led_set_alternate_if_changed(me, kColorRed,     kColorWhite); break;
        case kFaultNeoEskfFailBeacon:    led_set_alternate_if_changed(me, kColorRed,     kColorWhite); break;
        case kFaultNeoBaroFailBeacon:    led_set_alternate_if_changed(me, kColorOrange,  kColorWhite); break;
        case kFaultNeoPioWdtBeacon:      led_set_alternate_if_changed(me, kColorOrange,  kColorWhite); break;
        case kFaultNeoCore1StallBeacon:  led_set_alternate_if_changed(me, kColorMagenta, kColorWhite); break;
        // Flight phase overlays
        case kFdNeoArmed:        led_set_if_changed(me, WS2812_MODE_SOLID, kColorRed); break;      // Stage L AP parity (was Orange)
        case kFdNeoBoost:        led_set_if_changed(me, WS2812_MODE_SOLID, kColorRed); break;
        case kFdNeoCoast:        led_set_if_changed(me, WS2812_MODE_SOLID, kColorYellow); break;
        case kFdNeoDrogue:       led_set_if_changed(me, WS2812_MODE_BLINK, kColorRed); break;
        case kFdNeoMain:         led_set_if_changed(me, WS2812_MODE_BLINK, kColorRed); break;
        case kFdNeoLanded:       led_set_if_changed(me, WS2812_MODE_BLINK, kColorGreen); break;
        case kFdNeoAbort:        led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        case kFdNeoBeacon:       led_set_if_changed(me, WS2812_MODE_BLINK, kColorWhite); break;
        // Stage L: pre-arm fail = yellow double-flash (100/100/100/700 ms).
        // Driver WS2812_MODE_DOUBLE_FLASH handles frame stepping.
        case kFdNeoPreArmFail:   led_set_if_changed(me, WS2812_MODE_DOUBLE_FLASH, kColorYellow); break;
        // Stage L: boot init rainbow (shares MODE_RAINBOW with mag-cal;
        // time-separated so no collision — mag-cal is explicit post-boot action).
        case kFdNeoBootInit:     led_set_if_changed(me, WS2812_MODE_RAINBOW, kColorWhite); break;
        // Sensor status patterns
        case kSensorNeoEskfInit: led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        case kSensorNeoGps3d:    led_set_if_changed(me, WS2812_MODE_SOLID, kColorGreen); break;
        case kSensorNeoGps2d:    led_set_if_changed(me, WS2812_MODE_BLINK, kColorGreen); break;
        case kSensorNeoGpsSearch: led_set_if_changed(me, WS2812_MODE_BLINK, kColorYellow); break;
        case kSensorNeoGpsNoNmea: led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorCyan); break;
        case kSensorNeoNoGps:    led_set_if_changed(me, WS2812_MODE_BLINK, kColorBlue); break;
        case kSensorNeoTimeout:  led_set_if_changed(me, WS2812_MODE_SOLID, kColorMagenta); break;
        // Fault patterns (ascending priority — IVP-106)
        case kFaultNeoPioWdt:     led_set_if_changed(me, WS2812_MODE_SOLID, kColorOrange); break;
        case kFaultNeoBaroFail:   led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorOrange); break;
        case kFaultNeoEskfFail:   led_set_if_changed(me, WS2812_MODE_BLINK, kColorRed); break;
        case kFaultNeoImuFail:    led_set_if_changed(me, WS2812_MODE_BLINK_FAST, kColorRed); break;
        case kFaultNeoSafeMode:   led_set_alternate_if_changed(me, kColorBlue, kColorWhite); break;  // Stage L: blue+white 2Hz (was SOLID+Red)
        case kFaultNeoCore1Stall: led_set_if_changed(me, WS2812_MODE_SOLID, kColorMagenta); break;
        default: break;
    }
}

//----------------------------------------------------------------------------
// Core 1 vitality check (Council A1 defense-in-depth)
//
// IVP-116: health fault decoding moved to AO_Notify. LedEngine retains
// only the Core 1 vitality check as a local safety net — if AO_Notify
// or AO_HealthMonitor crash, this still fires the Core1 stall pattern.
//
// Reads core1_loop_count from seqlock snapshot. If unchanged for
// kCore1StallThreshold consecutive ticks (~500ms), sets fault layer.

static void led_check_core1_vitality(LedEngine * const me,
                                      const shared_sensor_data_t* snap) {
    if (snap->core1_loop_count != me->last_core1_count) {
        me->last_core1_count = snap->core1_loop_count;
        me->core1_stall_ticks = 0;
        me->layers[kLayerFault] = 0;  // Clear fault when Core 1 recovers
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
    // Stage L IVP-L1: dev override beats all layers (for LED pattern test CLI).
    if (g_devOverridePattern != 0) {
        led_apply_pattern(me, g_devOverridePattern);
        return;
    }

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
    me->last_alt_color = kColorOff;
    me->last_core1_count = 0;
    me->core1_stall_ticks = 0;

    // Subscribe to LED events (IVP-116: only SIG_LED_PATTERN, from AO_Notify)
    QActive_subscribe(&me->super, rc::SIG_LED_PATTERN);

    // ~33Hz animation tick (every 3 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 3U, 3U);
    return Q_TRAN(&LedEngine_running);
}

static QState LedEngine_running(LedEngine * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_LED_TICK: {
        // IVP-117: only Core 1 vitality fallback reads the seqlock now.
        // Sensor status + GPS/ESKF evaluation moved to AO_Notify.
        shared_sensor_data_t snap{};
        if (seqlock_read(&g_sensorSeqlock, &snap)) {
            led_check_core1_vitality(me, &snap);
        }

        // Apply priority compositor and drive animation frame
        led_apply_compositor(me);
        ws2812_update();
        return Q_HANDLED();
    }

    case rc::SIG_LED_PATTERN: {
        // IVP-116: resolved pattern from AO_Notify's LED backend.
        const rc::LedPatternEvt* pe =
            rc::evt_cast<rc::LedPatternEvt>(e);
        me->layers[kLayerNotify] = pe->pattern;  // 0 clears the layer
        return Q_HANDLED();
    }

    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

//----------------------------------------------------------------------------
// Public interface

QActive * const AO_LedEngine = &g_ledEngine.super;

void AO_LedEngine_start(uint8_t prio) {
    g_ledEngineStarted = true;
    QActive_ctor(&g_ledEngine.super,
                 Q_STATE_CAST(&LedEngine_initial));

    QTimeEvt_ctorX(&g_ledEngine.tick_timer, &g_ledEngine.super,
                   SIG_LED_TICK, 0U);

    QActive_start(&g_ledEngine.super,
                  Q_PRIO(prio, 0U),
                  g_ledEngineQueue,
                  Q_DIM(g_ledEngineQueue),
                  nullptr, 0U,
                  nullptr);
}

static uint8_t g_lastPostedPattern = 0;

void AO_LedEngine_post_pattern(uint8_t pattern) {
    // Guard: no-op if AO hasn't been started yet (init-time LED callbacks
    // fire before QF_run -- e.g., flight_director_init IDLE entry action).
    if (!g_ledEngineStarted) {
        return;
    }
    // Deduplicate: don't post if pattern hasn't changed. Callers like
    // telemetry_radio_tick post every cycle -- without this, queue overflows.
    if (pattern == g_lastPostedPattern) {
        return;
    }
    g_lastPostedPattern = pattern;

    // Static event — QV does NOT copy posted events; the queue stores
    // the pointer. A stack-local event becomes a use-after-free once the
    // caller returns (QV dispatches the receiver later in its while loop
    // after the caller's stack has been reclaimed). Static storage is
    // safe here because this function is only called from handler context
    // on Core 0 under the cooperative QV scheduler — no ISR posts the
    // same event, and dedup prevents overlapping posts within one tick.
    // (Discovered during IVP-117 HW verify — see LESSONS_LEARNED.)
    static rc::LedPatternEvt g_evt;
    g_evt.super = QEVT_INITIALIZER(rc::SIG_LED_PATTERN);
    g_evt.pattern = pattern;
    QACTIVE_POST(&g_ledEngine.super, &g_evt.super, nullptr);
}

// IVP-116: AO_LedEngine_post_override() removed — RCOS now calls
// AO_Notify_post_cal_intent() instead.

// Stage L IVP-L1: dev-only forced pattern (for LED test CLI).
// Writes a single static variable that the compositor checks first.
// Safe from Core 0 handler context; not intended for flight code.
void AO_LedEngine_dev_force_fault_layer(uint8_t pattern) {
    g_devOverridePattern = pattern;
}
