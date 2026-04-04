// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_Logger — Flight Data Logger Active Object (IVP-79, Phase 4)
//
// Phase 4 migration: owns ring buffer, decimator, flight table, FusedState
// builder, and event logging. 50Hz time event drives logging_tick() which
// reads ESKF epoch, builds FusedState, decimates, encodes PCM frame, and
// pushes to ring buffer.
//
// CLI commands (cmd_flush_log, cmd_erase_all_flights, etc.) remain in
// main.cpp — they call AO_Logger public accessors. Phase 7 will move
// CLI commands to cli_commands.cpp.
//============================================================================

#include "ao_logger.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/fused_state.h"
#include "rocketchip/pcm_frame.h"
#include "rocketchip/telemetry_state.h"
#include "logging/psram_init.h"
#include "logging/log_decimator.h"
#include "logging/ring_buffer.h"
#include "logging/data_convert.h"
#include "logging/flight_table.h"
#include "logging/flash_flush.h"
#include "fusion/eskf_runner.h"
#include "fusion/eskf.h"
#include "fusion/mahony_ahrs.h"
#include "fusion/confidence_gate.h"
#include "calibration/calibration_manager.h"
#include "ao_flight_director.h"
#include "flight_director/flight_director.h"
#include "ao_telemetry.h"
#include "core1/sensor_core1.h"  // g_eskf, g_eskfInitialized

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#endif

#include <math.h>

// ============================================================================
// Internal signal (private to this AO)
// ============================================================================
enum : uint16_t {
    SIG_LOG_TICK = rc::SIG_AO_MAX + 4
};

// ============================================================================
// Constants (moved from main.cpp)
// ============================================================================

// ESKF propagation rate (Hz)
static constexpr uint32_t kEskfRateHz = 200;

// Rad/deg conversion for Mahony divergence
static constexpr float kRadToDeg = 180.0F / 3.14159265F;

// SRAM fallback: 200KB ring buffer at 25Hz if PSRAM unavailable.
// 200KB / 55B = 3636 frames / 25Hz = 145 seconds.
static constexpr uint32_t kSramRingSize = 200U * 1024U;

// Decimation: 4:1 for PSRAM (200->50Hz), 8:1 for SRAM (200->25Hz)
static constexpr uint32_t kDecimationPsram = 4;
static constexpr uint32_t kDecimationSram = 8;

// Header sync: every 50 frames (~1 second at 50Hz)
static constexpr uint32_t kHeaderSyncDiv = 50;

// ============================================================================
// Module-scoped globals (moved from main.cpp)
// ============================================================================

static rc::RingBuffer g_ringBuffer;
static rc::LogDecimator g_decimator;
static bool g_loggingInitialized = false;
static uint8_t g_sramRingBuf[kSramRingSize];

// Flight table (IVP-53b) — flash-persistent flight log index.
static rc::FlightTableState g_flightTable;

// Epoch tracking — only produce output when ESKF has advanced.
static uint32_t g_lastLogEpoch = 0;

// PSRAM parameters — received from caller at start time.
static size_t g_loggerPsramSize = 0;
static bool g_loggerPsramSelfTestPassed = false;

// ============================================================================
// Extern declarations — globals still owned by main.cpp / other modules.
// ============================================================================
extern sensor_seqlock_t g_sensorSeqlock;

// ============================================================================
// FusedState builder (moved from main.cpp)
// ============================================================================

static void fused_copy_eskf_state(rc::FusedState& fused) {
    fused.q_w = g_eskf.q.w;
    fused.q_x = g_eskf.q.x;
    fused.q_y = g_eskf.q.y;
    fused.q_z = g_eskf.q.z;

    fused.pos_n = g_eskf.p.x;
    fused.pos_e = g_eskf.p.y;
    fused.pos_d = g_eskf.p.z;

    fused.vel_n = g_eskf.v.x;
    fused.vel_e = g_eskf.v.y;
    fused.vel_d = g_eskf.v.z;

    fused.accel_bias_x = g_eskf.accel_bias.x;
    fused.accel_bias_y = g_eskf.accel_bias.y;
    fused.accel_bias_z = g_eskf.accel_bias.z;

    fused.gyro_bias_x = g_eskf.gyro_bias.x;
    fused.gyro_bias_y = g_eskf.gyro_bias.y;
    fused.gyro_bias_z = g_eskf.gyro_bias.z;

    // Covariance diagnostics (max diagonal in each block)
    using rc::eskf::kIdxAttitude;
    using rc::eskf::kIdxPosition;
    using rc::eskf::kIdxVelocity;

    float patt = g_eskf.P(kIdxAttitude + 0, kIdxAttitude + 0);
    if (g_eskf.P(kIdxAttitude + 1, kIdxAttitude + 1) > patt) { patt = g_eskf.P(kIdxAttitude + 1, kIdxAttitude + 1); }
    if (g_eskf.P(kIdxAttitude + 2, kIdxAttitude + 2) > patt) { patt = g_eskf.P(kIdxAttitude + 2, kIdxAttitude + 2); }
    fused.sig_att = patt;

    float ppos = g_eskf.P(kIdxPosition + 0, kIdxPosition + 0);
    if (g_eskf.P(kIdxPosition + 1, kIdxPosition + 1) > ppos) { ppos = g_eskf.P(kIdxPosition + 1, kIdxPosition + 1); }
    if (g_eskf.P(kIdxPosition + 2, kIdxPosition + 2) > ppos) { ppos = g_eskf.P(kIdxPosition + 2, kIdxPosition + 2); }
    fused.sig_pos = ppos;

    float pvel = g_eskf.P(kIdxVelocity + 0, kIdxVelocity + 0);
    if (g_eskf.P(kIdxVelocity + 1, kIdxVelocity + 1) > pvel) { pvel = g_eskf.P(kIdxVelocity + 1, kIdxVelocity + 1); }
    if (g_eskf.P(kIdxVelocity + 2, kIdxVelocity + 2) > pvel) { pvel = g_eskf.P(kIdxVelocity + 2, kIdxVelocity + 2); }
    fused.sig_vel = pvel;

    fused.eskf_healthy = g_eskf.healthy();
    fused.zupt_active = g_eskf.last_zupt_active_;
}

// Non-static: shared between AO_FlightDirector (guard evaluation) and
// logging_tick (PCM frame encoding). Public via ao_logger.h.
void AO_Logger_populate_fused_state(rc::FusedState& fused,
                                    const shared_sensor_data_t& snap) {
    fused_copy_eskf_state(fused);

    // Baro AGL and vertical velocity
    if (snap.baro_valid) {
        fused.baro_alt_agl = calibration_get_altitude_agl(snap.pressure_pa);
        fused.baro_vvel = g_eskf.v.z;  // NED down = positive descent
    }

    fused.baro_temperature_c = snap.baro_temperature_c;
    fused.imu_temperature_c = snap.imu_temperature_c;

    // Mahony divergence
    const rc::MahonyAHRS* mahony_fused = eskf_runner_get_mahony();
    fused.mahony_div_deg = (eskf_runner_is_mahony_initialized() && mahony_fused->healthy())
        ? rc::MahonyAHRS::divergence_rad(g_eskf.q, mahony_fused->q) * kRadToDeg
        : -1.0f;

    // GPS from sensor snapshot
    fused.gps_lat_1e7 = snap.gps_lat_1e7;
    fused.gps_lon_1e7 = snap.gps_lon_1e7;
    fused.gps_alt_msl_m = snap.gps_alt_msl_m;
    fused.gps_ground_speed_mps = snap.gps_ground_speed_mps;
    fused.gps_fix_type = snap.gps_fix_type;
    fused.gps_satellites = snap.gps_satellites;

    // Confidence gate (IVP-85)
    const rc::ConfidenceState* conf_fused = eskf_runner_get_confidence();
    fused.confident = conf_fused->confident;
    fused.confidence_div_deg = conf_fused->ahrs_divergence_deg;
    fused.uncertain_ms = conf_fused->time_since_confident_ms;

    fused.flight_state = AO_FlightDirector_is_initialized()
        ? static_cast<uint8_t>(AO_FlightDirector_get_director()->state.current_phase)
        : 0;
#ifndef ROCKETCHIP_HOST_TEST
    fused.met_ms = to_ms_since_boot(get_absolute_time());
#else
    fused.met_ms = 0;
#endif
}

// ============================================================================
// Event logging (moved from main.cpp)
// ============================================================================

void AO_Logger_log_event(rc::LogEventId id,
                         uint8_t d0, uint8_t d1,
                         uint8_t d2, uint8_t d3) {
#ifndef ROCKETCHIP_HOST_TEST
    if (!g_loggingInitialized) {
        return;
    }
    rc::PcmFrameEvent frame{};
    uint8_t data[4] = {d0, d1, d2, d3};
    uint32_t met = to_ms_since_boot(get_absolute_time());
    rc::pcm_encode_event(static_cast<uint8_t>(id), data, met, frame);
    rc::ring_push(&g_ringBuffer, &frame);
#else
    (void)id; (void)d0; (void)d1; (void)d2; (void)d3;
#endif
}

// ============================================================================
// Ring buffer init (moved from main.cpp init_logging_ring)
// ============================================================================

static void init_logging_ring() {
    // Initialize logging ring buffer (IVP-52c).
    // Uses PSRAM if available (8MB at 50Hz = ~48 min), SRAM fallback otherwise
    // (200KB at 25Hz = ~145 sec). Ring buffer init writes header to memory.
    uint8_t* ringMem = nullptr;
    uint32_t ringSize = 0;
    uint32_t decRatio = kDecimationSram;

    if (g_loggerPsramSize > 0 && g_loggerPsramSelfTestPassed) {
        ringMem = rc::psram_base_ptr();
        ringSize = static_cast<uint32_t>(g_loggerPsramSize);
        decRatio = kDecimationPsram;
    } else {
        ringMem = g_sramRingBuf;
        ringSize = kSramRingSize;
        decRatio = kDecimationSram;
    }

    if (ringMem != nullptr) {
        g_loggingInitialized =
            rc::ring_init(&g_ringBuffer, ringMem, ringSize,
                          rc::kPcmFrameStandardSize, kHeaderSyncDiv);
        if (g_loggingInitialized) {
            rc::decimator_init(&g_decimator, decRatio);
        }
    }
}

// ============================================================================
// Logging tick (moved from main.cpp logging_tick)
// ============================================================================

static void logging_tick() {
    if (!g_loggingInitialized || !g_eskfInitialized) {
        return;
    }

    // Only run when ESKF has produced a new propagation
    uint32_t currentEpoch = eskf_runner_get_epoch();
    if (currentEpoch == g_lastLogEpoch) {
        return;
    }
    g_lastLogEpoch = currentEpoch;

    // Read sensor snapshot for GPS/baro/temp fields
    shared_sensor_data_t snap = {};
    if (!seqlock_read(&g_sensorSeqlock, &snap)) {
        return;  // Seqlock contention — skip this cycle
    }

    rc::FusedState fused = {};
    AO_Logger_populate_fused_state(fused, snap);

    // Feed to decimator
    rc::FusedState averaged = {};
    if (!rc::decimator_push(&g_decimator, fused, averaged)) {
        return;  // Not enough samples yet
    }

    // Convert to wire format and push to ring buffer
    rc::TelemetryState telem = {};
    rc::fused_to_telemetry(averaged, telem);

    rc::PcmFrameStandard frame = {};
    rc::pcm_encode_standard(telem, averaged.met_ms, frame);

    rc::ring_push(&g_ringBuffer, &frame);

    // Push to AO_Telemetry for radio TX encoding (IVP-94)
    AO_Telemetry_set_telem_snapshot(telem);
}

// ============================================================================
// QP/C Active Object structure and state machine
// ============================================================================

struct LoggerAo {
    QActive super;
    QTimeEvt tick_timer;    // 50Hz (every 2 ticks at 100Hz base)
};

static LoggerAo l_loggerAo;

// Queue depth 32: tick events at 50Hz accumulate during LoRa TX blocking in
// idle (50-150ms). At 50Hz, 150ms = ~8 events. Depth 32 gives ample margin.
static QEvtPtr l_loggerAoQueue[32];

static QState LoggerAo_initial(LoggerAo * const me, QEvt const * const e);
static QState LoggerAo_running(LoggerAo * const me, QEvt const * const e);

static QState LoggerAo_initial(LoggerAo * const me, QEvt const * const e) {
    (void)e;
    // 50Hz tick (every 2 ticks at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 2U, 2U);
    return Q_TRAN(&LoggerAo_running);
}

static QState LoggerAo_running(LoggerAo * const me, QEvt const * const e) {
    (void)me;
    switch (e->sig) {
    case SIG_LOG_TICK:
        logging_tick();
        return Q_HANDLED();
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Public API
// ============================================================================

QActive * const AO_Logger = &l_loggerAo.super;

void AO_Logger_start(uint8_t prio, size_t psram_size, bool psram_self_test_passed) {
    // Store PSRAM parameters for init_logging_ring()
    g_loggerPsramSize = psram_size;
    g_loggerPsramSelfTestPassed = psram_self_test_passed;

    // Initialize ring buffer and decimator
    init_logging_ring();

    // Load flight table from flash (IVP-53b)
    rc::flight_table_load(&g_flightTable);

    QActive_ctor(&l_loggerAo.super,
                 Q_STATE_CAST(&LoggerAo_initial));

    QTimeEvt_ctorX(&l_loggerAo.tick_timer, &l_loggerAo.super,
                   SIG_LOG_TICK, 0U);

    QActive_start(&l_loggerAo.super,
                  Q_PRIO(prio, 0U),
                  l_loggerAoQueue,
                  Q_DIM(l_loggerAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}

const rc::RingBuffer* AO_Logger_get_ring() {
    return &g_ringBuffer;
}

rc::RingBuffer* AO_Logger_get_ring_mut() {
    return &g_ringBuffer;
}

const rc::FlightTableState* AO_Logger_get_flight_table() {
    return &g_flightTable;
}

rc::FlightTableState* AO_Logger_get_flight_table_mut() {
    return &g_flightTable;
}

bool AO_Logger_is_initialized() {
    return g_loggingInitialized;
}
