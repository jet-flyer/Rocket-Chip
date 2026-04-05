// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_RCOS — CLI / Terminal Active Object (Phase D: non-blocking cal UI)
//
// 20Hz tick handler: polls USB key input, dispatches to rc_os menu system,
// renders ANSI dashboard, manages output mode cycling, and drives the
// calibration UI state machine.
//
// All calibration wizards are non-blocking. The cal UI state machine runs
// at 20Hz, polling calibration_manager for completion and reading keys
// for user interaction (Enter/ESC).
//============================================================================

#include "ao_rcos.h"
#include "ao_telemetry.h"
#include "ao_radio.h"
#include "ao_led_engine.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/board.h"
#include "rocketchip/config.h"
#include "rocketchip/job.h"
#include "rocketchip/led_patterns.h"
#include "cli/rc_os_dashboard.h"
#include "cli/rc_os.h"
#include "calibration/calibration_manager.h"
#include "calibration/cal_hooks.h"
#include "drivers/i2c_bus.h"
#include "ao_logger.h"
#include "logging/flight_table.h"
#include "logging/flash_flush.h"

// Forward declarations for non-blocking erase/download completion
// (actual implementations in rc_os_commands.cpp)
void cli_do_erase_flights();
void cli_do_download_flight(int flight_num);

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include "pico/stdio_usb.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
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
// Calibration UI State Machine (Phase D3)
// ============================================================================

#ifndef ROCKETCHIP_HOST_TEST

enum class CalUiState : uint8_t {
    kIdle = 0,
    // Async cals (gyro, level, baro) — Core 1 feeds, poll completion
    kAsyncPrompt,        // "ENTER to start, ESC to cancel" — wait for keypress
    kAsyncWaiting,
    // 6-position accel
    k6posPrompt,         // Print "Place in position N, press Enter"
    k6posSampling,       // Collecting, show progress
    k6posValidating,     // Finalize position, check result
    // Compass
    kMagPrompt,          // "ENTER to start, ESC to cancel" for mag
    kMagCollecting,      // Collecting mag samples, show progress/coverage
    // Common
    kComputing,          // Running fit (6pos or mag)
    kResult,             // Show pass/fail, auto-save
    // Blocking-replacement states (non-blocking char-by-char input)
    kResetConfirm,       // Reading "YES" + Enter for cal reset
    kEraseConfirm,       // Reading "yes" + Enter for flight erase
    kFlightNumInput,     // Reading flight number for download
    // Wizard mode
    kWizardNext,         // Advance to next wizard step
};

// Wizard step indices
static constexpr uint8_t kWizardGyro   = 0;
static constexpr uint8_t kWizardLevel  = 1;
static constexpr uint8_t kWizard6pos   = 2;
static constexpr uint8_t kWizardMag    = 3;
static constexpr uint8_t kWizardDone   = 4;

// Progress display granularity
static constexpr uint8_t  kCalProgressStep = 10;   // Print dot every 10%
static constexpr uint32_t kNeoFlashMs      = 1000;  // LED flash after step

// Mag cal display intervals
static constexpr uint16_t kMagProgressInterval = 25;   // Print every N samples
static constexpr uint16_t kMagMinSamples       = 50;   // Minimum for fit

// 6-pos position count (mirrors calibration_manager)
static constexpr uint8_t kAccel6posPositions = 6;

// Per-position instructions
static const char* const kPositionInstructions[kAccel6posPositions] = {
    "Place board FLAT on table, component side UP",
    "Stand board on its LEFT edge",
    "Stand board on its RIGHT edge",
    "Stand board on USB connector end (nose down)",
    "Stand board on opposite end from USB (nose up)",
    "Place board FLAT on table, component side DOWN (inverted)"
};

// ============================================================================
// AO State (extended for cal UI)
// ============================================================================

struct RcosAo {
    QActive   super;
    QTimeEvt  tick_timer;     // 20Hz

    // ANSI dashboard state
    uint32_t last_ansi_rx_count;
    uint32_t last_ansi_render_ms;

    // Calibration UI state
    CalUiState cal_ui_state;
    uint8_t    cal_async_type;       // 0=gyro, 1=level, 2=baro (for kAsyncPrompt)
    uint8_t    cal_6pos_position;     // 0-5 for current position
    uint8_t    cal_wizard_step;       // 0-3 for wizard sequence
    bool       cal_wizard_active;     // Running full wizard vs single cal
    uint8_t    cal_last_progress;     // Last progress % for dot printing
    bool       cal_is_6pos;           // True if current compute is 6pos (vs mag)

    // Mag cal tracking
    uint16_t   mag_last_printed_count;
    uint32_t   mag_total_reads;
    uint32_t   mag_reject_close;
    uint32_t   mag_reject_range;

    // Reset confirmation state
    char       confirm_buf[8];
    uint8_t    confirm_idx;
    uint8_t    confirm_timeout_ticks;   // 10s at 20Hz = 200 ticks

    // Wizard counters
    uint8_t    wizard_passed;
    uint8_t    wizard_failed;
    uint8_t    wizard_skipped;
};

static RcosAo l_rcosAo;
static QEvtPtr l_rcosAoQueue[16];

// Forward declarations
static QState RcosAo_initial(RcosAo * const me, QEvt const * const e);
static QState RcosAo_running(RcosAo * const me, QEvt const * const e);
static void cal_ui_tick(RcosAo* me);

// ============================================================================
// Helpers (moved from main.cpp)
// ============================================================================

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
// Menu/CSV modes: rc_os_update() called from AO tick (Phase D5).
static void cli_dispatch() {
    auto mode = AO_RCOS_get_output_mode();

    if (mode == StationOutputMode::kMavlink || mode == StationOutputMode::kAnsi) {
        poll_dashboard_keys();
    }
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

// ============================================================================
// Calibration UI — LED helper
// ============================================================================

static void cal_neo(uint8_t mode) {
    AO_LedEngine_post_override(mode);
}

// ============================================================================
// Calibration UI — save helper (shared by 6-pos and mag)
// ============================================================================

static void cal_save_to_flash() {
    printf("Saving to flash...");
    (void)fflush(stdout);
    cal_result_t saveResult = calibration_save();
    if (saveResult == CAL_RESULT_OK) {
        printf(" OK!\n");
        if (!i2c_bus_reset()) {
            printf("[WARN] I2C bus reset failed after save\n");
        }
    } else {
        printf(" FAILED (%d)\n", saveResult);
    }
    // Signal Core 1 to reload calibration
    cal_post_hook();
}

// ============================================================================
// Calibration UI — State Machine Tick (20Hz)
// ============================================================================

// Read one key without blocking. Returns the character, or PICO_ERROR_TIMEOUT.
static int cal_ui_read_key() {
    return getchar_timeout_us(0);
}

// Transition helper: go to wizard next step or idle
static CalUiState cal_ui_next_or_idle(RcosAo* me) {
    return me->cal_wizard_active ? CalUiState::kWizardNext : CalUiState::kIdle;
}

// ---- Async cal prompt: wait for ENTER before starting ----
static constexpr uint8_t kAsyncGyro  = 0;
static constexpr uint8_t kAsyncLevel = 1;
static constexpr uint8_t kAsyncBaro  = 2;

static void cal_ui_start_async_cal(RcosAo* me) {
    cal_result_t r = CAL_RESULT_BUSY;
    uint8_t neo = kCalNeoOff;
    switch (me->cal_async_type) {
    case kAsyncGyro:
        r = calibration_start_gyro();
        neo = kCalNeoGyro;
        break;
    case kAsyncLevel:
        r = calibration_start_accel_level();
        neo = kCalNeoLevel;
        break;
    case kAsyncBaro:
        r = calibration_start_baro();
        neo = kCalNeoBaro;
        break;
    }
    if (r != CAL_RESULT_OK) {
        printf("ERROR: Failed to start cal (%d)\n", static_cast<int>(r));
        cal_ui_next_or_idle(me);
        return;
    }
    printf("Sampling");
    (void)fflush(stdout);
    cal_neo(neo);
    me->cal_last_progress = 0;
    me->cal_ui_state = CalUiState::kAsyncWaiting;
}

static void cal_ui_handle_async_prompt(RcosAo* me) {
    int ch = cal_ui_read_key();
    if (ch == 27 || ch == 'x' || ch == 'X') {
        printf("Skipped.\n");
        cal_ui_next_or_idle(me);
        return;
    }
    if (ch == '\r' || ch == '\n') {
        cal_ui_start_async_cal(me);
        return;
    }
    // Ignore other keys — keep waiting
}

// ---- Async cals (gyro, level, baro) ----
static void cal_ui_handle_async_waiting(RcosAo* me) {
    int ch = cal_ui_read_key();
    if (ch == 27 || ch == 'x' || ch == 'X') {
        calibration_cancel();
        printf("\nCancelled.\n");
        cal_neo(kCalNeoOff);
        me->cal_ui_state = CalUiState::kIdle;
        if (me->cal_wizard_active) {
            me->wizard_skipped++;
            me->cal_ui_state = CalUiState::kWizardNext;
        }
        return;
    }
    if (!stdio_usb_connected()) {
        calibration_cancel();
        cal_neo(kCalNeoOff);
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }

    // Progress dots
    uint8_t progress = calibration_get_progress();
    if (progress / kCalProgressStep > me->cal_last_progress / kCalProgressStep) {
        printf(".");
        (void)fflush(stdout);
    }
    me->cal_last_progress = progress;

    // Check completion
    if (!calibration_is_active()) {
        cal_state_t state = calibration_manager_get_state();
        if (state == CAL_STATE_COMPLETE) {
            printf(" OK!\n");
            calibration_reset_state();
            cal_save_to_flash();
            cal_neo(kCalNeoSuccess);
            if (me->cal_wizard_active) { me->wizard_passed++; }
        } else {
            cal_result_t result = calibration_get_result();
            if (result == CAL_RESULT_MOTION_DETECTED) {
                printf(" FAILED - motion detected!\n");
            } else {
                printf(" FAILED (%d)\n", result);
            }
            calibration_reset_state();
            cal_neo(kCalNeoFail);
            if (me->cal_wizard_active) { me->wizard_failed++; }
        }
        me->cal_ui_state = cal_ui_next_or_idle(me);
    }
}

// ---- 6-pos: prompt for position ----
static void cal_ui_handle_6pos_prompt(RcosAo* me) {
    int ch = cal_ui_read_key();
    if (ch == 27 || ch == 'x' || ch == 'X') {
        printf("\nCalibration cancelled.\n");
        calibration_reset_6pos();
        cal_neo(kCalNeoOff);
        if (me->cal_wizard_active) { me->wizard_skipped++; }
        me->cal_ui_state = cal_ui_next_or_idle(me);
        return;
    }
    if (!stdio_usb_connected()) {
        calibration_reset_6pos();
        cal_neo(kCalNeoOff);
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    if (ch == '\r' || ch == '\n') {
        cal_result_t r = calibration_start_6pos_position(me->cal_6pos_position);
        if (r != CAL_RESULT_OK) {
            printf("ERROR: Failed to start position %d (%d)\n",
                   me->cal_6pos_position + 1, r);
            calibration_reset_6pos();
            cal_neo(kCalNeoFail);
            if (me->cal_wizard_active) { me->wizard_failed++; }
            me->cal_ui_state = cal_ui_next_or_idle(me);
            return;
        }
        printf("  Sampling... hold still");
        (void)fflush(stdout);
        cal_neo(kCalNeoAccelSample);
        me->cal_ui_state = CalUiState::k6posSampling;
    }
}

// ---- 6-pos: collecting samples ----
static void cal_ui_handle_6pos_sampling(RcosAo* me) {
    int ch = cal_ui_read_key();
    if (ch == 27 || ch == 'x' || ch == 'X') {
        printf("\nCalibration cancelled.\n");
        calibration_reset_6pos();
        cal_neo(kCalNeoOff);
        if (me->cal_wizard_active) { me->wizard_skipped++; }
        me->cal_ui_state = cal_ui_next_or_idle(me);
        return;
    }

    uint16_t count = calibration_6pos_position_sample_count();
    if (count > 0 && count % 13 == 0) {
        printf(".");
        (void)fflush(stdout);
    }

    if (calibration_6pos_position_done()) {
        me->cal_ui_state = CalUiState::k6posValidating;
    }
}

// Print the prompt for the next 6-pos position
static void cal_ui_print_6pos_position(RcosAo* me) {
    printf("\n--- Position %d/%d: %s ---\n",
           me->cal_6pos_position + 1, kAccel6posPositions,
           calibration_get_6pos_name(me->cal_6pos_position));
    printf("  %s\n", kPositionInstructions[me->cal_6pos_position]);
    printf("  Press ENTER when ready, ESC to cancel.\n");
    (void)fflush(stdout);
    cal_neo(kCalNeoAccelWait);
    me->cal_ui_state = CalUiState::k6posPrompt;
}

// ---- 6-pos: finalize position ----
static void cal_ui_handle_6pos_validating(RcosAo* me) {
    cal_result_t r = calibration_finalize_6pos_position();
    if (r == CAL_RESULT_OK) {
        const float* avg = calibration_get_6pos_avg(me->cal_6pos_position);
        printf(" OK!\n");
        printf("  Avg: X=%.2f Y=%.2f Z=%.2f\n",
               (double)avg[0], (double)avg[1], (double)avg[2]);

        me->cal_6pos_position++;
        if (me->cal_6pos_position >= kAccel6posPositions) {
            printf("\nAll 6 positions collected!\n");
            printf("Computing ellipsoid fit...");
            (void)fflush(stdout);
            me->cal_is_6pos = true;
            me->cal_ui_state = CalUiState::kComputing;
        } else {
            cal_ui_print_6pos_position(me);
        }
    } else {
        printf(" FAILED (%d)\n", r);
        calibration_reset_6pos();
        cal_neo(kCalNeoFail);
        if (me->cal_wizard_active) { me->wizard_failed++; }
        me->cal_ui_state = cal_ui_next_or_idle(me);
    }
}

// Process a single mag sample and update progress display
static void cal_ui_mag_process_sample(RcosAo* me,
                                      float mx, float my, float mz) {
    me->mag_total_reads++;
    mag_feed_result_t result = calibration_feed_mag_sample(mx, my, mz);
    uint16_t count = calibration_get_mag_sample_count();

    if (result == mag_feed_result_t::REJECTED_CLOSE) {
        me->mag_reject_close++;
    } else if (result == mag_feed_result_t::REJECTED_RANGE) {
        me->mag_reject_range++;
    }

    if (result == mag_feed_result_t::ACCEPTED && count == 1) {
        float mag = sqrtf(mx*mx + my*my + mz*mz);
        printf("  First sample: |M|=%.1f uT (read %lu)\n",
               (double)mag, (unsigned long)me->mag_total_reads);
        (void)fflush(stdout);
    }
    if (result == mag_feed_result_t::ACCEPTED &&
        count >= me->mag_last_printed_count + kMagProgressInterval) {
        me->mag_last_printed_count = count;
        uint8_t coverage = calibration_get_mag_coverage_pct();
        printf("  Samples: %u  Coverage: %u%%  (read %lu, close:%lu range:%lu)\n",
               count, coverage, (unsigned long)me->mag_total_reads,
               (unsigned long)me->mag_reject_close,
               (unsigned long)me->mag_reject_range);
        (void)fflush(stdout);
    }

    if (result == mag_feed_result_t::BUFFER_FULL) {
        uint16_t finalCount = calibration_get_mag_sample_count();
        uint8_t finalCoverage = calibration_get_mag_coverage_pct();
        printf("\nCollection complete: %u samples, %u%% coverage\n",
               finalCount, finalCoverage);

        if (finalCount < kMagMinSamples) {
            printf("ERROR: Not enough samples (%u < %u)\n",
                   finalCount, kMagMinSamples);
            calibration_reset_mag_cal();
            rc_os_mag_cal_active.store(false, std::memory_order_release);
            cal_neo(kCalNeoFail);
            if (me->cal_wizard_active) { me->wizard_failed++; }
            me->cal_ui_state = cal_ui_next_or_idle(me);
        } else {
            printf("Computing ellipsoid fit...");
            (void)fflush(stdout);
            me->cal_is_6pos = false;
            me->cal_ui_state = CalUiState::kComputing;
        }
    }
}

// ---- Mag prompt: wait for ENTER before starting collection ----
static void cal_ui_begin_mag_collection(RcosAo* me) {
    printf("Collecting...\n");
    (void)fflush(stdout);
    calibration_reset_mag_cal();
    if (rc_os_reset_mag_staleness != nullptr) {
        rc_os_reset_mag_staleness();
    }
    rc_os_mag_cal_active.store(true, std::memory_order_release);
    cal_neo(kCalNeoMag);
    me->mag_last_printed_count = 0;
    me->mag_total_reads = 0;
    me->mag_reject_close = 0;
    me->mag_reject_range = 0;
    me->cal_ui_state = CalUiState::kMagCollecting;
}

static void cal_ui_handle_mag_prompt(RcosAo* me) {
    int ch = cal_ui_read_key();
    if (ch == 27 || ch == 'x' || ch == 'X') {
        printf("Skipped.\n");
        cal_ui_next_or_idle(me);
        return;
    }
    if (ch == '\r' || ch == '\n') {
        cal_ui_begin_mag_collection(me);
        return;
    }
}

// ---- Mag collecting ----
static void cal_ui_handle_mag_collecting(RcosAo* me) {
    int ch = cal_ui_read_key();
    if (ch == 27 || ch == 'x' || ch == 'X') {
        printf("\nCompass calibration cancelled.\n");
        calibration_reset_mag_cal();
        rc_os_mag_cal_active.store(false, std::memory_order_release);
        cal_neo(kCalNeoOff);
        if (me->cal_wizard_active) { me->wizard_skipped++; }
        me->cal_ui_state = cal_ui_next_or_idle(me);
        return;
    }
    if (!stdio_usb_connected()) {
        calibration_reset_mag_cal();
        rc_os_mag_cal_active.store(false, std::memory_order_release);
        cal_neo(kCalNeoOff);
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }

    float mx = 0.0F;
    float my = 0.0F;
    float mz = 0.0F;
    if (rc_os_read_mag != nullptr && rc_os_read_mag(&mx, &my, &mz)) {
        cal_ui_mag_process_sample(me, mx, my, mz);
    }
}

// ---- Computing fit (6pos or mag) ----
static void cal_ui_handle_computing(RcosAo* me) {
    cal_result_t fitResult;
    if (me->cal_is_6pos) {
        fitResult = calibration_compute_6pos();
    } else {
        fitResult = calibration_compute_mag_cal();
    }

    if (fitResult != CAL_RESULT_OK) {
        printf(" FAILED (%d)\n", fitResult);
        if (me->cal_is_6pos) {
            printf("Fit failed - params out of range.\n");
            calibration_reset_6pos();
        } else {
            printf("Ellipsoid fit did not converge or params out of range.\n");
            calibration_reset_mag_cal();
            rc_os_mag_cal_active.store(false, std::memory_order_release);
        }
        cal_neo(kCalNeoFail);
        if (me->cal_wizard_active) { me->wizard_failed++; }
        me->cal_ui_state = cal_ui_next_or_idle(me);
    } else {
        printf(" OK!\n\n");
        me->cal_ui_state = CalUiState::kResult;
    }
}

// ---- Result: print and auto-save ----
static void cal_ui_handle_result(RcosAo* me) {
    const calibration_store_t* cal = calibration_manager_get();
    if (me->cal_is_6pos) {
        printf("Results:\n");
        printf("  Offset: X=%.4f Y=%.4f Z=%.4f\n",
               (double)cal->accel.offset.x,
               (double)cal->accel.offset.y,
               (double)cal->accel.offset.z);
        printf("  Scale:  X=%.4f Y=%.4f Z=%.4f\n",
               (double)cal->accel.scale.x,
               (double)cal->accel.scale.y,
               (double)cal->accel.scale.z);
        printf("\n");
    } else {
        printf("Results:\n");
        printf("  Offset: X=%.2f Y=%.2f Z=%.2f uT\n",
               (double)cal->mag.offset.x,
               (double)cal->mag.offset.y,
               (double)cal->mag.offset.z);
        printf("  Scale:  X=%.4f Y=%.4f Z=%.4f\n",
               (double)cal->mag.scale.x,
               (double)cal->mag.scale.y,
               (double)cal->mag.scale.z);
        printf("  Offdiag: XY=%.4f XZ=%.4f YZ=%.4f\n",
               (double)cal->mag.offdiag.x,
               (double)cal->mag.offdiag.y,
               (double)cal->mag.offdiag.z);
        printf("  Expected radius: %.2f uT\n",
               (double)cal->mag.expected_radius);
        printf("  Fitness (RMS): %.3f uT\n",
               (double)calibration_get_mag_fitness());
        printf("\n");
    }

    cal_save_to_flash();

    if (me->cal_is_6pos) {
        calibration_reset_6pos();
        printf("\n6-position accel calibration complete.\n");
    } else {
        calibration_reset_mag_cal();
        rc_os_mag_cal_active.store(false, std::memory_order_release);
        printf("\nCompass calibration complete.\n");
    }
    cal_neo(kCalNeoSuccess);

    if (me->cal_wizard_active) { me->wizard_passed++; }
    me->cal_ui_state = cal_ui_next_or_idle(me);
}

// Print the wizard completion summary
static void cal_ui_wizard_print_summary(RcosAo* me) {
    const calibration_store_t* calFinal = calibration_manager_get();
    printf("\n========================================\n");
    printf("  Wizard Complete\n");
    printf("========================================\n");
    printf("  Passed:  %d\n", me->wizard_passed);
    printf("  Failed:  %d\n", me->wizard_failed);
    printf("  Skipped: %d\n", me->wizard_skipped);
    printf("\nCalibration flags: 0x%02lX\n",
           (unsigned long)calFinal->cal_flags);
    printf("  Gyro:  %s\n",
           (calFinal->cal_flags & CAL_STATUS_GYRO) != 0 ? "OK" : "--");
    printf("  Level: %s\n",
           (calFinal->cal_flags & CAL_STATUS_LEVEL) != 0 ? "OK" : "--");
    printf("  Baro:  %s\n",
           (calFinal->cal_flags & CAL_STATUS_BARO) != 0 ? "OK" : "--");
    printf("  Accel: %s\n",
           (calFinal->cal_flags & CAL_STATUS_ACCEL_6POS) != 0 ? "6POS" : "--");
    printf("  Mag:   %s\n",
           (calFinal->cal_flags & CAL_STATUS_MAG) != 0 ? "OK" : "--");
    printf("========================================\n\n");
    cal_neo(kCalNeoOff);
    me->cal_wizard_active = false;
    me->cal_ui_state = CalUiState::kIdle;
}

// Start wizard step: gyro
static void cal_ui_wizard_start_gyro(RcosAo* me) {
    printf("\n--- Step 1/4: Gyro Calibration ---\n");
    if (!rc_os_imu_available) {
        printf("  SKIPPED (IMU not available)\n");
        me->wizard_skipped++;
        return;  // stays in kWizardNext
    }
    printf("Keep device STILL for ~2s.\n");
    printf("ENTER to start, 'x' to skip.\n");
    me->cal_async_type = kAsyncGyro;
    me->cal_ui_state = CalUiState::kAsyncPrompt;
}

// Start wizard step: level
static void cal_ui_wizard_start_level(RcosAo* me) {
    printf("\n--- Step 2/4: Level Calibration ---\n");
    if (!rc_os_imu_available) {
        printf("  SKIPPED (IMU not available)\n");
        me->wizard_skipped++;
        return;
    }
    printf("Keep device FLAT and STILL for ~1s.\n");
    printf("ENTER to start, 'x' to skip.\n");
    me->cal_async_type = kAsyncLevel;
    me->cal_ui_state = CalUiState::kAsyncPrompt;
}

// Start wizard step: 6-position accel
static void cal_ui_wizard_start_6pos(RcosAo* me) {
    printf("\n--- Step 3/4: 6-Position Accel Calibration ---\n");
    if (!rc_os_imu_available) {
        printf("  SKIPPED (IMU not available)\n");
        me->wizard_skipped++;
        return;
    }
    printf("Calibrates offset, scale, and cross-axis coupling.\n");
    calibration_reset_6pos();
    me->cal_6pos_position = 0;
    printf("--- Position 1/%d: %s ---\n",
           kAccel6posPositions,
           calibration_get_6pos_name(0));
    printf("  %s\n", kPositionInstructions[0]);
    printf("  Press ENTER when ready, ESC to cancel.\n");
    (void)fflush(stdout);
    cal_neo(kCalNeoAccelWait);
    me->cal_ui_state = CalUiState::k6posPrompt;
}

// Start wizard step: compass
static void cal_ui_wizard_start_mag(RcosAo* me) {
    printf("\n--- Step 4/4: Compass Calibration ---\n");
    if (rc_os_read_mag == nullptr) {
        printf("  SKIPPED (mag read callback not set)\n");
        me->wizard_skipped++;
        return;
    }
    printf("Rotate device through all orientations.\n");
    printf("ENTER to start, 'x' to skip.\n");
    me->cal_ui_state = CalUiState::kMagPrompt;
}

// Dispatch to the appropriate wizard step handler
static void cal_ui_wizard_start_step(RcosAo* me) {
    switch (me->cal_wizard_step) {
    case kWizardGyro:  cal_ui_wizard_start_gyro(me);  break;
    case kWizardLevel: cal_ui_wizard_start_level(me); break;
    case kWizard6pos:  cal_ui_wizard_start_6pos(me);  break;
    case kWizardMag:   cal_ui_wizard_start_mag(me);   break;
    default:           me->cal_ui_state = CalUiState::kIdle; break;
    }
}

// ---- Wizard: advance to next step ----
static void cal_ui_handle_wizard_next(RcosAo* me) {
    me->cal_wizard_step++;

    if (me->cal_wizard_step >= kWizardDone) {
        cal_ui_wizard_print_summary(me);
        return;
    }

    cal_ui_wizard_start_step(me);
}

// ---- Reset confirmation (non-blocking, one char per tick) ----
static constexpr uint8_t kResetTimeoutTicks = 200;  // 10s at 20Hz

static void cal_ui_handle_reset_confirm(RcosAo* me) {
    me->confirm_timeout_ticks++;
    if (me->confirm_timeout_ticks >= kResetTimeoutTicks) {
        printf("\nTimeout - cancelled.\n");
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }

    int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT) {
        return;  // No input this tick — keep waiting
    }
    if (ch == 27) {  // ESC
        printf("\nCancelled.\n");
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    if (ch == '\r' || ch == '\n') {
        me->confirm_buf[me->confirm_idx] = '\0';
        printf("\n");
        if (strcmp(me->confirm_buf, "YES") == 0) {
            printf("Resetting all calibration data...");
            (void)fflush(stdout);
            cal_result_t result = calibration_reset();
            if (result == CAL_RESULT_OK) {
                printf(" OK!\n");
            } else {
                printf(" FAILED (%d)\n", static_cast<int>(result));
            }
        } else {
            printf("Cancelled (need to type 'YES' then ENTER).\n");
        }
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    if (me->confirm_idx < 7) {
        me->confirm_buf[me->confirm_idx++] = static_cast<char>(ch);
        putchar(ch);
        (void)fflush(stdout);
    }
}

// ---- Erase-all-flights confirmation (non-blocking) ----
static void cal_ui_handle_erase_confirm(RcosAo* me) {
    me->confirm_timeout_ticks++;
    if (me->confirm_timeout_ticks >= kResetTimeoutTicks) {
        printf("\nTimeout - erase cancelled.\n");
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT) { return; }
    if (ch == 27) {
        printf("\nErase cancelled.\n");
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    if (ch == '\r' || ch == '\n') {
        me->confirm_buf[me->confirm_idx] = '\0';
        printf("\n");
        if (me->confirm_idx == 3 &&
            me->confirm_buf[0] == 'y' &&
            me->confirm_buf[1] == 'e' &&
            me->confirm_buf[2] == 's') {
            // Perform the erase — this calls flash_safe_execute internally
            cli_do_erase_flights();
        } else {
            printf("Erase cancelled.\n");
        }
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    if (me->confirm_idx < 7) {
        me->confirm_buf[me->confirm_idx++] = static_cast<char>(ch);
        putchar(ch);
        (void)fflush(stdout);
    }
}

// ---- Flight number input for download (non-blocking) ----
static void cal_ui_handle_flight_num(RcosAo* me) {
    me->confirm_timeout_ticks++;
    if (me->confirm_timeout_ticks >= 60) {  // 3s at 20Hz
        printf("\nTimeout.\n");
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT) { return; }
    if (ch == 27) {
        printf("\nCancelled.\n");
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    if (ch == '\r' || ch == '\n') {
        me->confirm_buf[me->confirm_idx] = '\0';
        printf("\n");
        int num = 0;
        for (uint8_t i = 0; i < me->confirm_idx; i++) {
            if (me->confirm_buf[i] >= '0' && me->confirm_buf[i] <= '9') {
                num = num * 10 + (me->confirm_buf[i] - '0');
            }
        }
        if (num > 0) {
            cli_do_download_flight(num);
        } else {
            printf("Invalid flight number.\n");
        }
        me->cal_ui_state = CalUiState::kIdle;
        return;
    }
    if (ch >= '0' && ch <= '9' && me->confirm_idx < 3) {
        me->confirm_buf[me->confirm_idx++] = static_cast<char>(ch);
        putchar(ch);
        (void)fflush(stdout);
    }
}

// ---- Cal UI dispatcher (20Hz) ----
static void cal_ui_tick(RcosAo* me) {
    switch (me->cal_ui_state) {
    case CalUiState::kIdle:           break;
    case CalUiState::kAsyncPrompt:    cal_ui_handle_async_prompt(me);     break;
    case CalUiState::kAsyncWaiting:   cal_ui_handle_async_waiting(me);    break;
    case CalUiState::k6posPrompt:     cal_ui_handle_6pos_prompt(me);      break;
    case CalUiState::k6posSampling:   cal_ui_handle_6pos_sampling(me);    break;
    case CalUiState::k6posValidating: cal_ui_handle_6pos_validating(me);  break;
    case CalUiState::kMagPrompt:      cal_ui_handle_mag_prompt(me);       break;
    case CalUiState::kMagCollecting:  cal_ui_handle_mag_collecting(me);   break;
    case CalUiState::kComputing:      cal_ui_handle_computing(me);        break;
    case CalUiState::kResult:         cal_ui_handle_result(me);           break;
    case CalUiState::kResetConfirm:   cal_ui_handle_reset_confirm(me);    break;
    case CalUiState::kEraseConfirm:   cal_ui_handle_erase_confirm(me);    break;
    case CalUiState::kFlightNumInput: cal_ui_handle_flight_num(me);       break;
    case CalUiState::kWizardNext:     cal_ui_handle_wizard_next(me);      break;
    }
}

#endif // !ROCKETCHIP_HOST_TEST

// ============================================================================
// State Handlers
// ============================================================================

static QState RcosAo_initial(RcosAo * const me, QEvt const * const e) {
    (void)e;

    me->last_ansi_rx_count = 0;
    me->last_ansi_render_ms = 0;

#ifndef ROCKETCHIP_HOST_TEST
    me->cal_ui_state = CalUiState::kIdle;
    me->cal_6pos_position = 0;
    me->cal_wizard_step = 0;
    me->cal_wizard_active = false;
    me->cal_last_progress = 0;
    me->cal_is_6pos = false;
    me->mag_last_printed_count = 0;
    me->mag_total_reads = 0;
    me->mag_reject_close = 0;
    me->mag_reject_range = 0;
    me->wizard_passed = 0;
    me->wizard_failed = 0;
    me->wizard_skipped = 0;
#endif

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

        // rc_os_update() handles USB connect detection, boot banner, and
        // menu key dispatch. Called every tick regardless of output mode —
        // USB connect/banner must work in ANSI mode too.
        rc_os_update();

        // Calibration UI state machine
        cal_ui_tick(me);
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

// ============================================================================
// Calibration UI — Public Trigger Functions (Phase D3)
// ============================================================================

#ifndef ROCKETCHIP_HOST_TEST

void AO_RCOS_start_cal_gyro() {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }
    if (l_rcosAo.cal_ui_state != CalUiState::kIdle) {
        printf("\nCalibration already in progress.\n");
        return;
    }
    printf("\nGyro Calibration — keep device STILL for ~2s.\n");
    printf("ENTER to start, 'x' to skip.\n");
    l_rcosAo.cal_async_type = kAsyncGyro;
    l_rcosAo.cal_wizard_active = false;
    l_rcosAo.cal_ui_state = CalUiState::kAsyncPrompt;
}

void AO_RCOS_start_cal_level() {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }
    if (l_rcosAo.cal_ui_state != CalUiState::kIdle) {
        printf("\nCalibration already in progress.\n");
        return;
    }
    printf("\nLevel Calibration — keep device FLAT and STILL for ~1s.\n");
    printf("ENTER to start, 'x' to skip.\n");
    l_rcosAo.cal_async_type = kAsyncLevel;
    l_rcosAo.cal_wizard_active = false;
    l_rcosAo.cal_ui_state = CalUiState::kAsyncPrompt;
}

void AO_RCOS_start_cal_baro() {
    if (!rc_os_baro_available) {
        printf("\nERROR: Barometer not available\n");
        return;
    }
    if (l_rcosAo.cal_ui_state != CalUiState::kIdle) {
        printf("\nCalibration already in progress.\n");
        return;
    }
    printf("\nBaro Calibration — setting ground reference (~1s).\n");
    printf("ENTER to start, 'x' to skip.\n");
    l_rcosAo.cal_async_type = kAsyncBaro;
    l_rcosAo.cal_wizard_active = false;
    l_rcosAo.cal_ui_state = CalUiState::kAsyncPrompt;
}

void AO_RCOS_start_cal_6pos() {
    if (!rc_os_imu_available) {
        printf("\nERROR: IMU not available\n");
        return;
    }
    if (l_rcosAo.cal_ui_state != CalUiState::kIdle) {
        printf("\nCalibration already in progress.\n");
        return;
    }
    printf("\n========================================\n");
    printf("  6-Position Accelerometer Calibration\n");
    printf("========================================\n");
    printf("Calibrates accel offset, scale, and\n");
    printf("cross-axis coupling by measuring gravity\n");
    printf("in 6 orientations.\n\n");

    calibration_reset_6pos();
    l_rcosAo.cal_6pos_position = 0;
    l_rcosAo.cal_wizard_active = false;
    l_rcosAo.cal_is_6pos = true;

    printf("--- Position 1/%d: %s ---\n",
           kAccel6posPositions,
           calibration_get_6pos_name(0));
    printf("  %s\n", kPositionInstructions[0]);
    printf("  Press ENTER when ready, ESC to cancel.\n");
    (void)fflush(stdout);
    cal_neo(kCalNeoAccelWait);
    l_rcosAo.cal_ui_state = CalUiState::k6posPrompt;
}

void AO_RCOS_start_cal_mag() {
    if (rc_os_read_mag == nullptr) {
        printf("\nERROR: Mag read callback not set\n");
        return;
    }
    if (l_rcosAo.cal_ui_state != CalUiState::kIdle) {
        printf("\nCalibration already in progress.\n");
        return;
    }
    printf("\n========================================\n");
    printf("  Compass Calibration\n");
    printf("========================================\n");
    printf("Rotate the device slowly through all\n");
    printf("orientations to cover the full sphere.\n");
    printf("ENTER to start, ESC/'x' to cancel.\n");
    printf("========================================\n");

    l_rcosAo.cal_wizard_active = false;
    l_rcosAo.cal_is_6pos = false;
    l_rcosAo.cal_ui_state = CalUiState::kMagPrompt;
}

void AO_RCOS_start_cal_wizard() {
    if (l_rcosAo.cal_ui_state != CalUiState::kIdle) {
        printf("\nCalibration already in progress.\n");
        return;
    }
    printf("\n========================================\n");
    printf("  Full Calibration Wizard\n");
    printf("========================================\n");
    printf("This wizard runs all calibrations:\n");
    printf("  1. Gyro (keep still, ~2s)\n");
    printf("  2. Level (keep flat, ~1s)\n");
    printf("  3. 6-position accel\n");
    printf("  4. Compass calibration\n");
    printf("(Baro ground ref runs automatically at boot)\n");
    printf("========================================\n\n");

    l_rcosAo.cal_wizard_active = true;
    l_rcosAo.cal_wizard_step = 0;  // Will be incremented to kWizardGyro in kWizardNext
    l_rcosAo.wizard_passed = 0;
    l_rcosAo.wizard_failed = 0;
    l_rcosAo.wizard_skipped = 0;

    // Start from step 0 by entering kWizardNext which increments then dispatches
    // But step 0 is gyro, and kWizardNext increments first. So set to 255 so it wraps to 0.
    // Actually, kWizardNext does me->cal_wizard_step++ first. So set step to max-1 = kWizardGyro - 1.
    // kWizardGyro = 0. So we need to set step to 255 (wraps to 0). But uint8_t 255+1 = 0. That works.
    // Hmm, but step >= kWizardDone (4) would fire immediately on 255. Let me rethink.
    // Better: set step to 0, but handle step 0 as the first transition.
    // Actually simplest: don't increment, just dispatch based on current step.
    // Let me refactor kWizardNext to check current step, not post-increment.
    // No wait, the current code does me->cal_wizard_step++ FIRST. So if step starts at 0,
    // after increment it becomes 1 (kWizardLevel). That skips gyro.
    // Fix: initialize to UINT8_MAX so increment wraps to 0.
    l_rcosAo.cal_wizard_step = UINT8_MAX;  // kWizardNext increments to 0 (kWizardGyro)
    l_rcosAo.cal_ui_state = CalUiState::kWizardNext;
}

bool AO_RCOS_cal_active() {
    return l_rcosAo.cal_ui_state != CalUiState::kIdle;
}

void AO_RCOS_start_erase_flights() {
    if (AO_RCOS_cal_active()) { return; }
    rc::FlightTableState* ft = AO_Logger_get_flight_table_mut();
    if (!ft->loaded) {
        printf("Flight table not loaded.\n");
        return;
    }
    uint32_t count = rc::flight_table_count(ft);
    if (count == 0) {
        printf("No flights to erase.\n");
        return;
    }
    printf("Erase ALL %lu flights? Type 'yes' + Enter to confirm: ",
           (unsigned long)count);
    (void)fflush(stdout);
    memset(l_rcosAo.confirm_buf, 0, sizeof(l_rcosAo.confirm_buf));
    l_rcosAo.confirm_idx = 0;
    l_rcosAo.confirm_timeout_ticks = 0;
    l_rcosAo.cal_ui_state = CalUiState::kEraseConfirm;
}

void AO_RCOS_start_download_flight() {
    if (AO_RCOS_cal_active()) { return; }
    const rc::FlightTableState* ft = AO_Logger_get_flight_table();
    if (!ft->loaded) {
        printf("Flight table not loaded.\n");
        return;
    }
    uint32_t count = rc::flight_table_count(ft);
    if (count == 0) {
        printf("No flights stored.\n");
        return;
    }
    printf("Flight # (1-%lu): ", (unsigned long)count);
    (void)fflush(stdout);
    memset(l_rcosAo.confirm_buf, 0, sizeof(l_rcosAo.confirm_buf));
    l_rcosAo.confirm_idx = 0;
    l_rcosAo.confirm_timeout_ticks = 0;
    l_rcosAo.cal_ui_state = CalUiState::kFlightNumInput;
}

void AO_RCOS_start_cal_reset() {
    if (AO_RCOS_cal_active()) { return; }
    printf("\n*** RESET ALL CALIBRATION ***\n");
    printf("This will erase all calibration data!\n");
    printf("Type 'YES' + ENTER to confirm: ");
    (void)fflush(stdout);
    memset(l_rcosAo.confirm_buf, 0, sizeof(l_rcosAo.confirm_buf));
    l_rcosAo.confirm_idx = 0;
    l_rcosAo.confirm_timeout_ticks = 0;
    l_rcosAo.cal_ui_state = CalUiState::kResetConfirm;
}

void AO_RCOS_start_cal_save() {
    if (AO_RCOS_cal_active()) { return; }
    printf("\nSaving calibration to flash...");
    (void)fflush(stdout);
    cal_result_t result = calibration_save();
    if (result == CAL_RESULT_OK) {
        printf(" OK!\n");
        if (!i2c_bus_reset()) {
            printf("[WARN] I2C bus reset failed after save\n");
        }
    } else {
        printf(" FAILED (%d)\n", static_cast<int>(result));
    }
    // Note: calibration_save() calls flash_safe_execute() which blocks
    // ~100-500ms. At 100Hz tick rate with queue depth 32, the 320ms
    // headroom is tight but sufficient for typical flash writes (~200ms).
}

#else
// Host test stubs
void AO_RCOS_start_cal_gyro() {}
void AO_RCOS_start_cal_level() {}
void AO_RCOS_start_cal_baro() {}
void AO_RCOS_start_cal_6pos() {}
void AO_RCOS_start_cal_mag() {}
void AO_RCOS_start_cal_wizard() {}
void AO_RCOS_start_cal_reset() {}
void AO_RCOS_start_cal_save() {}
void AO_RCOS_start_erase_flights() {}
void AO_RCOS_start_download_flight() {}
bool AO_RCOS_cal_active() { return false; }
#endif
