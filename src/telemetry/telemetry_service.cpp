// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file telemetry_service.cpp
 * @brief Radio telemetry TX scheduling + RX decode
 *
 * IVP-59: TX service (CCSDS encoder + rfm95w_send)
 * IVP-60: RX service (rfm95w_recv + CCSDS decode + CSV output)
 */

#include "rocketchip/telemetry_service.h"
#include "drivers/rfm95w.h"
#include "pico/time.h"
#include <stdio.h>

namespace rc {

// Duty cycle measurement window (10 seconds)
static constexpr uint32_t kDutyWindowMs = 10000;

// Valid TX rates — cycle_rate rotates through these
static constexpr uint8_t kRates[] = {2, 5, 10};
static constexpr uint8_t kNumRates = 3;

static uint32_t rate_to_interval(uint8_t hz) {
    if (hz == 0) { return 1000; }
    return 1000 / hz;
}

// ============================================================================
// Init
// ============================================================================

void telemetry_service_init(TelemetryServiceState* state,
                            rfm95w_t* radio, uint8_t rate_hz) {
    state->encoder.init();
    state->radio = radio;
    state->mode = RadioMode::kTx;
    state->rate_hz = rate_hz;
    state->interval_ms = rate_to_interval(rate_hz);
    state->last_tx_ms = 0;
    state->tx_count = 0;
    state->tx_fail_count = 0;
    state->last_airtime_us = 0;
    state->duty_cycle_sum_us = 0;
    state->duty_window_start = 0;
    state->rx_count = 0;
    state->rx_crc_errors = 0;
    state->last_rx_ms = 0;
    state->last_rx_rssi = 0;
    state->last_rx_snr = 0;
    state->last_rx_seq = 0;
}

// ============================================================================
// TX Mode
// ============================================================================

void telemetry_service_tick(TelemetryServiceState* state,
                            const TelemetryState* telem, uint32_t now_ms) {
    if (state->mode != RadioMode::kTx) { return; }
    if (state->radio == nullptr || telem == nullptr) {
        return;
    }

    // Rate limiting
    if (now_ms - state->last_tx_ms < state->interval_ms) {
        return;
    }
    state->last_tx_ms = now_ms;

    // Encode CCSDS nav packet
    EncodeResult result = {};
    state->encoder.encode_nav(*telem, telem->met_ms, result);
    if (!result.ok) {
        return;
    }

    // TX with airtime measurement
    uint64_t tx_start = time_us_64();
    bool sent = rfm95w_send(state->radio, result.buf, result.len);
    uint64_t tx_end = time_us_64();

    if (sent) {
        state->tx_count++;
        state->last_airtime_us = static_cast<uint32_t>(tx_end - tx_start);

        // Duty cycle tracking — reset window every 10s
        if (now_ms - state->duty_window_start >= kDutyWindowMs) {
            state->duty_cycle_sum_us = 0;
            state->duty_window_start = now_ms;
        }
        state->duty_cycle_sum_us += state->last_airtime_us;
    } else {
        state->tx_fail_count++;
    }
}

uint8_t telemetry_service_cycle_rate(TelemetryServiceState* state) {
    // Find current rate in the list and advance
    for (uint8_t i = 0; i < kNumRates; i++) {
        if (kRates[i] == state->rate_hz) {
            uint8_t next = kRates[(i + 1) % kNumRates];
            state->rate_hz = next;
            state->interval_ms = rate_to_interval(next);
            return next;
        }
    }
    // Current rate not in list — reset to 2 Hz
    state->rate_hz = kRates[0];
    state->interval_ms = rate_to_interval(kRates[0]);
    return kRates[0];
}

uint8_t telemetry_service_duty_pct(const TelemetryServiceState* state,
                                   uint32_t now_ms) {
    uint32_t window_elapsed_ms = now_ms - state->duty_window_start;
    if (window_elapsed_ms == 0) {
        return 0;
    }
    // duty% = (sum_airtime_us / window_elapsed_us) * 100
    uint32_t window_elapsed_us = window_elapsed_ms * 1000;
    return static_cast<uint8_t>(
        (state->duty_cycle_sum_us * 100) / window_elapsed_us);
}

// ============================================================================
// RX Mode (IVP-60)
// ============================================================================

void telemetry_service_start_rx(TelemetryServiceState* state) {
    if (state->radio == nullptr) { return; }
    state->mode = RadioMode::kRx;
    rfm95w_start_rx(state->radio);
}

void telemetry_service_stop_rx(TelemetryServiceState* state) {
    if (state->radio == nullptr) { return; }
    state->mode = RadioMode::kTx;
    // Radio returns to standby — rfm95w_send() handles mode transitions
}

static void print_rx_csv(const TelemetryState& telem, uint16_t seq,
                         uint32_t met_ms, int16_t rssi, float snr) {
    float lat = static_cast<float>(telem.lat_1e7) * 1e-7F;
    float lon = static_cast<float>(telem.lon_1e7) * 1e-7F;
    float alt_m = static_cast<float>(telem.alt_mm) * 0.001F;
    float vel_n = static_cast<float>(telem.vel_n_cms) * 0.01F;
    float vel_e = static_cast<float>(telem.vel_e_cms) * 0.01F;
    float vel_d = static_cast<float>(telem.vel_d_cms) * 0.01F;
    float baro_alt = static_cast<float>(telem.baro_alt_mm) * 0.001F;
    float q_w = static_cast<float>(telem.q_w) / 32767.0F;
    float q_x = static_cast<float>(telem.q_x) / 32767.0F;
    float q_y = static_cast<float>(telem.q_y) / 32767.0F;
    float q_z = static_cast<float>(telem.q_z) / 32767.0F;

    printf("RX,%u,%d,%.1f,%.7f,%.7f,%.3f,%.2f,%.2f,%.2f,%.3f,%.4f,%.4f,%.4f,%.4f,%u,%lu\n",
           static_cast<unsigned>(seq),
           static_cast<int>(rssi),
           static_cast<double>(snr),
           static_cast<double>(lat),
           static_cast<double>(lon),
           static_cast<double>(alt_m),
           static_cast<double>(vel_n),
           static_cast<double>(vel_e),
           static_cast<double>(vel_d),
           static_cast<double>(baro_alt),
           static_cast<double>(q_w),
           static_cast<double>(q_x),
           static_cast<double>(q_y),
           static_cast<double>(q_z),
           static_cast<unsigned>(telem.flight_state),
           (unsigned long)met_ms);
}

void telemetry_service_rx_tick(TelemetryServiceState* state, uint32_t now_ms) {
    if (state->mode != RadioMode::kRx) { return; }
    if (state->radio == nullptr) { return; }
    if (!rfm95w_available(state->radio)) { return; }

    uint8_t buf[128];
    uint8_t len = rfm95w_recv(state->radio, buf, sizeof(buf));
    if (len == 0) { return; }

    state->last_rx_rssi = rfm95w_rssi(state->radio);
    state->last_rx_snr = state->radio->last_snr;

    TelemetryState telem = {};
    uint16_t seq = 0;
    uint32_t met_ms = 0;
    if (!ccsds_decode_nav(buf, len, telem, seq, met_ms)) {
        state->rx_crc_errors++;
        return;
    }

    state->rx_count++;
    state->last_rx_ms = now_ms;
    state->last_rx_seq = seq;

    print_rx_csv(telem, seq, met_ms, state->last_rx_rssi, state->last_rx_snr);
}

} // namespace rc
