// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file telemetry_service.cpp
 * @brief Radio telemetry TX scheduling — CCSDS encoder + rfm95w_send()
 *
 * IVP-59: Telemetry Service (Stage 7: Radio & Telemetry)
 */

#include "rocketchip/telemetry_service.h"
#include "drivers/rfm95w.h"
#include "pico/time.h"

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

void telemetry_service_init(TelemetryServiceState* state,
                            rfm95w_t* radio, uint8_t rate_hz) {
    state->encoder.init();
    state->radio = radio;
    state->rate_hz = rate_hz;
    state->interval_ms = rate_to_interval(rate_hz);
    state->last_tx_ms = 0;
    state->tx_count = 0;
    state->tx_fail_count = 0;
    state->last_airtime_us = 0;
    state->duty_cycle_sum_us = 0;
    state->duty_window_start = 0;
}

void telemetry_service_tick(TelemetryServiceState* state,
                            const TelemetryState* telem, uint32_t now_ms) {
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

} // namespace rc
