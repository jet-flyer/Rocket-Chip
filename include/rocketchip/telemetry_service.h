// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file telemetry_service.h
 * @brief Radio telemetry service — TX scheduling and RX decode
 *
 * Core 0 downlink service. Replaces radio_test_tx_tick().
 * CCSDS encoder (IVP-58) at configurable rate (2/5/10 Hz).
 *
 * TX mode: caller passes latest TelemetryState each tick.
 *          Service encodes CCSDS and sends via rfm95w_send().
 * RX mode: rfm95w_recv() → CCSDS decode → CSV output (IVP-60)
 *
 * IVP-59: Telemetry Service (Stage 7: Radio & Telemetry)
 */

#ifndef ROCKETCHIP_TELEMETRY_SERVICE_H
#define ROCKETCHIP_TELEMETRY_SERVICE_H

#include <stdint.h>
#include "rocketchip/telemetry_encoder.h"

// Forward declarations — avoid pulling full driver headers into callers
struct rfm95w_t;

namespace rc {

// ============================================================================
// Telemetry Service State
// ============================================================================

struct TelemetryServiceState {
    CcsdsEncoder encoder;
    rfm95w_t*    radio;          // Borrowed pointer — caller owns lifetime

    uint8_t  rate_hz;            // Current TX rate (2, 5, or 10)
    uint32_t interval_ms;        // 1000 / rate_hz
    uint32_t last_tx_ms;         // Timestamp of last TX
    uint32_t tx_count;           // Total packets sent
    uint32_t tx_fail_count;      // TX failures (timeout)
    uint32_t last_airtime_us;    // Airtime of last packet
    uint32_t duty_cycle_sum_us;  // Running sum of airtime over duty window
    uint32_t duty_window_start;  // Start of current duty cycle window (ms)
};

// ============================================================================
// TX Mode API
// ============================================================================

/**
 * @brief Initialize telemetry service for TX mode
 *
 * @param state   Service state (caller-owned, zero-initialized)
 * @param radio   Initialized radio handle (must outlive service)
 * @param rate_hz Initial TX rate: 2, 5, or 10
 */
void telemetry_service_init(TelemetryServiceState* state,
                            rfm95w_t* radio, uint8_t rate_hz);

/**
 * @brief TX mode tick — call from Core 0 main loop
 *
 * Checks interval, encodes CCSDS from provided TelemetryState,
 * sends via radio, records airtime and duty cycle.
 *
 * @param state  Initialized service state
 * @param telem  Latest telemetry snapshot (caller-owned)
 * @param now_ms Current time (ms since boot)
 */
void telemetry_service_tick(TelemetryServiceState* state,
                            const TelemetryState* telem, uint32_t now_ms);

/**
 * @brief Cycle TX rate: 2 → 5 → 10 → 2
 *
 * @param state  Service state
 * @return New rate in Hz
 */
uint8_t telemetry_service_cycle_rate(TelemetryServiceState* state);

/**
 * @brief Get duty cycle percentage (over last 10s window)
 *
 * @param state  Service state
 * @param now_ms Current time
 * @return Duty cycle as integer percent (0-100)
 */
uint8_t telemetry_service_duty_pct(const TelemetryServiceState* state,
                                   uint32_t now_ms);

} // namespace rc

#endif // ROCKETCHIP_TELEMETRY_SERVICE_H
