// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file telemetry_service.h
 * @brief Radio telemetry service — TX scheduling and RX decode
 *
 * Core 0 downlink service.
 * CCSDS encoder (IVP-58) at configurable rate (2/5/10 Hz).
 *
 * TX mode: caller passes latest TelemetryState each tick.
 *          Service encodes CCSDS and sends via rfm95w_send().
 * RX mode: rfm95w_recv() → CCSDS decode → CSV output (IVP-60)
 *
 * IVP-59/60: Telemetry Service (Stage 7: Radio & Telemetry)
 */

#ifndef ROCKETCHIP_TELEMETRY_SERVICE_H
#define ROCKETCHIP_TELEMETRY_SERVICE_H

#include <stdint.h>
#include "rocketchip/telemetry_encoder.h"

// Forward declarations — avoid pulling full driver headers into callers
struct rfm95w_t;

namespace rc {

// ============================================================================
// Radio Mode
// ============================================================================

enum class RadioMode : uint8_t {
    kTx = 0,    // Transmitting CCSDS telemetry
    kRx = 1,    // Receiving and decoding CCSDS packets
};

// ============================================================================
// Telemetry Service State
// ============================================================================

struct TelemetryServiceState {
    CcsdsEncoder encoder;
    rfm95w_t*    radio;          // Borrowed pointer — caller owns lifetime
    RadioMode    mode;           // Current radio mode (TX or RX)

    // TX state
    uint8_t  rate_hz;            // Current TX rate (2, 5, or 10)
    uint32_t interval_ms;        // 1000 / rate_hz
    uint32_t last_tx_ms;         // Timestamp of last TX
    uint32_t tx_count;           // Total packets sent
    uint32_t tx_fail_count;      // TX failures (timeout)
    uint32_t last_airtime_us;    // Airtime of last packet
    uint32_t duty_cycle_sum_us;  // Running sum of airtime over duty window
    uint32_t duty_window_start;  // Start of current duty cycle window (ms)

    // RX state
    uint32_t rx_count;           // Total valid packets received
    uint32_t rx_crc_errors;      // Packets that failed CRC or header check
    uint32_t last_rx_ms;         // Timestamp of last valid RX (ms)
    int16_t  last_rx_rssi;       // RSSI of last received packet (dBm)
    int8_t   last_rx_snr;        // SNR of last received packet (dB)
    uint16_t last_rx_seq;        // Sequence counter of last received packet

    // MAVLink RX output (IVP-61)
    bool           mavlink_output;    // true = MAVLink binary, false = CSV text
    uint32_t       last_heartbeat_ms; // 1 Hz heartbeat timer
    MavlinkEncoder mav_encoder;       // Single instance for monotonic seq
};

// ============================================================================
// TX Mode API
// ============================================================================

/**
 * @brief Initialize telemetry service (starts in TX mode)
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
 * Only operates when mode == kTx. Checks interval, encodes CCSDS
 * from provided TelemetryState, sends via radio.
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

// ============================================================================
// RX Mode API (IVP-60)
// ============================================================================

/**
 * @brief Enter RX mode — starts radio in RX continuous
 *
 * Stops TX, puts radio into RxContinuous mode. DIO0 maps to RxDone.
 * Call once when switching to RX mode.
 *
 * @param state  Initialized service state
 */
void telemetry_service_start_rx(TelemetryServiceState* state);

/**
 * @brief Exit RX mode — returns radio to standby
 *
 * Radio returns to standby. Mode set to TX. TX resumes on next tick.
 *
 * @param state  Service state
 */
void telemetry_service_stop_rx(TelemetryServiceState* state);

/**
 * @brief RX mode tick — call from Core 0 main loop
 *
 * Only operates when mode == kRx. Polls rfm95w_available(), receives
 * packet, decodes CCSDS, prints CSV line to USB serial.
 *
 * @param state  Initialized service state
 * @param now_ms Current time (ms since boot)
 */
void telemetry_service_rx_tick(TelemetryServiceState* state, uint32_t now_ms);

/**
 * @brief Set MAVLink output mode (IVP-61)
 *
 * When enabled, RX tick emits MAVLink v2 binary frames on stdout
 * instead of CSV text. Heartbeat emitted at 1Hz even without packets.
 *
 * @param state  Service state
 * @param enable true = MAVLink binary, false = CSV text
 */
void telemetry_service_set_mavlink_output(TelemetryServiceState* state,
                                           bool enable);

} // namespace rc

#endif // ROCKETCHIP_TELEMETRY_SERVICE_H
