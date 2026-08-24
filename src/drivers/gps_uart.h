// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// UART GPS driver — interrupt-driven hardware UART transport backend
// UART GPS (e.g., Adafruit Ultimate GPS FeatherWing #3133) via UART0.
// Uses transport-neutral types from gps.h.
// NMEA parsing by lwGPS (same parser as I2C backend).
// Receive architecture: UART0 RX interrupt (Core 0) drains hardware FIFO
// into a 512-byte ring buffer. Application code on Core 1 reads from the
// ring buffer — zero bytes lost at operating baud rate.
// Baud: try 57600 first (CR1220 sticky), fall back to 9600 + PMTK251;
// leave at 57600 before enabling IRQ. Required for 10Hz operation
// (9600 baud saturates at ~4.8 NMEA bursts/sec; 57600 gives 2.8x headroom).
// NVIC / exclusive handler: Core 0 only (the core that ran gps_uart_init).
// Pin assignment: GPIO0 (TX), GPIO1 (RX) — Feather standard UART0.

#ifndef ROCKETCHIP_GPS_UART_H
#define ROCKETCHIP_GPS_UART_H

#include "gps.h"

// ============================================================================
// Configuration
// ============================================================================

// MT3339 baud — Adafruit Ultimate GPS FeatherWing's CR1220 makes the
// module's baud sticky across host power-cycles. Init handles both
// factory 9600 and sticky 57600. See acquire_at_target_baud().
constexpr uint32_t kGpsUartBaudTarget  = 57600;
constexpr uint32_t kGpsUartBaudFactory = 9600;
constexpr uint32_t kGpsUartTxPin   = 0;       // GPIO0 — Feather UART0 TX
constexpr uint32_t kGpsUartRxPin   = 1;       // GPIO1 — Feather UART0 RX

// ============================================================================
// API (mirrors gps_pa1010d.h — same contract, different transport)
// ============================================================================

// Dual-baud presence probe (57600 then 9600), then leave UART at 57600.
// Worst case two 2 s windows plus 250 ms negotiate. Registers the RX
// handler and enables NVIC on the calling core (Core 0).
[[nodiscard]] bool gps_uart_init(void);

[[nodiscard]] bool gps_uart_ready(void);

// Reads bytes accumulated by the RX ISR and feeds them to lwGPS.
// Called internally by gps_uart_update(). Safe to call from Core 1
// at any rate — the ISR handles byte capture independently.
void gps_uart_drain(void);

// Drains ring buffer and extracts parsed data from lwGPS.
// Call at 10Hz for 10Hz GPS position updates.
[[nodiscard]] bool gps_uart_update(void);

// true if data is valid (has fix)
[[nodiscard]] bool gps_uart_get_data(gps_data_t* data);

// true if GPS has 2D or 3D fix
[[nodiscard]] bool gps_uart_has_fix(void);

// Returns the number of bytes dropped because the ring buffer was full.
// Should be 0 in normal operation. Non-zero indicates the consumer
// (Core 1) isn't draining fast enough.
[[nodiscard]] uint32_t gps_uart_get_overflow_count(void);

// Deinits UART, resets ring buffer, renegotiates baud, re-enables IRQ
// on THIS core. Call only from the core that ran gps_uart_init() (Core 0).
// Blocks up to two presence windows (~4 s) plus 250 ms negotiate.
// Do not call from Core 1 — use gps_uart_request_reinit() instead.
[[nodiscard]] bool gps_uart_reinit(void);

// Core 1: request a Core-0 reinit after UART GPS staleness. Non-blocking.
void gps_uart_request_reinit(void);

// Core 0: consume a pending request. true = caller should run gps_uart_reinit().
[[nodiscard]] bool gps_uart_take_reinit_request(void);

#endif // ROCKETCHIP_GPS_UART_H
