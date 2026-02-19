/**
 * @file gps_uart.h
 * @brief UART GPS driver — interrupt-driven hardware UART transport backend
 *
 * UART GPS (e.g., Adafruit Ultimate GPS FeatherWing #3133) via UART0.
 * Uses transport-neutral types from gps.h.
 * NMEA parsing by lwGPS (same parser as I2C backend).
 *
 * Receive architecture: UART0 RX interrupt (Core 0) drains hardware FIFO
 * into a 512-byte ring buffer. Application code on Core 1 reads from the
 * ring buffer — zero bytes lost at 9600 baud.
 *
 * Pin assignment: GPIO0 (TX), GPIO1 (RX) — Feather standard UART0.
 */

#ifndef ROCKETCHIP_GPS_UART_H
#define ROCKETCHIP_GPS_UART_H

#include "gps.h"

// ============================================================================
// Configuration
// ============================================================================

constexpr uint32_t kGpsUartBaud     = 9600;    // MT3339 default (PA1616D)
constexpr uint32_t kGpsUartTxPin   = 0;       // GPIO0 — Feather UART0 TX
constexpr uint32_t kGpsUartRxPin   = 1;       // GPIO1 — Feather UART0 RX

// ============================================================================
// API (mirrors gps_pa1010d.h — same contract, different transport)
// ============================================================================

/**
 * @brief Initialize the UART GPS module
 *
 * Configures UART0 at 9600 baud, drains for up to 2 seconds looking for
 * NMEA '$' start byte. On success, registers RX interrupt handler on
 * Core 0 for background byte capture into ring buffer.
 *
 * @return true on success (GPS detected on UART)
 */
bool gps_uart_init(void);

/**
 * @brief Check if UART GPS is initialized
 * @return true if initialized
 */
bool gps_uart_ready(void);

/**
 * @brief Drain ring buffer into lwGPS parser (no data extraction)
 *
 * Reads bytes accumulated by the RX ISR and feeds them to lwGPS.
 * Called internally by gps_uart_update(). Safe to call from Core 1
 * at any rate — the ISR handles byte capture independently.
 */
void gps_uart_drain(void);

/**
 * @brief Poll UART GPS for new data
 *
 * Drains ring buffer and extracts parsed data from lwGPS.
 * Call at 10Hz for 10Hz GPS position updates.
 *
 * @return true if UART read succeeded, false on error
 */
bool gps_uart_update(void);

/**
 * @brief Get latest GPS data
 * @param data Output data structure
 * @return true if data is valid (has fix)
 */
bool gps_uart_get_data(gps_data_t* data);

/**
 * @brief Check if GPS has a valid fix
 * @return true if GPS has 2D or 3D fix
 */
bool gps_uart_has_fix(void);

/**
 * @brief Get ring buffer overflow count (diagnostic)
 *
 * Returns the number of bytes dropped because the ring buffer was full.
 * Should be 0 in normal operation. Non-zero indicates the consumer
 * (Core 1) isn't draining fast enough.
 *
 * @return overflow byte count since init
 */
uint32_t gps_uart_get_overflow_count(void);

/**
 * @brief Send PMTK command to GPS
 * @param cmd PMTK command string (without $, *, or checksum)
 * @return true on success
 */
bool gps_uart_send_command(const char* cmd);

/**
 * @brief Set GPS update rate
 * @param rateHz Update rate (1, 5, or 10 Hz)
 * @return true on success
 */
bool gps_uart_set_rate(uint8_t rateHz);

#endif // ROCKETCHIP_GPS_UART_H
