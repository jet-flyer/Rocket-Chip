/**
 * @file gps_uart.h
 * @brief UART GPS driver — hardware UART transport backend
 *
 * UART GPS (e.g., Adafruit Ultimate GPS FeatherWing #3133) via UART0.
 * Uses transport-neutral types from gps.h.
 * NMEA parsing by lwGPS (same parser as I2C backend).
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
 * NMEA '$' start byte. Returns false if no GPS data detected.
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
 * @brief Poll UART GPS for new data
 *
 * Non-blocking drain of UART RX FIFO. Feeds bytes to lwGPS parser.
 * Call at least 10Hz for 10Hz GPS.
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
