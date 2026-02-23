// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file gps_pa1010d.h
 * @brief PA1010D GPS module driver — I2C transport backend
 *
 * CDTop PA1010D via I2C with NMEA parsing by lwGPS.
 * Uses transport-neutral types from gps.h.
 */

#ifndef ROCKETCHIP_GPS_PA1010D_H
#define ROCKETCHIP_GPS_PA1010D_H

#include "gps.h"
#include <stddef.h>

// ============================================================================
// Configuration
// ============================================================================

constexpr uint8_t kGpsPa1010dAddr   = 0x10;

// ============================================================================
// API
// ============================================================================

/**
 * @brief Initialize the GPS module
 * @return true on success
 */
bool gps_pa1010d_init(void);

/**
 * @brief Check if GPS is initialized
 * @return true if initialized
 */
bool gps_pa1010d_ready(void);

/**
 * @brief Poll GPS for new data
 *
 * Call this periodically (at least 10Hz for 10Hz GPS).
 * Reads available NMEA data from I2C and parses it.
 *
 * @return true if I2C read succeeded (data received), false on I2C error
 */
bool gps_pa1010d_update(void);

/**
 * @brief Get latest GPS data
 * @param data Output data structure
 * @return true if data is valid
 */
bool gps_pa1010d_get_data(gps_data_t* data);

/**
 * @brief Check if GPS has a valid fix
 * @return true if GPS has 2D or 3D fix
 */
bool gps_pa1010d_has_fix(void);

/**
 * @brief Send PMTK command to GPS
 * @param cmd PMTK command string (e.g., "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
 * @return true on success
 */
bool gps_pa1010d_send_command(const char* cmd);

/**
 * @brief Set GPS update rate
 * @param rate_hz Update rate (1, 5, or 10 Hz)
 * @return true on success
 */
bool gps_pa1010d_set_rate(uint8_t rateHz);

/**
 * @brief Get pointer to last raw NMEA read buffer
 * @param buf Output pointer to internal buffer (valid until next update)
 * @param len Output length of last read data
 * @return true if data available
 */
bool gps_pa1010d_get_last_raw(const uint8_t** buf, size_t* len);

#endif // ROCKETCHIP_GPS_PA1010D_H
