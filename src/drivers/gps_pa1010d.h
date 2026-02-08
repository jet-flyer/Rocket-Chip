/**
 * @file gps_pa1010d.h
 * @brief PA1010D GPS module driver using lwGPS library
 *
 * CDTop PA1010D via I2C with NMEA parsing by lwGPS.
 */

#ifndef ROCKETCHIP_GPS_PA1010D_H
#define ROCKETCHIP_GPS_PA1010D_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration
// ============================================================================

constexpr uint8_t kGpsPa1010dAddr   = 0x10;

// ============================================================================
// Types
// ============================================================================

/**
 * @brief GPS fix type
 */
typedef enum {
    GPS_FIX_NONE = 0,   // No fix
    GPS_FIX_2D   = 2,   // 2D fix (no altitude)
    GPS_FIX_3D   = 3,   // 3D fix
} gps_fix_t;

/**
 * @brief GPS data
 */
typedef struct {
    // Position
    double latitude;        ///< Latitude in degrees (+ = North)
    double longitude;       ///< Longitude in degrees (+ = East)
    float altitude_m;       ///< Altitude above MSL in meters

    // Motion
    float speed_knots;      ///< Ground speed in knots
    float speed_mps;        ///< Ground speed in m/s
    float course_deg;       ///< True course in degrees

    // Quality
    gps_fix_t fix;          ///< Fix type
    uint8_t satellites;     ///< Number of satellites in use
    float hdop;             ///< Horizontal dilution of precision
    float vdop;             ///< Vertical dilution of precision
    float pdop;             ///< Position dilution of precision

    // Time (UTC)
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    // Date (UTC)
    uint8_t day;
    uint8_t month;
    uint16_t year;

    // Status
    bool valid;             ///< Position is valid
    bool time_valid;        ///< Time data is valid
    bool date_valid;        ///< Date data is valid
} gps_pa1010d_data_t;

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
 * @return true if new valid position data is available
 */
bool gps_pa1010d_update(void);

/**
 * @brief Get latest GPS data
 * @param data Output data structure
 * @return true if data is valid
 */
bool gps_pa1010d_get_data(gps_pa1010d_data_t* data);

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
bool gps_pa1010d_set_rate(uint8_t rate_hz);

#endif // ROCKETCHIP_GPS_PA1010D_H
