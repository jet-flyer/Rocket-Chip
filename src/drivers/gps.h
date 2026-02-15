/**
 * @file gps.h
 * @brief Transport-neutral GPS data types
 *
 * Shared by all GPS transport backends (I2C, UART, SPI).
 * No transport-specific includes — pure data definitions.
 *
 * Pattern: transport-neutral types + transport-specific backends.
 * See also: gps_pa1010d.h (I2C), gps_uart.h (UART).
 */

#ifndef ROCKETCHIP_GPS_H
#define ROCKETCHIP_GPS_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Types
// ============================================================================

/**
 * @brief GPS fix type (NMEA standard values)
 */
typedef enum {
    GPS_FIX_NONE = 0,   // No fix
    GPS_FIX_2D   = 2,   // 2D fix (no altitude)
    GPS_FIX_3D   = 3,   // 3D fix
} gps_fix_t;

/**
 * @brief Transport-neutral GPS data
 *
 * All GPS backends (I2C, UART) produce this struct.
 * Consumers (seqlock, ESKF) depend only on this type.
 */
typedef struct {
    // Position
    double latitude;        ///< Latitude in degrees (+ = North)
    double longitude;       ///< Longitude in degrees (+ = East)
    float altitudeM;        ///< Altitude above MSL in meters

    // Motion
    float speedKnots;       ///< Ground speed in knots
    float speedMps;         ///< Ground speed in m/s
    float courseDeg;        ///< True course in degrees

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
    bool valid;             ///< Position is valid (RMC active AND GGA fix)
    bool timeValid;         ///< Time data is valid
    bool dateValid;         ///< Date data is valid

    // Diagnostic (raw lwGPS fields for debugging fix detection)
    uint8_t ggaFix;         ///< GGA fix quality (0=none, 1=GPS, 2=DGPS)
    uint8_t gsaFixMode;     ///< GSA fix mode (1=none, 2=2D, 3=3D)
    bool rmcValid;          ///< RMC status ('A' = valid)
} gps_data_t;

/**
 * @brief GPS transport type (for boot banner / diagnostics)
 */
typedef enum {
    GPS_TRANSPORT_NONE = 0,
    GPS_TRANSPORT_I2C  = 1,
    GPS_TRANSPORT_UART = 2,
} gps_transport_t;

#endif // ROCKETCHIP_GPS_H
