/**
 * @file lwgps_opts.h
 * @brief LwGPS configuration for RocketChip
 */

#ifndef LWGPS_OPTS_H
#define LWGPS_OPTS_H

// Use double precision for latitude/longitude (better accuracy)
#define LWGPS_CFG_DOUBLE            1

// Enable status callback
#define LWGPS_CFG_STATUS            0

// Enable standard NMEA sentences
#define LWGPS_CFG_STATEMENT_GPGGA   1   // Position, altitude, fix quality
#define LWGPS_CFG_STATEMENT_GPGSA   1   // DOP and active satellites
#define LWGPS_CFG_STATEMENT_GPRMC   1   // Recommended minimum (speed, course, date)
#define LWGPS_CFG_STATEMENT_GPGSV   1   // Satellites in view

// Disable detailed satellite info (saves memory)
#define LWGPS_CFG_STATEMENT_GPGSV_SAT_DET 0

// Disable uBlox proprietary messages
#define LWGPS_CFG_STATEMENT_PUBX    0
#define LWGPS_CFG_STATEMENT_PUBX_TIME 0

// Enable CRC checking
#define LWGPS_CFG_CRC               1

#endif // LWGPS_OPTS_H
