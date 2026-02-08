/**
 * @file gps_pa1010d.c
 * @brief PA1010D GPS module driver using lwGPS library
 *
 * Reads NMEA sentences from PA1010D via I2C and parses with lwGPS.
 */

#include "gps_pa1010d.h"
#include "i2c_bus.h"
#include "lwgps/lwgps.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// Private State
// ============================================================================

static bool g_initialized = false;
static lwgps_t g_gps;
static gps_pa1010d_data_t g_data;

// I2C read buffer — full 255-byte MT3333 TX buffer per vendor recommendation.
// GlobalTop/Quectel app notes: "read full buffer, partial reads not recommended."
// Pico SDK i2c_read_blocking() has no upper limit (unlike Arduino Wire.h's 32).
// At 400kHz, 255 bytes takes ~5.8ms. At 10Hz GPS poll, this affects 10 of 1000
// IMU cycles/sec — negligible jitter for a 200Hz fusion consumer.
// Ref: pico-examples/i2c/pa1010d_i2c uses 250-byte reads.
#define GPS_MAX_READ    255
static uint8_t g_buffer[GPS_MAX_READ + 1];  // +1 for null terminator

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Calculate NMEA checksum
 */
static uint8_t nmea_checksum(const char* sentence) {
    uint8_t checksum = 0;
    // Skip leading '$' if present
    if (*sentence == '$') sentence++;
    // XOR all characters until '*' or end
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence++;
    }
    return checksum;
}

/**
 * @brief Read NMEA data from PA1010D via I2C
 *
 * The PA1010D has a 255-byte buffer. Reading returns available NMEA data.
 * Empty reads return 0x0A (newline) bytes.
 */
static int read_nmea_data(uint8_t* buffer, size_t max_len) {
    // PA1010D I2C protocol: just read bytes, no register address needed
    int32_t ret = i2c_bus_read(GPS_PA1010D_ADDR, buffer, max_len);
    if (ret <= 0) {
        return 0;
    }

    // Find actual data length (PA1010D pads with 0x0A or 0xFF)
    int32_t len = 0;
    for (int32_t i = 0; i < ret; i++) {
        // Stop at padding bytes
        if (buffer[i] == 0x0A && (i == 0 || buffer[i-1] == 0x0A)) {
            break;
        }
        if (buffer[i] == 0xFF) {
            break;
        }
        len = i + 1;
    }

    return len;
}

/**
 * @brief Update internal data structure from lwGPS
 */
static void update_data_from_lwgps(void) {
    g_data.latitude = g_gps.latitude;
    g_data.longitude = g_gps.longitude;
    g_data.altitude_m = (float)g_gps.altitude;

    g_data.speed_knots = (float)g_gps.speed;
    g_data.speed_mps = (float)lwgps_to_speed(g_gps.speed, LWGPS_SPEED_MPS);
    g_data.course_deg = (float)g_gps.course;

    // Fix type from GSA
    if (g_gps.fix_mode <= 1) {
        g_data.fix = GPS_FIX_NONE;
    } else if (g_gps.fix_mode == 2) {
        g_data.fix = GPS_FIX_2D;
    } else {
        g_data.fix = GPS_FIX_3D;
    }

    g_data.satellites = g_gps.sats_in_use;
    g_data.hdop = (float)g_gps.dop_h;
    g_data.vdop = (float)g_gps.dop_v;
    g_data.pdop = (float)g_gps.dop_p;

    g_data.hour = g_gps.hours;
    g_data.minute = g_gps.minutes;
    g_data.second = g_gps.seconds;

    g_data.day = g_gps.date;
    g_data.month = g_gps.month;
    g_data.year = 2000 + g_gps.year;  // lwGPS stores 2-digit year

    g_data.valid = lwgps_is_valid(&g_gps) && (g_data.fix >= GPS_FIX_2D);
    g_data.time_valid = g_gps.time_valid;
    g_data.date_valid = g_gps.date_valid;
}

// ============================================================================
// Public API
// ============================================================================

bool gps_pa1010d_init(void) {
    // Initialize lwGPS parser
    lwgps_init(&g_gps);

    // Clear data
    memset(&g_data, 0, sizeof(g_data));

    // Check if device is present
    if (!i2c_bus_probe(GPS_PA1010D_ADDR)) {
        return false;
    }

    g_initialized = true;

    // Configure NMEA output: enable only RMC + GGA sentences.
    // RMC provides speed, course, date/time. GGA provides position, altitude, fix quality.
    // Reduces parsing overhead and keeps output to ~139 bytes/sec (vs ~449 default).
    // PMTK314 fields: GLL,RMC,VTG,GGA,GSA,GSV,...
    gps_pa1010d_send_command("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

    return true;
}

bool gps_pa1010d_ready(void) {
    return g_initialized;
}

bool gps_pa1010d_update(void) {
    if (!g_initialized) {
        return false;
    }

    // Read available NMEA data (full 255-byte MT3333 TX buffer)
    int len = read_nmea_data(g_buffer, GPS_MAX_READ);
    if (len <= 0) {
        return false;
    }

    // Null-terminate for safety
    g_buffer[len] = '\0';

    // Process through lwGPS parser
    lwgps_process(&g_gps, g_buffer, len);

    // Update our data structure
    update_data_from_lwgps();

    return g_data.valid;
}

bool gps_pa1010d_get_data(gps_pa1010d_data_t* data) {
    if (data == NULL) {
        return false;
    }

    *data = g_data;
    return g_data.valid;
}

bool gps_pa1010d_has_fix(void) {
    return g_data.valid && (g_data.fix >= GPS_FIX_2D);
}

bool gps_pa1010d_send_command(const char* cmd) {
    if (!g_initialized || cmd == NULL) {
        return false;
    }

    // Build full NMEA sentence with checksum
    char sentence[128];
    int len = snprintf(sentence, sizeof(sentence) - 5, "$%s*", cmd);
    if (len < 0 || len >= (int)(sizeof(sentence) - 5)) {
        return false;
    }

    // Calculate and append checksum
    uint8_t cs = nmea_checksum(sentence);
    snprintf(sentence + len, 5, "%02X\r\n", cs);
    len += 4;

    // Send via I2C
    int ret = i2c_bus_write(GPS_PA1010D_ADDR, (uint8_t*)sentence, len);
    return (ret == len);
}

bool gps_pa1010d_set_rate(uint8_t rate_hz) {
    // PMTK220 - Set NMEA update rate
    // Parameter is update interval in milliseconds
    char cmd[32];
    uint16_t interval_ms;

    switch (rate_hz) {
        case 1:  interval_ms = 1000; break;
        case 5:  interval_ms = 200;  break;
        case 10: interval_ms = 100;  break;
        default: return false;
    }

    snprintf(cmd, sizeof(cmd), "PMTK220,%u", interval_ms);
    return gps_pa1010d_send_command(cmd);
}
