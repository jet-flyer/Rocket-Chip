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
constexpr size_t kGpsMaxRead = 255;
static uint8_t g_buffer[kGpsMaxRead + 1];  // +1 for null terminator
static size_t g_lastReadLen = 0;           // Last successful read length

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
 * @brief Read NMEA data from PA1010D via I2C, filtering padding bytes
 *
 * The PA1010D (MT3333) pads its 255-byte I2C buffer with 0x0A when empty.
 * Three packet types can arrive (per GlobalTop/Quectel app notes):
 *   Type 1: [NMEA data][0x0A padding...]   — normal
 *   Type 2: [all 0x0A]                     — buffer was empty
 *   Type 3: [0x0A padding...][NMEA data]   — read caught tail of prev buffer
 *
 * Adafruit approach: keep 0x0A only when preceded by 0x0D (legitimate \r\n
 * NMEA terminator). Discard all standalone 0x0A (padding). This handles
 * all three packet types and preserves sentence framing for lwGPS.
 *
 * Ref: Adafruit_GPS.cpp, SparkFun I2C GPS library, Quectel L76-L app note.
 */
static int read_nmea_data(uint8_t* buffer, size_t max_len) {
    // Read raw I2C data into a local buffer, then filter in-place
    static uint8_t raw[kGpsMaxRead];
    int32_t ret = i2c_bus_read(kGpsPa1010dAddr, raw, max_len);
    if (ret <= 0) {
        return -1;  // I2C failure (NACK or timeout)
    }

    // Filter: copy valid bytes, discard padding 0x0A and 0xFF
    int32_t out = 0;
    for (int32_t i = 0; i < ret; i++) {
        if (raw[i] == 0xFF) {
            continue;  // Bus error byte — discard
        }
        if (raw[i] == 0x0A) {
            // Keep LF only if it follows CR (legitimate \r\n terminator)
            if (out > 0 && buffer[out - 1] == 0x0D) {
                buffer[out++] = raw[i];
            }
            // Otherwise discard — it's padding
            continue;
        }
        buffer[out++] = raw[i];
    }

    g_lastReadLen = (size_t)out;
    return out;
}

/**
 * @brief Update internal data structure from lwGPS
 */
static void update_data_from_lwgps() {
    g_data.latitude = g_gps.latitude;
    g_data.longitude = g_gps.longitude;
    g_data.altitude_m = (float)g_gps.altitude;

    g_data.speed_knots = (float)g_gps.speed;
    g_data.speed_mps = (float)lwgps_to_speed(g_gps.speed, LWGPS_SPEED_MPS);
    g_data.course_deg = (float)g_gps.course;

    // Fix type: prefer GGA fix quality (most reliably updated by lwGPS),
    // fall back to GSA fix_mode for 2D/3D distinction.
    // GGA fix: 0=none, 1=GPS, 2=DGPS, 3=PPS, 4+=RTK
    // GSA fix_mode: 1=none, 2=2D, 3=3D
    if (g_gps.fix >= 1) {
        // GGA says we have a fix — use GSA for 2D/3D if available
        if (g_gps.fix_mode >= 3) {
            g_data.fix = GPS_FIX_3D;
        } else if (g_gps.fix_mode == 2) {
            g_data.fix = GPS_FIX_2D;
        } else {
            // GGA has fix but GSA hasn't updated yet — assume 3D
            g_data.fix = GPS_FIX_3D;
        }
    } else {
        g_data.fix = GPS_FIX_NONE;
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

    // Valid when RMC reports active AND GGA shows fix
    g_data.valid = lwgps_is_valid(&g_gps) && (g_data.fix >= GPS_FIX_2D);
    g_data.time_valid = g_gps.time_valid;
    g_data.date_valid = g_gps.date_valid;

    // Diagnostic: raw lwGPS fields for sensor status debugging
    g_data.gga_fix = g_gps.fix;
    g_data.gsa_fix_mode = g_gps.fix_mode;
    g_data.rmc_valid = lwgps_is_valid(&g_gps);
}

// ============================================================================
// Public API
// ============================================================================

bool gps_pa1010d_init() {
    // Initialize lwGPS parser
    lwgps_init(&g_gps);

    // Clear data
    memset(&g_data, 0, sizeof(g_data));

    // Presence check: read a full buffer instead of single-byte probe.
    // The PA1010D is a UART-over-I2C device — it doesn't ACK single-byte
    // probes reliably. All reference implementations (pico-examples,
    // Adafruit, SparkFun) skip probing and go straight to data reads.
    // A successful 255-byte read with any '$' character confirms presence.
    int32_t ret = i2c_bus_read(kGpsPa1010dAddr, g_buffer, kGpsMaxRead);
    if (ret <= 0) {
        return false;
    }

    // Check for NMEA start character anywhere in the buffer
    bool found_nmea = false;
    for (int32_t i = 0; i < ret; i++) {
        if (g_buffer[i] == '$') {
            found_nmea = true;
            break;
        }
    }
    if (!found_nmea) {
        return false;
    }

    g_initialized = true;

    // Configure NMEA output: enable RMC + GGA + GSA sentences.
    // RMC: speed, course, date/time. GGA: position, altitude, fix quality.
    // GSA: fix mode (2D/3D) and DOP values — required for gps_valid flag.
    // Output ~180 bytes/sec (vs ~449 default with all sentences).
    // PMTK314 fields: GLL,RMC,VTG,GGA,GSA,GSV,...
    gps_pa1010d_send_command("PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

    return true;
}

bool gps_pa1010d_ready() {
    return g_initialized;
}

bool gps_pa1010d_update() {
    if (!g_initialized) {
        return false;
    }

    // Read available NMEA data (full 255-byte MT3333 TX buffer)
    // Returns: >0 = NMEA bytes, 0 = padding only (no new data), <0 = I2C error
    int len = read_nmea_data(g_buffer, kGpsMaxRead);
    if (len < 0) {
        return false;  // I2C failure — caller should count as error
    }
    if (len == 0) {
        return true;   // I2C OK but no new NMEA sentence — not an error
    }

    // Null-terminate for safety
    g_buffer[len] = '\0';

    // Process through lwGPS parser
    lwgps_process(&g_gps, g_buffer, len);

    // Update our data structure
    update_data_from_lwgps();

    return true;
}

bool gps_pa1010d_get_data(gps_pa1010d_data_t* data) {
    if (data == nullptr) {
        return false;
    }

    *data = g_data;
    return g_data.valid;
}

bool gps_pa1010d_has_fix() {
    return g_data.valid && (g_data.fix >= GPS_FIX_2D);
}

bool gps_pa1010d_send_command(const char* cmd) {
    if (!g_initialized || cmd == nullptr) {
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
    int ret = i2c_bus_write(kGpsPa1010dAddr, reinterpret_cast<const uint8_t*>(sentence), len);
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

bool gps_pa1010d_get_last_raw(const uint8_t** buf, size_t* len) {
    if (!g_initialized || g_lastReadLen == 0) {
        return false;
    }
    *buf = g_buffer;
    *len = g_lastReadLen;
    return true;
}
