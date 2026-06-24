// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file gps_pa1010d.cpp
 * @brief PA1010D GPS module driver using lwGPS library
 *
 * Reads NMEA sentences from PA1010D via I2C and parses with lwGPS.
 *
 * Prior Art:
 *   - CDTop PA1010D datasheet (MT3333 chipset)
 *   - Adafruit PA1010D Arduino/CircuitPython GPS library (I2C chunked reads)
 *   - lwGPS library (vendored in lib/lwgps/) — NMEA parser
 */

#include "gps_pa1010d.h"
#include "i2c_bus.h"
#include "lwgps/lwgps.h"
#include "etl/string.h"
#include "etl/to_string.h"
#include <string.h>

// NMEA/I2C protocol constants
constexpr uint8_t  kNmeaLf           = 0x0A;   // Line feed — padding byte when GPS buffer empty
constexpr uint8_t  kNmeaCr           = 0x0D;   // Carriage return — NMEA line terminator
constexpr uint8_t  kI2cBusErrorByte  = 0xFF;   // Bus error indicator
constexpr char     kNmeaStart        = '$';    // NMEA sentence start delimiter

// lwGPS 2-digit year base
constexpr uint16_t kGpsYearBase      = 2000;

// Fix type thresholds (GSA fixMode values per NMEA spec)
constexpr uint8_t  kGsaFixMode3d     = 3;      // GSA fixMode >= 3 = 3D fix
constexpr uint8_t  kGsaFixMode2d     = 2;      // GSA fixMode == 2 = 2D fix

// ============================================================================
// R-2 absorbed (R-5 Unit D part 2a, 2026-05-16, council-approved): the 3
// blind PMTK writes at init are now precomputed const arrays with
// compile-time checksum + length verification via static_assert. No more
// snprintf in the init path. Pattern source: R-2's prior council
// (referenced in `docs/plans/R5_STDIO_REMOVAL.md` line 76 + PROBLEM_REPORTS
// row R-2). NASA/JPL framing: correct-by-construction at compile time
// instead of correct-by-runtime-formatting. Trades ~120 bytes of .rodata
// (negligible on 4 MB flash) for zero snprintf attack surface.
// ============================================================================

// Compile-time NMEA checksum: XOR of all bytes between '$' and '*' (exclusive).
// Same algorithm as the now-deleted runtime nmea_checksum() helper.
constexpr uint8_t nmea_checksum_constexpr(const char* body) {
    uint8_t c = 0;
    while (*body != '\0') {
        c ^= static_cast<uint8_t>(*body);
        ++body;
    }
    return c;
}

// PMTK314 — enable RMC + GGA + GSA + GSV sentences (rest disabled).
// Sent twice during init per the cold-boot window discovery (see init()).
constexpr char kPmtk314Body[] = "PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
constexpr char kPmtk314Sentence[] =
    "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
static_assert(nmea_checksum_constexpr(kPmtk314Body) == 0x29,
              "PMTK314 checksum mismatch — verify literal matches sentence body");
static_assert(sizeof(kPmtk314Sentence) - 1 == 51,
              "PMTK314 sentence byte length mismatch");

// PMTK220,1000 — set NMEA output interval to 1000ms = 1 Hz.
constexpr char kPmtk220_1HzBody[] = "PMTK220,1000";
constexpr char kPmtk220_1HzSentence[] = "$PMTK220,1000*1F\r\n";
static_assert(nmea_checksum_constexpr(kPmtk220_1HzBody) == 0x1F,
              "PMTK220,1000 checksum mismatch — verify literal matches sentence body");
static_assert(sizeof(kPmtk220_1HzSentence) - 1 == 18,
              "PMTK220,1000 sentence byte length mismatch");

// ============================================================================
// Private State
// ============================================================================

static bool g_initialized = false;

// Grok-triage instrumentation: capture blind-PMTK write return codes and
// whether the post-config probe hit. Rendered by gps_pa1010d_get_debug_status()
// for display in Hardware Status (`b`) since init_early_hw() runs before
// USB CDC, so any printf() from init is dropped.
static int  g_pmtkWriteResults[3] = { -999, -999, -999 };
static bool g_pmtkWindowHit = false;
static lwgps_t g_gps;
static gps_data_t g_data;

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
static int read_nmea_data(uint8_t* buffer, size_t maxLen) {
    // Read raw I2C data into a local buffer, then filter in-place
    static uint8_t g_raw[kGpsMaxRead];
    int32_t ret = i2c_bus_read(kGpsPa1010dAddr, g_raw, maxLen);
    if (ret <= 0) {
        return -1;  // I2C failure (NACK or timeout)
    }

    // Filter: copy valid bytes, discard padding 0x0A and 0xFF
    int32_t out = 0;
    for (int32_t i = 0; i < ret; i++) {
        if (g_raw[i] == kI2cBusErrorByte) {
            // Bus error byte — discard
        } else if (g_raw[i] == kNmeaLf) {
            // Keep LF only if it follows CR (legitimate \r\n terminator)
            if (out > 0 && buffer[out - 1] == kNmeaCr) {
                buffer[out++] = g_raw[i];
            }
            // Otherwise discard — it's padding
        } else {
            buffer[out++] = g_raw[i];
        }
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
    g_data.altitudeM = static_cast<float>(g_gps.altitude);

    g_data.speedKnots = static_cast<float>(g_gps.speed);
    g_data.speedMps = static_cast<float>(lwgps_to_speed(g_gps.speed, LWGPS_SPEED_MPS));
    g_data.courseDeg = static_cast<float>(g_gps.course);

    // Fix type: prefer GGA fix quality (most reliably updated by lwGPS),
    // fall back to GSA fixMode for 2D/3D distinction.
    // GGA fix: 0=none, 1=GPS, 2=DGPS, 3=PPS, 4+=RTK
    // GSA fixMode: 1=none, 2=2D, 3=3D
    if (g_gps.fix >= 1) {
        // GGA says we have a fix — use GSA for 2D/3D if available.
        // GSA fixMode==2 is 2D; anything else (3, or not yet updated) → 3D.
        if (g_gps.fix_mode == kGsaFixMode2d) {
            g_data.fix = GPS_FIX_2D;
        } else {
            g_data.fix = GPS_FIX_3D;
        }
    } else {
        g_data.fix = GPS_FIX_NONE;
    }

    g_data.satellites = g_gps.sats_in_use;
    g_data.hdop = static_cast<float>(g_gps.dop_h);
    g_data.vdop = static_cast<float>(g_gps.dop_v);
    g_data.pdop = static_cast<float>(g_gps.dop_p);

    g_data.hour = g_gps.hours;
    g_data.minute = g_gps.minutes;
    g_data.second = g_gps.seconds;

    g_data.day = g_gps.date;
    g_data.month = g_gps.month;
    g_data.year = kGpsYearBase + g_gps.year;  // lwGPS stores 2-digit year

    // Valid when RMC reports active AND GGA shows fix
    g_data.valid = lwgps_is_valid(&g_gps) && (g_data.fix >= GPS_FIX_2D);
    g_data.timeValid = (g_gps.time_valid != 0U);
    g_data.dateValid = (g_gps.date_valid != 0U);

    // Diagnostic: raw lwGPS fields for sensor status debugging
    g_data.ggaFix = g_gps.fix;
    g_data.gsaFixMode = g_gps.fix_mode;
    g_data.rmcValid = lwgps_is_valid(&g_gps);
}

// ============================================================================
// Public API
// ============================================================================

bool gps_pa1010d_init() {
    // Initialize lwGPS parser
    lwgps_init(&g_gps);

    // Clear data
    memset(&g_data, 0, sizeof(g_data));

    // Fresh bus state right before GPS (Grok triage — window is tiny).
    i2c_bus_recover();
    sleep_ms(20);

    // BLIND PMTK config — send the full sequence even if probe fails.
    // This is the fix for the "GPS never detected" regression: the PA1010D
    // exposes a transient ACK window at cold boot, then drops to low-power
    // if no command arrives. Probing first misses that window.
    //
    // R-5 Unit D part 2a (2026-05-16): const-array sentences with
    // compile-time checksum verification. No snprintf in this path; the
    // three byte-on-wire sequences are correct-by-construction.
    g_pmtkWriteResults[0] = i2c_bus_write(
        kGpsPa1010dAddr,
        reinterpret_cast<const uint8_t*>(kPmtk314Sentence),
        sizeof(kPmtk314Sentence) - 1);
    sleep_ms(50);
    g_pmtkWriteResults[1] = i2c_bus_write(
        kGpsPa1010dAddr,
        reinterpret_cast<const uint8_t*>(kPmtk220_1HzSentence),
        sizeof(kPmtk220_1HzSentence) - 1);
    sleep_ms(50);
    g_pmtkWriteResults[2] = i2c_bus_write(
        kGpsPa1010dAddr,
        reinterpret_cast<const uint8_t*>(kPmtk314Sentence),
        sizeof(kPmtk314Sentence) - 1);
    sleep_ms(50);

    // Aggressive probe retry — now that the module should be locked into
    // full-power mode by blind PMTK, read a full buffer and look for NMEA.
    bool foundNmea = false;
    for (int retry = 0; retry < 8 && !foundNmea; retry++) {
        int32_t ret = i2c_bus_read(kGpsPa1010dAddr, g_buffer, kGpsMaxRead);
        if (ret > 0) {
            for (int32_t i = 0; i < ret; i++) {
                if (g_buffer[i] == kNmeaStart) {
                    foundNmea = true;
                    break;
                }
            }
        }
        if (!foundNmea) {
            sleep_ms(150);
        }
    }
    if (!foundNmea) {
        return false;
    }

    g_pmtkWindowHit = true;
    g_initialized = true;
    return true;
}

void gps_pa1010d_get_debug_status(char* buf, size_t len) {
    if (buf == nullptr || len == 0) return;

    // R-5 Unit D part 2a (2026-05-16, council Option X): hand-rolled
    // etl::string + etl::to_string into caller buffer. Preserves the
    // (buf, len) API surface so the single caller
    // (`src/cli/rc_os_commands.cpp:643`) is untouched — keeps Tier 5
    // (rc_os_commands.cpp) scope out of this Tier 2 commit.
    //
    // Byte-on-wire identity vs prior snprintf format is required
    // (see commit message for the captured baseline). Double-space
    // between `]` and `window_hit:`, and between `:N` and `init:`,
    // is load-bearing — preserved verbatim below.
    etl::string<96> tmp;
    tmp.append("PMTK writes: [");
    etl::to_string(g_pmtkWriteResults[0], tmp, true);
    tmp.append(",");
    etl::to_string(g_pmtkWriteResults[1], tmp, true);
    tmp.append(",");
    etl::to_string(g_pmtkWriteResults[2], tmp, true);
    tmp.append("]  window_hit:");
    etl::to_string(g_pmtkWindowHit ? 1 : 0, tmp, true);
    tmp.append("  init:");
    etl::to_string(g_initialized ? 1 : 0, tmp, true);

    // Copy to caller buffer; truncate at len-1 to leave room for NUL.
    const size_t writable = (tmp.size() < (len - 1U)) ? tmp.size() : (len - 1U);
    memcpy(buf, tmp.data(), writable);
    buf[writable] = '\0';
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

bool gps_pa1010d_get_data(gps_data_t* data) {
    if (data == nullptr) {
        return false;
    }

    *data = g_data;
    return g_data.valid;
}

bool gps_pa1010d_has_fix() {
    return g_data.valid && (g_data.fix >= GPS_FIX_2D);
}

bool gps_pa1010d_get_last_raw(const uint8_t** buf, size_t* len) {
    if (!g_initialized || g_lastReadLen == 0) {
        return false;
    }
    *buf = g_buffer;
    *len = g_lastReadLen;
    return true;
}
