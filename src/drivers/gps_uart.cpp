/**
 * @file gps_uart.cpp
 * @brief UART GPS driver using lwGPS library
 *
 * Reads NMEA sentences from a UART GPS module (e.g., Adafruit Ultimate GPS
 * FeatherWing #3133 / PA1616D) and parses with lwGPS.
 *
 * Architecture mirrors gps_pa1010d.cpp:
 *   - Same lwGPS parser (NMEA parsing is transport-agnostic)
 *   - Same gps_data_t output (transport-neutral, defined in gps.h)
 *   - Same PMTK command interface
 *
 * Key differences from I2C backend:
 *   - No padding filter (UART gives clean bytes, no 0x0A padding)
 *   - No settling delay (point-to-point, no bus contention)
 *   - Non-blocking FIFO drain instead of bulk I2C read
 *   - 2-second presence detection timeout at init
 *
 * Ref: Adafruit Ultimate GPS FeatherWing product page, MT3339 datasheet.
 */

#include "gps_uart.h"
#include "lwgps/lwgps.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// Constants
// ============================================================================

// UART instance — uart0 on GPIO0/1 per Feather pinout.
// uart0 is a macro (reinterpret_cast), not constexpr — use #define.
#define GPS_UART_INST uart0

// NMEA protocol constants (shared with I2C backend by value, not by reference)
constexpr char     kNmeaStart       = '$';
constexpr uint16_t kGpsYearBase     = 2000;
constexpr uint8_t  kGsaFixMode3d    = 3;
constexpr uint8_t  kGsaFixMode2d    = 2;

// PMTK220 rate intervals (ms)
constexpr uint16_t kGpsRate1Hz      = 1000;
constexpr uint16_t kGpsRate5Hz      = 200;
constexpr uint16_t kGpsRate10Hz     = 100;

// NMEA command buffer
constexpr size_t   kNmeaCmdBufSize  = 128;
constexpr size_t   kNmeaChecksumLen = 5;       // "*XX\r\n"

// Init: presence detection timeout
// MT3339 outputs NMEA at 1Hz default — 2 seconds guarantees at least one sentence.
constexpr uint32_t kInitTimeoutUs   = 2000000; // 2 seconds

// Update: max bytes to drain per call.
// At 9600 baud / 10Hz poll = ~96 bytes/interval. 512 gives 5x margin.
// Bounded to prevent monopolizing Core 1 if FIFO backed up.
constexpr size_t   kMaxDrainBytes   = 512;

// ============================================================================
// Private State
// ============================================================================

static bool g_initialized = false;
static lwgps_t g_gps;
static gps_data_t g_data;

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Calculate NMEA checksum (XOR of bytes between '$' and '*')
 */
static uint8_t nmea_checksum(const char* sentence) {
    uint8_t checksum = 0;
    if (*sentence == '$') {
        sentence++;
    }
    while (*sentence != '\0' && *sentence != '*') {
        checksum ^= static_cast<uint8_t>(*sentence++);
    }
    return checksum;
}

/**
 * @brief Update internal data structure from lwGPS parser state
 *
 * Duplicated from gps_pa1010d.cpp — both backends produce identical
 * gps_data_t from the same lwGPS state. Factoring into a shared helper
 * would couple two otherwise-independent drivers for ~40 lines of
 * trivial field copies. Not worth the dependency.
 */
static void update_data_from_lwgps() {
    g_data.latitude = g_gps.latitude;
    g_data.longitude = g_gps.longitude;
    g_data.altitudeM = static_cast<float>(g_gps.altitude);

    g_data.speedKnots = static_cast<float>(g_gps.speed);
    g_data.speedMps = static_cast<float>(lwgps_to_speed(g_gps.speed, LWGPS_SPEED_MPS));
    g_data.courseDeg = static_cast<float>(g_gps.course);

    // Fix type: prefer GGA fix quality, fall back to GSA fixMode for 2D/3D
    if (g_gps.fix >= 1) {
        if (g_gps.fix_mode >= kGsaFixMode3d) {
            g_data.fix = GPS_FIX_3D;
        } else if (g_gps.fix_mode == kGsaFixMode2d) {
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
    g_data.year = kGpsYearBase + g_gps.year;

    g_data.valid = lwgps_is_valid(&g_gps) && (g_data.fix >= GPS_FIX_2D);
    g_data.timeValid = g_gps.time_valid;
    g_data.dateValid = g_gps.date_valid;

    g_data.ggaFix = g_gps.fix;
    g_data.gsaFixMode = g_gps.fix_mode;
    g_data.rmcValid = lwgps_is_valid(&g_gps);
}

// ============================================================================
// Public API
// ============================================================================

bool gps_uart_init() {
    // Initialize lwGPS parser
    lwgps_init(&g_gps);
    memset(&g_data, 0, sizeof(g_data));

    // Configure UART0
    uart_init(GPS_UART_INST, kGpsUartBaud);
    gpio_set_function(kGpsUartTxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartTxPin));
    gpio_set_function(kGpsUartRxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartRxPin));

    // Presence detection: drain UART for up to 2 seconds looking for '$'.
    // MT3339 outputs NMEA at 1Hz by default, so 2 seconds guarantees at
    // least one full sentence cycle if a GPS is connected.
    // This only adds delay when NO UART GPS is connected.
    bool foundNmea = false;
    absolute_time_t deadline = make_timeout_time_us(kInitTimeoutUs);

    while (!time_reached(deadline)) {
        if (uart_is_readable(GPS_UART_INST)) {
            char c = static_cast<char>(uart_getc(GPS_UART_INST));
            if (c == kNmeaStart) {
                foundNmea = true;
                break;
            }
        }
    }

    if (!foundNmea) {
        // No GPS detected — deinit UART so pins are free
        uart_deinit(GPS_UART_INST);
        return false;
    }

    g_initialized = true;

    // Configure NMEA output: RMC + GGA + GSA (same as I2C backend)
    gps_uart_send_command("PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

    return true;
}

bool gps_uart_ready() {
    return g_initialized;
}

bool gps_uart_update() {
    if (!g_initialized) {
        return false;
    }

    // Non-blocking drain: read available bytes from UART RX FIFO.
    // Bounded at kMaxDrainBytes to prevent monopolizing Core 1.
    uint8_t buf[kMaxDrainBytes];
    size_t count = 0;

    while (count < kMaxDrainBytes && uart_is_readable(GPS_UART_INST)) {
        buf[count++] = static_cast<uint8_t>(uart_getc(GPS_UART_INST));
    }

    if (count == 0) {
        return true;  // No data available — not an error
    }

    // Feed to lwGPS parser
    lwgps_process(&g_gps, buf, count);

    // Update data structure
    update_data_from_lwgps();

    return true;
}

bool gps_uart_get_data(gps_data_t* data) {
    if (data == nullptr) {
        return false;
    }
    *data = g_data;
    return g_data.valid;
}

bool gps_uart_has_fix() {
    return g_data.valid && (g_data.fix >= GPS_FIX_2D);
}

bool gps_uart_send_command(const char* cmd) {
    if (!g_initialized || cmd == nullptr) {
        return false;
    }

    // Build full NMEA sentence with checksum
    char sentence[kNmeaCmdBufSize];
    int len = snprintf(sentence, sizeof(sentence) - kNmeaChecksumLen, "$%s*", cmd);
    if (len < 0 || len >= static_cast<int>(sizeof(sentence) - kNmeaChecksumLen)) {
        return false;
    }

    // Calculate and append checksum
    uint8_t cs = nmea_checksum(sentence);
    snprintf(sentence + len, kNmeaChecksumLen, "%02X\r\n", cs);
    len += 4;  // "XX\r\n"

    // Send via UART
    uart_write_blocking(GPS_UART_INST, reinterpret_cast<const uint8_t*>(sentence), static_cast<size_t>(len));

    return true;
}

bool gps_uart_set_rate(uint8_t rateHz) {
    char cmd[32];
    uint16_t intervalMs = 0;

    switch (rateHz) {
        case 1:  intervalMs = kGpsRate1Hz;  break;
        case 5:  intervalMs = kGpsRate5Hz;  break;
        case 10: intervalMs = kGpsRate10Hz; break;
        default: return false;
    }

    snprintf(cmd, sizeof(cmd), "PMTK220,%u", intervalMs);
    return gps_uart_send_command(cmd);
}
