// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// PA1010D I2C NMEA. Prior Art: GlobalTop v1.2; Adafruit GPS I2C; lwGPS.

#include "gps_pa1010d.h"
#include "i2c_master.h"
#include "lwgps/lwgps.h"
#include "etl/string.h"
#include "etl/to_string.h"
#include "pico/time.h"
#include <string.h>

constexpr uint8_t  kNmeaLf = 0x0A;
constexpr uint8_t  kNmeaCr = 0x0D;
constexpr uint8_t  kBusErr = 0xFF;
constexpr char     kNmeaStart = '$';
constexpr uint16_t kGpsYearBase = 2000;
constexpr uint8_t  kGsaFixMode3d = 3;
constexpr uint8_t  kGsaFixMode2d = 2;
constexpr size_t   kGpsMaxRead = 255;             // GlobalTop NMEA-over-I2C
constexpr uint32_t kGpsReadTimeoutUs = 50000;     // GlobalTop 50 ms
constexpr uint32_t kPollMinUs = 500000;           // GlobalTop 500 ms poll
constexpr uint32_t kPostReadUs = 2000;            // GlobalTop 2 ms refill
constexpr int      kNmeaMaxBody = 82;             // NMEA-0183 max
constexpr int      kPmtkUnset = -999;
constexpr uint8_t  kPmtkCount = 3;
constexpr uint8_t  kNmeaHuntTries = 8;
constexpr uint32_t kPmtkGapMs = 50;
constexpr uint32_t kWakeGapMs = 20;
constexpr uint32_t kNmeaHuntGapMs = 150;
constexpr size_t   kPmtk314Len = 51;
constexpr size_t   kPmtk220Len = 18;

constexpr uint8_t nmea_checksum_constexpr(const char* body) {
    uint8_t c = 0;
    for (int i = 0; i < kNmeaMaxBody; i++) {
        if (body[i] == '\0') {
            break;
        }
        c ^= static_cast<uint8_t>(body[i]);
    }
    return c;
}

constexpr char kPmtk314Body[] = "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
constexpr uint8_t kPmtk314Sentence[] =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
static_assert(nmea_checksum_constexpr(kPmtk314Body) == 0x28, "PMTK314 checksum");
static_assert(sizeof(kPmtk314Sentence) - 1 == kPmtk314Len, "PMTK314 length");

constexpr char kPmtk220Body[] = "PMTK220,1000";
constexpr uint8_t kPmtk220Sentence[] = "$PMTK220,1000*1F\r\n";
static_assert(nmea_checksum_constexpr(kPmtk220Body) == 0x1F, "PMTK220 checksum");
static_assert(sizeof(kPmtk220Sentence) - 1 == kPmtk220Len, "PMTK220 length");

static bool g_initialized = false;
static int g_pmtkWriteResults[kPmtkCount] = {kPmtkUnset, kPmtkUnset, kPmtkUnset};
static bool g_pmtkWindowHit = false;
static lwgps_t g_gps;
static gps_data_t g_data;
static uint32_t g_lastPollUs = 0;
static uint8_t g_buffer[kGpsMaxRead + 1];
static size_t g_lastReadLen = 0;
static uint8_t g_raw[kGpsMaxRead];

static int read_nmea_data(uint8_t* buffer, size_t max_len) {
    int ret = i2c_master_read(kGpsPa1010dAddr, g_raw, max_len, kGpsReadTimeoutUs);
    if (ret <= 0) {
        return -1;
    }
    int32_t out = 0;
    for (int i = 0; i < ret; i++) {
        const uint8_t b = g_raw[static_cast<size_t>(i)];
        if (b != kBusErr) {
            if (static_cast<size_t>(out) >= max_len) {
                break;
            }
            if (b == kNmeaLf) {
                if (out > 0 && buffer[static_cast<size_t>(out - 1)] == kNmeaCr) {
                    buffer[static_cast<size_t>(out)] = b;
                    out++;
                }
            } else {
                buffer[static_cast<size_t>(out)] = b;
                out++;
            }
        }
    }
    g_lastReadLen = static_cast<size_t>(out);
    return out;
}

static void update_from_lwgps() {
    g_data.latitude = g_gps.latitude;
    g_data.longitude = g_gps.longitude;
    g_data.altitudeM = static_cast<float>(g_gps.altitude);
    g_data.speedKnots = static_cast<float>(g_gps.speed);
    g_data.speedMps = static_cast<float>(lwgps_to_speed(g_gps.speed, LWGPS_SPEED_MPS));
    g_data.courseDeg = static_cast<float>(g_gps.course);
    if (g_gps.fix >= 1) {
        if (g_gps.fix_mode == kGsaFixMode2d) {
            g_data.fix = GPS_FIX_2D;
        } else if (g_gps.fix_mode == kGsaFixMode3d) {
            g_data.fix = GPS_FIX_3D;
        } else {
            g_data.fix = GPS_FIX_3D;  // GGA fix, GSA mode not yet latched
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
    g_data.timeValid = (g_gps.time_valid != 0U);
    g_data.dateValid = (g_gps.date_valid != 0U);
    g_data.ggaFix = g_gps.fix;
    g_data.gsaFixMode = g_gps.fix_mode;
    g_data.rmcValid = lwgps_is_valid(&g_gps);
}

static void wake_bridge() {
    uint8_t wake = 0;
    (void)i2c_master_read(kGpsPa1010dAddr, &wake, 1, kGpsReadTimeoutUs);
    (void)i2c_master_read(kGpsPa1010dAddr, g_buffer, kGpsMaxRead, kGpsReadTimeoutUs);
    sleep_ms(kWakeGapMs);
}

static bool send_pmtk() {
    g_pmtkWriteResults[0] = i2c_master_write(
        kGpsPa1010dAddr, kPmtk314Sentence,
        sizeof(kPmtk314Sentence) - 1, kGpsReadTimeoutUs);
    sleep_ms(kPmtkGapMs);
    g_pmtkWriteResults[1] = i2c_master_write(
        kGpsPa1010dAddr, kPmtk220Sentence,
        sizeof(kPmtk220Sentence) - 1, kGpsReadTimeoutUs);
    sleep_ms(kPmtkGapMs);
    g_pmtkWriteResults[2] = i2c_master_write(
        kGpsPa1010dAddr, kPmtk314Sentence,
        sizeof(kPmtk314Sentence) - 1, kGpsReadTimeoutUs);
    sleep_ms(kPmtkGapMs);
    return (g_pmtkWriteResults[0] > 0) && (g_pmtkWriteResults[1] > 0) &&
           (g_pmtkWriteResults[2] > 0);
}

static bool find_nmea() {
    for (int retry = 0; retry < kNmeaHuntTries; retry++) {
        int ret = i2c_master_read(kGpsPa1010dAddr, g_buffer, kGpsMaxRead,
                                  kGpsReadTimeoutUs);
        if (ret > 0) {
            for (int i = 0; i < ret; i++) {
                if (g_buffer[static_cast<size_t>(i)] == kNmeaStart) {
                    return true;
                }
            }
        }
        sleep_ms(kNmeaHuntGapMs);
    }
    return false;
}

bool gps_pa1010d_init() {
    lwgps_init(&g_gps);
    memset(&g_data, 0, sizeof(g_data));
    wake_bridge();
    if (!send_pmtk()) {
        sleep_ms(kWakeGapMs);
        wake_bridge();
        (void)send_pmtk();
    }
    g_pmtkWindowHit = find_nmea();
    g_initialized = (g_pmtkWriteResults[0] > 0) &&
                    (g_pmtkWriteResults[1] > 0) &&
                    (g_pmtkWriteResults[2] > 0);
    return g_initialized;
}

void gps_pa1010d_get_debug_status(char* buf, size_t len) {
    if (buf == nullptr || len == 0) {
        return;
    }
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
    const size_t n = (tmp.size() < (len - 1U)) ? tmp.size() : (len - 1U);
    memcpy(buf, tmp.data(), n);
    buf[n] = '\0';
}

bool gps_pa1010d_ready() {
    return g_initialized;
}

bool gps_pa1010d_update() {
    if (!g_initialized) {
        return false;
    }
    const uint32_t now_us = time_us_32();
    if ((now_us - g_lastPollUs) < kPollMinUs) {
        return true;
    }
    g_lastPollUs = now_us;
    int len = read_nmea_data(g_buffer, kGpsMaxRead);
    busy_wait_us(kPostReadUs);
    if (len < 0) {
        return false;
    }
    if (len == 0) {
        return true;
    }
    g_buffer[static_cast<size_t>(len)] = '\0';
    lwgps_process(&g_gps, g_buffer, static_cast<size_t>(len));
    update_from_lwgps();
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
    if (!g_initialized || g_lastReadLen == 0 || buf == nullptr || len == nullptr) {
        return false;
    }
    *buf = g_buffer;
    *len = g_lastReadLen;
    return true;
}
