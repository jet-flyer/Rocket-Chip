// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Pad ANSI row formatters. Header-only for host tests.
// RSSI is SX1276 RegPktRssiValue (dBm), not 211.2 Annex B Quality Indicator.
// Zulu is GPS UTC on this board. MET is the vehicle's mission clock.
#ifndef ROCKETCHIP_RC_OS_DASHBOARD_FORMAT_H
#define ROCKETCHIP_RC_OS_DASHBOARD_FORMAT_H

#include "rocketchip/rc_log.h"
#include <stddef.h>
#include <stdint.h>

namespace rc {
namespace dash {

static constexpr uint32_t kSecPerDay = 86400U;
static constexpr uint32_t kSecPerHour = 3600U;
static constexpr uint32_t kSecPerMin = 60U;
static constexpr int32_t kDeg1e7 = 10000000;
static constexpr int32_t kZone15_1e7 = 15 * kDeg1e7;
static constexpr int32_t kZoneHalf_1e7 = 75 * (kDeg1e7 / 10);
static constexpr int16_t kDstShiftMin = 60;

inline void utc_plus_minutes(uint8_t h, uint8_t m, uint8_t s, int16_t off_min,
                             uint8_t* oh, uint8_t* om, uint8_t* os) {
    int32_t t = (static_cast<int32_t>(h) * static_cast<int32_t>(kSecPerHour)) +
                (static_cast<int32_t>(m) * static_cast<int32_t>(kSecPerMin)) +
                static_cast<int32_t>(s);
    t += static_cast<int32_t>(off_min) * static_cast<int32_t>(kSecPerMin);
    t %= static_cast<int32_t>(kSecPerDay);
    if (t < 0) {
        t += static_cast<int32_t>(kSecPerDay);
    }
    *oh = static_cast<uint8_t>(t / static_cast<int32_t>(kSecPerHour));
    t %= static_cast<int32_t>(kSecPerHour);
    *om = static_cast<uint8_t>(t / static_cast<int32_t>(kSecPerMin));
    *os = static_cast<uint8_t>(t % static_cast<int32_t>(kSecPerMin));
}

// Local GPS wins. A side with no GPS may use the peer. Two GPS clocks
// are not disciplined to each other. A vehicle whose mission clock is
// already running does not take a time from the station.
enum class TimePick : uint8_t { kNone = 0, kLocalGps, kPeerGps };

inline TimePick choose_time_source(bool local_gps, bool peer_gps,
                                   bool vehicle_met_running, bool i_am_vehicle) {
    if (local_gps) {
        return TimePick::kLocalGps;
    }
    if (peer_gps && !(i_am_vehicle && vehicle_met_running)) {
        return TimePick::kPeerGps;
    }
    return TimePick::kNone;
}

// Last NMEA second is the anchor. The pad redraw is 1 Hz, so a stuck
// NMEA second still advances from the station monotonic clock.
struct ZuluRun {
    bool anchored;
    uint32_t nmea_sod;
    uint32_t anchor_sod;
    uint32_t anchor_ms;
};

inline uint32_t hms_to_sod(uint8_t h, uint8_t m, uint8_t s) {
    return (static_cast<uint32_t>(h) * kSecPerHour) +
           (static_cast<uint32_t>(m) * kSecPerMin) +
           static_cast<uint32_t>(s);
}

inline void sod_to_hms(uint32_t sod, uint8_t* h, uint8_t* m, uint8_t* s) {
    sod %= kSecPerDay;
    *h = static_cast<uint8_t>(sod / kSecPerHour);
    sod %= kSecPerHour;
    *m = static_cast<uint8_t>(sod / kSecPerMin);
    *s = static_cast<uint8_t>(sod % kSecPerMin);
}

inline void zulu_run_apply(ZuluRun* run, bool nmea_ok, uint8_t h, uint8_t m,
                           uint8_t s, uint32_t mono_ms, bool* out_ok,
                           uint8_t* oh, uint8_t* om, uint8_t* os) {
    if (nmea_ok) {
        const uint32_t sod = hms_to_sod(h, m, s);
        if (!run->anchored || sod != run->nmea_sod) {
            run->anchored = true;
            run->nmea_sod = sod;
            run->anchor_sod = sod;
            run->anchor_ms = mono_ms;
        }
    }
    if (!run->anchored) {
        *out_ok = false;
        return;
    }
    const uint32_t elapsed_s = (mono_ms - run->anchor_ms) / 1000U;
    sod_to_hms(run->anchor_sod + elapsed_s, oh, om, os);
    *out_ok = true;
}

// Civil offset, minutes east of UTC.
// US DST: Energy Policy Act of 2005, 2nd Sunday in March 02:00 standard
// through 1st Sunday in November 02:00 daylight.
// EU DST: Directive 2000/84/EC, last Sunday in March 01:00 UTC through
// last Sunday in October 01:00 UTC.
// Boxes are coarse on purpose. First match wins. Anywhere else is the
// 15° nautical zone with no daylight rule.
enum class TzDst : uint8_t { kNone = 0, kUs = 1, kEu = 2 };

struct TzBox {
    int16_t lat0;
    int16_t lat1;
    int16_t lon0;
    int16_t lon1;
    int16_t std_min;
    TzDst dst;
};

inline bool leap_year(int year) {
    if ((year % 400) == 0) {
        return true;
    }
    if ((year % 100) == 0) {
        return false;
    }
    return (year % 4) == 0;
}

// 0 = Sunday. Sakamoto.
inline int dow_sun0(int year, int month, int day) {
    static const int kTbl[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    int y = year;
    if (month < 3) {
        y -= 1;
    }
    int v = y + (y / 4) - (y / 100) + (y / 400) + kTbl[month - 1] + day;
    v %= 7;
    if (v < 0) {
        v += 7;
    }
    return v;
}

inline int nth_sunday(int year, int month, int n) {
    const int first = dow_sun0(year, month, 1);
    const int day = 1 + ((7 - first) % 7);
    return day + ((n - 1) * 7);
}

inline int last_sunday(int year, int month) {
    static const int kDim[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int dim = kDim[month];
    if (month == 2 && leap_year(year)) {
        dim = 29;
    }
    return dim - dow_sun0(year, month, dim);
}

inline int32_t utc_sod(uint8_t h, uint8_t m, uint8_t s) {
    return static_cast<int32_t>(hms_to_sod(h, m, s));
}

inline bool us_dst_active(int year, int month, int day, int32_t utc,
                          int16_t std_min) {
    if (month < 3 || month > 11) {
        return false;
    }
    if (month > 3 && month < 11) {
        return true;
    }
    int32_t local = utc + (static_cast<int32_t>(std_min) * 60);
    local %= static_cast<int32_t>(kSecPerDay);
    if (local < 0) {
        local += static_cast<int32_t>(kSecPerDay);
    }
    if (month == 3) {
        const int start = nth_sunday(year, 3, 2);
        if (day < start) {
            return false;
        }
        if (day > start) {
            return true;
        }
        return local >= (2 * static_cast<int32_t>(kSecPerHour));
    }
    const int end = nth_sunday(year, 11, 1);
    if (day < end) {
        return true;
    }
    if (day > end) {
        return false;
    }
    return local < static_cast<int32_t>(kSecPerHour);
}

inline bool eu_dst_active(int year, int month, int day, int32_t utc) {
    if (month < 3 || month > 10) {
        return false;
    }
    if (month > 3 && month < 10) {
        return true;
    }
    if (month == 3) {
        const int start = last_sunday(year, 3);
        if (day < start) {
            return false;
        }
        if (day > start) {
            return true;
        }
        return utc >= static_cast<int32_t>(kSecPerHour);
    }
    const int end = last_sunday(year, 10);
    if (day < end) {
        return true;
    }
    if (day > end) {
        return false;
    }
    return utc < static_cast<int32_t>(kSecPerHour);
}

inline int16_t solar_offset_min(int32_t lon_1e7) {
    const int32_t q = lon_1e7 / kZone15_1e7;
    const int32_t r = lon_1e7 % kZone15_1e7;
    int32_t hours = q;
    if (r >= kZoneHalf_1e7) {
        hours += 1;
    } else if (r <= -kZoneHalf_1e7) {
        hours -= 1;
    }
    if (hours > 14) {
        hours = 14;
    }
    if (hours < -12) {
        hours = -12;
    }
    return static_cast<int16_t>(hours * static_cast<int32_t>(kSecPerMin));
}

inline bool in_tz_box(const TzBox& box, int32_t lat_1e7, int32_t lon_1e7) {
    const int32_t lat0 = static_cast<int32_t>(box.lat0) * kDeg1e7;
    const int32_t lat1 = static_cast<int32_t>(box.lat1) * kDeg1e7;
    const int32_t lon0 = static_cast<int32_t>(box.lon0) * kDeg1e7;
    const int32_t lon1 = static_cast<int32_t>(box.lon1) * kDeg1e7;
    return lat_1e7 >= lat0 && lat_1e7 < lat1 && lon_1e7 >= lon0 && lon_1e7 < lon1;
}

inline int16_t tz_offset_min(int32_t lat_1e7, int32_t lon_1e7, int year,
                             int month, int day, uint8_t h, uint8_t m,
                             uint8_t s) {
    static const TzBox kBoxes[] = {
        {31, 37, -115, -109, -420, TzDst::kNone},  // Arizona
        {18, 23, -161, -154, -600, TzDst::kNone},  // Hawaii
        {51, 72, -170, -129, -540, TzDst::kUs},    // Alaska
        {32, 50, -125, -114, -480, TzDst::kUs},    // Pacific
        {31, 49, -114, -102, -420, TzDst::kUs},    // Mountain
        {25, 49, -102, -85, -360, TzDst::kUs},     // Central
        {24, 49, -85, -66, -300, TzDst::kUs},      // Eastern
        {6, 36, 68, 98, 330, TzDst::kNone},        // India +5:30
        {18, 54, 73, 135, 480, TzDst::kNone},      // China +8
        {36, 43, -10, -6, 0, TzDst::kEu},          // Portugal
        {36, 44, -10, 4, 60, TzDst::kEu},          // Spain
        {49, 61, -11, 2, 0, TzDst::kEu},           // UK, Ireland
        {35, 72, -5, 30, 60, TzDst::kEu},          // Central Europe
    };
    const int32_t utc = utc_sod(h, m, s);
    for (const TzBox& box : kBoxes) {
        if (!in_tz_box(box, lat_1e7, lon_1e7)) {
            continue;
        }
        bool dst = false;
        if (box.dst == TzDst::kUs && month >= 1 && month <= 12) {
            dst = us_dst_active(year, month, day, utc, box.std_min);
        } else if (box.dst == TzDst::kEu && month >= 1 && month <= 12) {
            dst = eu_dst_active(year, month, day, utc);
        }
        int32_t off = box.std_min;
        if (dst) {
            off += kDstShiftMin;
        }
        return static_cast<int16_t>(off);
    }
    return solar_offset_min(lon_1e7);
}

// One UTC day. A longer count belongs on tomorrow's Zulu mark.
static constexpr uint16_t kTMinusMaxMin = 1440;

struct TMinus {
    bool on;
    uint32_t target_ms;
};

inline bool parse_tminus_minutes(const char* s, uint16_t* minutes) {
    if (s == nullptr || s[0] == '\0' || minutes == nullptr) {
        return false;
    }
    uint32_t v = 0;
    for (const char* p = s; *p != '\0'; ++p) {
        if (*p < '0' || *p > '9') {
            return false;
        }
        v = (v * 10U) + static_cast<uint32_t>(*p - '0');
        if (v > kTMinusMaxMin) {
            return false;
        }
    }
    if (v == 0U) {
        return false;
    }
    *minutes = static_cast<uint16_t>(v);
    return true;
}

inline bool parse_tminus_zulu(const char* s, uint8_t* h, uint8_t* m, uint8_t* sec) {
    if (s == nullptr || h == nullptr || m == nullptr || sec == nullptr) {
        return false;
    }
    char dig[6] = {};
    int n = 0;
    for (const char* p = s; *p != '\0'; ++p) {
        if (*p == ':') {
            continue;
        }
        if (*p < '0' || *p > '9' || n >= 6) {
            return false;
        }
        dig[n++] = *p;
    }
    if (n != 6) {
        return false;
    }
    const uint8_t hh = static_cast<uint8_t>((dig[0] - '0') * 10 + (dig[1] - '0'));
    const uint8_t mm = static_cast<uint8_t>((dig[2] - '0') * 10 + (dig[3] - '0'));
    const uint8_t ss = static_cast<uint8_t>((dig[4] - '0') * 10 + (dig[5] - '0'));
    if (hh > 23 || mm > 59 || ss > 59) {
        return false;
    }
    *h = hh;
    *m = mm;
    *sec = ss;
    return true;
}

// Equal Zulu is this instant. An earlier clock time is the next UTC day.
inline uint32_t tminus_zulu_delta_s(uint32_t now_sod, uint32_t tgt_sod) {
    if (tgt_sod >= now_sod) {
        return tgt_sod - now_sod;
    }
    return (kSecPerDay - now_sod) + tgt_sod;
}

inline void tminus_arm(TMinus* t, uint32_t now_ms, uint32_t delta_s) {
    t->on = true;
    t->target_ms = now_ms + (delta_s * 1000U);
}

inline int32_t tminus_remaining_s(const TMinus& t, uint32_t now_ms) {
    const uint32_t delta_ms = t.target_ms - now_ms;
    return static_cast<int32_t>(delta_ms) / 1000;
}

// NASA writes MET as day/hh:mm:ss (science.nasa.gov: "2/03:45:18 MET";
// JSC voice procedure: "3/02:19:00"). The Kennedy countdown clock is the
// same count with a minus before T-0 and a plus after. One field.
// show_days comes from the mission profile (HAB yes, rocket no).
inline int format_met_mark(char* out, size_t n, bool on, int32_t signed_s,
                           bool show_days) {
    if (!on) {
        return static_cast<int>(rc_snprintf(out, n, "MET -:--:--:--"));
    }
    const bool neg = signed_s < 0;
    uint32_t abs_s = neg ? static_cast<uint32_t>(-signed_s)
                         : static_cast<uint32_t>(signed_s);
    if (!show_days) {
        if (abs_s > (99U * kSecPerHour + 59U * kSecPerMin + 59U)) {
            abs_s = 99U * kSecPerHour + 59U * kSecPerMin + 59U;
        }
        return static_cast<int>(rc_snprintf(
            out, n, "MET %c%02lu:%02lu:%02lu", neg ? '-' : '+',
            (unsigned long)(abs_s / kSecPerHour),
            (unsigned long)((abs_s % kSecPerHour) / kSecPerMin),
            (unsigned long)(abs_s % kSecPerMin)));
    }
    const uint32_t cap = 999U * kSecPerDay + 23U * kSecPerHour +
                         59U * kSecPerMin + 59U;
    if (abs_s > cap) {
        abs_s = cap;
    }
    const uint32_t days = abs_s / kSecPerDay;
    abs_s %= kSecPerDay;
    return static_cast<int>(rc_snprintf(
        out, n, "MET %c%lu/%02lu:%02lu:%02lu", neg ? '-' : '+',
        (unsigned long)days,
        (unsigned long)(abs_s / kSecPerHour),
        (unsigned long)((abs_s % kSecPerHour) / kSecPerMin),
        (unsigned long)(abs_s % kSecPerMin)));
}

// Station receiver, not the vehicle "GPS (veh)" line.
// fix: 0 none, 2 = 2D, 3 = 3D. time_ok is NMEA UTC present.
inline int format_station_gps_row(char* out, size_t n, bool have_reads,
                                  uint8_t fix, uint8_t sats, bool time_ok,
                                  uint32_t up_s) {
    const char* kind = "no I2C";
    if (have_reads) {
        if (fix >= 3U) {
            kind = "3D";
        } else if (fix == 2U) {
            kind = "2D";
        } else if (time_ok) {
            kind = "time";
        } else {
            kind = "none";
        }
    }
    if (time_ok) {
        return static_cast<int>(rc_snprintf(
            out, n, "Stn GPS: %-6s %2u sat  Up: %lus", kind,
            static_cast<unsigned>(sats), (unsigned long)up_s));
    }
    return static_cast<int>(rc_snprintf(
        out, n, "Stn GPS: %-6s %2u sat  Up: %lus  no time", kind,
        static_cast<unsigned>(sats), (unsigned long)up_s));
}

// Clock line is its own row. State is the line under it. 80 columns.
inline int format_met_zulu(char* out, size_t n, bool met_on, int32_t met_signed_s,
                           bool show_days, bool zulu_ok, uint8_t zh, uint8_t zm,
                           uint8_t zs, bool local_ok, int16_t local_off_min) {
    char met[24];
    format_met_mark(met, sizeof(met), met_on, met_signed_s, show_days);
    if (!zulu_ok) {
        return static_cast<int>(rc_snprintf(
            out, n, "%s  Zulu: --:--:--Z  Local: --:--:--", met));
    }
    if (!local_ok) {
        return static_cast<int>(rc_snprintf(
            out, n, "%s  Zulu: %02u:%02u:%02uZ  Local: --:--:--", met,
            static_cast<unsigned>(zh), static_cast<unsigned>(zm),
            static_cast<unsigned>(zs)));
    }
    uint8_t lh = 0;
    uint8_t lm = 0;
    uint8_t ls = 0;
    utc_plus_minutes(zh, zm, zs, local_off_min, &lh, &lm, &ls);
    return static_cast<int>(rc_snprintf(
        out, n, "%s  Zulu: %02u:%02u:%02uZ  Local: %02u:%02u:%02u", met,
        static_cast<unsigned>(zh), static_cast<unsigned>(zm),
        static_cast<unsigned>(zs),
        static_cast<unsigned>(lh), static_cast<unsigned>(lm),
        static_cast<unsigned>(ls)));
}

// COP-P glance. V(R) is FARM; N(R)/V(S) are FOP (211.0 §7).
inline int format_air_row(char* out, size_t n, const char* dialect, bool on,
                          bool peer_plcw, bool rf_live, bool nav_sdu,
                          uint8_t nn_r, uint8_t v_s, uint8_t farm_vr,
                          uint8_t mac_mode, uint8_t mac_state) {
    if (!on) {
        return static_cast<int>(rc_snprintf(out, n, "Air: %s", dialect));
    }
    static const char kMacMode[] = "ILTA";
    const char mc = (mac_mode < 4U) ? kMacMode[mac_mode] : '?';
    // MacMode::active == 3. PLCW-heard can stick through connecting-T.
    const bool locked = peer_plcw && rf_live && (mac_mode == 3U);
    if (locked) {
        return static_cast<int>(rc_snprintf(
            out, n, "Air: %s  COP-P lock  N(R)=%u V(S)=%u V(R)=%u  MAC %c/s%u%s",
            dialect,
            static_cast<unsigned>(nn_r),
            static_cast<unsigned>(v_s),
            static_cast<unsigned>(farm_vr),
            mc, static_cast<unsigned>(mac_state),
            nav_sdu ? "  nav" : ""));
    }
    return static_cast<int>(rc_snprintf(
        out, n, "Air: %s  COP-P waiting  MAC %c/s%u",
        dialect, mc, static_cast<unsigned>(mac_state)));
}

// CRC-ok fraction. 211.2 Annex B Quality Indicator is uncorrectable-error,
// not 10 Hz miss slots (HD table 6-10 does not owe commanded nav_hz).
inline uint8_t crc_lq_pct(uint32_t ok, uint32_t fail) {
    const uint64_t tot = static_cast<uint64_t>(ok) + fail;
    if (tot == 0U) {
        return 0U;
    }
    return static_cast<uint8_t>((static_cast<uint64_t>(ok) * 100U) / tot);
}

// "RF Link: COP-P  LQ 100%  RX 2.3 Hz  [OK]"
inline int format_rf_link_row(char* out, size_t n, bool copp_lock, bool rf_live,
                              uint8_t lq, bool hz_ok, uint32_t hz10) {
    const char* kind = "NO RX";
    const char* tag = "[!!]";
    if (copp_lock && rf_live) {
        kind = "COP-P";
        tag = "[OK]";
    } else if (rf_live) {
        kind = "RF";
        tag = "[--]";
    }
    if (!hz_ok) {
        return static_cast<int>(rc_snprintf(
            out, n, "RF Link: %s  LQ %u%%  RX -- Hz  %s", kind,
            static_cast<unsigned>(lq), tag));
    }
    return static_cast<int>(rc_snprintf(
        out, n, "RF Link: %s  LQ %u%%  RX %lu.%lu Hz  %s", kind,
        static_cast<unsigned>(lq),
        (unsigned long)(hz10 / 10U), (unsigned long)(hz10 % 10U), tag));
}

}  // namespace dash
}  // namespace rc

#endif
