// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Pad ANSI row formatters. Header-only for host tests.
// RSSI is SX1276 RegPktRssiValue (dBm), not 211.2 Annex B Quality Indicator.
// Zulu is NMEA UTC from the station GPS; MET is vehicle nav SDU met_ms.
#ifndef ROCKETCHIP_RC_OS_DASHBOARD_FORMAT_H
#define ROCKETCHIP_RC_OS_DASHBOARD_FORMAT_H

#include "rocketchip/rc_log.h"
#include <stddef.h>
#include <stdint.h>

namespace rc {
namespace dash {

// Minutes east of UTC. 0 = omit Local (Zulu only). MDT = -360.
static constexpr int16_t kPadLocalUtcOffsetMin = 0;

static constexpr uint32_t kSecPerDay = 86400U;
static constexpr uint32_t kSecPerHour = 3600U;
static constexpr uint32_t kSecPerMin = 60U;

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

// One header clock strip. Local stays on the line; offset 0 means Local == Zulu.
// Longest pad case (MET 999:59.9 + both clocks) is 48 chars; State prefix is 23.
inline int format_met_zulu(char* out, size_t n, uint32_t met_ms,
                           bool zulu_ok, uint8_t zh, uint8_t zm, uint8_t zs,
                           int16_t local_off_min) {
    const uint32_t met_s = met_ms / 1000U;
    const uint32_t met_ds = (met_ms % 1000U) / 100U;
    const uint32_t met_min = met_s / 60U;
    const uint32_t met_sec = met_s % 60U;
    if (!zulu_ok) {
        return static_cast<int>(rc_snprintf(
            out, n, "MET: %lu:%02lu.%lu  Zulu: --:--:--Z  Local: --:--:--",
            (unsigned long)met_min, (unsigned long)met_sec,
            (unsigned long)met_ds));
    }
    uint8_t lh = 0;
    uint8_t lm = 0;
    uint8_t ls = 0;
    utc_plus_minutes(zh, zm, zs, local_off_min, &lh, &lm, &ls);
    return static_cast<int>(rc_snprintf(
        out, n, "MET: %lu:%02lu.%lu  Zulu: %02u:%02u:%02uZ  Local: %02u:%02u:%02u",
        (unsigned long)met_min, (unsigned long)met_sec,
        (unsigned long)met_ds,
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
