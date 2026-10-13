// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Live ANSI terminal dashboard — ground station telemetry display
// Renders a fixed-layout dashboard using ANSI escape codes.
// Technique: \033[H (cursor home) + \033[K (clear to EOL) per line.
// No \033[2J (clear screen) — that causes visible flicker.
// Entire frame built in a static buffer, written via tud_cdc_write.
// Target: 80 columns, ~19 rows. 115200 baud USB CDC.
// Colors: green/yellow/red/cyan/default. No bold, no background.

#include "rc_os_dashboard.h"
#include "rc_os_dashboard_format.h"
#include "rocketchip/rc_debug.h"
#include "rocketchip/rc_log.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/time_sync.h"
#include "diag/radio_rate_counters.h"
#include "safety/health_monitor.h"
#include "active_objects/ao_radio.h"
#include "active_objects/ao_telemetry.h"
#include "active_objects/station_bar_mode.h"
#include "starcom_adapt/sc_air.h"
#include "flight_director/flight_state.h"
#include <string.h>
#include <stdint.h>
#include <cstdint>
#include <math.h>

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#include "tusb.h"
#endif

// ============================================================================
// ANSI escape helpers
// ============================================================================

// Colors
static constexpr const char* kReset  = "\033[0m";
static constexpr const char* kRed    = "\033[31m";
static constexpr const char* kGreen  = "\033[32m";
static constexpr const char* kYellow = "\033[33m";
static constexpr const char* kCyan   = "\033[36m";
// Cursor home (top-left, no clear)
static constexpr const char* kHome   = "\033[H";
// Clear to end of line
static constexpr const char* kClrEol = "\033[K";

// ============================================================================
// Conversion constants
// ============================================================================

static constexpr float kMmToM   = 0.001F;
static constexpr float kCmsToMs = 0.01F;

// ============================================================================
// Station-side computed state (persists across renders)
// ============================================================================

static float    g_maxAltM  = 0.0F;
static float    g_maxVelMs = 0.0F;
static uint8_t  g_prevFlightState = 255;  // for transition detection

// ============================================================================
// Helpers
// ============================================================================

static const char* flight_phase_color(uint8_t state) {
    switch (static_cast<rc::FlightPhase>(state)) {
    case rc::FlightPhase::kIdle:          return kGreen;
    case rc::FlightPhase::kArmed:         return kYellow;
    case rc::FlightPhase::kBoost:         return kRed;
    case rc::FlightPhase::kCoast:         return kCyan;
    case rc::FlightPhase::kDrogueDescent: return kCyan;
    case rc::FlightPhase::kMainDescent:   return kCyan;
    case rc::FlightPhase::kLanded:        return kGreen;
    case rc::FlightPhase::kAbort:         return kRed;
    case rc::FlightPhase::kFault:         return kRed;
    default:                              return kReset;
    }
}

static const char* rssi_color(int16_t rssi) {
    if (rssi > -80)  return kGreen;
    if (rssi > -100) return kYellow;
    return kRed;
}

static const char* signal_age_color(uint32_t age_ms) {
    if (age_ms < 2000) return kGreen;
    if (age_ms < 5000) return kYellow;
    return kRed;
}

// Build RSSI bar: [########  ] with 10 positions
// Maps -120 to -40 dBm → 0 to 10
static int rssi_bar(char* buf, int max, int16_t rssi) {
    int level = (rssi + 120) / 8;  // -120→0, -40→10
    if (level < 0) level = 0;
    if (level > 10) level = 10;
    int n = 0;
    buf[n++] = '[';
    for (int i = 0; i < 10; i++) {
        buf[n++] = (i < level) ? '#' : ' ';
    }
    buf[n++] = ']';
    buf[n] = '\0';
    (void)max;
    return n;
}

// ============================================================================
// Main render
// ============================================================================

// Static frame buffer — avoids stack allocation (LL Entry 1).
static char g_frame[2048];

// Direct CDC write, bypasses stdio (LL Entry 39 pattern); drops on full.
static void send_frame(const char* buf, size_t len) {
#ifndef ROCKETCHIP_HOST_TEST
    if (!tud_cdc_connected()) { return; }
    if (tud_cdc_write_available() < len) { return; }
    tud_cdc_write(buf, static_cast<uint32_t>(len));
#else
    (void)buf; (void)len;
#endif
}

// Vehicle GPS civil time from the nav packet. Used when this board
// has no GPS time of its own. Not the mission clock.
struct VehClock {
    bool time_ok;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    bool date_ok;
    int year;
    uint8_t month;
    uint8_t day;
    bool pos_ok;
    int32_t lat_1e7;
    int32_t lon_1e7;
};

// Decoded display values — populated by decode_telem_fields()
struct DisplayFields {
    float alt_m, baro_m, vvel, speed, batt_v;
    double lat, lon;
    uint32_t lost, age_ms, met_s, met_ds, age_s, age_ds;
    uint8_t fix, sats;
    int8_t temp_c;
    bool eskf_ok;
    int rssi_pct;
    const char* phase;
    const char* phase_clr;
    const char* sig_clr;
    const char* rssi_clr;
    const char* fix_str;
    char bar[16];
    // Radio config row (station vs last vehicle echo).
    uint16_t stn_bw;  uint8_t stn_nav;  uint8_t stn_sf;  uint8_t stn_cr;
    uint16_t veh_bw;  uint8_t veh_nav;  uint8_t veh_sf;  uint8_t veh_cr;
    bool     veh_cfg_known;    // false -> dashboard shows "?" for vehicle
    bool     cfg_mismatch;     // true -> yellow-highlight the row
    bool     cfg_just_changed; // true for one frame after a transition
    bool     met_mission;
    bool     met_show_days;
    VehClock veh_clock;
};

// Decode telemetry + radio state into display-ready values
static void decode_telem_fields(const rc::TelemetryState& t,
                                 const RadioAoState* rs,
                                 uint32_t met_ms, uint16_t seq,
                                 DisplayFields& d) {
    d.alt_m   = static_cast<float>(t.alt_mm) * kMmToM;       // MSL
    d.baro_m  = static_cast<float>(t.baro_alt_mm) * kMmToM;  // AGL
    d.temp_c  = t.temperature_c;
    d.vvel    = static_cast<float>(t.baro_vvel_cms) * kCmsToMs;
    float vel_n = static_cast<float>(t.vel_n_cms) * kCmsToMs;
    float vel_e = static_cast<float>(t.vel_e_cms) * kCmsToMs;
    float vel_d = static_cast<float>(t.vel_d_cms) * kCmsToMs;
    d.speed   = sqrtf(vel_n * vel_n + vel_e * vel_e + vel_d * vel_d);
    d.fix     = (t.gps_fix_sats >> 4) & 0x0F;
    d.sats    = t.gps_fix_sats & 0x0F;
    d.eskf_ok = (rc::health_eskf(t.health) >= rc::kHealthDegraded);
    d.batt_v  = static_cast<float>(t.battery_mv) * 0.001F;
    d.lat     = static_cast<double>(t.lat_1e7) * 1e-7;
    d.lon     = static_cast<double>(t.lon_1e7) * 1e-7;

    // Running max AGL
    if (d.baro_m > g_maxAltM) g_maxAltM = d.baro_m;
    float abs_vel = fabsf(d.vvel);
    if (abs_vel > g_maxVelMs) g_maxVelMs = abs_vel;
    if (t.flight_state == 0 && g_prevFlightState != 0 && g_prevFlightState != 255) {
        g_maxAltM = 0.0F;
        g_maxVelMs = 0.0F;
    }
    g_prevFlightState = t.flight_state;

    // CRC fail count (211.2 Quality Indicator analog). Not RfManager
    // 10 Hz miss slots — HD table 6-10 does not owe commanded nav_hz.
    d.lost = rs->rx_crc_errors;
    (void)seq;

    // Signal age
#ifndef ROCKETCHIP_HOST_TEST
    d.age_ms = to_ms_since_boot(get_absolute_time()) - rs->last_rx_ms;
#else
    d.age_ms = 0;
#endif

    d.phase     = rc::flight_phase_name(static_cast<rc::FlightPhase>(t.flight_state));
    d.phase_clr = flight_phase_color(t.flight_state);
    d.sig_clr   = signal_age_color(d.age_ms);
    d.rssi_clr  = rssi_color(rs->last_rx_rssi);
    {
        const StarcomLinkStatus sc = AO_Telemetry_get_starcom_link();
        const bool copp_lock =
            sc.peer_plcw && (sc.mac_mode == 3U);
        const StationBarMode bar = station_bar_mode(
            copp_lock, rs->rx_count, d.age_ms);
        if (bar == StationBarMode::Waiting) {
            d.rssi_clr = kYellow;
        } else if (bar == StationBarMode::RfHeard) {
            d.rssi_clr = kCyan;
        }
    }

    d.met_s  = met_ms / 1000;
    d.met_ds = (met_ms % 1000) / 100;
    d.age_s  = d.age_ms / 1000;
    d.age_ds = (d.age_ms % 1000) / 100;

    rssi_bar(d.bar, sizeof(d.bar), rs->last_rx_rssi);
    d.rssi_pct = ((rs->last_rx_rssi + 120) * 100) / 80;
    if (d.rssi_pct < 0) d.rssi_pct = 0;
    if (d.rssi_pct > 100) d.rssi_pct = 100;

    d.fix_str = "None";
    if (d.fix == 3) d.fix_str = "3D";
    else if (d.fix == 2) d.fix_str = "2D";
}

// "Radio:" row (no ANSI colour; caller wraps). "?" until vehicle cfg known.
// Returns number of chars written, not including the trailing kClrEol.
static int format_radio_row(char* out, size_t n, const DisplayFields& d) {
    if (!d.veh_cfg_known) {
        return rc::rc_snprintf(out, n,
            "Radio: BW%u %uHz SF%u CR%u",
            static_cast<unsigned>(d.stn_bw),
            static_cast<unsigned>(d.stn_nav),
            static_cast<unsigned>(d.stn_sf),
            static_cast<unsigned>(d.stn_cr));
    }
    return rc::rc_snprintf(out, n,
        "Radio: BW%u %uHz SF%u CR%u  |  Vehicle: BW%u %uHz SF%u CR%u%s",
        static_cast<unsigned>(d.stn_bw),
        static_cast<unsigned>(d.stn_nav),
        static_cast<unsigned>(d.stn_sf),
        static_cast<unsigned>(d.stn_cr),
        static_cast<unsigned>(d.veh_bw),
        static_cast<unsigned>(d.veh_nav),
        static_cast<unsigned>(d.veh_sf),
        static_cast<unsigned>(d.veh_cr),
        d.cfg_just_changed ? " [CHANGED]" : "");
}

// RF Link glance — COP-P lock + CRC-ok LQ. Not RfManager 10 Hz TRACK.
static void format_rf_link_row(char* out, size_t n, const char*& colour,
                               const DisplayFields& d) {
    (void)d;
    const StarcomLinkStatus sc = AO_Telemetry_get_starcom_link();
    const RadioAoState* rs = AO_Radio_get_state();
    uint32_t gap = kStationBarHoldMs;
    uint32_t rxn = 0;
    if (rs != nullptr) {
        rxn = rs->rx_count;
#ifndef ROCKETCHIP_HOST_TEST
        gap = to_ms_since_boot(get_absolute_time()) - rs->last_rx_ms;
#endif
    }
    const bool rf_live = (rxn > 0U) && (gap < kStationBarHoldMs);
    const bool copp_lock = sc.peer_plcw && (sc.mac_mode == 3U) && rf_live;
#ifndef ROCKETCHIP_HOST_TEST
    const uint32_t window_ms = to_ms_since_boot(get_absolute_time());
#else
    const uint32_t window_ms = 0;
#endif
    const uint32_t ok = g_radioRateCounters.rx_crc_ok_n;
    const uint32_t fail = g_radioRateCounters.rx_crc_fail_n;
    const uint8_t lq = rc::dash::crc_lq_pct(ok, fail);
    const bool hz_ok = (window_ms >= 1000U);
    uint32_t hz10 = 0;
    if (hz_ok) {
        hz10 = static_cast<uint32_t>(
            (static_cast<uint64_t>(ok) * 10U) / (window_ms / 1000U));
    }
    if (copp_lock) {
        colour = kGreen;
    } else if (rf_live) {
        colour = kYellow;
    } else {
        colour = kRed;
    }
    rc::dash::format_rf_link_row(out, n, copp_lock, rf_live, lq, hz_ok, hz10);
}

// Short command name for the CMD row. Stable across retries.
static const char* cmd_id_short_name(uint16_t cmd_id) {
    switch (cmd_id) {
    case 400:   return "ARM/DISARM";
    case 185:   return "ABORT";
    case 31010: return "BEACON";
    case 31011: return "SET RF";
    case 31012: return "QUERY RF";
    default:    return "CMD";
    }
}

// CMD row. Three display states:
//  - pending: "CMD: ARM/DISARM  Try 3/8" (yellow while retrying, reset color on initial send)
//  - ACK:     "CMD: ARM/DISARM  ACK 180ms" (green, held ~3s after ACK)
//  - FAIL:    "CMD: ARM/DISARM  FAILED"    (red, held ~5s after exhaustion)
//  - idle:    "CMD: ---"                   (reset color)
static void format_cmd_status_row(char* out, size_t n, const char*& colour) {
    constexpr uint32_t kAckHoldMs  = 3000U;
    constexpr uint32_t kFailHoldMs = 5000U;

    PendingCmdStatus st{};
    AO_Telemetry_get_pending_cmd_status(&st);

#ifndef ROCKETCHIP_HOST_TEST
    const uint32_t now = to_ms_since_boot(get_absolute_time());
#else
    const uint32_t now = 0;
#endif

    if (st.pending) {
        colour = (st.retries_used == 0) ? kReset : kYellow;
        rc::rc_snprintf(out, n, "CMD: %-10s Try %u/%u",
                        cmd_id_short_name(st.cmd_id),
                        static_cast<unsigned>(st.retries_used + 1),
                        static_cast<unsigned>(st.max_retries + 1));
        return;
    }

    if (st.last_result_valid) {
        const uint32_t age = now - st.last_result_ms;
        if (st.last_result_ok && age < kAckHoldMs) {
            colour = kGreen;
            rc::rc_snprintf(out, n, "CMD: %-10s ACK %ums",
                            cmd_id_short_name(st.last_cmd_id),
                            static_cast<unsigned>(st.last_rtt_ms));
            return;
        }
        if (!st.last_result_ok && age < kFailHoldMs) {
            colour = kRed;
            rc::rc_snprintf(out, n, "CMD: %-10s FAILED",
                            cmd_id_short_name(st.last_cmd_id));
            return;
        }
    }

    colour = kReset;
    rc::rc_snprintf(out, n, "CMD: ---");
}

static rc::dash::ZuluRun g_zuluRun = {};
static rc::dash::TMinus g_tminus = {};
static rc::MetRun g_metRun = {};
static bool g_latencyValid = false;
static int32_t g_latencyS = 0;

static uint32_t pad_mono_ms() {
#ifndef ROCKETCHIP_HOST_TEST
    return to_ms_since_boot(get_absolute_time());
#else
    return 0;
#endif
}

// MET is not consulted. Station GPS time wins. Otherwise the vehicle's.
static void format_clock_row(char* out, size_t n, bool met_mission,
                             uint32_t veh_met_ms, bool show_days,
                             const VehClock& veh) {
    rc::met_run_observe(&g_metRun, met_mission, veh_met_ms, pad_mono_ms());
    const uint32_t shown_met = rc::met_run_at(g_metRun, pad_mono_ms());
    shared_sensor_data_t snap = {};
    const bool got = seqlock_read(&g_sensorSeqlock, &snap);
    const bool local_gps = got && snap.gps_time_valid;
    const rc::dash::TimePick time_pick = rc::dash::choose_time_source(
        local_gps, veh.time_ok, false, false);
    const bool use_local = time_pick == rc::dash::TimePick::kLocalGps;
    const bool use_peer = time_pick == rc::dash::TimePick::kPeerGps;
    const uint8_t src_h = use_local ? snap.gps_hour : veh.hour;
    const uint8_t src_m = use_local ? snap.gps_minute : veh.minute;
    const uint8_t src_s = use_local ? snap.gps_second : veh.second;
    bool zulu_ok = false;
    uint8_t zh = 0;
    uint8_t zm = 0;
    uint8_t zs = 0;
    rc::dash::zulu_run_apply(&g_zuluRun, use_local || use_peer, src_h, src_m,
                             src_s, pad_mono_ms(), &zulu_ok, &zh, &zm, &zs);

    bool local_ok = false;
    int16_t off = 0;
    const bool stn_pos = got && (snap.gps_valid || snap.gps_lat_1e7 != 0 ||
                                 snap.gps_lon_1e7 != 0);
    const bool stn_date = got && snap.gps_date_valid;
    int32_t lat = 0;
    int32_t lon = 0;
    int year = 0;
    int month = 0;
    int day = 0;
    bool have_pos = false;
    bool have_date = false;
    if (stn_pos) {
        lat = snap.gps_lat_1e7;
        lon = snap.gps_lon_1e7;
        have_pos = true;
    } else if (veh.pos_ok) {
        lat = veh.lat_1e7;
        lon = veh.lon_1e7;
        have_pos = true;
    }
    if (stn_date) {
        year = snap.gps_year;
        month = snap.gps_month;
        day = snap.gps_day;
        have_date = true;
    } else if (veh.date_ok) {
        year = veh.year;
        month = veh.month;
        day = veh.day;
        have_date = true;
    }
    if (zulu_ok && have_pos && have_date) {
        off = rc::dash::tz_offset_min(lat, lon, year, month, day, zh, zm, zs);
        local_ok = true;
    }
    bool met_on = false;
    int32_t met_signed_s = 0;
    if (g_tminus.on) {
        // Remaining time until the mark is a negative MET.
        const int32_t left_s = rc::dash::tminus_remaining_s(g_tminus, pad_mono_ms());
        met_on = true;
        met_signed_s = -left_s;
    } else if (g_metRun.on) {
        met_on = true;
        met_signed_s = static_cast<int32_t>(shown_met / 1000U);
    }
    rc::dash::format_met_zulu(out, n, met_on, met_signed_s, show_days,
                              zulu_ok, zh, zm, zs, local_ok, off);
}

static bool current_zulu_sod(uint32_t* sod) {
    bool ok = false;
    uint8_t h = 0;
    uint8_t m = 0;
    uint8_t s = 0;
    rc::dash::zulu_run_apply(&g_zuluRun, false, 0, 0, 0, pad_mono_ms(),
                             &ok, &h, &m, &s);
    if (!ok || sod == nullptr) {
        return false;
    }
    *sod = rc::dash::hms_to_sod(h, m, s);
    return true;
}

static void fill_station_gps(char* out, size_t n) {
    shared_sensor_data_t snap = {};
    const bool got = seqlock_read(&g_sensorSeqlock, &snap);
    rc::dash::format_station_gps_row(
        out, n, got && snap.gps_read_count > 0U,
        got ? snap.gps_fix_type : 0U, got ? snap.gps_satellites : 0U,
        got && snap.gps_time_valid, pad_mono_ms() / 1000U);
}

void ansi_dashboard_set_tminus_minutes(uint16_t minutes, uint32_t now_ms) {
    rc::dash::tminus_arm(&g_tminus, now_ms,
                         static_cast<uint32_t>(minutes) * 60U);
}

bool ansi_dashboard_set_tminus_zulu(uint8_t h, uint8_t m, uint8_t s,
                                    uint32_t now_ms) {
    uint32_t now_sod = 0;
    if (!current_zulu_sod(&now_sod)) {
        return false;
    }
    const uint32_t delta = rc::dash::tminus_zulu_delta_s(
        now_sod, rc::dash::hms_to_sod(h, m, s));
    rc::dash::tminus_arm(&g_tminus, now_ms, delta);
    return true;
}

void ansi_dashboard_clear_tminus() {
    g_tminus.on = false;
}

bool ansi_dashboard_retarget_tminus_sod(uint32_t accepted_sod, uint32_t now_ms) {
    uint32_t now_sod = 0;
    if (!current_zulu_sod(&now_sod)) {
        return false;
    }
    const uint32_t delta = rc::dash::tminus_zulu_delta_s(now_sod, accepted_sod);
    rc::dash::tminus_arm(&g_tminus, now_ms, delta);
    return true;
}

void ansi_dashboard_note_latency(bool valid, int32_t latency_s) {
    g_latencyValid = valid;
    g_latencyS = latency_s;
}

bool ansi_dashboard_latency(int32_t* latency_s) {
    if (!g_latencyValid || latency_s == nullptr) {
        return false;
    }
    *latency_s = g_latencyS;
    return true;
}

static void format_starcom_row(char* out, int n, const char*& colour) {
    const StarcomLinkStatus sc = AO_Telemetry_get_starcom_link();
    const RadioAoState* rs = AO_Radio_get_state();
    uint32_t gap = kStationBarHoldMs;
    uint32_t rxn = 0;
    if (rs != nullptr) {
        rxn = rs->rx_count;
#ifndef ROCKETCHIP_HOST_TEST
        gap = to_ms_since_boot(get_absolute_time()) - rs->last_rx_ms;
#endif
    }
    const bool rf_live = (rxn > 0U) && (gap < kStationBarHoldMs);
    if (sc.on && sc.peer_plcw && rf_live && (sc.mac_mode == 3U)) {
        colour = kGreen;
    } else if (sc.on) {
        colour = kYellow;
    } else {
        colour = kReset;
    }
    rc::dash::format_air_row(out, static_cast<size_t>(n), rc::kAirDialect, sc.on,
                             sc.peer_plcw, rf_live, sc.nav_sdu,
                             sc.nn_r, sc.v_s, sc.farm_vr,
                             sc.mac_mode, sc.mac_state);
}

static int build_frame(const DisplayFields& d, const RadioAoState* rs,
                        uint16_t seq, uint32_t met_ms) {
    // Radio row colour: cyan one frame after a change, yellow if mismatch.
    char radio_row[96];
    format_radio_row(radio_row, sizeof(radio_row), d);
    const char* radio_clr = kReset;
    if (d.cfg_just_changed)      { radio_clr = kCyan;   }
    else if (d.cfg_mismatch)     { radio_clr = kYellow; }

    // RF Link row (pre-arm parity).
    char rflink_row[96];
    const char* rflink_clr = kReset;
    format_rf_link_row(rflink_row, sizeof(rflink_row), rflink_clr, d);

    // CMD row — pending/ACK/FAIL with auto-clear.
    char cmd_row[80];
    const char* cmd_clr = kReset;
    format_cmd_status_row(cmd_row, sizeof(cmd_row), cmd_clr);

    char clock_row[80];
    format_clock_row(clock_row, sizeof(clock_row), d.met_mission, met_ms,
                     d.met_show_days, d.veh_clock);
    char stn_row[72];
    fill_station_gps(stn_row, sizeof(stn_row));

    rc::strbuf sb;
    rc::strbuf_init(&sb, g_frame, sizeof(g_frame));

    rc::strbuf_printf(&sb,
        "%s"
        "=== RocketChip Ground Station ===%s\n"
        "%s%s\n"
        "State: %s%-14s%s%s\n"
        "%s%s\n"
        "------------------------------------------------------------------------%s\n",
        kHome,
        kClrEol,
        clock_row, kClrEol,
        d.phase_clr, d.phase, kReset, kClrEol,
        stn_row, kClrEol,
        kClrEol);

    rc::strbuf_printf(&sb,
        "Alt:  %7.1f m        Max: %.1f m%s\n"
        "Vvel: %+6.1f m/s      Spd: %.1f m/s%s\n"
        "Baro: %7.1f m         GPS (veh): %s (%usat)%s\n"
        "------------------------------------------------------------------------%s\n",
        static_cast<double>(d.alt_m),
        static_cast<double>(g_maxAltM), kClrEol,
        static_cast<double>(d.vvel), static_cast<double>(d.speed), kClrEol,
        static_cast<double>(d.baro_m), d.fix_str,
        static_cast<unsigned>(d.sats), kClrEol,
        kClrEol);

    rc::strbuf_printf(&sb,
        "RSSI: %s%d dBm%s  SNR: %d dB  %s%s%s  %d%%%s\n"
        "Pkts: %-6lu CRC: %-4lu  %sLast: %lu.%lus%s%s\n",
        d.rssi_clr, static_cast<int>(rs->last_rx_rssi), kReset,
        static_cast<int>(rs->last_rx_snr),
        d.rssi_clr, d.bar, kReset, d.rssi_pct, kClrEol,
        (unsigned long)rs->rx_count, (unsigned long)d.lost,
        d.sig_clr, (unsigned long)d.age_s, (unsigned long)d.age_ds,
        kReset, kClrEol);

    rc::strbuf_printf(&sb, "%s%s%s%s\n",
        radio_clr, radio_row, kReset, kClrEol);
    rc::strbuf_printf(&sb, "%s%s%s%s\n",
        rflink_clr, rflink_row, kReset, kClrEol);
    rc::strbuf_printf(&sb, "%s%s%s%s\n",
        cmd_clr, cmd_row, kReset, kClrEol);

    char sc_row[96];
    const char* sc_clr = kReset;
    format_starcom_row(sc_row, static_cast<int>(sizeof(sc_row)), sc_clr);
    rc::strbuf_printf(&sb, "%s%s%s%s\n", sc_clr, sc_row, kReset, kClrEol);

    char up_row[24];
    if (d.met_mission) {
        rc::rc_snprintf(up_row, sizeof(up_row), "Veh up: --");
    } else {
        rc::rc_snprintf(up_row, sizeof(up_row), "Veh up: %lus",
                        (unsigned long)(met_ms / 1000U));
    }
    rc::strbuf_printf(&sb,
        "------------------------------------------------------------------------%s\n"
        "Batt: %.2fV  Temp: %dC  ESKF: %s%s%s  Seq: %u%s\n"
        "Lat: %.7f  Lon: %.7f%s\n"
        "%s%s\n"
        "'a' ARM  'D' DISARM  'x' menu%s\n",
        kClrEol,
        static_cast<double>(d.batt_v), static_cast<int>(d.temp_c),
        d.eskf_ok ? kGreen : kRed, d.eskf_ok ? "OK" : "FAIL", kReset,
        static_cast<unsigned>(seq), kClrEol,
        d.lat, d.lon, kClrEol,
        up_row, kClrEol,
        kClrEol);

    return static_cast<int>(rc::strbuf_len(&sb));
}

// Pause render so ARM confirm text is not overwritten.
static bool g_dashboardPaused = false;

void ansi_dashboard_pause()  { g_dashboardPaused = true; }
void ansi_dashboard_resume() { g_dashboardPaused = false; }

void ansi_dashboard_render(const rc::TelemetryState& t,
                            const RadioAoState* rs,
                            uint32_t met_ms, uint16_t seq, bool valid,
                            const RxTelemSnapshot* rx) {
    if (g_dashboardPaused) return;

    if (!valid) {
        ansi_dashboard_render_waiting(rs);
        return;
    }

    DisplayFields d = {};
    decode_telem_fields(t, rs, met_ms, seq, d);

    // Populate radio config row.
    d.stn_bw  = rs->runtime_config.bandwidth_khz;
    d.stn_nav = rs->runtime_config.nav_rate_hz;
    d.stn_sf  = rs->runtime_config.spreading_factor;
    d.stn_cr  = rs->runtime_config.coding_rate;
    d.veh_cfg_known = (rx != nullptr && rx->echo_bw_khz != 0);
    if (d.veh_cfg_known) {
        d.veh_bw  = rx->echo_bw_khz;
        d.veh_nav = rx->echo_nav_hz;
        d.veh_sf  = rx->echo_sf;
        d.veh_cr  = rx->echo_cr;
        d.cfg_mismatch =
            (d.stn_bw != d.veh_bw) || (d.stn_nav != d.veh_nav) ||
            (d.stn_sf != d.veh_sf) || (d.stn_cr != d.veh_cr);
        d.cfg_just_changed = rx->echo_just_changed;
    }

    d.met_mission = (t.flags & rc::kFlagsMetMission) != 0;
    d.met_show_days = (t.flags & rc::kFlagsMetDays) != 0;
    d.veh_clock.time_ok = (t.flags & rc::kFlagsUtcValid) != 0;
    d.veh_clock.hour = t.utc_hour;
    d.veh_clock.minute = t.utc_minute;
    d.veh_clock.second = t.utc_second;
    d.veh_clock.date_ok = t.utc_month >= 1 && t.utc_month <= 12;
    d.veh_clock.year = 2000 + t.utc_year;
    d.veh_clock.month = t.utc_month;
    d.veh_clock.day = t.utc_day;
    d.veh_clock.pos_ok = ((t.gps_fix_sats >> 4) & 0x0F) >= 2;
    d.veh_clock.lat_1e7 = t.lat_1e7;
    d.veh_clock.lon_1e7 = t.lon_1e7;
    int pos = build_frame(d, rs, seq, met_ms);
    send_frame(g_frame, static_cast<size_t>(pos));
}

void ansi_dashboard_render_waiting(const RadioAoState* rs) {
    const char* sig_msg = "\033[33mWaiting for vehicle packets...\033[0m";

    rc::strbuf sb;
    rc::strbuf_init(&sb, g_frame, sizeof(g_frame));
    char sc_row[96];
    const char* sc_clr = kReset;
    format_starcom_row(sc_row, static_cast<int>(sizeof(sc_row)), sc_clr);
    char clock_row[80];
    format_clock_row(clock_row, sizeof(clock_row), false, 0, false, VehClock{});
    char stn_row[72];
    fill_station_gps(stn_row, sizeof(stn_row));
    rc::strbuf_printf(&sb,
        "%s"
        "=== RocketChip Ground Station ===%s\n"
        "%s%s\n"
        "%s%s\n"
        "%s%s\n"
        "%s%s%s%s\n"
        "------------------------------------------------------------------------%s\n"
        "RX: %lu pkts  CRC err: %lu%s\n"
        "%s\n"
        "'a' ARM  'D' DISARM  'x' menu%s\n",
        kHome,
        kClrEol,
        sig_msg, kClrEol,
        clock_row, kClrEol,
        stn_row, kClrEol,
        sc_clr, sc_row, kReset, kClrEol,
        kClrEol,
        (unsigned long)rs->rx_count, (unsigned long)rs->rx_crc_errors, kClrEol,
        kClrEol,
        kClrEol);
    send_frame(g_frame, rc::strbuf_len(&sb));
}
