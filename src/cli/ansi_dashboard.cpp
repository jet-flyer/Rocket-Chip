// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file ansi_dashboard.cpp
 * @brief Live ANSI terminal dashboard — ground station telemetry display
 *
 * Renders a fixed-layout dashboard using ANSI escape codes.
 * Technique: \033[H (cursor home) + \033[K (clear to EOL) per line.
 * No \033[2J (clear screen) — that causes visible flicker.
 * Entire frame built in a static buffer, written in one fwrite() call.
 *
 * Target: 80 columns, ~18 rows. 115200 baud USB CDC.
 * Colors: green/yellow/red/cyan/default. No bold, no background.
 */

#include "ansi_dashboard.h"
#include "rocketchip/config.h"
#include "active_objects/ao_radio.h"
#include "flight_director/flight_state.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
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

static constexpr float kMmToM   = 0.001f;
static constexpr float kCmsToMs = 0.01f;

// ============================================================================
// Station-side computed state (persists across renders)
// ============================================================================

static float    s_max_alt_m  = 0.0f;
static float    s_max_vel_ms = 0.0f;
static uint32_t s_first_seq  = 0;
static bool     s_first_seq_valid = false;
static uint8_t  s_prev_flight_state = 255;  // for transition detection

// ============================================================================
// Helpers
// ============================================================================

static const char* flight_phase_color(uint8_t state) {
    switch (state) {
    case 0: return kGreen;   // IDLE
    case 1: return kYellow;  // ARMED
    case 2: return kRed;     // BOOST
    case 3: return kCyan;    // COAST
    case 4: return kCyan;    // DESCENT
    case 5: return kGreen;   // LANDED
    case 6: return kRed;     // ERROR
    default: return kReset;
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

// Static frame buffer — avoids stack allocation (LL Entry 1)
// and enables single fwrite() to prevent CDC tearing.
static char s_frame[2048];

// Decoded display values — populated by decode_telem_fields()
struct DisplayFields {
    float alt_m, vvel, speed, batt_v;
    double lat, lon;
    uint32_t lost, age_ms, met_s, met_ds, age_s, age_ds;
    uint8_t fix, sats;
    bool eskf_ok;
    int rssi_pct;
    const char* phase;
    const char* phase_clr;
    const char* sig_clr;
    const char* rssi_clr;
    const char* fix_str;
    char bar[16];
};

// Decode telemetry + radio state into display-ready values
static void decode_telem_fields(const rc::TelemetryState& t,
                                 const RadioAoState* rs,
                                 uint32_t met_ms, uint16_t seq,
                                 DisplayFields& d) {
    d.alt_m   = static_cast<float>(t.baro_alt_mm) * kMmToM;
    d.vvel    = static_cast<float>(t.baro_vvel_cms) * kCmsToMs;
    float vel_n = static_cast<float>(t.vel_n_cms) * kCmsToMs;
    float vel_e = static_cast<float>(t.vel_e_cms) * kCmsToMs;
    float vel_d = static_cast<float>(t.vel_d_cms) * kCmsToMs;
    d.speed   = sqrtf(vel_n * vel_n + vel_e * vel_e + vel_d * vel_d);
    d.fix     = (t.gps_fix_sats >> 4) & 0x0F;
    d.sats    = t.gps_fix_sats & 0x0F;
    d.eskf_ok = (t.health & rc::kHealthEskfHealthy) != 0;
    d.batt_v  = static_cast<float>(t.battery_mv) * 0.001f;
    d.lat     = static_cast<double>(t.lat_1e7) * 1e-7;
    d.lon     = static_cast<double>(t.lon_1e7) * 1e-7;

    // Running max
    if (d.alt_m > s_max_alt_m) s_max_alt_m = d.alt_m;
    float abs_vel = fabsf(d.vvel);
    if (abs_vel > s_max_vel_ms) s_max_vel_ms = abs_vel;
    if (t.flight_state == 0 && s_prev_flight_state != 0 && s_prev_flight_state != 255) {
        s_max_alt_m = 0.0f;
        s_max_vel_ms = 0.0f;
    }
    s_prev_flight_state = t.flight_state;

    // Packet loss
    if (!s_first_seq_valid && rs->rx_count > 0) {
        s_first_seq = seq;
        s_first_seq_valid = true;
    }
    d.lost = 0;
    if (s_first_seq_valid && rs->rx_count > 0) {
        uint32_t expected = (seq - s_first_seq + 1) & 0x3FFF;
        if (expected > rs->rx_count) d.lost = expected - rs->rx_count;
    }

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

// Build ANSI frame string into s_frame, return length
static int build_frame(const DisplayFields& d, const RadioAoState* rs,
                        uint16_t seq) {
    return snprintf(s_frame, sizeof(s_frame),
        "%s"
        "=== RocketChip Ground Station ===%s\n"
        "State: %s%-8s%s     MET: %lu:%02lu.%lu%s\n"
        "-------------------------------------------%s\n"
        "Alt:  %7.1f m            Max: %.1f m%s\n"
        "Vvel: %+6.1f m/s          Spd: %.1f m/s%s\n"
        "Baro: %7.1f m            GPS: %s (%usat)%s\n"
        "-------------------------------------------%s\n"
        "RSSI: %s%d dBm%s  SNR: %d dB  %s%s%s  %d%%%s\n"
        "Pkts: %-6lu Lost: %-4lu  %sLast: %lu.%lus%s%s\n"
        "-------------------------------------------%s\n"
        "Batt: %.2fV  Temp: %dC  ESKF: %s%s%s  Seq: %u%s\n"
        "Lat: %.7f  Lon: %.7f%s\n"
        "%s\n"
        "'m' mode cycle  'x' menu%s\n",
        kHome,
        kClrEol,
        d.phase_clr, d.phase, kReset,
        (unsigned long)(d.met_s / 60), (unsigned long)(d.met_s % 60),
        (unsigned long)d.met_ds, kClrEol,
        kClrEol,
        static_cast<double>(d.alt_m),
        static_cast<double>(s_max_alt_m), kClrEol,
        static_cast<double>(d.vvel), static_cast<double>(d.speed), kClrEol,
        static_cast<double>(d.alt_m), d.fix_str,
        static_cast<unsigned>(d.sats), kClrEol,
        kClrEol,
        d.rssi_clr, static_cast<int>(rs->last_rx_rssi), kReset,
        static_cast<int>(rs->last_rx_snr),
        d.rssi_clr, d.bar, kReset, d.rssi_pct, kClrEol,
        (unsigned long)rs->rx_count, (unsigned long)d.lost,
        d.sig_clr, (unsigned long)d.age_s, (unsigned long)d.age_ds,
        kReset, kClrEol,
        kClrEol,
        static_cast<double>(d.batt_v), static_cast<int>(0),
        d.eskf_ok ? kGreen : kRed, d.eskf_ok ? "OK" : "FAIL", kReset,
        static_cast<unsigned>(seq), kClrEol,
        d.lat, d.lon, kClrEol,
        kClrEol,
        kClrEol);
}

void ansi_dashboard_render(const rc::TelemetryState& t,
                            const RadioAoState* rs,
                            uint32_t met_ms, uint16_t seq, bool valid) {
    if (!valid) {
        ansi_dashboard_render_waiting(rs);
        return;
    }

    DisplayFields d = {};
    decode_telem_fields(t, rs, met_ms, seq, d);
    int pos = build_frame(d, rs, seq);

    fwrite(s_frame, 1, static_cast<size_t>(pos), stdout);
    fflush(stdout);
}

void ansi_dashboard_render_waiting(const RadioAoState* rs) {
    int pos = 0;
    const char* sig_msg = "\033[33mWaiting for vehicle packets...\033[0m";

#ifndef ROCKETCHIP_HOST_TEST
    uint32_t uptime_s = to_ms_since_boot(get_absolute_time()) / 1000;
#else
    uint32_t uptime_s = 0;
#endif

    pos += snprintf(s_frame + pos, sizeof(s_frame) - static_cast<size_t>(pos),
        "%s"
        "=== RocketChip Ground Station ===%s\n"
        "%s%s\n"
        "-------------------------------------------%s\n"
        "RX: %lu pkts  CRC err: %lu%s\n"
        "Uptime: %lus%s\n"
        "%s\n"
        "'m' mode cycle  'x' menu%s\n",
        kHome,
        kClrEol,
        sig_msg, kClrEol,
        kClrEol,
        (unsigned long)rs->rx_count, (unsigned long)rs->rx_crc_errors, kClrEol,
        (unsigned long)uptime_s, kClrEol,
        kClrEol,
        kClrEol);

    fwrite(s_frame, 1, static_cast<size_t>(pos), stdout);
    fflush(stdout);
}
