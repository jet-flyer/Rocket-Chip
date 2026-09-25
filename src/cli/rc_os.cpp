// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// USB / lockout / ARM-confirm modes + engine kKey dispatch.
// Menu meaning lives in cli_menus.h (data). AO_RCOS ticks this.

#include "rc_os.h"
#include "rc_os_dashboard.h"
#include "cli/cli_engine.h"
#include "cli/cli_menus.h"
#include "cli/rc_os_dashboard_format.h"
#include "rocketchip/time_sync.h"
#include "active_objects/ao_telemetry.h"
#include "cli/cli_actions.h"
#include "cli/rc_os_commands.h"
#include "cli/rc_os_debug.h"
#include "ao_rcos.h"
#include "rocketchip/job.h"
#include "rocketchip/rc_log.h"
#include "rocketchip/station_output_mode.h"
#include "station/gcs_mavlink.h"
#include "calibration/calibration_manager.h"
#include "ao_flight_director.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#ifndef ROCKETCHIP_HOST_TEST
#include "tusb.h"
#endif

constexpr uint8_t  kUsbSettlePolls      = 5;
constexpr uint32_t kArmConfirmTimeoutMs = 5000;
constexpr uint8_t  kMavlinkV2Stx        = 0xFDU;
constexpr uint8_t  kMavlinkV1Stx        = 0xFEU;
constexpr uint16_t kMavCmdArmDisarm     = 400;

static bool is_mavlink_stx(uint8_t b) {
    return (b == kMavlinkV2Stx) || (b == kMavlinkV1Stx);
}

static rc::cli::Engine g_eng{};
static bool g_wasConnected  = false;
static bool g_bannerPrinted = false;
static uint8_t g_settleCount = 0;
static bool g_mavlinkDetected = false;
static uint32_t g_usbDisconnectSinceMs = 0;

static bool g_armConfirmActive = false;
static char g_armBuf[4] = {};
static uint8_t g_armBufPos = 0;
static uint32_t g_armStartMs = 0;

enum class TMinusAsk : uint8_t { kNone = 0, kMinutes, kZulu };
static TMinusAsk g_tminusAsk = TMinusAsk::kNone;
static char g_tminusBuf[12] = {};
static uint8_t g_tminusPos = 0;

#if defined(ROCKETCHIP_DEV_MODE)
static bool g_devModeRuntime = false;
#endif

bool rc_os_imu_available = false;
bool rc_os_baro_available = false;

#if defined(ROCKETCHIP_JOB_STATION)
static const rc::cli::Item* items() { return rc::cli::kStationItems; }
static constexpr size_t item_count() { return rc::cli::kStationItemCount; }
#else
static const rc::cli::Item* items() { return rc::cli::kVehicleItems; }
static constexpr size_t item_count() { return rc::cli::kVehicleItemCount; }
#endif

static void show_help() {
    rc::cli::print_help(items(), item_count(), rc::cli::top(g_eng));
}

// Cable-up (tud_ready / CONNECTION_WITHOUT_DTR) is not a terminal session.
// CLI banner follows DTR so a closed COM port can reconnect and reprint
// flight-<sha> for host classify.
static bool cli_terminal_connected() {
#ifdef ROCKETCHIP_HOST_TEST
    return stdio_usb_connected();
#else
    return tud_cdc_connected();
#endif
}

static void show_prompt() {
    rc::cli::print_prompt(rc::cli::top(g_eng));
}

void rc_os_init() {
    rc::cli::init(g_eng);
    g_wasConnected  = false;
    g_bannerPrinted = false;
    g_usbDisconnectSinceMs = 0;
#if defined(ROCKETCHIP_DEV_MODE)
    g_devModeRuntime = false;
#endif
}

bool rc_os_dev_mode_runtime() {
#if defined(ROCKETCHIP_DEV_MODE)
    return g_devModeRuntime;
#else
    return false;
#endif
}

void rc_os_dev_mode_toggle() {
#if defined(ROCKETCHIP_DEV_MODE)
    if (g_devModeRuntime) {
        g_devModeRuntime = false;
        rc::rc_log("DEV_MODE off\n");
        return;
    }
    if (!stdio_usb_connected() || !AO_FlightDirector_is_ground_state()) {
        rc::rc_log("DEV_MODE refused (need USB + idle)\n");
        return;
    }
    g_devModeRuntime = true;
    rc::rc_log("DEV_MODE on\n");
#endif
}

void rc_os_reset_to_main() {
    rc::cli::init(g_eng);
}

void rc_os_print_help() {
    show_help();
    show_prompt();
}

bool rc_os_arm_confirm_active() {
    return g_armConfirmActive;
}

static void tminus_prompt() {
    if (g_tminusAsk == TMinusAsk::kMinutes) {
        rc::rc_log("T- minutes (1-1440): ");
        return;
    }
    rc::rc_log("T- Zulu (HHMMSS or HH:MM:SS): ");
}

static void commit_tminus_line() {
    g_tminusBuf[g_tminusPos] = '\0';
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (g_tminusAsk == TMinusAsk::kMinutes) {
        uint16_t minutes = 0;
        if (!rc::dash::parse_tminus_minutes(g_tminusBuf, &minutes)) {
            rc::rc_log("\nT- needs 1-1440 minutes\n");
            return;
        }
        ansi_dashboard_set_tminus_minutes(minutes, now_ms);
        AO_Telemetry_send_tracked_command(rc::kCmdTMinus, 0.0F,
                                          static_cast<float>(minutes),
                                          0.0F, 0.0F, 0.0F);
        rc::rc_log("\nT- %u min sent\n", static_cast<unsigned>(minutes));
        return;
    }
    uint8_t h = 0;
    uint8_t m = 0;
    uint8_t s = 0;
    if (!rc::dash::parse_tminus_zulu(g_tminusBuf, &h, &m, &s)) {
        rc::rc_log("\nT- needs HHMMSS or HH:MM:SS\n");
        return;
    }
    if (!ansi_dashboard_set_tminus_zulu(h, m, s, now_ms)) {
        rc::rc_log("\nT- needs GPS time\n");
        return;
    }
    const uint32_t sod = (static_cast<uint32_t>(h) * 3600U) +
                         (static_cast<uint32_t>(m) * 60U) +
                         static_cast<uint32_t>(s);
    AO_Telemetry_send_tracked_command(rc::kCmdTMinus, 1.0F,
                                      static_cast<float>(sod),
                                      0.0F, 0.0F, 0.0F);
    rc::rc_log("\nT- at %02u:%02u:%02uZ sent\n",
               static_cast<unsigned>(h), static_cast<unsigned>(m),
               static_cast<unsigned>(s));
}

void rc_os_start_tminus_minutes() {
    g_tminusAsk = TMinusAsk::kMinutes;
    g_tminusPos = 0;
    tminus_prompt();
}

void rc_os_start_tminus_zulu() {
    g_tminusAsk = TMinusAsk::kZulu;
    g_tminusPos = 0;
    tminus_prompt();
}

static int handle_tminus_line() {
    if (g_tminusAsk == TMinusAsk::kNone) {
        return -1;
    }
    const int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) {
        return 0;
    }
    if (c == rc::cli::kEsc) {
        rc::rc_log("\nT- cancelled\n");
        g_tminusAsk = TMinusAsk::kNone;
        show_prompt();
        return 1;
    }
    if (c == '\r' || c == '\n') {
        commit_tminus_line();
        g_tminusAsk = TMinusAsk::kNone;
        show_prompt();
        return 1;
    }
    if (c == 8 || c == 127) {
        if (g_tminusPos > 0) {
            g_tminusPos--;
            rc::rc_log("\b \b");
        }
        return 1;
    }
    if (g_tminusPos + 1U < sizeof(g_tminusBuf)) {
        rc::rc_log("%c", c);
        g_tminusBuf[g_tminusPos++] = static_cast<char>(c);
    }
    return 1;
}

void rc_os_start_arm_confirm() {
#ifndef ROCKETCHIP_HOST_TEST
    g_armConfirmActive = true;
    g_armBufPos = 0;
    g_armStartMs = to_ms_since_boot(get_absolute_time());
    ansi_dashboard_pause();
    rc::rc_log("Type ARM in caps then Enter to confirm (5s): ");
#endif
}

static void enter_gcs_exclusive() {
    g_mavlinkDetected = true;
    g_settleCount = 0;
    AO_RCOS_set_output_mode(StationOutputMode::kMavlink);
}

static bool drain_gcs_exclusive() {
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        AO_Telemetry_feed_usb_byte(static_cast<uint8_t>(c));
        c = getchar_timeout_us(0);
    }
    return true;
}

// First MAVLink STX takes USB. Pad/CLI stay off until disconnect.
static bool sniff_gcs_stx() {
    if (AO_RCOS_get_output_mode() == StationOutputMode::kMavlink) {
        return drain_gcs_exclusive();
    }
    int c;
    bool mav = false;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        const uint8_t b = static_cast<uint8_t>(c);
        if (is_mavlink_stx(b)) { mav = true; }
        if (mav) {
            AO_Telemetry_feed_usb_byte(b);
        }
    }
    if (mav) {
        enter_gcs_exclusive();
        return true;
    }
    return false;
}

static bool handle_mavlink_lockout(int c) {
    if (is_mavlink_stx(static_cast<uint8_t>(c))) {
        enter_gcs_exclusive();
        return true;
    }
    if (g_mavlinkDetected ||
        AO_Telemetry_is_gcs_connected() ||
        AO_RCOS_get_output_mode() == StationOutputMode::kMavlink) {
        AO_Telemetry_feed_usb_byte(static_cast<uint8_t>(c));
        return true;
    }
    return false;
}

static bool handle_usb_connect() {
    if (!g_wasConnected) {
        g_settleCount = 1;
        g_wasConnected = true;
    }
    if (g_settleCount > 0) {
        if (sniff_gcs_stx()) {
            return true;
        }
        if (g_settleCount < kUsbSettlePolls) {
            g_settleCount++;
            return false;
        }
        g_settleCount = 0;
        if (AO_RCOS_get_output_mode() == StationOutputMode::kMavlink) {
            return true;
        }
        cli_print_boot_summary();
        if (!g_bannerPrinted) {
            rc::rc_log("\n");
            show_help();
            g_bannerPrinted = true;
        }
        rc::cli::init(g_eng);
        show_prompt();
    }
    return true;
}

static int handle_arm_confirm() {
    if (!g_armConfirmActive) {
        return -1;
    }
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - g_armStartMs > kArmConfirmTimeoutMs) {
        rc::rc_log("ARM aborted (timeout)\n");
        g_armConfirmActive = false;
        ansi_dashboard_resume();
        show_prompt();
        return 1;
    }
    int ac = getchar_timeout_us(0);
    if (ac == PICO_ERROR_TIMEOUT) {
        return 0;
    }
    if (ac == '\r' || ac == '\n') {
        g_armBuf[g_armBufPos] = '\0';
        if (g_armBufPos == 3 &&
            g_armBuf[0] == 'A' && g_armBuf[1] == 'R' && g_armBuf[2] == 'M') {
            AO_Telemetry_send_tracked_command(kMavCmdArmDisarm, 1.0F);
            rc::rc_log("[CMD] ARM sent, waiting for ACK...\n");
        } else {
            rc::rc_log("ARM aborted (bad input: '%s')\n", g_armBuf);
        }
        g_armConfirmActive = false;
        ansi_dashboard_resume();
        show_prompt();
        return 1;
    }
    if (g_armBufPos < 3) {
        rc::rc_log("%c", ac);
        g_armBuf[g_armBufPos++] = static_cast<char>(ac);
    } else {
        rc::rc_log("ARM aborted (overflow)\n");
        g_armConfirmActive = false;
        ansi_dashboard_resume();
        show_prompt();
    }
    return 1;
}

static void dispatch_key(int c) {
    const rc::cli::Result r =
        rc::cli::on_key(g_eng, items(), item_count(), c);
    switch (r.ev) {
        case rc::cli::Event::kHelp:
            show_help();
            break;
        case rc::cli::Event::kPushed:
            show_help();
            break;
        case rc::cli::Event::kPopped:
            rc::rc_log("Returning to previous menu.\n");
            break;
        case rc::cli::Event::kAction:
            rc::cli::run_action(r.act);
            if (r.act == rc::cli::ActionId::kReturnPad) {
                return;
            }
            break;
        case rc::cli::Event::kUnknown:
            break;
    }
    if (g_tminusAsk == TMinusAsk::kNone) {
        show_prompt();
    }
}

bool rc_os_update() {
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (rc::gcs_usb_disconnect_expired(stdio_usb_connected(), now_ms,
                                       &g_usbDisconnectSinceMs)) {
#if defined(ROCKETCHIP_DEV_MODE)
        g_devModeRuntime = false;
#endif
        g_wasConnected  = false;
        g_bannerPrinted = false;
        g_mavlinkDetected = false;
        if constexpr (job::kRadioModeRx) {
            if (AO_RCOS_get_output_mode() == StationOutputMode::kMavlink) {
                AO_RCOS_set_output_mode(StationOutputMode::kAnsi);
            }
        }
        return false;
    }
    if (!stdio_usb_connected()) {
        return false;
    }
    if (!cli_terminal_connected()) {
        g_wasConnected = false;
        g_bannerPrinted = false;
        g_settleCount = 0;
        return false;
    }
    if (!handle_usb_connect()) {
        return false;
    }
    if (debug_eskf_live_poll()) {
        return false;
    }

    const int arm_result = handle_arm_confirm();
    if (arm_result == 0) {
        return false;
    }
    if (arm_result == 1) {
        return true;
    }

    const int tminus_result = handle_tminus_line();
    if (tminus_result == 0) {
        return false;
    }
    if (tminus_result == 1) {
        return true;
    }

    if (AO_RCOS_cal_active()) {
        return false;
    }

    if (AO_RCOS_get_output_mode() == StationOutputMode::kMavlink) {
        return drain_gcs_exclusive();
    }

    const int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) {
        return false;
    }

    if (handle_mavlink_lockout(c)) {
        return drain_gcs_exclusive();
    }

    dispatch_key(c);
    return true;
}

bool rc_os_usb_settling() {
    return g_settleCount > 0;
}

bool rc_os_is_connected() {
    return stdio_usb_connected();
}

bool rc_os_is_calibrating() {
    return calibration_is_active();
}

rc_os_menu_t rc_os_get_menu() {
    return static_cast<rc_os_menu_t>(rc::cli::top(g_eng));
}
