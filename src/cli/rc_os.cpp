// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// USB / lockout / ARM-confirm modes + engine kKey dispatch.
// Menu meaning lives in cli_menus.h (data). AO_RCOS ticks this.

#include "rc_os.h"
#include "rc_os_dashboard.h"
#include "cli/cli_engine.h"
#include "cli/cli_menus.h"
#include "cli/cli_actions.h"
#include "cli/rc_os_commands.h"
#include "cli/rc_os_debug.h"
#include "active_objects/ao_telemetry.h"
#include "ao_rcos.h"
#include "rocketchip/job.h"
#include "rocketchip/rc_log.h"
#include "rocketchip/station_output_mode.h"
#include "calibration/calibration_manager.h"
#include "ao_flight_director.h"
#include "pico/stdlib.h"
#include "pico/time.h"

constexpr uint8_t  kUsbSettlePolls      = 5;
constexpr uint32_t kArmConfirmTimeoutMs = 5000;
constexpr uint8_t  kMavlinkV2Stx        = 0xFDU;
constexpr uint16_t kMavCmdArmDisarm     = 400;

static rc::cli::Engine g_eng{};
static bool g_wasConnected  = false;
static bool g_bannerPrinted = false;
static uint8_t g_settleCount = 0;
static bool g_mavlinkDetected = false;

static bool g_armConfirmActive = false;
static char g_armBuf[4] = {};
static uint8_t g_armBufPos = 0;
static uint32_t g_armStartMs = 0;

#if defined(ROCKETCHIP_DEV_MODE)
static bool g_devModeRuntime = false;
#endif

bool rc_os_imu_available = false;
bool rc_os_baro_available = false;
bool rc_os_i2c_scan_allowed = true;
std::atomic<bool> rc_os_mag_cal_active{false};

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

static void show_prompt() {
    rc::cli::print_prompt(rc::cli::top(g_eng));
}

void rc_os_init() {
    rc::cli::init(g_eng);
    g_wasConnected  = false;
    g_bannerPrinted = false;
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

void rc_os_start_arm_confirm() {
#ifndef ROCKETCHIP_HOST_TEST
    g_armConfirmActive = true;
    g_armBufPos = 0;
    g_armStartMs = to_ms_since_boot(get_absolute_time());
    rc_os_dashboard_pause();
    rc::rc_log("Type ARM in caps then Enter to confirm (5s): ");
#endif
}

static bool handle_mavlink_lockout(int c) {
    if (static_cast<uint8_t>(c) == kMavlinkV2Stx) {
        g_mavlinkDetected = true;
    }
    if (g_mavlinkDetected ||
        AO_Telemetry_is_gcs_connected() ||
        AO_RCOS_get_output_mode() == StationOutputMode::kMavlink) {
        if (c == rc::cli::kEsc) {
            g_mavlinkDetected = false;
            AO_RCOS_set_output_mode(StationOutputMode::kMenu);
            rc::rc_log("\n[GCS mode exited]\n");
            show_prompt();
        }
        return true;
    }
    return false;
}

static bool handle_mavlink_input(int c) {
    if (AO_RCOS_get_output_mode() != StationOutputMode::kMavlink) {
        return false;
    }
    if (rc::cli::top(g_eng) != rc::cli::MenuId::kMain) {
        return false;
    }
    if (c == 'm' || c == 'M') {
        AO_RCOS_set_output_mode(StationOutputMode::kMenu);
        rc::rc_log("\nMAVLink mode off. CLI active.\n");
        show_prompt();
        return true;
    }
    AO_Telemetry_feed_usb_byte(static_cast<uint8_t>(c));
    return true;
}

static bool handle_usb_connect() {
    if (!g_wasConnected) {
        g_settleCount = 1;
        g_wasConnected = true;
        return false;
    }
    if (g_settleCount > 0 && g_settleCount < kUsbSettlePolls) {
        g_settleCount++;
        return false;
    }
    if (g_settleCount == kUsbSettlePolls) {
        g_settleCount = 0;
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}
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
        rc_os_dashboard_resume();
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
        rc_os_dashboard_resume();
        show_prompt();
        return 1;
    }
    if (g_armBufPos < 3) {
        rc::rc_log("%c", ac);
        g_armBuf[g_armBufPos++] = static_cast<char>(ac);
    } else {
        rc::rc_log("ARM aborted (overflow)\n");
        g_armConfirmActive = false;
        rc_os_dashboard_resume();
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
    show_prompt();
}

bool rc_os_update() {
    if (!stdio_usb_connected()) {
#if defined(ROCKETCHIP_DEV_MODE)
        g_devModeRuntime = false;
#endif
        g_wasConnected  = false;
        g_bannerPrinted = false;
        return false;
    }
    if (!handle_usb_connect()) {
        return false;
    }
    if (dev_eskf_live_poll()) {
        return false;
    }

    const int arm_result = handle_arm_confirm();
    if (arm_result == 0) {
        return false;
    }
    if (arm_result == 1) {
        return true;
    }

    if (AO_RCOS_cal_active()) {
        return false;
    }

    const int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) {
        return false;
    }

    AO_Telemetry_feed_usb_byte(static_cast<uint8_t>(c));

    if (handle_mavlink_lockout(c)) {
        return true;
    }
    if (handle_mavlink_input(c)) {
        return true;
    }

    dispatch_key(c);
    return true;
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
