// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// ActionId → domain. Direct switch (P10-9: no function pointers).

#include "cli/cli_actions.h"
#include "cli/rc_os.h"
#include "cli/rc_os_commands.h"
#include "cli/rc_os_debug.h"
#include "cli/rc_os_dashboard.h"
#include "active_objects/ao_rcos.h"
#include "ao_flight_director.h"
#include "rocketchip/ao_signals.h"
#include "ao_notify.h"
#include "active_objects/ao_telemetry.h"
#include "drivers/i2c_bus.h"
#include "diag/diag_stats.h"
#include "safety/pyro_edge_logger.h"
#include "flight_director/command_handler.h"
#include "rocketchip/job.h"
#include "rocketchip/rc_log.h"
#include "rocketchip/station_output_mode.h"

namespace rc {
namespace cli {

static constexpr uint16_t kMavCmdArmDisarm = 400;

void print_prompt(MenuId menu) {
    switch (menu) {
        case MenuId::kMain:     rc::rc_log("[main] "); break;
        case MenuId::kCal:      rc::rc_log("[cal] "); break;
        case MenuId::kFlight:   rc::rc_log("[flight] "); break;
        case MenuId::kDebug:    rc::rc_log("[debug] "); break;
        case MenuId::kSettings: rc::rc_log("[settings] "); break;
        default:                rc::rc_log("> "); break;
    }
}

void print_help(const Item* table, size_t n, MenuId menu) {
    rc::rc_log("\n");
    for (size_t i = 0; i < n; ++i) {
        if (table[i].menu == menu &&
            table[i].key != static_cast<char>(kEsc) &&
            (table[i].gate != Gate::kDevRuntime || rc_os_dev_mode_runtime())) {
            rc::rc_log("  %c  %s\n", table[i].key, table[i].label);
        }
    }
}

#if defined(ROCKETCHIP_DEV_MODE)
static bool inject_allowed() {
    if (rc_os_dev_mode_runtime()) {
        return true;
    }
    rc::rc_log("DEV_MODE off (v on main, USB + idle)\n");
    return false;
}
#endif

static void flight_command(rc::CommandType cmd) {
    const bool accepted =
        AO_FlightDirector_process_command(static_cast<int>(cmd));
    if (!accepted && cmd == rc::CommandType::kArm) {
        AO_Notify_post_prearm_fail();
    }
}

void run_action(ActionId act) {
    switch (act) {
        case ActionId::kNone:
        case ActionId::kHelp:
        case ActionId::kBack:
            break;
        case ActionId::kReturnPad:
            AO_RCOS_set_output_mode(StationOutputMode::kAnsi);
            break;
        case ActionId::kPreflight:
            cli_print_preflight();
            break;
        case ActionId::kBeacon:
            cmd_findme_beacon();
            break;
        case ActionId::kRadioStatus:
            cmd_radio_status();
            break;
        case ActionId::kFlushLog:
            cmd_flush_log();
            break;
        case ActionId::kEraseFlights:
            AO_RCOS_start_erase_flights();
            break;
        case ActionId::kListFlights:
            cmd_list_flights();
            break;
        case ActionId::kDownloadFlight:
            AO_RCOS_start_download_flight();
            break;
        case ActionId::kCycleOutputMode:
            AO_RCOS_cycle_output_mode();
            break;
        case ActionId::kStationGps:
            cmd_station_gps();
            break;
        case ActionId::kStationDistance:
            cmd_station_distance();
            break;
        case ActionId::kStationArmConfirm:
            rc_os_start_arm_confirm();
            break;
        case ActionId::kStationDisarm:
            AO_Telemetry_send_tracked_command(kMavCmdArmDisarm, 0.0F);
            break;
        case ActionId::kCalGyro:
            AO_RCOS_start_cal_gyro();
            break;
        case ActionId::kCalLevel:
            AO_RCOS_start_cal_level();
            break;
        case ActionId::kCalBaro:
            AO_RCOS_start_cal_baro();
            break;
        case ActionId::kCalAccel6pos:
            AO_RCOS_start_cal_6pos();
            break;
        case ActionId::kCalMag:
            AO_RCOS_start_cal_mag();
            break;
        case ActionId::kCalWizard:
            AO_RCOS_start_cal_wizard();
            break;
        case ActionId::kCalSave:
            AO_RCOS_start_cal_save();
            break;
#if defined(ROCKETCHIP_DEV_MODE)
        case ActionId::kCalReset:
            if (!inject_allowed()) {
                break;
            }
            AO_RCOS_start_cal_reset();
            break;
        case ActionId::kDevModeToggle:
            rc_os_dev_mode_toggle();
            break;
#else
        case ActionId::kCalReset:
        case ActionId::kDevModeToggle:
            break;
#endif
        case ActionId::kFlightArm:
            flight_command(rc::CommandType::kArm);
            break;
        case ActionId::kFlightDisarm:
            flight_command(rc::CommandType::kDisarm);
            break;
        case ActionId::kFlightAbort:
            flight_command(rc::CommandType::kAbort);
            break;
        case ActionId::kFlightReset:
            flight_command(rc::CommandType::kReset);
            break;
#if defined(ROCKETCHIP_DEV_MODE)
        case ActionId::kInjectLaunch:
            if (!inject_allowed()) {
                break;
            }
            AO_FlightDirector_dispatch_signal(SIG_LAUNCH);
            break;
        case ActionId::kInjectBurnout:
            if (!inject_allowed()) {
                break;
            }
            AO_FlightDirector_dispatch_signal(SIG_BURNOUT);
            break;
        case ActionId::kInjectApogee:
            if (!inject_allowed()) {
                break;
            }
            AO_FlightDirector_dispatch_signal(SIG_APOGEE);
            break;
        case ActionId::kInjectMain:
            if (!inject_allowed()) {
                break;
            }
            AO_FlightDirector_dispatch_signal(SIG_MAIN_DEPLOY);
            break;
        case ActionId::kInjectLanding:
            if (!inject_allowed()) {
                break;
            }
            AO_FlightDirector_dispatch_signal(SIG_LANDING);
            break;
#else
        case ActionId::kInjectLaunch:
        case ActionId::kInjectBurnout:
        case ActionId::kInjectApogee:
        case ActionId::kInjectMain:
        case ActionId::kInjectLanding:
            break;
#endif
        case ActionId::kFlightStatus:
            AO_FlightDirector_print_status();
            break;
        case ActionId::kDebugSensors:
            if constexpr (job::kRadioModeRx) {
                cli_print_station_status();
            } else {
                cli_print_sensor_status();
            }
            break;
        case ActionId::kDebugI2cScan:
            if (rc_os_i2c_scan_allowed) {
                rc::rc_log("\nRescanning I2C bus...\n");
                i2c_bus_scan();
            } else {
                rc::rc_log("\nI2C scan disabled (Core 1 owns bus)\n");
            }
            break;
        case ActionId::kDebugBootHw:
            cli_print_hw_status();
            break;
        case ActionId::kDebugEskfLive:
            cli_debug_start_eskf_live();
            break;
        case ActionId::kDebugPyroLog:
            rc::pyro_edge_logger_dump_cli();
            break;
        case ActionId::kDebugDiag:
            diag_stats_dump();
            break;
        case ActionId::kSettingsStub:
            rc::rc_log("settings catalog not wired (sitting 5)\n");
            break;
    }
}

}  // namespace cli
}  // namespace rc
