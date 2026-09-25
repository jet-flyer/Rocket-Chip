// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// ActionId → domain. Direct switch (P10-9: no function pointers).

#include "cli/cli_actions.h"
#include "cli/cli_catalog.h"
#include "cli/rc_os.h"
#include "cli/rc_os_commands.h"
#include "cli/rc_os_debug.h"
#include "cli/rc_os_dashboard.h"
#include "active_objects/ao_rcos.h"
#include "active_objects/ao_radio.h"
#include "ao_flight_director.h"
#include "rocketchip/ao_signals.h"
#include "ao_notify.h"
#include "active_objects/ao_telemetry.h"
#include "drivers/i2c_master.h"
#include "rocketchip/shared_state.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/radio_config.h"
#include "rocketchip/radio_config_table.h"
#include "rocketchip/version.h"
#include "rocketchip/board.h"
#include "hardware/watchdog.h"
#include "pico/stdio_usb.h"
#include "tusb.h"
#include "diag/diag_stats.h"
#include "diag/radio_rate_counters.h"
#include "safety/pyro_edge_logger.h"
#include "flight_director/command_handler.h"
#include "flight_director/mission_profile.h"
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
        case MenuId::kClock:    rc::rc_log("[clock] "); break;
        case MenuId::kGps:      rc::rc_log("[gps] "); break;
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
#if !defined(ROCKETCHIP_DEV_MODE)
    if (cmd == rc::CommandType::kArm &&
        rc::kDefaultRocketProfile.usb_arm_inhibit &&
        stdio_usb_connected()) {
        rc::rc_log("ARM refused (USB_ARM_INH)\n");
        AO_Notify_post_prearm_fail();
        return;
    }
#endif
    const bool accepted =
        AO_FlightDirector_process_command(static_cast<int>(cmd));
    if (!accepted && cmd == rc::CommandType::kArm) {
        AO_Notify_post_prearm_fail();
    }
}

static const char* stn_out_name(StationOutputMode m) {
    switch (m) {
        case StationOutputMode::kAnsi:    return "ansi";
        case StationOutputMode::kCsv:     return "csv";
        case StationOutputMode::kMavlink: return "mavlink";
        case StationOutputMode::kMenu:    return "menu";
    }
    return "?";
}

static void catalog_list() {
    const bool veh = !job::kRadioModeRx;
    rc::rc_log("\n--- catalog ---\n");
    rc::rc_log("  IDENTITY    %s %s-%s  board=%s  profile=%s\n",
               kVersionString, kBuildConfig, kGitHash, board::kBoardName,
               rc::kDefaultRocketProfile.name);
    const rc::RadioConfig* cfg = AO_Radio_get_runtime_config();
    if (cfg != nullptr) {
        rc::rc_log("  NAV_PRESET  basic  BW%u %uHz SF%u CR%u  (n=next, not saved)\n",
                   static_cast<unsigned>(cfg->bandwidth_khz),
                   static_cast<unsigned>(cfg->nav_rate_hz),
                   static_cast<unsigned>(cfg->spreading_factor),
                   static_cast<unsigned>(cfg->coding_rate));
        rc::rc_log("  TX_POWER    locked %udBm\n",
                   static_cast<unsigned>(cfg->power_dbm));
    }
    if (veh) {
        rc::rc_log("  USB_ARM_INH locked %s\n",
                   rc::kDefaultRocketProfile.usb_arm_inhibit ? "ON" : "OFF");
        rc::rc_log("  USB_CFG_EN  locked %s\n",
                   rc::kDefaultRocketProfile.usb_config_en ? "ON" : "OFF");
    } else {
        rc::rc_log("  STN_OUTPUT  basic  %s  (main m cycles)\n",
                   stn_out_name(AO_RCOS_get_output_mode()));
    }
}

static void catalog_nav_next() {
    const rc::RadioConfig* cur = AO_Radio_get_runtime_config();
    if (cur == nullptr) {
        rc::rc_log("NAV_PRESET: no radio\n");
        return;
    }
    size_t idx = kRadioConfigTableSize;
    for (size_t i = 0; i < kRadioConfigTableSize; ++i) {
        const auto& e = kRadioConfigTable[i];
        if (e.bw_khz == cur->bandwidth_khz &&
            e.nav_rate_hz == cur->nav_rate_hz &&
            e.sf == cur->spreading_factor &&
            e.cr == cur->coding_rate) {
            idx = i;
            break;
        }
    }
    const uint8_t next = radio_config_next_fit(idx);
    if (next == kRadioConfigNoIndex) {
        rc::rc_log("NAV_PRESET: no catalog row fits nav ToA\n");
        return;
    }
    const auto& e = kRadioConfigTable[next];
    const uint32_t toa_us = radio_config_nav_airtime_us(
        e.sf, e.bw_khz, kRadioConfigNavPltuBytes);
    const uint32_t slot_us = (e.nav_rate_hz == 0)
                                 ? 0
                                 : (1000000U / e.nav_rate_hz);
    const uint32_t pct = (slot_us == 0) ? 999U : (toa_us * 100U / slot_us);
    if (!AO_Telemetry_request_comm_change(next)) {
        rc::rc_log("NAV_PRESET COMM_CHANGE refused idx=%u\n",
                   static_cast<unsigned>(next));
        return;
    }
    rc::rc_log("NAV_PRESET COMM_CHANGE BW%u %uHz SF%u (idx %u, ToA %u%%)\n",
               static_cast<unsigned>(e.bw_khz),
               static_cast<unsigned>(e.nav_rate_hz),
               static_cast<unsigned>(e.sf),
               static_cast<unsigned>(next),
               static_cast<unsigned>(pct));
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
        case ActionId::kTMinusMinutes:
            rc_os_start_tminus_minutes();
            break;
        case ActionId::kTMinusZulu:
            rc_os_start_tminus_zulu();
            break;
        case ActionId::kTMinusClear:
            ansi_dashboard_clear_tminus();
            rc::rc_log("T- cleared\n");
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
            if (g_i2c_scan_allowed) {
                rc::rc_log("\nRescanning I2C bus...\n");
                cli_print_i2c_scan();
            } else {
                rc::rc_log("\nI2C scan disabled (Core 1 owns bus)\n");
            }
            break;
        case ActionId::kDebugI2cQuiesce: {
            shared_sensor_data_t snap{};
            (void)seqlock_read(&g_sensorSeqlock, &snap);
            rc::rc_log("[I2C] quiesce+reboot  I=%lu e=%lu B=%lu e=%lu G=%lu e=%lu\n",
                       static_cast<unsigned long>(snap.imu_read_count),
                       static_cast<unsigned long>(snap.imu_error_count),
                       static_cast<unsigned long>(snap.baro_read_count),
                       static_cast<unsigned long>(snap.baro_error_count),
                       static_cast<unsigned long>(snap.gps_read_count),
                       static_cast<unsigned long>(snap.gps_error_count));
            i2c_master_quiesce(kI2cQuiesceViaCli);
            constexpr uint32_t kI2cRebootDelayMs = 100;
            watchdog_reboot(0, 0, kI2cRebootDelayMs);
            break;
        }
        case ActionId::kDebugI2cPark:
            rc::rc_log("[I2C] park for probe flash (WFI until reset)\n");
            (void)tud_disconnect();
            i2c_master_park();
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
        case ActionId::kDebugRfRates:
            radio_rate_counters_dump();
            break;
        case ActionId::kCatalogList:
            catalog_list();
            break;
        case ActionId::kCatalogNavNext:
            catalog_nav_next();
            break;
    }
}

}  // namespace cli
}  // namespace rc
