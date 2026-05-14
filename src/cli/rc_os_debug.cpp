// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// rc_os_debug — operator Debug sub-menu (`q` from main).
//
// R-25-exec step 2 of 13 (per docs/decisions/BENCH_TIER_DEPRECATION_
// 2026-05-13.md, council-APPROVED Approach A): migrated from
// src/dev/dev_cli.cpp. No longer #ifdef-gated by ROCKETCHIP_INCLUDES_
// DEV_DIAGNOSTICS — lives in the single flight binary always. State-
// mutating commands (digit-key radio-config set, LED test pattern
// force) are runtime-gated by test_mode_active(); diagnostic reads
// (sensors, HW status, I2C scan, ESKF live, diag dump, pyro log)
// work always (observational, no safety risk).

#include "cli/rc_os_debug.h"
// R-25-exec step 5 (2026-05-13): src/dev/replay_inject.h removed.
// R-25-exec step 6 (2026-05-13): src/dev/station_replay.h removed.
// Replay coverage moved host-side per council amendment #4.
#include "cli/rc_os.h"
#include "cli/rc_os_commands.h"
#include "safety/test_mode.h"
#include "drivers/i2c_bus.h"
#include "safety/pyro_edge_logger.h"
#include "diag/diag_stats.h"
#include "active_objects/ao_led_engine.h"
#include "active_objects/ao_radio.h"           // T6: local config set
#include "rocketchip/led_patterns.h"
#include "rocketchip/config.h"
#include "rocketchip/job.h"
#include "rocketchip/radio_config_table.h"     // T6: whitelist for digit keys
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool s_eskfLiveActive = false;
static uint32_t s_eskfLiveLastPrintUs = 0;
static constexpr uint32_t kEskfLivePeriodUs = 1000000;

bool dev_debug_menu_enter() {
    printf("\n--- Debug ---\n");
    printf("s-Sensors  i-I2C scan  b-Boot/HW  e-ESKF live\n");
    printf("y-Pyro log  r-Replay  d-Diag stats  l-LED test  h-Help  z-Back\n");
    return true;
}

// Stage L IVP-L1 HW verify: force a specific LED pattern code for visual check.
// Blocking input is unsafe in a handler — it would overflow AO event queues
// (LL Entry 32). Instead, set a "LED test pending" flag; the next keypress
// received by the main dispatcher falls through to dev_led_test_poll().
static bool s_ledTestPending = false;

static void dev_led_test_force(uint8_t code) {
    printf("[led_test] forcing pattern code %u (dev override)\n", code);
    // Use the dev-override path, not post_pattern, to beat AO_Notify's
    // continuous re-publish of SIG_LED_PATTERN. Pass 0 to clear.
    AO_LedEngine_dev_force_fault_layer(code);
}

static void dev_led_test_menu() {
    printf("\n--- LED pattern test (Stage L IVP-L1) ---\n");
    printf("Pick one (press the digit):\n");
    printf("  1 = kCalGyro            (yellow blink, AP)\n");
    printf("  2 = kFdArmed            (red solid, AP)\n");
    printf("  3 = kFdLandedBeacon     (green+white alt 2Hz)\n");
    printf("  4 = kFdAbortBeacon      (red+white   alt 2Hz)\n");
    printf("  5 = kFaultSafeMode      (blue+white  alt 2Hz)\n");
    printf("  6 = kFdPreArmFail       (yellow double-flash)\n");
    printf("  7 = kFdBootInit         (rainbow)\n");
    printf("  8 = kFdBeacon           (white blink - distress)\n");
    printf("  9 = kFdLanded           (green blink - no beacon baseline)\n");
    printf("  0 = clear (return to normal resolver)\n");
    printf("  any other key = cancel\n");
    printf("[led_test] > ");
    s_ledTestPending = true;
}

bool dev_led_test_pending() { return s_ledTestPending; }

void dev_led_test_feed(int c) {
    s_ledTestPending = false;
    printf("%c\n", (char)c);
    switch (c) {
        case '1': dev_led_test_force(rc::led::kCalGyro);        break;
        case '2': dev_led_test_force(rc::led::kFdArmed);        break;
        case '3': dev_led_test_force(rc::led::kFdLandedBeacon); break;
        case '4': dev_led_test_force(rc::led::kFdAbortBeacon);  break;
        case '5': dev_led_test_force(rc::led::kFaultSafeMode);  break;
        case '6': dev_led_test_force(rc::led::kFdPreArmFail);   break;
        case '7': dev_led_test_force(rc::led::kFdBootInit);     break;
        case '8': dev_led_test_force(rc::led::kFdBeacon);       break;
        case '9': dev_led_test_force(rc::led::kFdLanded);       break;
        case '0': dev_led_test_force(0);                        break;
        default: printf("[led_test] cancelled\n"); break;
    }
}

bool dev_debug_menu_dispatch(int c) {
    switch (c) {
        case 's': case 'S':
            if constexpr (kRadioModeRx) {
                cli_print_station_status();
            } else {
                cli_print_sensor_status();
            }
            break;
        case 'i': case 'I':
            if (rc_os_i2c_scan_allowed) {
                printf("\nRescanning I2C bus...\n");
                i2c_bus_scan();
            } else {
                printf("\nI2C scan disabled (Core 1 owns bus)\n");
            }
            break;
        case 'b': case 'B':
            cli_print_hw_status();
            break;
        case 'e':
            s_eskfLiveActive = true;
            s_eskfLiveLastPrintUs = time_us_32();
            printf("\n--- ESKF live (1Hz) --- any key to stop ---\n");
            cli_print_eskf_live();
            break;
        case 'y': case 'Y':
            rc::pyro_edge_logger_dump_cli();
            break;
        case 'r': case 'R':
            // R-25-exec steps 5+6 (2026-05-13): both vehicle replay-inject
            // and station replay-inject DELETED per council amendment #4.
            // Replay coverage moved host-side (see
            // scripts/replay_harness_host.py for vehicle; station replay
            // covered by a host-side hex-injection harness rewrite in
            // step 7).
            printf("[debug] replay retired; see scripts/replay_harness_host.py\n");
            break;
        case 'd': case 'D':
            diag_stats_dump();
            break;
        case 'l': case 'L':
            // R-25-exec: LED test is state-mutating (writes the LED
            // engine override path). Test-mode-gated per council
            // Approach A.
            if (!rc::test_mode_active()) {
                printf("[debug] LED test gated; arm test mode via probe "
                       "(see safety/test_mode.h)\n");
                break;
            }
            dev_led_test_menu();
            break;
        case '0': case '1': case '2': case '3': case '4': case '5': {
            // R-25-exec: radio config set is state-mutating (drives
            // AO_Radio to reconfigure the radio). Test-mode-gated per
            // council Approach A.
            if (!rc::test_mode_active()) {
                printf("[debug] radio config set gated; arm test mode "
                       "via probe (see safety/test_mode.h)\n");
                break;
            }
            // Stage T IVP-T6 — local radio config set (no RF).
            // Digit = whitelist index (0=BW125/5 default, 1=BW125/10,
            // 2=BW250/10, 3=BW500/10, 4=BW125/2, 5=BW250/5).
            size_t idx = static_cast<size_t>(c - '0');
            if (idx >= rc::kRadioConfigTableSize) {
                printf("[cfg] idx %u out of range\n",
                       static_cast<unsigned>(idx));
                break;
            }
            const auto& t = rc::kRadioConfigTable[idx];
            rc::RadioConfig cfg{};
            cfg.mode             = rc::RadioRole::kTx;
            cfg.protocol         = rc::EncoderType::kCcsds;
            cfg.bandwidth_khz    = t.bw_khz;
            cfg.nav_rate_hz      = t.nav_rate_hz;
            cfg.spreading_factor = t.sf;
            cfg.coding_rate      = t.cr;
            cfg.power_dbm        = t.power_dbm;
            AO_Radio_set_pending_config(cfg);
            printf("[cfg] local radio -> BW%u %uHz SF%u CR%u pwr%u (idx %u)\n",
                   static_cast<unsigned>(t.bw_khz),
                   static_cast<unsigned>(t.nav_rate_hz),
                   static_cast<unsigned>(t.sf),
                   static_cast<unsigned>(t.cr),
                   static_cast<unsigned>(t.power_dbm),
                   static_cast<unsigned>(idx));
            break;
        }
        case 'h': case 'H': case '?':
            printf("\n--- Debug Menu ---\n");
            printf("s-Sensors  i-I2C scan  b-Boot/HW  e-ESKF live\n");
            printf("y-Pyro log  r-Replay inject  d-Diag stats  l-LED test  z-Back\n");
            printf("0..5 = local radio cfg (0:BW125/5 1:BW125/10 2:BW250/10\n");
            printf("                        3:BW500/10 4:BW125/2 5:BW250/5)\n");
            break;
        case 'z': case 'Z': case 27:
            printf("Returning to main menu.\n");
            return false;
        default:
            break;
    }
    return true;
}

bool dev_eskf_live_poll() {
    if (!s_eskfLiveActive) { return false; }
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        s_eskfLiveActive = false;
        printf("\n--- ESKF live stopped ---\n");
        printf("[debug] ");
    } else {
        uint32_t nowUs = time_us_32();
        if (nowUs - s_eskfLiveLastPrintUs >= kEskfLivePeriodUs) {
            s_eskfLiveLastPrintUs = nowUs;
            cli_print_eskf_live();
        }
    }
    return true;
}

// R-25-exec step 5 (2026-05-13): dev_replay_poll() + parse_and_inject()
// + s_replayBuf DELETED per council amendment #4. Vehicle CSV-streamer
// replay retired; algorithmic coverage moves to host-side
// scripts/replay_harness_host.py (same ESKF code, host workstation,
// no on-target attack surface, no flash cost).

// R-25-exec step 6 (2026-05-13): dev_station_replay_poll() +
// station_parse_and_inject() + hex_nibble() + s_stReplayBuf DELETED
// per council amendment #4. Station hex-injection replay retired;
// coverage moves host-side (rewrite tracked in step 7).
