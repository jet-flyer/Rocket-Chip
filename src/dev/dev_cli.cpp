// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Dev-only CLI: debug sub-menu + ESKF live streaming.
// Excluded from flight builds via CMake (BUILD_FOR_FLIGHT).

#ifndef BUILD_FOR_FLIGHT

#include "dev/dev_cli.h"
#include "dev/replay_inject.h"
#include "dev/station_replay.h"
#include "cli/rc_os.h"
#include "cli/rc_os_commands.h"
#include "drivers/i2c_bus.h"
#include "safety/pyro_edge_logger.h"
#include "dev/diag_stats.h"
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
            if constexpr (kRadioModeRx) {
                printf("\n--- Station replay (RX-path hex injection) --- send 'R,<hex>\\n' lines ---\n");
                station_replay_start();
            } else {
                replay_inject_start();
            }
            break;
        case 'd': case 'D':
            diag_stats_dump();
            break;
        case 'l': case 'L':
            dev_led_test_menu();
            break;
        case '0': case '1': case '2': case '3': case '4': case '5': {
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

// Replay mode: reads CSV lines from serial, injects into seqlock.
// Protocol: Python sends "S,ax,ay,az,gx,gy,gz,press,lat,lon,alt\n"
// End: Python sends "REPLAY_END\n"
static char s_replayBuf[128];
static uint8_t s_replayBufPos = 0;

static void parse_and_inject(const char* line) {
    if (strncmp(line, "REPLAY_END", 10) == 0) {
        replay_inject_stop();
        return;
    }
    if (line[0] != 'S' || line[1] != ',') { return; }

    static uint32_t s_replayTimestampUs = 0;
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, press = 0;
    int32_t lat = 0, lon = 0, alt = 0;
    bool gps_valid = false;

    // S,ax,ay,az,gx,gy,gz,press,lat,lon,alt
    int n = sscanf(line + 2, "%f,%f,%f,%f,%f,%f,%f,%ld,%ld,%ld",
                   &ax, &ay, &az, &gx, &gy, &gz, &press, &lat, &lon, &alt);
    gps_valid = (n >= 10 && lat != 0);
    s_replayTimestampUs += 10000;  // 10ms per sample (100Hz)

    replay_inject_sample(s_replayTimestampUs, ax, ay, az, gx, gy, gz,
                         press, lat, lon, alt, gps_valid);
}

bool dev_replay_poll() {
    if (!replay_inject_active()) { return false; }

    for (int drain = 0; drain < 256; ++drain) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) { break; }

        if (c == '\n' || c == '\r') {
            if (s_replayBufPos > 0) {
                s_replayBuf[s_replayBufPos] = '\0';
                parse_and_inject(s_replayBuf);
                s_replayBufPos = 0;
            }
        } else if (s_replayBufPos < sizeof(s_replayBuf) - 1) {
            s_replayBuf[s_replayBufPos++] = static_cast<char>(c);
        }
    }
    return true;
}

// Station replay mode: reads hex-encoded packet lines from serial,
// injects into AO_Telemetry RX path (IVP-132a.3).
// Protocol: Python sends "R,<hex bytes>\n" — hex is a complete packet.
// End: Python sends "REPLAY_END\n".
static char s_stReplayBuf[512];
static uint16_t s_stReplayBufPos = 0;

static uint8_t hex_nibble(char c) {
    if (c >= '0' && c <= '9') { return static_cast<uint8_t>(c - '0'); }
    if (c >= 'a' && c <= 'f') { return static_cast<uint8_t>(c - 'a' + 10); }
    if (c >= 'A' && c <= 'F') { return static_cast<uint8_t>(c - 'A' + 10); }
    return 0xFF;
}

static void station_parse_and_inject(const char* line) {
    if (strncmp(line, "REPLAY_END", 10) == 0) {
        station_replay_stop();
        printf("[station_replay] end (count=%lu)\n",
               (unsigned long)station_replay_get_inject_count());
        return;
    }
    if (line[0] != 'R' || line[1] != ',') { return; }

    const char* hex = line + 2;
    uint8_t buf[128];
    uint8_t len = 0;
    while (hex[0] && hex[1] && len < sizeof(buf)) {
        uint8_t hi = hex_nibble(hex[0]);
        uint8_t lo = hex_nibble(hex[1]);
        if (hi == 0xFF || lo == 0xFF) { break; }
        buf[len++] = static_cast<uint8_t>((hi << 4) | lo);
        hex += 2;
    }
    if (len > 0) {
        station_replay_inject_bytes(buf, len);
    }
}

bool dev_station_replay_poll() {
    if constexpr (!kRadioModeRx) { return false; }
    if (!station_replay_active()) { return false; }

    for (int drain = 0; drain < 256; ++drain) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) { break; }

        if (c == '\n' || c == '\r') {
            if (s_stReplayBufPos > 0) {
                s_stReplayBuf[s_stReplayBufPos] = '\0';
                station_parse_and_inject(s_stReplayBuf);
                s_stReplayBufPos = 0;
            }
        } else if (s_stReplayBufPos < sizeof(s_stReplayBuf) - 1) {
            s_stReplayBuf[s_stReplayBufPos++] = static_cast<char>(c);
        }
    }
    return true;
}

#endif // BUILD_FOR_FLIGHT
