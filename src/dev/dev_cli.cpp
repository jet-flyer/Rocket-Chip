// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Dev-only CLI: debug sub-menu + ESKF live streaming.
// Excluded from flight builds via CMake (BUILD_FOR_FLIGHT).

#ifndef BUILD_FOR_FLIGHT

#include "dev/dev_cli.h"
#include "dev/replay_inject.h"
#include "cli/rc_os.h"
#include "cli/rc_os_commands.h"
#include "drivers/i2c_bus.h"
#include "safety/pyro_edge_logger.h"
#include "rocketchip/config.h"
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
    printf("s-Sensors  i-I2C scan  b-Boot/HW  e-ESKF live  y-Pyro log\n");
    printf("h-Help  z-Back\n");
    return true;
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
            replay_inject_start();
            break;
        case 'h': case 'H': case '?':
            printf("\n--- Debug Menu ---\n");
            printf("s-Sensors  i-I2C scan  b-Boot/HW  e-ESKF live\n");
            printf("y-Pyro log  r-Replay inject  z-Back\n");
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

#endif // BUILD_FOR_FLIGHT
