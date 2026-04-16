// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Dev-only CLI: debug sub-menu + ESKF live streaming.
// Excluded from flight builds via CMake (BUILD_FOR_FLIGHT).

#ifndef BUILD_FOR_FLIGHT

#include "dev/dev_cli.h"
#include "cli/rc_os.h"
#include "cli/rc_os_commands.h"
#include "drivers/i2c_bus.h"
#include "safety/pyro_edge_logger.h"
#include "rocketchip/config.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

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
        case 'h': case 'H': case '?':
            printf("\n--- Debug Menu ---\n");
            printf("s-Sensors  i-I2C scan  b-Boot/HW  e-ESKF live  y-Pyro log\n");
            printf("z-Back to main\n");
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

#endif // BUILD_FOR_FLIGHT
