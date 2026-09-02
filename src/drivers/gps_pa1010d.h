// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// PA1010D I2C NMEA. Prior Art: GlobalTop NMEA-over-I2C v1.2; lwGPS.

#ifndef ROCKETCHIP_GPS_PA1010D_H
#define ROCKETCHIP_GPS_PA1010D_H

#include "gps.h"
#include "rocketchip/i2c_strap.h"
#include <stddef.h>

constexpr uint8_t kGpsPa1010dAddr = kI2cAddrPa1010d;

[[nodiscard]] bool gps_pa1010d_init(void);
[[nodiscard]] bool gps_pa1010d_ready(void);
[[nodiscard]] bool gps_pa1010d_update(void);
[[nodiscard]] bool gps_pa1010d_get_data(gps_data_t* data);
[[nodiscard]] bool gps_pa1010d_has_fix(void);
[[nodiscard]] bool gps_pa1010d_get_last_raw(const uint8_t** buf, size_t* len);
void gps_pa1010d_get_debug_status(char* buf, size_t len);

#endif
