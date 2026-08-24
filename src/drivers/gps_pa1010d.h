// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// PA1010D GPS module driver — I2C transport backend
// CDTop PA1010D via I2C with NMEA parsing by lwGPS.
// Uses transport-neutral types from gps.h.

#ifndef ROCKETCHIP_GPS_PA1010D_H
#define ROCKETCHIP_GPS_PA1010D_H

#include "gps.h"
#include "i2c_bus.h"
#include <stddef.h>

// ============================================================================
// Configuration
// ============================================================================

constexpr uint8_t kGpsPa1010dAddr = kI2cAddrPa1010d;

// ============================================================================
// API
// ============================================================================

[[nodiscard]] bool gps_pa1010d_init(void);

[[nodiscard]] bool gps_pa1010d_ready(void);

// Poll. Module is 1 Hz (PMTK220,1000). true if I2C read completed; false on error.
[[nodiscard]] bool gps_pa1010d_update(void);

[[nodiscard]] bool gps_pa1010d_get_data(gps_data_t* data);

[[nodiscard]] bool gps_pa1010d_has_fix(void);

// buf/len point at an internal buffer; invalid after the next update().
[[nodiscard]] bool gps_pa1010d_get_last_raw(const uint8_t** buf, size_t* len);

// PMTK write return codes from gps_pa1010d_init(), plus window-hit and init.
void gps_pa1010d_get_debug_status(char* buf, size_t len);

#endif // ROCKETCHIP_GPS_PA1010D_H
