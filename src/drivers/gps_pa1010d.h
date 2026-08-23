// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// PA1010D GPS module driver — I2C transport backend
// CDTop PA1010D via I2C with NMEA parsing by lwGPS.
// Uses transport-neutral types from gps.h.

#ifndef ROCKETCHIP_GPS_PA1010D_H
#define ROCKETCHIP_GPS_PA1010D_H

#include "gps.h"
#include <stddef.h>

// ============================================================================
// Configuration
// ============================================================================

constexpr uint8_t kGpsPa1010dAddr   = 0x10;

// ============================================================================
// API
// ============================================================================

[[nodiscard]] bool gps_pa1010d_init(void);

[[nodiscard]] bool gps_pa1010d_ready(void);

// Call this periodically (at least 10Hz for 10Hz GPS).
// Reads available NMEA data from I2C and parses it.
[[nodiscard]] bool gps_pa1010d_update(void);

[[nodiscard]] bool gps_pa1010d_get_data(gps_data_t* data);

[[nodiscard]] bool gps_pa1010d_has_fix(void);

// buf/len point at an internal buffer; invalid after the next update().
[[nodiscard]] bool gps_pa1010d_get_last_raw(const uint8_t** buf, size_t* len);

// Contents: PMTK write return codes from the ultra-early init path
// (captured in init_early_hw() before USB CDC is up), plus window-hit
// flag and init flag. Used by the Hardware Status (`b`) CLI handler
// so those early-init observations survive to post-USB display time.
void gps_pa1010d_get_debug_status(char* buf, size_t len);

#endif // ROCKETCHIP_GPS_PA1010D_H
