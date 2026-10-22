// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station debug bench for Fruit Jam RFM95 DIO1 (GPIO41) and DIO2 (GPIO42).
// Refuses on any other role, and while the link is armed or flying.
#ifndef ROCKETCHIP_DIAG_STATION_DIO_BENCH_H
#define ROCKETCHIP_DIAG_STATION_DIO_BENCH_H

void station_dio_toggle_check();
void station_dio_fifo_depth();

#endif
