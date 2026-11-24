// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// [Radio] lines of the T=0 preconditions (diag_stats_t0_preconditions).
// Split out of diag_stats.cpp so the host test can link it without QP.

#ifndef ROCKETCHIP_DIAG_DIAG_RADIO_PRECONDITION_H
#define ROCKETCHIP_DIAG_DIAG_RADIO_PRECONDITION_H

// radio_fitted is board::kRadioAvailable. Fitted: RegVersion readback and
// IRQ pin state. Not fitted: one "fitted=NO" line, and no SPI or GPIO read
// (that pack never runs spi_bus_init(), main.cpp init_peripherals).
void diag_log_radio_precondition(bool radio_fitted);

#endif // ROCKETCHIP_DIAG_DIAG_RADIO_PRECONDITION_H
