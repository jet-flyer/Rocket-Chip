// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// 100 Hz QF tick source (target only). lib/qep/bsp_qv.c QF_onStartup calls
// qf_tick_start. The tick does not replay missed periods after a stall
// (diag/tick_skip.h). The Debug 'w' dump prints the stats.
#ifndef ROCKETCHIP_DIAG_QF_TICK_H
#define ROCKETCHIP_DIAG_QF_TICK_H

#ifdef __cplusplus
extern "C" {
#endif

// Start the tick alarm. Returns false if the SDK has no alarm slot.
bool qf_tick_start(void);

#ifdef __cplusplus
}

#include "diag/tick_skip.h"

namespace rc {

// Copy of the stats. The alarm IRQ writes them; each field is one word.
TickSkipStats qf_tick_stats();

} // namespace rc
#endif

#endif // ROCKETCHIP_DIAG_QF_TICK_H
