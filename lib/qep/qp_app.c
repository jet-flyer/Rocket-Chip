// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// QP/C Application Callbacks for RocketChip
//
// Required by QP/C FuSa (Functional Safety) subsystem.
// Q_onError() is called when a QP assertion fails.
//
// Two implementations:
//   - Host test: prints to stderr and aborts (fails the test)
//   - Target: logs to debug output and halts (watchdog will reset)
//============================================================================

#include "qp_port.h"
#include "qsafe.h"

#ifdef ROCKETCHIP_HOST_TEST
#include <stdio.h>
#include <stdlib.h>

Q_NORETURN Q_onError(char const * const module, int_t const id) {
    fprintf(stderr, "[QP ASSERT] module=%s, id=%d\n", module, id);
    abort();
}

#else
// Target implementation provided in main.cpp to access watchdog and NeoPixel
#endif

// QP_versionStr now provided by qf_act.c (IVP-75)
