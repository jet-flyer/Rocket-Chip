// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_LINKER_SYMBOLS_H
#define ROCKETCHIP_LINKER_SYMBOLS_H

#include <stdint.h>

// Linker-script-defined stack-guard boundary symbols (accepted-via-vendoring,
// TP-2). The reserved `__`-names belong to the toolchain/linker — we only
// REFERENCE them (take their address for the MPU stack guard), never define
// them, so CERT DCL37-C ("don't create a reserved identifier") does not apply.
// Centralized here so the one toolchain boundary lives in a single
// documented place (L2-P5 council 2026-06-24; TP-2). clang-tidy naming
// ignores these two symbols via GlobalVariableIgnoredRegexp.
extern "C" {
extern uint32_t __StackBottom;     // Core 0 stack bottom — start of the MPU guard
extern uint32_t __StackOneBottom;  // Core 1 stack bottom (SCRATCH_X)
}

#endif // ROCKETCHIP_LINKER_SYMBOLS_H
