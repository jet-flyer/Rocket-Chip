// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Host stub: the subset of pico/time.h the drivers under test call.
// sleep_ms only counts; host tests do not wait.

#ifndef ROCKETCHIP_TEST_HOST_STUB_PICO_TIME_H
#define ROCKETCHIP_TEST_HOST_STUB_PICO_TIME_H

#include <stdint.h>
#include "host_stub_counters.h"

inline void sleep_ms(uint32_t ms) {
    host_stub::g_sleepCalls++;
    host_stub::g_sleepMsTotal += ms;
}

#endif  // ROCKETCHIP_TEST_HOST_STUB_PICO_TIME_H
