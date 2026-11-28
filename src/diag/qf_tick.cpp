// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// QF tick from a one-shot SDK alarm that the callback sets again.
// pico-sdk 2.2.0 time.c: a repeating timer that runs late replays each
// missed period in the same IRQ (next = last target + period, loop while
// next <= now). A full tick burst posts many time events at the same time
// and can fill an AO queue (QP assert 130). This callback fires one QF
// tick, counts the missed periods, and returns -(skipped + 1) * period.
// A negative return sets the next alarm from the last target, so the tick
// stays on the same 10 ms grid.
#include "diag/qf_tick.h"

#include "pico/time.h"

extern "C" {
#include "qp_port.h"
}

namespace {

constexpr uint32_t kQfTickPeriodUs = 10000U;  // 100 Hz

// Alarm IRQ (Core 0) writes these. qf_tick_stats reads them in task context.
uint64_t g_qfTickTargetUs = 0U;
rc::TickSkipStats g_qfTickStats{};

int64_t qf_tick_alarm_cb(alarm_id_t id, void* user_data) {
    (void)id;
    (void)user_data;
    const rc::TickSkipStep step =
        rc::tick_skip_step(g_qfTickTargetUs, time_us_64(), kQfTickPeriodUs);
    g_qfTickTargetUs = step.next_target_us;
    rc::tick_skip_note(g_qfTickStats, step.skipped);
    QTIMEEVT_TICK_X(0U, nullptr);
    return -static_cast<int64_t>(step.advance_us);
}

} // namespace

extern "C" bool qf_tick_start(void) {
    g_qfTickTargetUs = time_us_64() + kQfTickPeriodUs;
    const alarm_id_t id = add_alarm_at(from_us_since_boot(g_qfTickTargetUs),
                                       qf_tick_alarm_cb, nullptr, true);
    return id > 0;
}

namespace rc {

TickSkipStats qf_tick_stats() {
    TickSkipStats s{};
    s.fired = g_qfTickStats.fired;
    s.skipped_total = g_qfTickStats.skipped_total;
    s.max_skip = g_qfTickStats.max_skip;
    s.bursts = g_qfTickStats.bursts;
    return s;
}

} // namespace rc
