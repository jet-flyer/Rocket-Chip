// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// rc_log — project-owned bounded logging channel that replaces <stdio.h>.
//
// Purpose: provide a printf-shape call surface for diagnostic logging
// without using <stdio.h>. Required because JSF AV Rule 22, MISRA-C:2012
// Rule 21.6, and JPL Rule 1 (transitively via ISO 9899 Appendix J) all
// prohibit <stdio.h> due to its ~30 implementation-defined behaviors.
//
// See docs/decisions/STDIO_REPLACEMENT_PLAN.md for the safety rationale,
// docs/STDIO_TO_RC_LOG_MIGRATION_GUIDE.md for the migration cheat-sheet,
// and C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn.md for the
// R-5 migration plan (Unit B landed rc_log; Units C-J migrate callsites).
//
// CONTRACT (LOCKED at Unit B per council round 1+2):
//
//   - Bounded: each call writes at most kRcLogBufferBytes (128) before
//     truncation. No allocation, no blocking, no error return.
//   - Truncation: if formatted output exceeds the buffer, the message is
//     truncated and an explicit "...\n" marker is appended (within the
//     128-byte budget). Callers do NOT detect their own truncation —
//     callsites are diagnostic, not data-integrity-bearing.
//   - Sink: USB CDC via tud_cdc_write through a non-blocking ring buffer
//     drained by tud_task on Core 0's main loop. If the ring buffer is
//     full (USB CDC backpressure), the message is dropped on the floor.
//     Never blocks the caller. Same pattern as ArduPilot AP_HAL UART putchar.
//   - Format string: printf-style for the public API (rc_log(fmt, ...)).
//     Internal formatting routes through ETL's etl::format_to with the
//     printf string translated mechanically. Supported specs are
//     documented in docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md.
//   - Surface lock: the public API surface (signature + behavior contract)
//     is frozen at Unit B for the duration of R-5 migration (Units C-J).
//     Internal implementation (ETL adapter, ring buffer, format
//     translation) iterates freely as bugs surface.
//
// USAGE:
//
//   rc::rc_log("[FD] PYRO FIRED: DROGUE\n");
//   rc::rc_log("Sensor count = %d, status = %s\n", count, status);
//
// PROHIBITED (use a different mechanism):
//
//   - Hot-path logging (every-tick) — log via telemetry path instead.
//   - ISR / fault-handler context — handlers do not log; the post-boot
//     health_monitor logs prior-fault state.
//   - Output requiring more than 128 bytes — break into multiple calls.

#ifndef ROCKETCHIP_RC_LOG_H
#define ROCKETCHIP_RC_LOG_H

#include <stddef.h>
#include <stdint.h>

namespace rc {

// Fixed per-call stack buffer size. Council round 2 amendment #2: 128 bytes.
// Truncation marker "...\n" (4 bytes) reserved at end-of-buffer when output
// exceeds capacity.
constexpr size_t kRcLogBufferBytes = 128U;

// Primary call surface — printf-shape, bounded, drop-on-overflow.
// Format-string subset supported: see STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md.
// Returns nothing — caller cannot detect truncation or sink unavailability.
void rc_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

}  // namespace rc

#endif  // ROCKETCHIP_RC_LOG_H
