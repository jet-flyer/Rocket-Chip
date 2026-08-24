// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Bounded printf-shape log — no <stdio.h> (JSF AV 22 / MISRA 21.6).
// Rationale: docs/decisions/STDIO_REPLACEMENT_PLAN.md
// Format specs: docs/audits/STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md
//
// Each call ≤ kRcLogBufferBytes (128), then "...\n" truncation. No alloc,
// never blocks, no error return. Ring full → drop-oldest (evict oldest
// bytes so the new write lands). Drain from Core 0 idle:
// rc_log_drain_to_cdc in qv_idle_bridge — not tud_task.
// Not for hot-path, ISR, or Core 1. Q_onError is the exception
// (Q_NORETURN; IRQs already off; the preempted producer never resumes).

#ifndef ROCKETCHIP_RC_LOG_H
#define ROCKETCHIP_RC_LOG_H

#include <stddef.h>
#include <stdint.h>

namespace rc {

// Per-call stack buffer. Truncation marker "...\n" fits inside this.
constexpr size_t kRcLogBufferBytes = 128U;

// Primary call surface — printf-shape, bounded, drop-on-overflow.
// Format-string subset supported: see STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md.
// Returns nothing — caller cannot detect truncation or sink unavailability.
void rc_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Buffer-bound sibling of rc_log. Formats into a 256-byte work buffer
// (that cap stamps "...\n"). Copies min(work, n-1) into buf and NUL-
// terminates. Return is payload bytes, not including NUL. n smaller
// than work cuts the copy; it does not restamp the marker. n==0 or
// buf==nullptr returns 0.
size_t rc_snprintf(char* buf, size_t n, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

// Append-format helper for composing multi-piece output into one buffer
// (e.g. ANSI dashboard frame). Models Linux seq_file: sticky overflow,
// no underflow risk, callers never compute (cap - pos) themselves.
struct strbuf {
    char*  buf;
    size_t cap;
    size_t pos;
    bool   overflow;
};

void strbuf_init(strbuf* sb, char* buf, size_t cap);
void strbuf_printf(strbuf* sb, const char* fmt, ...)
    __attribute__((format(printf, 2, 3)));
inline size_t strbuf_len(const strbuf* sb) { return sb->pos; }
inline bool   strbuf_overflowed(const strbuf* sb) { return sb->overflow; }

// Target CDC ring (drop-oldest).
constexpr size_t kRcLogRingBytes = 8192U;

}  // namespace rc

// Drain the rc_log ring buffer to USB CDC. Non-blocking; only writes bytes
// the CDC has room for. Must be called periodically from Core 0's main
// loop (e.g., from qv_idle_bridge in main.cpp) — without it, rc_log
// output queues into the ring and is never emitted to the wire.
extern "C" void rc_log_drain_to_cdc(void);

// Ring health (HW_GATE Rule 1 / LL 36): dropped_bytes > 0 means output
// was lost; high_water approaching rc::kRcLogRingBytes means the ring is tight.
extern "C" uint32_t rc_log_dropped_bytes(void);
extern "C" uint32_t rc_log_high_water(void);

#endif  // ROCKETCHIP_RC_LOG_H
