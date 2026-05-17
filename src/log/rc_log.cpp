// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// rc_log implementation — hand-rolled printf-subset formatter dispatching
// to etl::to_string for the actual digit-by-digit conversion.
//
// Council decision (2026-05-15, 3-persona focused review — NASA/JPL,
// Professor, ArduPilot, unanimous Approach A): transcript at
// C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-b7c34e2af19a8b3d2.md
//
// Reasoning summary:
//   - One parser to debug, not two (rejects B's printf→{} translator).
//   - GCC __attribute__((format(printf,1,2))) on rc_log.h header works
//     naturally with this implementation.
//   - Smaller unowned surface — depends on etl::to_string (~150 LOC stable)
//     not etl::format_to (~2200 LOC, larger drift surface on ETL upgrades).
//
// Supported format specs (per Unit A's STDIO_FORMAT_SPEC_INVENTORY_2026-05-15.md):
//   %s, %d, %u, %lu, %llu, %zu, %c, %%
//   %02x, %02X, %04X, %08lx, %08lX, %02lX (hex variants)
//   %.Nf, %N.Mf (float with precision/width)
//   width specifiers: %6lu, %3u, %-8s, %-10s, %-20s, %6s, etc.
//   flags: '-' (left-align), '0' (zero-pad), '+' (force sign), ' ' (space-pad-sign)
//
// Not supported (zero usage in inventory):
//   %e, %g, %a, %p, %n, %i (alias for %d)
//   Octal %o is single-callsite; investigate per inventory finding before migration.

#include "rocketchip/rc_log.h"

#include <math.h>
#include <stdarg.h>
#include <string.h>

#include "etl/string.h"
#include "etl/to_string.h"
#include "etl/format_spec.h"

#ifndef ROCKETCHIP_HOST_TEST
#include "tusb.h"
#endif

namespace {

// Truncation marker appended when output exceeds buffer.
constexpr char kTruncMarker[] = "...\n";
constexpr size_t kTruncMarkerLen = sizeof(kTruncMarker) - 1U;  // 4 bytes

// Parsed printf-spec state (per-conversion).
struct ParsedSpec {
    bool left_align = false;
    bool zero_pad = false;
    bool force_sign = false;
    bool space_sign = false;
    uint32_t width = 0;
    uint32_t precision = 0;
    bool has_precision = false;
    enum class LengthMod : uint8_t { kNone, kH, kHH, kL, kLL, kZ, kT, kJ } length = LengthMod::kNone;
    char conversion = '\0';
};

// Parse a printf %-spec starting at *p_inout (which points one PAST the %).
// Advances *p_inout to one past the conversion character. Returns the parsed spec.
// On error (unknown conversion, malformed), returns spec with conversion='\0'.
ParsedSpec parse_spec(const char** p_inout) {
    ParsedSpec out;
    const char* p = *p_inout;

    // Flags
    while (true) {
        if (*p == '-') { out.left_align = true; ++p; }
        else if (*p == '0') { out.zero_pad = true; ++p; }
        else if (*p == '+') { out.force_sign = true; ++p; }
        else if (*p == ' ') { out.space_sign = true; ++p; }
        else if (*p == '#') { ++p; }  // alt-form flag accepted, ignored
        else break;
    }

    // Width (numeric only — varargs '*' not supported per inventory)
    while (*p >= '0' && *p <= '9') {
        out.width = out.width * 10U + static_cast<uint32_t>(*p - '0');
        ++p;
    }

    // Precision
    if (*p == '.') {
        ++p;
        out.has_precision = true;
        while (*p >= '0' && *p <= '9') {
            out.precision = out.precision * 10U + static_cast<uint32_t>(*p - '0');
            ++p;
        }
    }

    // Length modifier
    if (*p == 'h' && *(p+1) == 'h') { out.length = ParsedSpec::LengthMod::kHH; p += 2; }
    else if (*p == 'h') { out.length = ParsedSpec::LengthMod::kH; ++p; }
    else if (*p == 'l' && *(p+1) == 'l') { out.length = ParsedSpec::LengthMod::kLL; p += 2; }
    else if (*p == 'l') { out.length = ParsedSpec::LengthMod::kL; ++p; }
    else if (*p == 'z') { out.length = ParsedSpec::LengthMod::kZ; ++p; }
    else if (*p == 't') { out.length = ParsedSpec::LengthMod::kT; ++p; }
    else if (*p == 'j') { out.length = ParsedSpec::LengthMod::kJ; ++p; }

    // Conversion char
    if (*p != '\0') {
        out.conversion = *p;
        ++p;
    }

    *p_inout = p;
    return out;
}

// Build an etl::format_spec from a parsed printf spec for integer/hex conversion.
etl::format_spec build_etl_spec(const ParsedSpec& spec, bool is_hex_upper, bool is_hex_lower) {
    etl::format_spec fmt;
    fmt.width(spec.width);
    if (spec.zero_pad && !spec.left_align) {
        fmt.fill('0');
    } else {
        fmt.fill(' ');
    }
    if (spec.left_align) {
        fmt.left();
    } else {
        fmt.right();
    }
    if (is_hex_upper || is_hex_lower) {
        fmt.hex();
        fmt.upper_case(is_hex_upper);
    } else {
        fmt.decimal();
    }
    if (spec.has_precision) {
        fmt.precision(spec.precision);
    }
    return fmt;
}

// Format a signed integer with optional + or space flag prefix.
// ETL's format_spec doesn't natively express '+'/' ' flags, so we
// prepend the sign-char manually for non-negative values.
template <typename T>
void format_signed_int(etl::istring& out, T value, const ParsedSpec& spec) {
    etl::format_spec fmt = build_etl_spec(spec, false, false);
    if (value >= T{0}) {
        // Handle leading sign char for + and ' ' flags by reducing the
        // effective width by 1 and pre-appending the char.
        if (spec.force_sign) {
            out.push_back('+');
            if (fmt.get_width() > 0U) { fmt.width(fmt.get_width() - 1U); }
        } else if (spec.space_sign) {
            out.push_back(' ');
            if (fmt.get_width() > 0U) { fmt.width(fmt.get_width() - 1U); }
        }
    }
    etl::to_string(value, out, fmt, true);
}

// Format a string with width/alignment.
void format_string(etl::istring& out, const char* s, const ParsedSpec& spec) {
    if (s == nullptr) { s = "(null)"; }
    size_t slen = strlen(s);
    if (spec.has_precision && spec.precision < slen) {
        slen = spec.precision;
    }
    size_t pad = (spec.width > slen) ? (spec.width - slen) : 0U;
    if (spec.left_align) {
        out.append(s, slen);
        for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
    } else {
        for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
        out.append(s, slen);
    }
}

// Format a float with %.Nf style precision + optional width.
//
// Hand-rolled to match libc printf %.Nf byte-for-byte. ETL's
// add_floating_point uses round-half-away-from-zero (C `::round`) which
// differs from libc/IEEE 754's round-half-to-even on halfway values
// (1.25 → libc gives "1.2", round-away gives "1.3"). ETL also strips
// the sign of negative zero via etl::absolute. Both are real divergences
// from libc that downstream parsers depend on; council acceptance
// criterion #1 (2026-05-15) is fixed here.
//
// Algorithm:
//   1. Handle sign via signbit() (correctly detects negative zero).
//   2. Build absolute value's integer + fractional parts.
//   3. Scale fractional by 10^precision, round-half-to-even via
//      nearbyint() (uses FE_TONEAREST default = round-half-to-even).
//   4. Handle carry from fractional → integer.
//   5. Format "[-]integer.0Npaddedfractional" then apply width/alignment.
//
// NaN / Inf are emitted as "nan" / "inf" / "-inf" (libc convention).
void format_float(etl::istring& out, double value, const ParsedSpec& spec) {
    uint32_t prec = spec.has_precision ? spec.precision : 6U;  // libc default

    // NaN / Inf handling matches libc printf
    if (isnan(value)) {
        // Apply width to "nan" string
        const char* s = "nan";
        size_t pad = (spec.width > 3U) ? (spec.width - 3U) : 0U;
        if (spec.left_align) {
            out.append(s, 3);
            for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
        } else {
            for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
            out.append(s, 3);
        }
        return;
    }
    if (isinf(value)) {
        bool neg = (value < 0.0);
        const char* s = neg ? "-inf" : (spec.force_sign ? "+inf" : (spec.space_sign ? " inf" : "inf"));
        size_t slen = strlen(s);
        size_t pad = (spec.width > slen) ? (spec.width - slen) : 0U;
        if (spec.left_align) {
            out.append(s, slen);
            for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
        } else {
            for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
            out.append(s, slen);
        }
        return;
    }

    // Sign detection. signbit() correctly returns true for -0.0 (unlike
    // value < 0.0, which is false for negative zero).
    bool neg = signbit(value);
    double abs_v = neg ? -value : value;

    // Build the rendered string into a small scratch buffer; apply width
    // padding when we copy to out.
    char tmp[64];
    size_t tmp_len = 0U;

    // Sign char
    if (neg) {
        tmp[tmp_len++] = '-';
    } else if (spec.force_sign) {
        tmp[tmp_len++] = '+';
    } else if (spec.space_sign) {
        tmp[tmp_len++] = ' ';
    }

    // Scale fractional part to integer form: floor(abs_v * 10^prec)
    // Then add 0.5-equivalent via nearbyint for round-half-to-even.
    double scale = 1.0;
    for (uint32_t i = 0; i < prec; ++i) { scale *= 10.0; }
    double scaled = abs_v * scale;
    // nearbyint() uses the current rounding mode, default FE_TONEAREST
    // which IS round-half-to-even per IEEE 754 / C99 §7.12.9.3. This
    // matches libc printf's behavior exactly.
    double rounded = nearbyint(scaled);

    // Split into integer + fractional digits.
    // Use unsigned 64-bit to hold the value safely up to ~10^18.
    unsigned long long int_part;
    unsigned long long frac_part;
    if (prec == 0U) {
        int_part = static_cast<unsigned long long>(rounded);
        frac_part = 0U;
    } else {
        // integer.fractional split: divide by 10^prec
        unsigned long long combined = static_cast<unsigned long long>(rounded);
        unsigned long long denom = 1U;
        for (uint32_t i = 0; i < prec; ++i) { denom *= 10U; }
        int_part = combined / denom;
        frac_part = combined % denom;
    }

    // Render integer part. Manually convert digits (no allocations).
    char int_digits[24];
    size_t int_len = 0U;
    if (int_part == 0U) {
        int_digits[int_len++] = '0';
    } else {
        unsigned long long v = int_part;
        while (v > 0U) {
            int_digits[int_len++] = static_cast<char>('0' + (v % 10U));
            v /= 10U;
        }
        // Reverse
        for (size_t i = 0; i < int_len / 2; ++i) {
            char t = int_digits[i];
            int_digits[i] = int_digits[int_len - 1 - i];
            int_digits[int_len - 1 - i] = t;
        }
    }
    if (tmp_len + int_len < sizeof(tmp)) {
        memcpy(&tmp[tmp_len], int_digits, int_len);
        tmp_len += int_len;
    }

    // Decimal point + fractional digits (only if precision > 0)
    if (prec > 0U && tmp_len + 1U + prec < sizeof(tmp)) {
        tmp[tmp_len++] = '.';
        // Render fractional with leading zeros to `prec` width
        char frac_digits[24];
        size_t frac_len = 0U;
        if (frac_part == 0U) {
            frac_digits[frac_len++] = '0';
        } else {
            unsigned long long v = frac_part;
            while (v > 0U) {
                frac_digits[frac_len++] = static_cast<char>('0' + (v % 10U));
                v /= 10U;
            }
            for (size_t i = 0; i < frac_len / 2; ++i) {
                char t = frac_digits[i];
                frac_digits[i] = frac_digits[frac_len - 1 - i];
                frac_digits[frac_len - 1 - i] = t;
            }
        }
        // Leading zeros if frac_len < prec
        for (uint32_t i = 0; i < prec - frac_len; ++i) {
            tmp[tmp_len++] = '0';
        }
        memcpy(&tmp[tmp_len], frac_digits, frac_len);
        tmp_len += frac_len;
    }

    // Apply width with alignment
    size_t pad = (spec.width > tmp_len) ? (spec.width - tmp_len) : 0U;
    char pad_char = (spec.zero_pad && !spec.left_align) ? '0' : ' ';
    if (spec.left_align) {
        out.append(tmp, tmp_len);
        for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
    } else if (pad_char == '0' && tmp_len > 0 && (tmp[0] == '-' || tmp[0] == '+' || tmp[0] == ' ')) {
        // Zero-pad goes AFTER the sign char to match libc behavior
        out.push_back(tmp[0]);
        for (size_t i = 0; i < pad; ++i) { out.push_back('0'); }
        out.append(tmp + 1, tmp_len - 1);
    } else {
        for (size_t i = 0; i < pad; ++i) { out.push_back(pad_char); }
        out.append(tmp, tmp_len);
    }
}

// Format one printf-style conversion using a va_list-derived argument.
// Returns false if the conversion char is unsupported (caller writes raw spec).
bool format_conversion(etl::istring& out, const ParsedSpec& spec, va_list& args) {
    switch (spec.conversion) {
        case 's': {
            const char* s = va_arg(args, const char*);
            format_string(out, s, spec);
            return true;
        }
        case 'c': {
            int c = va_arg(args, int);
            char ch = static_cast<char>(c);
            // Width/alignment for %c
            size_t pad = (spec.width > 1U) ? (spec.width - 1U) : 0U;
            if (spec.left_align) {
                out.push_back(ch);
                for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
            } else {
                for (size_t i = 0; i < pad; ++i) { out.push_back(' '); }
                out.push_back(ch);
            }
            return true;
        }
        case 'd':
        case 'i': {
            // Sign-extended pull based on length modifier
            switch (spec.length) {
                case ParsedSpec::LengthMod::kLL: {
                    long long v = va_arg(args, long long);
                    format_signed_int(out, v, spec);
                    break;
                }
                case ParsedSpec::LengthMod::kL: {
                    long v = va_arg(args, long);
                    format_signed_int(out, v, spec);
                    break;
                }
                case ParsedSpec::LengthMod::kZ:
                case ParsedSpec::LengthMod::kT:
                case ParsedSpec::LengthMod::kJ: {
                    // ssize_t / ptrdiff_t / intmax_t — fold to long long
                    long long v = va_arg(args, long long);
                    format_signed_int(out, v, spec);
                    break;
                }
                default: {
                    int v = va_arg(args, int);
                    format_signed_int(out, v, spec);
                    break;
                }
            }
            return true;
        }
        case 'u': {
            etl::format_spec fmt = build_etl_spec(spec, false, false);
            switch (spec.length) {
                case ParsedSpec::LengthMod::kLL: {
                    unsigned long long v = va_arg(args, unsigned long long);
                    etl::to_string(v, out, fmt, true);
                    break;
                }
                case ParsedSpec::LengthMod::kL: {
                    unsigned long v = va_arg(args, unsigned long);
                    etl::to_string(v, out, fmt, true);
                    break;
                }
                case ParsedSpec::LengthMod::kZ: {
                    size_t v = va_arg(args, size_t);
                    etl::to_string(static_cast<unsigned long>(v), out, fmt, true);
                    break;
                }
                default: {
                    unsigned int v = va_arg(args, unsigned int);
                    etl::to_string(v, out, fmt, true);
                    break;
                }
            }
            return true;
        }
        case 'x':
        case 'X': {
            bool upper = (spec.conversion == 'X');
            etl::format_spec fmt = build_etl_spec(spec, upper, !upper);
            switch (spec.length) {
                case ParsedSpec::LengthMod::kLL: {
                    unsigned long long v = va_arg(args, unsigned long long);
                    etl::to_string(v, out, fmt, true);
                    break;
                }
                case ParsedSpec::LengthMod::kL: {
                    unsigned long v = va_arg(args, unsigned long);
                    etl::to_string(v, out, fmt, true);
                    break;
                }
                case ParsedSpec::LengthMod::kZ: {
                    size_t v = va_arg(args, size_t);
                    etl::to_string(static_cast<unsigned long>(v), out, fmt, true);
                    break;
                }
                default: {
                    unsigned int v = va_arg(args, unsigned int);
                    etl::to_string(v, out, fmt, true);
                    break;
                }
            }
            return true;
        }
        case 'f': {
            double v = va_arg(args, double);
            format_float(out, v, spec);
            return true;
        }
        case '%': {
            out.push_back('%');
            return true;
        }
        default:
            return false;
    }
}

// Push raw bytes to the output buffer until it fills, then handle truncation.
// Returns true if the buffer is now full (and the truncation marker has been
// appended); subsequent writes should be no-ops.
bool buffer_append(etl::istring& out, const char* src, size_t len) {
    size_t available = out.capacity() - out.size();
    if (available > kTruncMarkerLen) {
        size_t writable = (len < (available - kTruncMarkerLen)) ? len : (available - kTruncMarkerLen);
        out.append(src, writable);
        if (writable < len) {
            // Truncation happened
            out.append(kTruncMarker, kTruncMarkerLen);
            return true;
        }
    } else {
        // Buffer is too full to safely append more — only room for truncation marker if any
        if (available >= kTruncMarkerLen) {
            out.append(kTruncMarker, kTruncMarkerLen);
        }
        return true;
    }
    return false;
}

// Sink wiring. On target, drain to USB CDC via a small ring buffer
// drained by tud_task on Core 0. On host (ROCKETCHIP_HOST_TEST), capture
// into a stub buffer the ctest can inspect.
#ifdef ROCKETCHIP_HOST_TEST
// Host-side capture buffer for ctest assertions.
namespace host_test {
    constexpr size_t kHostCaptureBytes = 4096U;
    static char s_capture[kHostCaptureBytes];
    static size_t s_capture_len = 0U;
}

void emit(const char* buf, size_t len) {
    using namespace host_test;
    size_t writable = (s_capture_len + len <= kHostCaptureBytes) ? len : (kHostCaptureBytes - s_capture_len);
    memcpy(&s_capture[s_capture_len], buf, writable);
    s_capture_len += writable;
}
#else
// Target sink: non-blocking, ring-buffered, drained from qv_idle_bridge.
// Council round 2 amendment #1 (Cubesat): never blocks the caller.
//
// Implementation: 1024-byte ring buffer. emit() copies the message in
// full; if the ring is full, the OLDEST bytes are evicted to make room
// (drop-oldest semantics). rc_log_drain_to_cdc() drains the ring to
// USB CDC via tud_cdc_write/tud_cdc_write_flush from Core 0's main
// loop. While USB CDC is disconnected (host hasn't opened the port +
// asserted DTR yet), bytes accumulate in the ring; when the host
// finally attaches, drain emits whatever the ring currently holds.
//
// Rationale for drop-oldest (not drop-newest):
//   - Boot output is the diagnostic high-value content. Before the host
//     attaches, the ring fills naturally with banner + init logs. When
//     the host attaches, the most recent ~1KB is more useful than the
//     first 1KB followed by silence.
//   - Pattern source: ArduPilot AP_HAL UART putchar (newest wins).
//
// Concurrency: rc_log is called from Core 0 cooperative context only
// (per rc_log.h contract — never from ISR, never from Core 1). Drain
// is also Core 0 cooperative (from qv_idle_bridge). Producer and
// consumer never run concurrently; the volatile head/tail are
// belt-and-braces against compiler reordering, not against
// preemption. Coexistence with SDK's stdio_usb (un-migrated printf
// callers) is byte-stream-level: TinyUSB serializes its own TX FIFO,
// so our tud_cdc_write and the SDK's tud_cdc_write don't corrupt each
// other, they just interleave at byte boundaries — acceptable during
// R-5 migration; resolves at Unit J when stdio_usb is retired.

namespace target_sink {
    // 8192 bytes: sized for the burst-dump pattern.
    // Council 2026-05-16 (NASA/JPL + Prof + ArduPilot + Cubesat,
    // unanimous): diag_stats_dump() emits ~1.3 KB in tight succession
    // faster than the idle-bridge drain can keep up; the prior 1024-byte
    // ring hit drop-oldest mid-dump on the station, losing the T=0
    // Preconditions block (Identity/Radio RegVersion/NVIC_ISPR/SPI
    // error count) — that's a positive-control signal disappearing
    // silently, which is the LL Entry 36 pattern. 8 KB gives forward
    // margin for upcoming R-5 migrations (rc_os_dashboard ANSI render
    // + rc_os_commands.cpp 215-callsite file). 1.3% of 520 KB SRAM.
    //
    // Drop-oldest semantics unchanged — still the safety net for
    // boot-time bursts before host attaches. The two new counters
    // below (s_dropped_bytes + s_high_water) surface ring health so
    // future overflows are observable, not silent.
    //
    // Fault-window note: at 50ms visible-signal delay (per
    // memmanage_fault_handler's fault_reset_with_visible_signal),
    // 8 KB drain at ~150 KB/s USB CDC throughput requires ~55ms.
    // Acceptable because the fault-reset path only fires from kIdle
    // (pad faults) and the ring is typically near-empty at fault
    // time — rc_log is not on hot path.
    constexpr size_t kRingBytes = 8192U;
    static volatile char s_ring[kRingBytes];
    static volatile size_t s_head = 0U;  // producer (rc_log writes here)
    static volatile size_t s_tail = 0U;  // consumer (drain reads here)

    // Council 2026-05-16: mandatory observability counters. Without
    // these, drop-oldest events convert the diag-stats dump from a
    // hard gate into a soft gate (LL Entry 36 pattern). Exposed via
    // diag_stats so soak scripts can detect ring health.
    static volatile uint32_t s_dropped_bytes = 0U;  // cumulative evictions
    static volatile uint32_t s_high_water = 0U;     // max ring_used() seen

    inline size_t ring_used() {
        size_t h = s_head, t = s_tail;
        return (h >= t) ? (h - t) : (kRingBytes - (t - h));
    }
    inline size_t ring_free() { return kRingBytes - 1U - ring_used(); }
}

void emit(const char* buf, size_t len) {
    using namespace target_sink;
    // Drop-oldest: if the message doesn't fit, advance tail (evict
    // oldest bytes) until it does. Producer owns both head and tail
    // here, which is safe because rc_log is single-producer + the
    // drain consumer is also Core 0 cooperative (never preempts the
    // producer). Cap the eviction at message size so a giant message
    // can't loop forever on a small ring.
    if (len >= kRingBytes) {
        // Message larger than the entire ring (shouldn't happen — buf
        // capacity is kRcLogBufferBytes=128). Keep only the tail of
        // the message that fits, evict everything else.
        size_t evicted_pre = len - (kRingBytes - 1U);
        s_dropped_bytes = s_dropped_bytes + static_cast<uint32_t>(evicted_pre);
        buf += evicted_pre;
        len = kRingBytes - 1U;
    }
    size_t avail = ring_free();
    if (avail < len) {
        // Evict (len - avail) oldest bytes by advancing s_tail.
        size_t evict = len - avail;
        s_dropped_bytes = s_dropped_bytes + static_cast<uint32_t>(evict);
        s_tail = (s_tail + evict) % kRingBytes;
    }
    for (size_t i = 0; i < len; ++i) {
        size_t h = s_head;
        s_ring[h] = buf[i];
        s_head = (h + 1U) % kRingBytes;
    }
    // Track high-water mark post-write (after head advanced).
    size_t used_now = ring_used();
    if (used_now > s_high_water) {
        s_high_water = static_cast<uint32_t>(used_now);
    }
}

// Ring-health observability getters (council 2026-05-16, mandatory).
// Defined inside the anonymous namespace block so they can read the
// static volatile counters in target_sink. Declared extern "C" with
// `__attribute__((used))` so the symbols are emitted with their
// rc_log.h signatures despite living in an anonymous namespace.
extern "C" __attribute__((used))
uint32_t rc_log_dropped_bytes(void) {
    return target_sink::s_dropped_bytes;
}
extern "C" __attribute__((used))
uint32_t rc_log_high_water(void) {
    return target_sink::s_high_water;
}
#endif

// Shared parse+format loop for rc_log (CDC sink) and rc_snprintf (buf sink).
void format_into(etl::istring& out, const char* fmt, va_list args) {
    bool truncated = false;
    const char* p = fmt;
    while (*p != '\0' && !truncated) {
        if (*p == '%' && *(p + 1) != '\0') {
            ++p;  // Skip '%'
            ParsedSpec spec = parse_spec(&p);
            if (spec.conversion == '\0') {
                // Malformed spec — write '%' literal and continue
                if (buffer_append(out, "%", 1U)) { truncated = true; break; }
                continue;
            }
            bool ok = format_conversion(out, spec, args);
            if (!ok) {
                // Unsupported conversion — write the raw % + conv for visibility.
                char tmp[3] = { '%', spec.conversion, '\0' };
                if (buffer_append(out, tmp, 2U)) { truncated = true; break; }
            }
            // Truncation guard: if conversion overflowed the buffer beyond
            // the marker reserve, the buffer is full.
            if (out.size() + kTruncMarkerLen >= out.capacity()) {
                size_t avail = out.capacity() - out.size();
                if (avail >= kTruncMarkerLen) {
                    out.append(kTruncMarker, kTruncMarkerLen);
                }
                truncated = true;
                break;
            }
        } else {
            // Literal char (or trailing % with no spec, which is allowed: just emit %)
            if (buffer_append(out, p, 1U)) { truncated = true; break; }
            ++p;
        }
    }
}

}  // namespace (anonymous)

namespace rc {

void rc_log(const char* fmt, ...) {
    // Per-call stack buffer for formatted output.
    etl::string<kRcLogBufferBytes> buf;

    va_list args;
    va_start(args, fmt);
    format_into(buf, fmt, args);
    va_end(args);

    // Emit to sink (USB CDC ring on target, capture buffer on host).
    // Note: no inline drain here — drain runs from qv_idle_bridge in
    // Core 0's main loop. While USB CDC is disconnected (host hasn't
    // attached yet), bytes accumulate in the ring with drop-oldest
    // semantics, so the most-recent boot output is preserved for the
    // host to see when it finally attaches and asserts DTR.
    emit(buf.data(), buf.size());
}

constexpr size_t kRcSnprintfMaxBytes = 256U;

size_t rc_snprintf(char* buf, size_t n, const char* fmt, ...) {
    if (buf == nullptr || n == 0U) {
        return 0U;
    }
    etl::string<kRcSnprintfMaxBytes> work;
    va_list args;
    va_start(args, fmt);
    format_into(work, fmt, args);
    va_end(args);

    // Copy into caller buffer, truncating at n-1 to leave room for NUL.
    size_t writable = (work.size() < (n - 1U)) ? work.size() : (n - 1U);
    if (writable > 0U) {
        memcpy(buf, work.data(), writable);
    }
    buf[writable] = '\0';
    return writable;
}

void strbuf_init(strbuf* sb, char* buf, size_t cap) {
    sb->buf = buf;
    sb->cap = cap;
    sb->pos = 0U;
    sb->overflow = (buf == nullptr || cap == 0U);
    if (!sb->overflow) {
        buf[0] = '\0';
    }
}

void strbuf_printf(strbuf* sb, const char* fmt, ...) {
    if (sb == nullptr) {
        return;
    }
    if (sb->buf == nullptr || sb->cap == 0U) {
        sb->overflow = true;
        return;
    }
    if (sb->overflow) {
        return;
    }
    if (sb->pos + 1U >= sb->cap) {
        sb->overflow = true;
        return;
    }
    etl::string<kRcSnprintfMaxBytes> work;
    va_list args;
    va_start(args, fmt);
    format_into(work, fmt, args);
    va_end(args);

    size_t remaining = sb->cap - sb->pos - 1U;
    size_t writable = (work.size() < remaining) ? work.size() : remaining;
    if (writable > 0U) {
        memcpy(sb->buf + sb->pos, work.data(), writable);
        sb->pos += writable;
    }
    sb->buf[sb->pos] = '\0';
    if (writable < work.size()) {
        sb->overflow = true;
    }
}

}  // namespace rc

// ===========================================================================
// Sink drain — called by Core 0's main loop after tud_task to flush the ring
// into USB CDC. Non-blocking; only writes what the CDC has room for.
// ===========================================================================
#ifndef ROCKETCHIP_HOST_TEST
extern "C" void rc_log_drain_to_cdc(void) {
    using namespace target_sink;
    // Ring-empty fast path: when called from qv_idle_bridge every
    // idle tick, the ring is empty most of the time. Skip all
    // TinyUSB calls in that case to avoid mutex/IRQ contention with
    // the SDK's stdio_usb background task. Empirically required
    // 2026-05-16: every-idle-tick TinyUSB calls were correlated with
    // Core 1's I2C transactions failing (ICM-20948 stopped ACKing
    // mid-session). Reading two volatile size_t and comparing is
    // ~3 cycles, vs ~hundreds of cycles for a tud_cdc_* call.
    if (s_head == s_tail) {
        return;
    }
    // Hold-on-disconnect: if host hasn't opened the port + asserted
    // DTR yet, leave bytes in the ring. emit() applies drop-oldest
    // when the ring fills, so the most-recent boot output is what
    // the host sees on first attach. Returning early here (instead
    // of discarding s_tail = s_head) is the change that fixes boot
    // output loss.
    if (!tud_cdc_connected()) {
        return;
    }
    bool wrote_any = false;
    size_t cdc_avail = tud_cdc_write_available();
    while (cdc_avail > 0U && ring_used() > 0U) {
        size_t t = s_tail;
        // Compute contiguous span from tail to either head or end-of-ring
        size_t end = (s_head < t) ? kRingBytes : s_head;
        size_t span = end - t;
        if (span > cdc_avail) { span = cdc_avail; }
        // Cast away volatile for the const-data read; safe because we're the
        // only consumer and the bytes have been written by emit() already.
        uint32_t written = tud_cdc_write(const_cast<const char*>(reinterpret_cast<const volatile char*>(&s_ring[t])),
                                         static_cast<uint32_t>(span));
        if (written > 0U) {
            s_tail = (t + written) % kRingBytes;
            cdc_avail -= written;
            wrote_any = true;
        }
        if (written < span) { break; }  // CDC stalled — yield to next drain tick
    }
    // Push the TinyUSB TX FIFO toward the wire. Per pico SDK's
    // stdio_usb_out_chars pattern (sdk/2.2.0/.../stdio_usb.c:113).
    // tud_task() itself is driven by the SDK's IRQ-driven background
    // task (PICO_STDIO_USB_ENABLE_IRQ_BACKGROUND_TASK=1 by default),
    // so we don't call tud_task() here — that would race the SDK's
    // own mutex-protected invocation. tud_cdc_write_flush() is
    // thread-safe and just schedules the FIFO for transmission.
    if (wrote_any) {
        tud_cdc_write_flush();
    }
}
#endif

// ===========================================================================
// Host-test capture API — only compiled in ROCKETCHIP_HOST_TEST builds.
// ===========================================================================
#ifdef ROCKETCHIP_HOST_TEST
namespace rc {
namespace rc_log_test {
extern "C" const char* host_capture_data() { return ::host_test::s_capture; }
extern "C" size_t host_capture_size() { return ::host_test::s_capture_len; }
extern "C" void host_capture_reset() { ::host_test::s_capture_len = 0; }
}
}
// Host stubs for ring-health getters (no real ring on host).
extern "C" uint32_t rc_log_dropped_bytes(void) { return 0U; }
extern "C" uint32_t rc_log_high_water(void) { return 0U; }
#endif
