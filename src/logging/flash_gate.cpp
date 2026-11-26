// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Flash gate: time each flash_safe_execute call and each radio poll gap.

#include "logging/flash_gate.h"
#include "diag/qf_tick.h"
#include "logging/flash_scratch.h"
#include "rocketchip/rc_log.h"
#include "pico/flash.h"
#include "pico/time.h"
#include "hardware/flash.h"
#include "qp_port.h"

// QP registry of started AOs (qp_pkg.h). Read-only, for the queue line.
extern "C" QActive* QActive_registry_[QF_MAX_ACTIVE + 1U];

namespace rc {

namespace {

// JEDEC Read ID: opcode 9Fh, then manufacturer, memory type, capacity.
constexpr uint8_t kJedecReadIdCmd = 0x9FU;
constexpr size_t kJedecXferLen = 4U;
constexpr uint32_t kJedecTimeoutMs = 1000U;  // Same as other flash_safe_execute sites

struct JedecXfer {
    uint8_t rx[kJedecXferLen];
};

// Core 0 only (all writers run in Core 0 task context).
FlashStallStats g_flashStall{};
PollGapStats g_radioPoll{};
PollGapStats g_macPoll{};
JedecXfer g_jedec{};
bool g_jedecRead = false;
bool g_jedecOk = false;

void do_read_jedec(void* param) {
    auto* x = static_cast<JedecXfer*>(param);
    // tx is on the stack (SRAM). flash_do_cmd saves and restores the QMI
    // state, the same as flash_range_erase (PSRAM CS1 stays set).
    const uint8_t tx[kJedecXferLen] = {kJedecReadIdCmd, 0U, 0U, 0U};
    flash_do_cmd(tx, x->rx, kJedecXferLen);
}

void print_jedec() {
    if (!g_jedecRead) {
        rc::rc_log("[FLASH] jedec=not-read\n");
        return;
    }
    if (!g_jedecOk) {
        rc::rc_log("[FLASH] jedec=read-fail\n");
        return;
    }
    const uint8_t mfr = g_jedec.rx[1];
    const uint8_t typ = g_jedec.rx[2];
    const uint8_t cap = g_jedec.rx[3];
    const FlashPart* part = flash_part_from_jedec(mfr, typ, cap);
    if (part == nullptr) {
        rc::rc_log("[FLASH] jedec=%02X%02X%02X part=unknown\n",
                   static_cast<unsigned>(mfr), static_cast<unsigned>(typ),
                   static_cast<unsigned>(cap));
        return;
    }
    rc::rc_log("[FLASH] jedec=%02X%02X%02X part=%s prog_us=%lu/%lu erase_us=%lu/%lu (typ/max)\n",
               static_cast<unsigned>(mfr), static_cast<unsigned>(typ),
               static_cast<unsigned>(cap), part->name,
               static_cast<unsigned long>(part->prog_typ_us),
               static_cast<unsigned long>(part->prog_max_us),
               static_cast<unsigned long>(part->erase_typ_us),
               static_cast<unsigned long>(part->erase_max_us));
}

void print_op(FlashOpType type) {
    const FlashOpStats& s = g_flashStall.op[static_cast<size_t>(type)];
    const char* name = flash_op_type_name(type);
    rc::rc_log("[FLASH] %s n=%lu fail=%lu max_us=%lu total_ms=%lu\n", name,
               static_cast<unsigned long>(s.count),
               static_cast<unsigned long>(s.fail),
               static_cast<unsigned long>(s.max_us),
               static_cast<unsigned long>(s.total_us / 1000U));
    rc::rc_log("[FLASH] %s hist_ms <1=%lu 1-3=%lu 3-10=%lu 10-50=%lu 50-400=%lu 400+=%lu\n",
               name,
               static_cast<unsigned long>(s.bins[0]),
               static_cast<unsigned long>(s.bins[1]),
               static_cast<unsigned long>(s.bins[2]),
               static_cast<unsigned long>(s.bins[3]),
               static_cast<unsigned long>(s.bins[4]),
               static_cast<unsigned long>(s.bins[5]));
}

// Low-water mark of free event-queue slots per AO priority since boot.
// 0 means that queue was full one time (the next post is a QP assert).
// "-" means no AO at that priority (station has no FD).
void print_queue_min() {
    char line[96];
    size_t pos = 0U;
    pos = rc::rc_snprintf(line, sizeof(line), "[QUEUE] min_free");
    for (uint32_t p = 1U; (p <= QF_MAX_ACTIVE) && (pos < sizeof(line)); ++p) {
        const QActive* const a = QActive_registry_[p];
        size_t n = 0U;
        if (a == nullptr) {
            n = rc::rc_snprintf(&line[pos], sizeof(line) - pos, " p%lu=-",
                         static_cast<unsigned long>(p));
        } else {
            n = rc::rc_snprintf(&line[pos], sizeof(line) - pos, " p%lu=%u",
                         static_cast<unsigned long>(p),
                         static_cast<unsigned>(a->eQueue.nMin));
        }
        pos += n;
    }
    rc::rc_log("%s\n", line);
}

void print_tick() {
    const TickSkipStats t = qf_tick_stats();
    rc::rc_log("[TICK] skipped=%lu max_skip=%lu bursts=%lu fired=%lu\n",
               static_cast<unsigned long>(t.skipped_total),
               static_cast<unsigned long>(t.max_skip),
               static_cast<unsigned long>(t.bursts),
               static_cast<unsigned long>(t.fired));
}

void print_poll(const char* name, const PollGapStats& g) {
    rc::rc_log("[POLL] %s n=%lu max_gap_us=%lu gt10ms=%lu gt100ms=%lu\n", name,
               static_cast<unsigned long>(g.calls),
               static_cast<unsigned long>(g.max_gap_us),
               static_cast<unsigned long>(g.over_warn),
               static_cast<unsigned long>(g.over_long));
}

} // namespace

uint64_t flash_gate_begin() {
    return time_us_64();
}

void flash_gate_end(FlashOpType type, uint64_t start_us, int sdk_result) {
    const uint64_t end_us = time_us_64();
    flash_stall_note(g_flashStall, type, flash_stall_span_us(start_us, end_us),
                     sdk_result == PICO_OK);
}

void flash_gate_read_jedec() {
    if (g_jedecRead) {
        return;
    }
    const uint64_t t0 = flash_gate_begin();
    const int r = flash_safe_execute(do_read_jedec, &g_jedec, kJedecTimeoutMs);
    flash_gate_end(FlashOpType::kCommand, t0, r);
    g_jedecRead = true;
    g_jedecOk = (r == PICO_OK);
    print_jedec();
}

void flash_gate_note_radio_poll() {
    poll_gap_note(g_radioPoll, time_us_64());
}

void flash_gate_note_mac_poll() {
    poll_gap_note(g_macPoll, time_us_64());
}

void flash_gate_dump() {
    rc::rc_log("\n=== Flash stall (since boot) ===\n");
    print_jedec();
    print_op(FlashOpType::kErase);
    print_op(FlashOpType::kProgram);
    print_op(FlashOpType::kCommand);
    print_poll("radio_rx", g_radioPoll);
    print_poll("mac", g_macPoll);
    print_queue_min();
    print_tick();
    flash_scratch_dump();
}

} // namespace rc
