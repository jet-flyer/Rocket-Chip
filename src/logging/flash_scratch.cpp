// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Flash stall scratch loop. Test build only (see flash_scratch.h).

#include "logging/flash_scratch.h"
#include "rocketchip/rc_log.h"

#if defined(ROCKETCHIP_FLASH_STALL_SCRATCH)

#include "logging/flash_gate.h"
#include "logging/flight_table.h"
#include "rocketchip/flash_layout.h"
#include "rocketchip/shared_state.h"  // core1_i2c_pause/resume (R-17)
#include "drivers/i2c_master.h"
#include "ao_flight_director.h"
#include "ao_logger.h"
#include "ao_rcos.h"  // AO_RCOS event queue (backlog guard)
#include "pico/flash.h"
#include "pico/time.h"
#include "hardware/flash.h"

namespace rc {

namespace {

constexpr uint32_t kScratchTimeoutMs = 1000U;  // Same as other flash_safe_execute sites
constexpr uint32_t kScratchFirstSector = kFlashScratchStart / FLASH_SECTOR_SIZE;
constexpr uint32_t kScratchEndSector = kFlashScratchEnd / FLASH_SECTOR_SIZE;
constexpr uint32_t kScratchPages = (kFlashScratchEnd - kFlashScratchStart) / FLASH_PAGE_SIZE;
constexpr uint8_t kScratchFill = 0x5AU;

static_assert((kFlashScratchStart % FLASH_SECTOR_SIZE) == 0U, "Scratch start not sector-aligned");
static_assert(kFlashScratchEnd <= kFlashSafeTestOffset, "Scratch overlaps flash-safe test sector");
static_assert(kFlashScratchStart >= kFlashLogEnd, "Scratch overlaps flight-log region");

enum class ScratchState : uint8_t {
    kIdle = 0,
    kErasing,
    kProgramming,
    kDone,
    kStopped,
};

struct ScratchRun {
    ScratchState state;
    uint32_t sector;       // Next sector to erase (absolute index)
    uint32_t page;         // Next page to program (index in area)
    uint32_t erased;
    uint32_t programmed;
    uint32_t fail;
    uint32_t skipped;      // Ticks with no op because AO_RCOS had a backlog
    uint32_t cycle_max_us; // Core 1 pause + flash op + I2C reset
    uint64_t start_us;
    uint64_t end_us;
};

struct EraseParams {
    uint32_t offset;
    size_t len;
};

struct ProgramParams {
    uint32_t offset;
    const uint8_t* data;
    size_t len;
};

ScratchRun g_scratch{};
uint8_t g_scratchPage[FLASH_PAGE_SIZE];  // SRAM source for flash_range_program

void do_scratch_erase(void* param) {
    auto* p = static_cast<EraseParams*>(param);
    flash_range_erase(p->offset, p->len);
}

void do_scratch_program(void* param) {
    auto* p = static_cast<ProgramParams*>(param);
    flash_range_program(p->offset, p->data, p->len);
}

const char* state_name(ScratchState s) {
    switch (s) {
    case ScratchState::kIdle:        return "idle";
    case ScratchState::kErasing:     return "erase";
    case ScratchState::kProgramming: return "program";
    case ScratchState::kDone:        return "done";
    case ScratchState::kStopped:     return "stopped";
    default:                         return "?";
    }
}

// True if another AO_RCOS event waits behind the tick in progress.
// An erase (45-75 ms) is longer than the 50 ms tick period. Without this
// guard, ticks arrive faster than AO_RCOS handles them, and the 16-deep
// queue overflows (QP assert qf_actq 130, reset in about 1 s).
bool rcos_backlog() {
    const QEvtPtr volatile* const front = &AO_RCOS->eQueue.frontEvt;
    return *front != nullptr;
}

bool running() {
    return (g_scratch.state == ScratchState::kErasing) ||
           (g_scratch.state == ScratchState::kProgramming);
}

// True if a flight-table entry, or the next free sector, is in the area.
bool table_overlaps_area(const FlightTableState* ft) {
    if (ft->table.next_free_sector > kScratchFirstSector) {
        return true;
    }
    uint32_t n = ft->table.entry_count;
    if (n > kMaxFlightEntries) {
        n = kMaxFlightEntries;
    }
    bool overlap = false;
    for (uint32_t i = 0; i < n; ++i) {
        const FlightLogEntry& e = ft->table.entries[i];
        const uint32_t end = e.start_sector + e.sector_count;
        if ((e.start_sector < kScratchEndSector) && (end > kScratchFirstSector)) {
            overlap = true;
        }
    }
    return overlap;
}

void stop(const char* why) {
    g_scratch.state = ScratchState::kStopped;
    g_scratch.end_us = time_us_64();
    rc::rc_log("[SCRATCH] stop: %s\n", why);
}

// One flash op with the LL 31 / R-17 Core 1 I2C wrap. Returns PICO_OK or error.
int run_one_op(bool erase) {
    const uint64_t c0 = time_us_64();
    rc::core1_i2c_pause();
    int r = PICO_ERROR_GENERIC;
    if (erase) {
        EraseParams p{g_scratch.sector * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE};
        const uint64_t t0 = flash_gate_begin();
        r = flash_safe_execute(do_scratch_erase, &p, kScratchTimeoutMs);
        flash_gate_end(FlashOpType::kErase, t0, r);
    } else {
        g_scratchPage[0] = static_cast<uint8_t>(g_scratch.page & 0xFFU);
        g_scratchPage[1] = static_cast<uint8_t>((g_scratch.page >> 8U) & 0xFFU);
        ProgramParams p{kFlashScratchStart + (g_scratch.page * FLASH_PAGE_SIZE),
                        g_scratchPage, FLASH_PAGE_SIZE};
        const uint64_t t0 = flash_gate_begin();
        r = flash_safe_execute(do_scratch_program, &p, kScratchTimeoutMs);
        flash_gate_end(FlashOpType::kProgram, t0, r);
    }
    if (!i2c_master_reset()) {
        rc::rc_log("[WARN] I2C master reset failed after scratch op\n");
    }
    rc::core1_i2c_resume();
    const uint32_t cycle = flash_stall_span_us(c0, time_us_64());
    if (cycle > g_scratch.cycle_max_us) {
        g_scratch.cycle_max_us = cycle;
    }
    return r;
}

} // namespace

void flash_scratch_toggle() {
    if (running()) {
        stop("key");
        return;
    }
    if (!AO_FlightDirector_is_ground_state()) {
        rc::rc_log("[SCRATCH] refused: not ground state\n");
        return;
    }
    const FlightTableState* ft = AO_Logger_get_flight_table();
    if ((ft == nullptr) || !ft->loaded) {
        rc::rc_log("[SCRATCH] refused: flight table not loaded\n");
        return;
    }
    if (table_overlaps_area(ft)) {
        rc::rc_log("[SCRATCH] refused: flight log data in scratch area\n");
        return;
    }
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
        g_scratchPage[i] = kScratchFill;
    }
    g_scratch = ScratchRun{};
    g_scratch.state = ScratchState::kErasing;
    g_scratch.sector = kScratchFirstSector;
    g_scratch.start_us = time_us_64();
    rc::rc_log("[SCRATCH] start area=0x%08lX-0x%08lX sectors=%lu pages=%lu\n",
               static_cast<unsigned long>(kFlashScratchStart),
               static_cast<unsigned long>(kFlashScratchEnd),
               static_cast<unsigned long>(kScratchEndSector - kScratchFirstSector),
               static_cast<unsigned long>(kScratchPages));
}

void flash_scratch_tick() {
    if (!running()) {
        return;
    }
    // Never erase or program while ARMED or in flight.
    if (!AO_FlightDirector_is_ground_state()) {
        stop("not ground state");
        return;
    }
    if (rcos_backlog()) {
        g_scratch.skipped++;
        return;
    }
    if (g_scratch.state == ScratchState::kErasing) {
        if (run_one_op(true) == PICO_OK) {
            g_scratch.erased++;
        } else {
            g_scratch.fail++;
        }
        g_scratch.sector++;
        if (g_scratch.sector >= kScratchEndSector) {
            g_scratch.state = ScratchState::kProgramming;
            rc::rc_log("[SCRATCH] erase done, program phase\n");
        }
        return;
    }
    if (run_one_op(false) == PICO_OK) {
        g_scratch.programmed++;
    } else {
        g_scratch.fail++;
    }
    g_scratch.page++;
    if (g_scratch.page >= kScratchPages) {
        g_scratch.state = ScratchState::kDone;
        g_scratch.end_us = time_us_64();
        rc::rc_log("[SCRATCH] done\n");
    }
}

void flash_scratch_dump() {
    const uint64_t end = running() ? time_us_64() : g_scratch.end_us;
    const uint32_t elapsed_ms = (g_scratch.start_us == 0U)
        ? 0U : (flash_stall_span_us(g_scratch.start_us, end) / 1000U);
    rc::rc_log("[SCRATCH] state=%s erased=%lu/%lu programmed=%lu/%lu fail=%lu\n",
               state_name(g_scratch.state),
               static_cast<unsigned long>(g_scratch.erased),
               static_cast<unsigned long>(kScratchEndSector - kScratchFirstSector),
               static_cast<unsigned long>(g_scratch.programmed),
               static_cast<unsigned long>(kScratchPages),
               static_cast<unsigned long>(g_scratch.fail));
    rc::rc_log("[SCRATCH] cycle_max_us=%lu skipped=%lu elapsed_ms=%lu area=0x%08lX-0x%08lX\n",
               static_cast<unsigned long>(g_scratch.cycle_max_us),
               static_cast<unsigned long>(g_scratch.skipped),
               static_cast<unsigned long>(elapsed_ms),
               static_cast<unsigned long>(kFlashScratchStart),
               static_cast<unsigned long>(kFlashScratchEnd));
}

} // namespace rc

#else

namespace rc {

void flash_scratch_dump() {
    rc::rc_log("[SCRATCH] build=off\n");
}

} // namespace rc

#endif // ROCKETCHIP_FLASH_STALL_SCRATCH
