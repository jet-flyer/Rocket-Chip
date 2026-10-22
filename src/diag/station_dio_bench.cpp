// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station-only FSK FIFO bench. SIO inputs with pull-up. No PIO. Never Tx.
// SX1276 DS Rev. 4 is the register source (repo PDF, March 2015).

#include "diag/station_dio_bench.h"

#include "rocketchip/rc_log.h"

#if defined(ROCKETCHIP_JOB_STATION)

#include "diag/station_dio_eval.h"
#include "active_objects/ao_radio.h"
#include "active_objects/ao_telemetry.h"
#include "active_objects/ao_flight_director.h"
#include "flight_director/flight_state.h"
#include "drivers/spi_bus.h"
#include "rocketchip/board_fruit_jam.h"
#include "hardware/gpio.h"
#include "pico/time.h"

namespace {

enum class Kind : uint8_t {
    kToggle = 0,
    kDepth,
};

enum class Refuse : uint8_t {
    kOk = 0,
    kNoRadio,
    kTx,
    kAir,
};

// RegOpMode (0x01) p.93. LongRangeMode (bit 7) changes only in Sleep.
// 0x00 is FSK, high-frequency, Sleep. 0x01 is FSK Standby.
// This bench never writes FSTx (0x02) or Tx (0x03).
constexpr uint8_t kRegOpMode         = 0x01;
constexpr uint8_t kRegFifo           = 0x00;
constexpr uint8_t kRegPacketConfig2  = 0x31;
constexpr uint8_t kRegFifoThresh     = 0x35;
constexpr uint8_t kRegIrqFlags2      = 0x3F;
constexpr uint8_t kRegDioMapping1    = 0x40;
constexpr uint8_t kOpSleepLora       = 0x80;
constexpr uint8_t kOpSleepFsk        = 0x00;
constexpr uint8_t kOpStandbyFsk      = 0x01;
constexpr uint8_t kDataModePacket    = 0x40;
constexpr uint8_t kClearFifoOverrun  = rc::dio_bench::kIrqFifoOverrun;

// Same settle the LoRa driver uses between mode writes.
constexpr uint32_t kModeSettleMs = 10;

// RegFifoThresh bits [5:0] (p.101). FifoLevel sets at threshold + 1.
// 4 puts that edge at 5 bytes, before FifoFull, so DIO1 rises first.
constexpr uint8_t kBenchFifoThreshold = 4;

// Past both size quotes: 64 bytes (p.66) and 66 bytes (FifoFull, p.104).
constexpr uint8_t kFifoProbeBytes = 80;

constexpr uint8_t kSampleCap = static_cast<uint8_t>(kFifoProbeBytes + 1U);

static_assert(kBenchFifoThreshold <= 63, "FifoThreshold field is 6 bits");
static_assert(kSampleCap == 81, "before sample plus 80 writes");

void log_refuse(Refuse why) {
    if (why == Refuse::kNoRadio) {
        rc::rc_log("DIO bench refused: radio absent\n");
    } else if (why == Refuse::kTx) {
        rc::rc_log("DIO bench refused: radio TX active\n");
    } else if (why == Refuse::kAir) {
        rc::rc_log("DIO bench refused: armed or flying\n");
    }
}

Refuse refuse_reason() {
    const RadioAoState* st = AO_Radio_get_state();
    if (st == nullptr || !st->initialized || !st->radio.initialized) {
        return Refuse::kNoRadio;
    }
    if (st->tx_active) {
        return Refuse::kTx;
    }
    if (!AO_FlightDirector_is_ground_state()) {
        return Refuse::kAir;
    }
    const RxTelemSnapshot* rx = AO_Telemetry_get_rx_state();
    if (rx != nullptr && rx->valid) {
        const uint8_t fs = rx->telem.flight_state;
        const bool idle = fs == static_cast<uint8_t>(rc::FlightPhase::kIdle);
        const bool landed = fs == static_cast<uint8_t>(rc::FlightPhase::kLanded);
        if (!idle && !landed) {
            return Refuse::kAir;
        }
    }
    return Refuse::kOk;
}

void init_dio_inputs() {
    const uint8_t pins[2] = {board::kRadioDio1Pin, board::kRadioDio2Pin};
    for (uint8_t i = 0; i < 2; ++i) {
        gpio_init(pins[i]);
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
        gpio_set_dir(pins[i], GPIO_IN);
        // Plan §7.2: internal pull-up. Fruit Jam is A4, so E9 does not
        // apply. The pull-up still makes an undriven DIO read high.
        gpio_pull_up(pins[i]);
    }
}

bool enter_fsk_standby(uint8_t cs) {
    spi_bus_write_reg(cs, kRegOpMode, kOpSleepLora);
    sleep_ms(kModeSettleMs);
    spi_bus_write_reg(cs, kRegOpMode, kOpSleepFsk);
    sleep_ms(kModeSettleMs);

    // RegPacketConfig2 bit 6 DataMode: 1 is packet mode (p.100).
    uint8_t pc2 = spi_bus_read_reg(cs, kRegPacketConfig2);
    pc2 = static_cast<uint8_t>(pc2 | kDataModePacket);
    spi_bus_write_reg(cs, kRegPacketConfig2, pc2);

    // Table 30 p.69, packet mode, Standby column.
    // DIO1 mapping 00 is FifoLevel. DIO2 mapping 00 is FifoFull.
    // RegDioMapping1 [5:4] and [3:2] are those fields (p.105). 0x00 is both.
    spi_bus_write_reg(cs, kRegDioMapping1, 0x00);
    spi_bus_write_reg(cs, kRegFifoThresh, kBenchFifoThreshold);

    spi_bus_write_reg(cs, kRegOpMode, kOpStandbyFsk);
    sleep_ms(kModeSettleMs);
    return true;
}

void clear_fifo(uint8_t cs) {
    // p.66 and p.104: writing 1 to FifoOverrun clears the flag and the FIFO.
    spi_bus_write_reg(cs, kRegIrqFlags2, kClearFifoOverrun);
}

rc::dio_bench::DioSample read_sample(uint8_t cs, uint8_t n) {
    rc::dio_bench::DioSample s{};
    s.n = n;
    s.irq_flags2 = spi_bus_read_reg(cs, kRegIrqFlags2);
    s.gpio_dio1 = gpio_get(board::kRadioDio1Pin);
    s.gpio_dio2 = gpio_get(board::kRadioDio2Pin);
    return s;
}

void print_sample(const rc::dio_bench::DioSample& s) {
    const unsigned level = rc::dio_bench::irq_level(s.irq_flags2) ? 1U : 0U;
    const unsigned full = rc::dio_bench::irq_full(s.irq_flags2) ? 1U : 0U;
    const unsigned empty = (s.irq_flags2 & rc::dio_bench::kIrqFifoEmpty) ? 1U : 0U;
    rc::rc_log("DIO n=%u g41=%u g42=%u flg=%02x lvl=%u ful=%u emp=%u\n",
               static_cast<unsigned>(s.n),
               s.gpio_dio1 ? 1U : 0U,
               s.gpio_dio2 ? 1U : 0U,
               static_cast<unsigned>(s.irq_flags2),
               level, full, empty);
}

void print_toggle(const rc::dio_bench::ToggleVerdict& v) {
    if (v.pass) {
        rc::rc_log("PASS\n");
        return;
    }
    if (v.mismatch) {
        const char* pin = v.mismatch_dio1 ? "DIO1" : "DIO2";
        rc::rc_log("FAIL mismatch n=%u %s gpio=%u flag=%u\n",
                   static_cast<unsigned>(v.mismatch_n),
                   pin,
                   v.gpio_seen ? 1U : 0U,
                   v.flag_seen ? 1U : 0U);
        return;
    }
    rc::rc_log("FAIL pins not both low and high\n");
}

void print_depth(const rc::dio_bench::DepthVerdict& d) {
    if (d.full_seen && d.overrun_seen) {
        rc::rc_log("Q31 FifoFull at %u bytes (expect 64 or 66), overrun at %u, DIO2 %u\n",
                   static_cast<unsigned>(d.full_at),
                   static_cast<unsigned>(d.overrun_at),
                   d.dio2_at_full ? 1U : 0U);
    } else if (d.full_seen) {
        rc::rc_log("Q31 FifoFull at %u bytes (expect 64 or 66), overrun at none, DIO2 %u\n",
                   static_cast<unsigned>(d.full_at),
                   d.dio2_at_full ? 1U : 0U);
    } else if (d.overrun_seen) {
        rc::rc_log("Q31 FifoFull never (expect 64 or 66), overrun at %u\n",
                   static_cast<unsigned>(d.overrun_at));
    } else {
        rc::rc_log("Q31 FifoFull never (expect 64 or 66), overrun at none\n");
    }
    if (d.fail) {
        rc::rc_log("FAIL FifoFull never set\n");
    }
}

void restore_link() {
    if (AO_Radio_restore_station_link()) {
        rc::rc_log("DIO bench restored LoRa RX\n");
    } else {
        rc::rc_log("DIO bench restore failed\n");
    }
}

void run(Kind kind) {
    if (board::kRadioDio2Pin >= NUM_BANK0_GPIOS) {
        rc::rc_log("DIO bench refused: GPIO bank too small\n");
        return;
    }
    const Refuse why = refuse_reason();
    if (why != Refuse::kOk) {
        log_refuse(why);
        return;
    }

    const RadioAoState* st = AO_Radio_get_state();
    if (st == nullptr) {
        rc::rc_log("DIO bench refused: radio absent\n");
        return;
    }
    const uint8_t cs = st->radio.cs_pin;
    init_dio_inputs();
    if (!enter_fsk_standby(cs)) {
        restore_link();
        return;
    }
    clear_fifo(cs);

    rc::dio_bench::DioSample samples[kSampleCap] = {};
    uint8_t count = 0;
    samples[count] = read_sample(cs, 0);
    print_sample(samples[count]);
    count = static_cast<uint8_t>(count + 1U);

    for (uint8_t n = 1; n <= kFifoProbeBytes; ++n) {
        spi_bus_write_reg(cs, kRegFifo, n);
        samples[count] = read_sample(cs, n);
        print_sample(samples[count]);
        const bool full = rc::dio_bench::irq_full(samples[count].irq_flags2);
        count = static_cast<uint8_t>(count + 1U);
        if (kind == Kind::kToggle && full) {
            break;
        }
    }

    if (kind == Kind::kToggle) {
        print_toggle(rc::dio_bench::eval_dio_toggle(samples, count));
    } else {
        print_depth(rc::dio_bench::eval_fifo_depth(samples, count));
    }
    restore_link();
}

}  // namespace

#endif  // ROCKETCHIP_JOB_STATION

void station_dio_toggle_check() {
#if !defined(ROCKETCHIP_JOB_STATION)
    rc::rc_log("DIO bench refused: station only\n");
#else
    run(Kind::kToggle);
#endif
}

void station_dio_fifo_depth() {
#if !defined(ROCKETCHIP_JOB_STATION)
    rc::rc_log("DIO bench refused: station only\n");
#else
    run(Kind::kDepth);
#endif
}
