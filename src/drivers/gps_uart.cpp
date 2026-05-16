// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file gps_uart.cpp
 * @brief UART GPS driver using lwGPS library — interrupt-driven receive
 *
 * Reads NMEA sentences from a UART GPS module (e.g., Adafruit Ultimate GPS
 * FeatherWing #3133 / PA1616D) and parses with lwGPS.
 *
 * Architecture mirrors gps_pa1010d.cpp:
 *   - Same lwGPS parser (NMEA parsing is transport-agnostic)
 *   - Same gps_data_t output (transport-neutral, defined in gps.h)
 *   - PMTK commands sent as precomputed const-array sentences (R-2 / R-5 Unit D)
 *
 * Key differences from I2C backend:
 *   - No padding filter (UART gives clean bytes, no 0x0A padding)
 *   - No settling delay (point-to-point, no bus contention)
 *   - Interrupt-driven RX with 512-byte ring buffer (no FIFO overflow)
 *   - 2-second presence detection timeout at init
 *
 * Receive path:
 *   GPS module (9600 baud) -> UART0 hardware FIFO (32 bytes)
 *   -> ISR on Core 0 (drains FIFO -> ring buffer)
 *   -> gps_uart_drain() on Core 1 (drains ring buffer -> lwGPS)
 *   -> gps_uart_update() at 10Hz (drain + extract gps_data_t)
 *
 * Prior Art:
 *   - Adafruit Ultimate GPS FeatherWing product page, MT3339 datasheet
 *   - ArduPilot AP_HAL::UARTDriver (DMA + ring buffer pattern)
 *   - Pico SDK stdio_uart.c (IRQ handler pattern)
 */

#include "gps_uart.h"
#include "rocketchip/board.h"
#include "lwgps/lwgps.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include <string.h>

// ============================================================================
// Constants
// ============================================================================

// UART instance — uart0 on GPIO0/1 per Feather pinout.
// uart0 is a macro (reinterpret_cast), not constexpr — use #define.
#define GPS_UART_INST uart0

// NMEA protocol constants
constexpr char     kNmeaStart       = '$';
constexpr uint16_t kGpsYearBase     = 2000;
constexpr uint8_t  kGsaFixMode3d    = 3;
constexpr uint8_t  kGsaFixMode2d    = 2;

// PMTK251 baud rate negotiation
// MT3339 switches baud immediately on receiving PMTK251 — ACK arrives at the
// NEW baud rate, so it won't be readable at the old rate. Use a delay instead.
// Source: GlobalTop PMTK_A11 spec; Adafruit_GPS uses 1000ms delay after PMTK251.
// 57600 chosen: 5760 B/s / ~200 B per 10Hz burst = 28 Hz capacity (2.8× headroom).
// At 115200 the MT3339 PA1616D is documented to work but some units are unreliable
// above 57600 (Adafruit forum reports). 57600 is the safe high-speed choice.
// Source: Adafruit Ultimate GPS product page, user forum thread #71672.
constexpr uint32_t kGpsBaudNegotiateDelayMs = 250;  // ms — module stabilize time

// Init: presence detection timeout
// MT3339 outputs NMEA at 1Hz default — 2 seconds guarantees at least one sentence.
constexpr uint32_t kInitTimeoutUs   = 2000000; // 2 seconds

// ============================================================================
// R-2 absorbed (R-5 Unit D part 2b, 2026-05-16, council-approved): the PMTK
// command sentences are precomputed const arrays with compile-time checksum
// + length verification via static_assert. No snprintf in the file. Same
// pattern as gps_pa1010d.cpp Unit D part 2a (commit c71090e).
//
// Latent bug fix (HW_GATE_DISCIPLINE Rule 7 — bugs surfaced by verification
// are in-scope for the PR that surfaced them): the prior `negotiate_baud()`
// called `gps_uart_send_command()` BEFORE `g_initialized = true`, which
// `gps_uart_send_command` early-returned on `if (!g_initialized) return
// false`. The PMTK251 baud-change command was therefore never sent over the
// wire — the GPS module stayed at 9600 baud while the host UART switched to
// 57600. Result: rxOvf accumulated and lwGPS parser saw mostly garbage
// (pre-migration baseline: RMC=V GGA=0 GSA=0 rxOvf=776+). With const-array
// `uart_write_blocking` calls there is no `g_initialized` early-return path
// to trip over; the bug evaporates.
// ============================================================================

// Compile-time NMEA checksum: XOR of all bytes between '$' and '*' (exclusive).
constexpr uint8_t nmea_checksum_constexpr(const char* body) {
    uint8_t c = 0;
    while (*body != '\0') {
        c ^= static_cast<uint8_t>(*body);
        ++body;
    }
    return c;
}

// PMTK251,57600 — switch MT3339 to 57600 baud.
constexpr char kPmtk251_57600Body[] = "PMTK251,57600";
constexpr char kPmtk251_57600Sentence[] = "$PMTK251,57600*2C\r\n";
static_assert(nmea_checksum_constexpr(kPmtk251_57600Body) == 0x2C,
              "PMTK251,57600 checksum mismatch — verify literal matches sentence body");
static_assert(sizeof(kPmtk251_57600Sentence) - 1 == 19,
              "PMTK251,57600 sentence byte length mismatch");

// PMTK314 — enable RMC + GGA + GSA + GSV sentences (rest disabled).
// Same literal as gps_pa1010d's kPmtk314Sentence; kept separate per the
// council Tier 2 surgical-scope principle (don't share constants between
// driver TUs to avoid hidden coupling).
constexpr char kPmtk314Body[] = "PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
constexpr char kPmtk314Sentence[] =
    "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
static_assert(nmea_checksum_constexpr(kPmtk314Body) == 0x29,
              "PMTK314 checksum mismatch — verify literal matches sentence body");
static_assert(sizeof(kPmtk314Sentence) - 1 == 51,
              "PMTK314 sentence byte length mismatch");

// PMTK220,200 — set NMEA output interval to 200ms = 5 Hz.
// See AGENT_WHITEBOARD.md "UART GPS 10Hz + sticky-baud fix" for the
// active 10Hz investigation.
constexpr char kPmtk220_5HzBody[] = "PMTK220,200";
constexpr char kPmtk220_5HzSentence[] = "$PMTK220,200*2C\r\n";
static_assert(nmea_checksum_constexpr(kPmtk220_5HzBody) == 0x2C,
              "PMTK220,200 checksum mismatch — verify literal matches sentence body");
static_assert(sizeof(kPmtk220_5HzSentence) - 1 == 17,
              "PMTK220,200 sentence byte length mismatch");

// ============================================================================
// Interrupt-Driven Ring Buffer
// ============================================================================
//
// SPSC (single-producer single-consumer) lock-free ring buffer.
//
// Producer: gps_uart_rx_isr() on Core 0 (UART0 IRQ)
//   - Writes g_rxBuf[head], then advances g_rxHead
//   - Reads g_rxTail to check if buffer is full
//
// Consumer: gps_uart_drain() on Core 1 (polled at 10Hz+)
//   - Snapshots g_rxHead, reads g_rxBuf[tail..head], then advances g_rxTail
//
// Thread safety:
//   - volatile uint32_t for head/tail — ARM Cortex-M33 naturally-aligned
//     32-bit writes are atomic (ARMv8-M architecture guarantee)
//   - ISR writes buffer entry BEFORE advancing head; consumer reads head
//     BEFORE reading buffer entries
//   - RP2350 SRAM is cache-coherent across cores (no L1 data caches on
//     Cortex-M33) — volatile is sufficient, no __dmb() needed
//   - lwGPS state (g_gps, g_data) only touched by Core 1 — no cross-core
//     access on parser state
//
// Sizing: 512 bytes (power-of-2 for efficient masking).
//   At 57600 baud (~5760 B/s), 10Hz poll = ~576 bytes/interval worst case.
//   In practice each 10Hz burst is ~200 bytes (GGA+RMC+GSA sentences).
//   512 bytes holds 2+ full bursts; ISR drains FIFO faster than burst rate.
//   Overflow would only occur if Core 1 stalls for >88ms — not possible at 200Hz loop.
//
// Ref: ArduPilot ByteBuffer, Pico SDK stdio_uart.c, Linux serial core.

constexpr uint32_t kRxBufSize = 512;
constexpr uint32_t kRxBufMask = kRxBufSize - 1;
static_assert((kRxBufSize & kRxBufMask) == 0, "Ring buffer size must be power of 2");

static uint8_t           g_rxBuf[kRxBufSize];
static volatile uint32_t g_rxHead     = 0;  // Written by ISR, read by consumer
static volatile uint32_t g_rxTail     = 0;  // Written by consumer, read by ISR
static volatile uint32_t g_rxOverflow = 0;  // Overflow counter (diagnostic)

// ============================================================================
// Private State
// ============================================================================

static bool g_initialized = false;
static lwgps_t g_gps;
static gps_data_t g_data;

// ============================================================================
// ISR
// ============================================================================

/**
 * @brief UART0 RX interrupt handler — drains hardware FIFO into ring buffer
 *
 * Runs on Core 0 (where gps_uart_init() registered it). Fires on:
 *   - UARTRXINTR: RX FIFO reaches threshold (>= 4 bytes, default IFLS)
 *   - UARTRTINTR: >= 1 byte and no new bytes for 32 bit periods (~3.3ms at 9600)
 *
 * At 9600 baud, fires at most ~240 times/sec. Each invocation is <1us.
 * Total Core 0 CPU impact: <0.1%.
 */
static void gps_uart_rx_isr() {
    while (uart_is_readable(GPS_UART_INST)) {
        uint8_t byte = static_cast<uint8_t>(uart_get_hw(GPS_UART_INST)->dr);

        uint32_t head = g_rxHead;
        uint32_t next = (head + 1) & kRxBufMask;

        if (next == g_rxTail) {
            // Buffer full — drop byte, count overflow
            g_rxOverflow = g_rxOverflow + 1;
            continue;
        }

        g_rxBuf[head] = byte;
        g_rxHead = next;
    }
}

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Update internal data structure from lwGPS parser state
 *
 * Duplicated from gps_pa1010d.cpp — both backends produce identical
 * gps_data_t from the same lwGPS state. Factoring into a shared helper
 * would couple two otherwise-independent drivers for ~40 lines of
 * trivial field copies. Not worth the dependency.
 */
static void update_data_from_lwgps() {
    g_data.latitude = g_gps.latitude;
    g_data.longitude = g_gps.longitude;
    g_data.altitudeM = static_cast<float>(g_gps.altitude);

    g_data.speedKnots = static_cast<float>(g_gps.speed);
    g_data.speedMps = static_cast<float>(lwgps_to_speed(g_gps.speed, LWGPS_SPEED_MPS));
    g_data.courseDeg = static_cast<float>(g_gps.course);

    // Fix type: prefer GGA fix quality, fall back to GSA fixMode for 2D/3D.
    // GSA fixMode==2 is 2D; anything else (3, or not yet updated) → 3D.
    if (g_gps.fix >= 1) {
        if (g_gps.fix_mode == kGsaFixMode2d) {
            g_data.fix = GPS_FIX_2D;
        } else {
            g_data.fix = GPS_FIX_3D;
        }
    } else {
        g_data.fix = GPS_FIX_NONE;
    }

    g_data.satellites = g_gps.sats_in_use;
    g_data.hdop = static_cast<float>(g_gps.dop_h);
    g_data.vdop = static_cast<float>(g_gps.dop_v);
    g_data.pdop = static_cast<float>(g_gps.dop_p);

    g_data.hour = g_gps.hours;
    g_data.minute = g_gps.minutes;
    g_data.second = g_gps.seconds;

    g_data.day = g_gps.date;
    g_data.month = g_gps.month;
    g_data.year = kGpsYearBase + g_gps.year;

    // Valid if GGA reports a fix (quality >= 1) and fix type is 2D or 3D.
    // Deliberately does NOT require RMC=A — RMC can lag GGA by several sentences
    // on first acquisition, causing G=N even with a genuine 3D lock. GGA fix
    // quality is the authoritative positional validity indicator (ArduPilot pattern).
    g_data.valid = (g_gps.fix >= 1) && (g_data.fix >= GPS_FIX_2D);
    g_data.timeValid = (g_gps.time_valid != 0U);
    g_data.dateValid = (g_gps.date_valid != 0U);

    g_data.ggaFix = g_gps.fix;
    g_data.gsaFixMode = g_gps.fix_mode;
    g_data.rmcValid = lwgps_is_valid(&g_gps);
}

// ============================================================================
// Private Helpers
// ============================================================================

// Negotiate UART baud rate with MT3339.
// Must be called AFTER presence detection (GPS is confirmed present and talking).
// Sends PMTK251 at current baud, waits for module to switch, then reinits
// host UART at new baud. IRQ must NOT be enabled yet (called before IRQ setup).
//
// MT3339 behavior: switches baud immediately on receiving PMTK251. ACK arrives
// at the new rate — ignore it. Use delay as synchronization.
// Source: GlobalTop PMTK_A11; Adafruit_GPS library pattern.
//
// R-5 Unit D part 2b: writes the precomputed kPmtk251_57600Sentence directly
// via uart_write_blocking. The prior `gps_uart_send_command(cmd)` call here
// was dead — see file-top comment about the latent g_initialized bug.
static void negotiate_baud_to_57600() {
    // Send precomputed PMTK251,57600 at current baud rate. No checksum
    // computation, no snprintf — the bytes are byte-identical-to-intent
    // (static_assert at file scope verifies).
    uart_write_blocking(
        GPS_UART_INST,
        reinterpret_cast<const uint8_t*>(kPmtk251_57600Sentence),
        sizeof(kPmtk251_57600Sentence) - 1);

    // Wait for module to switch — ACK arrives at new rate, don't try to read it
    busy_wait_ms(kGpsBaudNegotiateDelayMs);

    // Reinit host UART at new baud
    uart_deinit(GPS_UART_INST);
    uart_init(GPS_UART_INST, 57600);
    gpio_set_function(kGpsUartTxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartTxPin));
    gpio_set_function(kGpsUartRxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartRxPin));
}

// Write a precomputed PMTK sentence directly. Used after baud negotiation
// for the static-config writes (PMTK314 sentence list + PMTK220 rate).
// No g_initialized guard — caller is responsible for ordering.
static void uart_write_pmtk(const char* sentence, size_t len) {
    uart_write_blocking(
        GPS_UART_INST,
        reinterpret_cast<const uint8_t*>(sentence),
        len);
}

// Drain UART for up to 2 seconds looking for '$' (NMEA start).
// MT3339 outputs NMEA at 1Hz by default, so 2 seconds guarantees at
// least one full sentence cycle if a GPS is connected.
// This only adds delay when NO UART GPS is connected.
static bool detect_gps_presence() {
    absolute_time_t deadline = make_timeout_time_us(kInitTimeoutUs);

    while (!time_reached(deadline)) {
        if (uart_is_readable(GPS_UART_INST)) {
            char c = static_cast<char>(uart_getc(GPS_UART_INST));
            if (c == kNmeaStart) {
                return true;
            }
        }
    }
    return false;
}

// ============================================================================
// Public API
// ============================================================================

bool gps_uart_init() {
    // [M3] UART GPS unavailable on boards where GPIO 0/1 are not UART pins
    if constexpr (!board::kUartGpsAvailable) {
        return false;
    }

    // Initialize lwGPS parser
    lwgps_init(&g_gps);
    memset(&g_data, 0, sizeof(g_data));

    // Reset ring buffer state
    g_rxHead = 0;
    g_rxTail = 0;
    g_rxOverflow = 0;

    // Configure UART0 at factory baud (9600 — MT3339 cold-start default)
    uart_init(GPS_UART_INST, kGpsUartBaud);
    gpio_set_function(kGpsUartTxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartTxPin));
    gpio_set_function(kGpsUartRxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartRxPin));

    // Presence detection
    if (!detect_gps_presence()) {
        // No GPS detected — deinit UART so pins are free
        uart_deinit(GPS_UART_INST);
        return false;
    }

    // Negotiate 57600 baud before enabling IRQ.
    // negotiate_baud_to_57600() deinits/reinits the host UART —
    // must run before IRQ registration.
    negotiate_baud_to_57600();

    // Configure NMEA output: RMC + GGA + GSA (same as I2C backend).
    // Must be called AFTER baud negotiation — both writes go at 57600.
    uart_write_pmtk(kPmtk314Sentence, sizeof(kPmtk314Sentence) - 1);

    // Set 5Hz update rate (PMTK220,200ms interval).
    uart_write_pmtk(kPmtk220_5HzSentence, sizeof(kPmtk220_5HzSentence) - 1);

    g_initialized = true;

    // Enable interrupt-driven receive.
    // ISR registered on Core 0 (the core running this init).
    // uart_set_irqs_enabled(rx=true, tx=false) enables both:
    //   UARTRXINTR — fires when RX FIFO reaches threshold (>= 4 bytes)
    //   UARTRTINTR — fires when >= 1 byte and no new bytes for 32 bit periods
    irq_set_exclusive_handler(UART_IRQ_NUM(GPS_UART_INST), gps_uart_rx_isr);
    irq_set_enabled(UART_IRQ_NUM(GPS_UART_INST), true);
    uart_set_irqs_enabled(GPS_UART_INST, true, false);

    return true;
}

bool gps_uart_ready() {
    return g_initialized;
}

void gps_uart_drain() {
    if (!g_initialized) {
        return;
    }

    // Snapshot head (written by ISR on Core 0) then read bytes up to that point.
    // After reading, advance tail. This is the SPSC consumer path.
    uint32_t head = g_rxHead;
    uint32_t tail = g_rxTail;

    if (head == tail) {
        return;  // Ring buffer empty
    }

    // Calculate contiguous bytes available.
    // Feed to lwGPS in up to two chunks (wrap-around).
    if (head > tail) {
        // Single contiguous region: [tail..head)
        lwgps_process(&g_gps, &g_rxBuf[tail], head - tail);
    } else {
        // Wrapped: [tail..end) then [0..head)
        lwgps_process(&g_gps, &g_rxBuf[tail], kRxBufSize - tail);
        if (head > 0) {
            lwgps_process(&g_gps, &g_rxBuf[0], head);
        }
    }

    g_rxTail = head;
}

bool gps_uart_update() {
    if (!g_initialized) {
        return false;
    }

    // Drain ring buffer into lwGPS parser
    gps_uart_drain();

    // Update data structure from whatever lwGPS has accumulated
    update_data_from_lwgps();

    return true;
}

bool gps_uart_get_data(gps_data_t* data) {
    if (data == nullptr) {
        return false;
    }
    *data = g_data;
    return g_data.valid;
}

bool gps_uart_has_fix() {
    return g_data.valid && (g_data.fix >= GPS_FIX_2D);
}

uint32_t gps_uart_get_overflow_count() {
    return g_rxOverflow;
}

bool gps_uart_reinit() {
    // [M3] UART GPS unavailable on boards where GPIO 0/1 are not UART pins
    if constexpr (!board::kUartGpsAvailable) {
        return false;
    }

    // Disable interrupt — prevents ISR from touching ring buffer during reset
    irq_set_enabled(UART_IRQ_NUM(GPS_UART_INST), false);
    uart_set_irqs_enabled(GPS_UART_INST, false, false);

    // Deinit UART
    uart_deinit(GPS_UART_INST);

    // Reset ring buffer
    g_rxHead = 0;
    g_rxTail = 0;
    g_rxOverflow = 0;

    // Reset parser
    lwgps_init(&g_gps);
    memset(&g_data, 0, sizeof(g_data));

    // Mark uninitialized — protects gps_uart_drain / gps_uart_update from
    // touching parser state mid-reinit. PMTK writes below bypass that
    // guard by using uart_write_blocking directly.
    g_initialized = false;

    // Reinit UART at factory baud for presence detection
    uart_init(GPS_UART_INST, kGpsUartBaud);
    gpio_set_function(kGpsUartTxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartTxPin));
    gpio_set_function(kGpsUartRxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartRxPin));

    // Presence detection (blocks up to 2s)
    if (!detect_gps_presence()) {
        uart_deinit(GPS_UART_INST);
        return false;
    }

    // Negotiate 57600 baud
    negotiate_baud_to_57600();

    // Reconfigure output sentences and rate (all writes at 57600 now)
    uart_write_pmtk(kPmtk314Sentence, sizeof(kPmtk314Sentence) - 1);
    uart_write_pmtk(kPmtk220_5HzSentence, sizeof(kPmtk220_5HzSentence) - 1);

    g_initialized = true;

    // Re-enable interrupt
    irq_set_enabled(UART_IRQ_NUM(GPS_UART_INST), true);
    uart_set_irqs_enabled(GPS_UART_INST, true, false);

    return true;
}
