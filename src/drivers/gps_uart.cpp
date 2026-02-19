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
 *   - Same PMTK command interface
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
 * Ref: Adafruit Ultimate GPS FeatherWing product page, MT3339 datasheet.
 *      ArduPilot AP_HAL::UARTDriver (DMA + ring buffer pattern).
 *      Pico SDK stdio_uart.c (IRQ handler pattern).
 */

#include "gps_uart.h"
#include "lwgps/lwgps.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// Constants
// ============================================================================

// UART instance — uart0 on GPIO0/1 per Feather pinout.
// uart0 is a macro (reinterpret_cast), not constexpr — use #define.
#define GPS_UART_INST uart0

// NMEA protocol constants (shared with I2C backend by value, not by reference)
constexpr char     kNmeaStart       = '$';
constexpr uint16_t kGpsYearBase     = 2000;
constexpr uint8_t  kGsaFixMode3d    = 3;
constexpr uint8_t  kGsaFixMode2d    = 2;

// PMTK220 rate intervals (ms)
constexpr uint16_t kGpsRate1Hz      = 1000;
constexpr uint16_t kGpsRate5Hz      = 200;
constexpr uint16_t kGpsRate10Hz     = 100;

// NMEA command buffer
constexpr size_t   kNmeaCmdBufSize  = 128;
constexpr size_t   kNmeaChecksumLen = 5;       // "*XX\r\n"

// Init: presence detection timeout
// MT3339 outputs NMEA at 1Hz default — 2 seconds guarantees at least one sentence.
constexpr uint32_t kInitTimeoutUs   = 2000000; // 2 seconds

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
//   At 9600 baud (~960 B/s), 10Hz poll = ~96 bytes/interval.
//   512 = 5.3x margin, holds 2+ full NMEA bursts.
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
 * @brief Calculate NMEA checksum (XOR of bytes between '$' and '*')
 */
static uint8_t nmea_checksum(const char* sentence) {
    uint8_t checksum = 0;
    if (*sentence == '$') {
        sentence++;
    }
    while (*sentence != '\0' && *sentence != '*') {
        checksum ^= static_cast<uint8_t>(*sentence++);
    }
    return checksum;
}

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

    // Fix type: prefer GGA fix quality, fall back to GSA fixMode for 2D/3D
    if (g_gps.fix >= 1) {
        if (g_gps.fix_mode >= kGsaFixMode3d) {
            g_data.fix = GPS_FIX_3D;
        } else if (g_gps.fix_mode == kGsaFixMode2d) {
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
    g_data.timeValid = g_gps.time_valid;
    g_data.dateValid = g_gps.date_valid;

    g_data.ggaFix = g_gps.fix;
    g_data.gsaFixMode = g_gps.fix_mode;
    g_data.rmcValid = lwgps_is_valid(&g_gps);
}

// ============================================================================
// Public API
// ============================================================================

bool gps_uart_init() {
    // Initialize lwGPS parser
    lwgps_init(&g_gps);
    memset(&g_data, 0, sizeof(g_data));

    // Reset ring buffer state
    g_rxHead = 0;
    g_rxTail = 0;
    g_rxOverflow = 0;

    // Configure UART0
    uart_init(GPS_UART_INST, kGpsUartBaud);
    gpio_set_function(kGpsUartTxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartTxPin));
    gpio_set_function(kGpsUartRxPin, UART_FUNCSEL_NUM(GPS_UART_INST, kGpsUartRxPin));

    // Presence detection: drain UART for up to 2 seconds looking for '$'.
    // MT3339 outputs NMEA at 1Hz by default, so 2 seconds guarantees at
    // least one full sentence cycle if a GPS is connected.
    // This only adds delay when NO UART GPS is connected.
    bool foundNmea = false;
    absolute_time_t deadline = make_timeout_time_us(kInitTimeoutUs);

    while (!time_reached(deadline)) {
        if (uart_is_readable(GPS_UART_INST)) {
            char c = static_cast<char>(uart_getc(GPS_UART_INST));
            if (c == kNmeaStart) {
                foundNmea = true;
                break;
            }
        }
    }

    if (!foundNmea) {
        // No GPS detected — deinit UART so pins are free
        uart_deinit(GPS_UART_INST);
        return false;
    }

    g_initialized = true;

    // Configure NMEA output: RMC + GGA + GSA (same as I2C backend)
    gps_uart_send_command("PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

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

bool gps_uart_send_command(const char* cmd) {
    if (!g_initialized || cmd == nullptr) {
        return false;
    }

    // Build full NMEA sentence with checksum
    char sentence[kNmeaCmdBufSize];
    int len = snprintf(sentence, sizeof(sentence) - kNmeaChecksumLen, "$%s*", cmd);
    if (len < 0 || len >= static_cast<int>(sizeof(sentence) - kNmeaChecksumLen)) {
        return false;
    }

    // Calculate and append checksum
    uint8_t cs = nmea_checksum(sentence);
    snprintf(sentence + len, kNmeaChecksumLen, "%02X\r\n", cs);
    len += 4;  // "XX\r\n"

    // Send via UART
    uart_write_blocking(GPS_UART_INST, reinterpret_cast<const uint8_t*>(sentence), static_cast<size_t>(len));

    return true;
}

bool gps_uart_set_rate(uint8_t rateHz) {
    char cmd[32];
    uint16_t intervalMs = 0;

    switch (rateHz) {
        case 1:  intervalMs = kGpsRate1Hz;  break;
        case 5:  intervalMs = kGpsRate5Hz;  break;
        case 10: intervalMs = kGpsRate10Hz; break;
        default: return false;
    }

    snprintf(cmd, sizeof(cmd), "PMTK220,%u", intervalMs);
    return gps_uart_send_command(cmd);
}
