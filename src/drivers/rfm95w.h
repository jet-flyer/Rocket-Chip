// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// RFM95W (SX1276) LoRa radio driver
// SPI-based driver for Adafruit LoRa Radio FeatherWing #3231.
// GPIO-controlled CS for burst FIFO access (see spi_bus.h).
// Register addresses and init sequence from SX1276 datasheet
// (Semtech DS_SX1276-7-8-9_W_APP_V7) and RadioHead RH_RF95.
// Optional peripheral — absent FeatherWing detected at init time
// via RegVersion read (returns 0x00/0xFF when not present).

#ifndef ROCKETCHIP_RFM95W_H
#define ROCKETCHIP_RFM95W_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Register Constants (SX1276 datasheet Table 41)
// Only registers used by v1 driver — Council Amendment #5
// ============================================================================

namespace rfm95w {
namespace reg {
    constexpr uint8_t kFifo           = 0x00;
    constexpr uint8_t kOpMode         = 0x01;
    constexpr uint8_t kFrMsb          = 0x06;
    constexpr uint8_t kFrMid          = 0x07;
    constexpr uint8_t kFrLsb          = 0x08;
    constexpr uint8_t kPaConfig       = 0x09;
    constexpr uint8_t kLna            = 0x0C;  // IVP-T11: LnaBoostHf +3 dB
    constexpr uint8_t kFifoAddrPtr    = 0x0D;
    constexpr uint8_t kFifoTxBase     = 0x0E;
    constexpr uint8_t kFifoRxBase     = 0x0F;
    constexpr uint8_t kFifoRxCurrent  = 0x10;
    constexpr uint8_t kIrqFlagsMask   = 0x11;
    constexpr uint8_t kIrqFlags       = 0x12;
    constexpr uint8_t kRxNbBytes      = 0x13;
    constexpr uint8_t kPktSnrValue    = 0x19;
    constexpr uint8_t kPktRssiValue   = 0x1A;
    constexpr uint8_t kModemConfig1   = 0x1D;
    constexpr uint8_t kModemConfig2   = 0x1E;
    constexpr uint8_t kPreambleMsb    = 0x20;
    constexpr uint8_t kPreambleLsb    = 0x21;
    constexpr uint8_t kPayloadLength  = 0x22;
    constexpr uint8_t kModemConfig3   = 0x26;  // IVP-T11: AgcAutoOn adaptive LNA
    constexpr uint8_t kSyncWord       = 0x39;
    constexpr uint8_t kInvertIQ       = 0x33;  // IVP-T11: boot-time audit only
    constexpr uint8_t kDioMapping1    = 0x40;
    constexpr uint8_t kVersion        = 0x42;
    constexpr uint8_t kPaDac          = 0x4D;
} // namespace reg

namespace mode {
    constexpr uint8_t kSleep          = 0x00;
    constexpr uint8_t kStandby        = 0x01;
    constexpr uint8_t kTx             = 0x03;
    constexpr uint8_t kRxContinuous   = 0x05;
} // namespace mode

// LoRa mode bit (set in RegOpMode to select LoRa vs FSK)
constexpr uint8_t kLoRaMode           = 0x80;

// Expected RegVersion value for SX1276
constexpr uint8_t kVersionExpected    = 0x12;

namespace irq {
    constexpr uint8_t kRxDone         = 0x40;
    constexpr uint8_t kPayloadCrcErr  = 0x20;
    constexpr uint8_t kTxDone         = 0x08;
    constexpr uint8_t kAll            = 0xFF;
} // namespace irq

// Max payload size (SX1276 FIFO is 256 bytes total, split TX/RX)
constexpr uint8_t kMaxPayload         = 128;

// TX timeout: SF7/BW250 airtime ~90ms for 105B (MAVLink worst case);
// 150ms = ~1.7× margin. Council Amendment #2 (Stage 7 plan).
constexpr uint32_t kTxTimeoutUs       = 150000;

// Bandwidth constants for rfm95w_set_bandwidth()
// SX1276 RegModemConfig1[7:4] bandwidth encoding
constexpr uint8_t kBw125  = 0x07;    // 125 kHz
constexpr uint8_t kBw250  = 0x08;    // 250 kHz
constexpr uint8_t kBw500  = 0x09;    // 500 kHz

} // namespace rfm95w

// ============================================================================
// Boot-Time Audit (IVP-T11)
// ============================================================================

// Snapshot of registers that MUST match between vehicle and station for the
// link to come up, but are easy to misconfigure silently. Read once post-init
// and logged by the caller (driver stays stdio-free per project-wide R-5
// stdio removal, 2026-05-17).
struct rfm95w_audit_t {
    uint8_t invert_iq;        // RegInvertIQ (0x33) — expect 0x27 (non-inverted)
    uint8_t modem_config2;    // RegModemConfig2 — bit 2 (RxPayloadCrcOn) must be 1
    uint8_t lna;              // RegLna (0x0C) — post-T11 expect 0x23
    uint8_t modem_config3;    // RegModemConfig3 (0x26) — post-T11 expect 0x04
};

// Expected values (for caller's compare-and-log).
constexpr uint8_t kAuditInvertIqExpected     = 0x27;
constexpr uint8_t kAuditModemCfg2CrcBitMask  = 0x04;   // RxPayloadCrcOn
constexpr uint8_t kAuditLnaExpected          = 0x23;
constexpr uint8_t kAuditModemCfg3Expected    = 0x04;

// ============================================================================
// TX Poll Result (non-blocking send)
// ============================================================================

enum class TxPollResult : uint8_t {
    kBusy    = 0,   // TX still in progress
    kDone    = 1,   // TX complete (TxDone IRQ flag set)
    kTimeout = 2,   // TX exceeded expected airtime (150ms)
};

// ============================================================================
// Device Handle
// ============================================================================

struct rfm95w_t {
    uint8_t  cs_pin;
    uint8_t  rst_pin;
    uint8_t  irq_pin;        // DIO0
    bool     initialized;
    uint8_t  mode;           // Current operating mode
    int16_t  last_rssi;      // RSSI of last received packet (dBm)
    int8_t   last_snr;       // SNR of last received packet (dB)
    uint64_t tx_start_us;    // Timestamp of send_start() for timeout
    uint32_t tx_timeout_us;  // Abort threshold for TX polling. Scales with
                             // SF/BW/payload via rfm95w_set_tx_timeout_us().
                             // Default kTxTimeoutUs until configured.
};

// ============================================================================
// Public API
// ============================================================================

// Performs hardware reset, checks RegVersion (0x12 expected), configures
// LoRa mode at 915 MHz, SF7, BW 125kHz, CR 4/5, +20 dBm via PA_BOOST.
// If the FeatherWing is not stacked (RegVersion reads 0x00 or 0xFF),
// returns false and boot continues normally (Council #3: optional peripheral).
bool rfm95w_init(rfm95w_t* dev, uint8_t cs, uint8_t rst, uint8_t irq);

// GDB-callable diagnostic helper (IVP-132a.4 re-eval). Reads RegVersion
// over SPI using the given CS pin. Returns 0x12 if an SX1276 is
// physically present and responding; 0x00 or 0xFF if SPI line is dead
// or no chip present. Does NOT require rfm95w_init() to have been
// called first — uses the raw SPI bus.
// Use from GDB as the T=0 precondition "is the radio physically
// reachable" check:
// (gdb) call rfm95w_read_version(10)  // Fruit Jam CS pin
uint8_t rfm95w_read_version(uint8_t cs);

// Reads RegInvertIQ, RegModemConfig2, RegLna, RegModemConfig3 over SPI and
// returns their values in audit. Caller is expected to log them and
// compare against kAudit*Expected constants. Driver stays stdio-free.
// Call after rfm95w_init(). If the radio is absent, values will be 0x00 or
// 0xFF — caller should skip the comparison when radio is not initialized.
void rfm95w_read_audit(rfm95w_t* dev, rfm95w_audit_t* audit);

// Writes payload to FIFO, sets TX mode, polls DIO0 for TxDone with
// 100ms timeout. Returns to Standby after completion.
bool rfm95w_send(rfm95w_t* dev, const uint8_t* data, uint8_t len);

// ============================================================================
// Non-Blocking TX API
//
// Split rfm95w_send() into start + poll for use inside AO handlers where
// blocking for 50-150ms causes QP/C queue overflow (LL Entry 32).
//
// Usage:
//   if (rfm95w_send_start(dev, data, len)) {
//       // ... on subsequent ticks:
//       TxPollResult r = rfm95w_send_poll(dev);
//       if (r == TxPollResult::kDone)    { /* success */ }
//       if (r == TxPollResult::kTimeout) { /* handle failure */ }
//   }
// ============================================================================

// Writes payload to FIFO, sets TX mode, returns immediately (~200µs).
// Call rfm95w_send_poll() on subsequent ticks to check completion.
bool rfm95w_send_start(rfm95w_t* dev, const uint8_t* data, uint8_t len);

// Reads RegIrqFlags register (latched, not GPIO DIO0 — Council C3-R3).
// Returns kBusy until TxDone flag sets, or kTimeout after 150ms.
// On kDone/kTimeout: clears IRQ flags, restores Standby mode.
TxPollResult rfm95w_send_poll(rfm95w_t* dev);

// Reads payload from FIFO, records RSSI and SNR.
uint8_t rfm95w_recv(rfm95w_t* dev, uint8_t* buf, uint8_t max_len);

// Polls DIO0 for RxDone flag. Call periodically in main loop.
bool rfm95w_available(rfm95w_t* dev);

// Isolated poll function structured for future ISR swap (Council #6).
bool rfm95w_poll_irq(rfm95w_t* dev);

// Uses 64-bit arithmetic to avoid overflow at 915 MHz (Council #1).
void rfm95w_set_frequency(rfm95w_t* dev, uint32_t freq_hz);

// Uses PA_BOOST pin. Valid range: 2-20 dBm.
void rfm95w_set_tx_power(rfm95w_t* dev, int8_t dbm);

// RSSI in dBm (negative value, e.g., -80)
int16_t rfm95w_rssi(const rfm95w_t* dev);

// Modifies RegModemConfig1[7:4]. Must be called while in Standby or Sleep.
// Both TX and RX must use the same bandwidth to communicate.
void rfm95w_set_bandwidth(rfm95w_t* dev, uint8_t bw);

// Modifies RegModemConfig2[7:4]. Must be called while in Standby or Sleep.
// Both TX and RX must use the same SF to communicate.
void rfm95w_set_spreading_factor(rfm95w_t* dev, uint8_t sf);

// Modifies RegModemConfig1[3:1]. Must be called while in Standby or Sleep.
void rfm95w_set_coding_rate(rfm95w_t* dev, uint8_t cr);

// Caller sets ~2× max packet airtime at current {SF, BW, CR, payload}.
void rfm95w_set_tx_timeout_us(rfm95w_t* dev, uint32_t timeout_us);

// Implements SX1276 datasheet §4.1.1.6 formula: T_preamble + T_payload.
// Returns microseconds. Used by callers to set TX timeout and ACK-window
// budgets. Assumes explicit header, CRC on, 8-symbol preamble, CR 4/5.
uint32_t rfm95w_airtime_us(uint8_t sf, uint16_t bw_khz, uint8_t payload_bytes);

void rfm95w_start_rx(rfm95w_t* dev);

#endif // ROCKETCHIP_RFM95W_H
