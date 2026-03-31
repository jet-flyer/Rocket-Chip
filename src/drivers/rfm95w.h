// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file rfm95w.h
 * @brief RFM95W (SX1276) LoRa radio driver
 *
 * SPI-based driver for Adafruit LoRa Radio FeatherWing #3231.
 * GPIO-controlled CS for burst FIFO access (see spi_bus.h).
 *
 * Register addresses and init sequence from SX1276 datasheet
 * (Semtech DS_SX1276-7-8-9_W_APP_V7) and RadioHead RH_RF95.
 *
 * Optional peripheral — absent FeatherWing detected at init time
 * via RegVersion read (returns 0x00/0xFF when not present).
 */

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
    constexpr uint8_t kSyncWord       = 0x39;
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
// TX Poll Result (IVP-92: non-blocking send)
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
    uint64_t tx_start_us;    // Timestamp of send_start() for timeout (IVP-92)
};

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize the RFM95W radio
 *
 * Performs hardware reset, checks RegVersion (0x12 expected), configures
 * LoRa mode at 915 MHz, SF7, BW 125kHz, CR 4/5, +20 dBm via PA_BOOST.
 *
 * If the FeatherWing is not stacked (RegVersion reads 0x00 or 0xFF),
 * returns false and boot continues normally (Council #3: optional peripheral).
 *
 * @param dev Device handle (caller-owned, zero-initialized)
 * @param cs  GPIO pin for chip select
 * @param rst GPIO pin for reset
 * @param irq GPIO pin for DIO0 (TX/RX done interrupt)
 * @return true if radio detected and configured, false if absent or error
 */
bool rfm95w_init(rfm95w_t* dev, uint8_t cs, uint8_t rst, uint8_t irq);

/**
 * @brief Send a packet
 *
 * Writes payload to FIFO, sets TX mode, polls DIO0 for TxDone with
 * 100ms timeout. Returns to Standby after completion.
 *
 * @param dev Initialized device handle
 * @param data Payload data
 * @param len Payload length (max 128 bytes)
 * @return true if TxDone received within timeout
 */
bool rfm95w_send(rfm95w_t* dev, const uint8_t* data, uint8_t len);

// ============================================================================
// Non-Blocking TX API (IVP-92)
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

/**
 * @brief Start a non-blocking packet transmission
 *
 * Writes payload to FIFO, sets TX mode, returns immediately (~200µs).
 * Call rfm95w_send_poll() on subsequent ticks to check completion.
 *
 * @param dev Initialized device handle
 * @param data Payload data
 * @param len Payload length (max 128 bytes)
 * @return true if TX started, false if not initialized or len invalid
 */
bool rfm95w_send_start(rfm95w_t* dev, const uint8_t* data, uint8_t len);

/**
 * @brief Poll for TX completion (non-blocking)
 *
 * Reads RegIrqFlags register (latched, not GPIO DIO0 — Council C3-R3).
 * Returns kBusy until TxDone flag sets, or kTimeout after 150ms.
 * On kDone/kTimeout: clears IRQ flags, restores Standby mode.
 *
 * @param dev Device handle (must have called send_start first)
 * @return TxPollResult: kBusy, kDone, or kTimeout
 */
TxPollResult rfm95w_send_poll(rfm95w_t* dev);

/**
 * @brief Receive a packet (call after rfm95w_available returns true)
 *
 * Reads payload from FIFO, records RSSI and SNR.
 *
 * @param dev Initialized device handle
 * @param buf Buffer to receive into
 * @param max_len Maximum bytes to read
 * @return Number of bytes received, or 0 on error/CRC failure
 */
uint8_t rfm95w_recv(rfm95w_t* dev, uint8_t* buf, uint8_t max_len);

/**
 * @brief Check if a packet has been received
 *
 * Polls DIO0 for RxDone flag. Call periodically in main loop.
 *
 * @param dev Initialized device handle
 * @return true if a packet is ready to read
 */
bool rfm95w_available(rfm95w_t* dev);

/**
 * @brief Poll DIO0 interrupt pin
 *
 * Isolated poll function structured for future ISR swap (Council #6).
 *
 * @param dev Initialized device handle
 * @return true if DIO0 is high (interrupt pending)
 */
bool rfm95w_poll_irq(rfm95w_t* dev);

/**
 * @brief Set operating frequency
 *
 * Uses 64-bit arithmetic to avoid overflow at 915 MHz (Council #1).
 *
 * @param dev Initialized device handle
 * @param freq_hz Frequency in Hz (e.g., 915000000)
 */
void rfm95w_set_frequency(rfm95w_t* dev, uint32_t freq_hz);

/**
 * @brief Set TX output power
 *
 * Uses PA_BOOST pin. Valid range: 2-20 dBm.
 *
 * @param dev Initialized device handle
 * @param dbm Power in dBm (clamped to 2-20)
 */
void rfm95w_set_tx_power(rfm95w_t* dev, int8_t dbm);

/**
 * @brief Get RSSI of last received packet
 *
 * @param dev Initialized device handle
 * @return RSSI in dBm (negative value, e.g., -80)
 */
int16_t rfm95w_rssi(const rfm95w_t* dev);

/**
 * @brief Set LoRa bandwidth
 *
 * Modifies RegModemConfig1[7:4]. Must be called while in Standby or Sleep.
 * Both TX and RX must use the same bandwidth to communicate.
 *
 * @param dev Initialized device handle
 * @param bw  Bandwidth code: rfm95w::kBw125, kBw250, or kBw500
 */
void rfm95w_set_bandwidth(rfm95w_t* dev, uint8_t bw);

/**
 * @brief Set LoRa spreading factor (IVP-92)
 *
 * Modifies RegModemConfig2[7:4]. Must be called while in Standby or Sleep.
 * Both TX and RX must use the same SF to communicate.
 *
 * @param dev Initialized device handle
 * @param sf  Spreading factor: 6-12
 */
void rfm95w_set_spreading_factor(rfm95w_t* dev, uint8_t sf);

/**
 * @brief Set LoRa coding rate (IVP-92)
 *
 * Modifies RegModemConfig1[3:1]. Must be called while in Standby or Sleep.
 *
 * @param dev Initialized device handle
 * @param cr  Coding rate denominator: 5-8 (meaning CR 4/5 through 4/8)
 */
void rfm95w_set_coding_rate(rfm95w_t* dev, uint8_t cr);

/**
 * @brief Set radio to RX continuous mode
 *
 * @param dev Initialized device handle
 */
void rfm95w_start_rx(rfm95w_t* dev);

#endif // ROCKETCHIP_RFM95W_H
