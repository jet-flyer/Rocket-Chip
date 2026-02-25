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

// TX timeout: SF7/BW125 airtime ~31ms for 16 bytes; 100ms = 3× margin,
// well under 5s watchdog. Council Amendment #1.
constexpr uint32_t kTxTimeoutUs       = 100000;

} // namespace rfm95w

// ============================================================================
// Device Handle
// ============================================================================

struct rfm95w_t {
    uint8_t cs_pin;
    uint8_t rst_pin;
    uint8_t irq_pin;        // DIO0
    bool    initialized;
    uint8_t mode;           // Current operating mode
    int16_t last_rssi;      // RSSI of last received packet (dBm)
    int8_t  last_snr;       // SNR of last received packet (dB)
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
 * @brief Set radio to RX continuous mode
 *
 * @param dev Initialized device handle
 */
void rfm95w_start_rx(rfm95w_t* dev);

#endif // ROCKETCHIP_RFM95W_H
