// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file rfm95w.cpp
 * @brief RFM95W (SX1276) LoRa radio driver implementation
 *
 * Init sequence and register usage based on:
 * - SX1276 datasheet (Semtech DS_SX1276-7-8-9_W_APP_V7)
 * - RadioHead RH_RF95 (airspayce.com)
 * - Adafruit CircuitPython adafruit_rfm9x
 *
 * Council amendments incorporated:
 * #1: 64-bit freq calculation, 100ms TX timeout
 * #2: GPIO-controlled CS (in spi_bus.cpp)
 * #3: Optional peripheral — absent HW returns false
 * #4: FIFO pointer set before every TX write
 * #5: Only used registers defined
 * #6: rfm95w_poll_irq() isolated for future ISR swap
 */

#include "rfm95w.h"
#include "spi_bus.h"
#include "rocketchip/config.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// ============================================================================
// Internal Helpers
// ============================================================================

static void set_mode(rfm95w_t* dev, uint8_t mode) {
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kOpMode,
                      rfm95w::kLoRaMode | mode);
    dev->mode = mode;
}

// Configure GPIO pins for CS (output high), RST (output), IRQ (input),
// then hardware reset the SX1276 (10ms low, 10ms high).
static void init_gpio_and_reset(uint8_t cs, uint8_t rst, uint8_t irq) {
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1);

    gpio_init(rst);
    gpio_set_dir(rst, GPIO_OUT);

    gpio_init(irq);
    gpio_set_dir(irq, GPIO_IN);

    // SX1276 datasheet Section 7.2.2 — POR after reset
    gpio_put(rst, 0);
    sleep_ms(10);
    gpio_put(rst, 1);
    sleep_ms(10);
}

// Configure LoRa modem parameters: SF7, BW 125kHz, CR 4/5, CRC on,
// preamble 8, sync word 0x12, +20 dBm PA_BOOST, 915 MHz.
static void configure_modem(rfm95w_t* dev) {
    uint8_t cs = dev->cs_pin;

    rfm95w_set_frequency(dev, 915000000u);

    // FIFO base addresses: TX at 0x80, RX at 0x00 (RadioHead default)
    spi_bus_write_reg(cs, rfm95w::reg::kFifoTxBase, 0x80);
    spi_bus_write_reg(cs, rfm95w::reg::kFifoRxBase, 0x00);

    // RegModemConfig1: BW[7:4]=0111 (125kHz), CR[3:1]=001 (4/5), ImplicitHeader=0
    spi_bus_write_reg(cs, rfm95w::reg::kModemConfig1, 0x72);
    // RegModemConfig2: SF7[7:4]=0111, CRC on[2]=1
    spi_bus_write_reg(cs, rfm95w::reg::kModemConfig2, 0x74);

    spi_bus_write_reg(cs, rfm95w::reg::kPreambleMsb, 0x00);
    spi_bus_write_reg(cs, rfm95w::reg::kPreambleLsb, 0x08);

    rfm95w_set_tx_power(dev, 5);  // Bench testing — raise to 20 for field use
    spi_bus_write_reg(cs, rfm95w::reg::kSyncWord, 0x12);
    spi_bus_write_reg(cs, rfm95w::reg::kDioMapping1, 0x00);
}

// ============================================================================
// Public API
// ============================================================================

bool rfm95w_init(rfm95w_t* dev, uint8_t cs, uint8_t rst, uint8_t irq) {
    dev->cs_pin      = cs;
    dev->rst_pin     = rst;
    dev->irq_pin     = irq;
    dev->initialized = false;
    dev->mode        = rfm95w::mode::kSleep;
    dev->last_rssi   = 0;
    dev->last_snr    = 0;

    init_gpio_and_reset(cs, rst, irq);

    // Read RegVersion — expect 0x12 for SX1276
    // If 0x00 or 0xFF, FeatherWing is not stacked (Council #3)
    uint8_t version = spi_bus_read_reg(cs, rfm95w::reg::kVersion);
    if (version != rfm95w::kVersionExpected) {
        return false;
    }

    // Enter Sleep → LoRa mode (must set LoRa bit while in Sleep)
    spi_bus_write_reg(cs, rfm95w::reg::kOpMode, rfm95w::mode::kSleep);
    sleep_ms(10);
    set_mode(dev, rfm95w::mode::kSleep);
    sleep_ms(10);

    configure_modem(dev);
    set_mode(dev, rfm95w::mode::kStandby);

    dev->initialized = true;
    return true;
}

bool rfm95w_send(rfm95w_t* dev, const uint8_t* data, uint8_t len) {
    if (!dev->initialized || len == 0 || len > rfm95w::kMaxPayload) {
        return false;
    }

    // Set Standby before writing FIFO
    set_mode(dev, rfm95w::mode::kStandby);

    // Council #4: Set FIFO pointer to TX base before every write
    // Stale RX pointer corrupts TX data if not reset
    uint8_t tx_base = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kFifoTxBase);
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFifoAddrPtr, tx_base);

    // Write payload to FIFO
    spi_bus_write_burst(dev->cs_pin, rfm95w::reg::kFifo, data, len);

    // Set payload length
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPayloadLength, len);

    // Clear all IRQ flags before TX
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kIrqFlags, rfm95w::irq::kAll);

    // Set TX mode
    set_mode(dev, rfm95w::mode::kTx);

    // Poll DIO0 for TxDone with timeout (Council #1: no bare while(!DIO0))
    uint64_t start = time_us_64();
    while (!rfm95w_poll_irq(dev)) {
        if ((time_us_64() - start) > rfm95w::kTxTimeoutUs) {
            // Timeout — return to Standby
            set_mode(dev, rfm95w::mode::kStandby);
            return false;
        }
    }

    // Clear IRQ flags
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kIrqFlags, rfm95w::irq::kAll);

    // Return to Standby
    set_mode(dev, rfm95w::mode::kStandby);

    return true;
}

uint8_t rfm95w_recv(rfm95w_t* dev, uint8_t* buf, uint8_t max_len) {
    if (!dev->initialized) {
        return 0;
    }

    // Read IRQ flags
    uint8_t irq_flags = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kIrqFlags);

    // Clear all IRQ flags
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kIrqFlags, rfm95w::irq::kAll);

    // Check for CRC error
    if (irq_flags & rfm95w::irq::kPayloadCrcErr) {
        return 0;
    }

    // Read payload length
    uint8_t nb_bytes = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kRxNbBytes);
    if (nb_bytes > max_len) {
        nb_bytes = max_len;
    }

    // Set FIFO pointer to current RX address
    uint8_t rx_addr = spi_bus_read_reg(dev->cs_pin,
                                       rfm95w::reg::kFifoRxCurrent);
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFifoAddrPtr, rx_addr);

    // Read payload from FIFO
    spi_bus_read_burst(dev->cs_pin, rfm95w::reg::kFifo, buf, nb_bytes);

    // Record RSSI: RSSI(dBm) = -157 + RegPktRssiValue (for HF port, 868/915 MHz)
    // (SX1276 datasheet Section 5.5.5)
    uint8_t raw_rssi = spi_bus_read_reg(dev->cs_pin,
                                        rfm95w::reg::kPktRssiValue);
    dev->last_rssi = static_cast<int16_t>(-157 + raw_rssi);

    // Record SNR: SNR(dB) = RegPktSnrValue / 4 (signed, 2's complement)
    int8_t raw_snr = static_cast<int8_t>(
        spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kPktSnrValue));
    dev->last_snr = static_cast<int8_t>(raw_snr / 4);

    return nb_bytes;
}

bool rfm95w_available(rfm95w_t* dev) {
    if (!dev->initialized) {
        return false;
    }

    // Check DIO0 pin (RxDone interrupt)
    if (rfm95w_poll_irq(dev)) {
        // Verify it's actually RxDone, not some other IRQ
        uint8_t irq_flags = spi_bus_read_reg(dev->cs_pin,
                                             rfm95w::reg::kIrqFlags);
        return (irq_flags & rfm95w::irq::kRxDone) != 0;
    }
    return false;
}

bool rfm95w_poll_irq(rfm95w_t* dev) {
    // Council #6: Isolated poll function for future ISR swap.
    // Currently polls GPIO; future version can check a flag set by ISR.
    return gpio_get(dev->irq_pin) != 0;
}

void rfm95w_set_frequency(rfm95w_t* dev, uint32_t freq_hz) {
    // Council #1: Use 64-bit arithmetic — 32-bit overflows at 915 MHz
    // Frf = (freq_hz * 2^19) / F_XOSC, where F_XOSC = 32 MHz
    uint64_t frf = (static_cast<uint64_t>(freq_hz) << 19) / 32000000ULL;

    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFrMsb,
                      static_cast<uint8_t>((frf >> 16) & 0xFF));
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFrMid,
                      static_cast<uint8_t>((frf >> 8) & 0xFF));
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFrLsb,
                      static_cast<uint8_t>(frf & 0xFF));
}

void rfm95w_set_tx_power(rfm95w_t* dev, int8_t dbm) {
    // Clamp to valid range for PA_BOOST
    if (dbm < 2) { dbm = 2; }
    if (dbm > 20) { dbm = 20; }

    if (dbm > 17) {
        // High power mode: enable PA_DAC for +20 dBm
        // RegPaDac: 0x87 = +20 dBm mode (SX1276 datasheet Section 5.4.3)
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPaDac, 0x87);
        // RegPaConfig: PA_BOOST=1, MaxPower=7, OutputPower = dbm - 5
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPaConfig,
                          static_cast<uint8_t>(0x80 | 0x70 | (dbm - 5)));
    } else {
        // Normal mode: disable PA_DAC
        // RegPaDac: 0x84 = default (normal mode)
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPaDac, 0x84);
        // RegPaConfig: PA_BOOST=1, MaxPower=7, OutputPower = dbm - 2
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPaConfig,
                          static_cast<uint8_t>(0x80 | 0x70 | (dbm - 2)));
    }
}

int16_t rfm95w_rssi(const rfm95w_t* dev) {
    return dev->last_rssi;
}

void rfm95w_start_rx(rfm95w_t* dev) {
    if (!dev->initialized) {
        return;
    }

    // Clear all IRQ flags
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kIrqFlags, rfm95w::irq::kAll);

    // Set FIFO pointer to RX base
    uint8_t rx_base = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kFifoRxBase);
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFifoAddrPtr, rx_base);

    // Set RX Continuous mode
    set_mode(dev, rfm95w::mode::kRxContinuous);
}
