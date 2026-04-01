// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file rfm95w.cpp
 * @brief RFM95W (SX1276) LoRa radio driver implementation
 *
 * Prior Art:
 *   - SX1276 datasheet (Semtech DS_SX1276-7-8-9_W_APP_V7)
 *   - RadioHead RH_RF95 (airspayce.com)
 *   - Adafruit CircuitPython adafruit_rfm9x
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
// File-scope constants (JSF AV Rule 151)
// ============================================================================

// Hardware reset timing (SX1276 datasheet Section 7.2.2)
static constexpr uint32_t kResetPulseLowMs  = 10;  // RST low duration
static constexpr uint32_t kResetPulseHighMs = 10;  // RST high settle

// Mode transition settling time
static constexpr uint32_t kModeSettleMs = 10;

// ISM band frequency (US, FCC Part 15)
static constexpr uint32_t kDefaultFreqHz = 915000000U;

// SX1276 crystal oscillator frequency (datasheet Section 3)
static constexpr uint64_t kFxoscHz = 32000000ULL;

// Frequency register shift (2^19 per SX1276 datasheet Section 4.1.4)
static constexpr uint32_t kFreqRegShift = 19;

// ModemConfig1: BW=125kHz[7:4]=0111, CR=4/5[3:1]=001, ImplicitHdr=0
static constexpr uint8_t kModemCfg1Default = 0x72;

// ModemConfig2: SF7[7:4]=0111, CRC on[2]=1
static constexpr uint8_t kModemCfg2Default = 0x74;

// Default TX power (dBm) — overridden by RadioConfig from Mission Profile
static constexpr int8_t kDefaultTxPowerDbm = 20;

// Private sync word (SX1276 default LoRa sync)
static constexpr uint8_t kSyncWordDefault = 0x12;

// RSSI offset for HF port (868/915 MHz) per SX1276 datasheet Section 5.5.5
static constexpr int16_t kRssiOffsetHf = -157;

// PA_BOOST power configuration (SX1276 datasheet Section 5.4.3)
static constexpr int8_t kPaBoostMinDbm     = 2;   // Minimum PA_BOOST output
static constexpr int8_t kPaBoostMaxDbm     = 20;  // Maximum PA_BOOST output
static constexpr int8_t kPaDacThreshDbm    = 17;  // Above this, use PA_DAC high-power mode
static constexpr uint8_t kPaDacHighPower   = 0x87; // PA_DAC: +20 dBm mode
static constexpr uint8_t kPaDacNormal      = 0x84; // PA_DAC: default (normal mode)
static constexpr uint8_t kPaBoostBit       = 0x80; // PA_SELECT = PA_BOOST
static constexpr uint8_t kMaxPowerBits     = 0x70; // MaxPower=7 (bits[6:4])
static constexpr int8_t kPaOffsetHighPower = 5;    // OutputPower = dbm - 5 in high-power mode
static constexpr int8_t kPaOffsetNormal    = 2;    // OutputPower = dbm - 2 in normal mode

// Bandwidth register mask: lower nibble preserved (CR + header bits)
static constexpr uint8_t kBwLowerNibbleMask = 0x0F;

// SNR register divisor (SX1276: SNR = RegPktSnrValue / 4)
static constexpr int8_t kSnrDivisor = 4;

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
    gpio_disable_pulls(irq);  // No internal pulls — SX1276 DIO0 is push-pull

    // SX1276 datasheet Section 7.2.2 — POR after reset
    gpio_put(rst, 0);
    sleep_ms(kResetPulseLowMs);
    gpio_put(rst, 1);
    sleep_ms(kResetPulseHighMs);
}

// Configure LoRa modem parameters: SF7, BW 125kHz, CR 4/5, CRC on,
// preamble 8, sync word 0x12, +20 dBm PA_BOOST, 915 MHz.
static void configure_modem(rfm95w_t* dev) {
    uint8_t cs = dev->cs_pin;

    rfm95w_set_frequency(dev, kDefaultFreqHz);

    // FIFO base addresses: TX at 0x80, RX at 0x00 (RadioHead default)
    spi_bus_write_reg(cs, rfm95w::reg::kFifoTxBase, 0x80);
    spi_bus_write_reg(cs, rfm95w::reg::kFifoRxBase, 0x00);

    // RegModemConfig1: BW[7:4]=0111 (125kHz), CR[3:1]=001 (4/5), ImplicitHeader=0
    spi_bus_write_reg(cs, rfm95w::reg::kModemConfig1, kModemCfg1Default);
    // RegModemConfig2: SF7[7:4]=0111, CRC on[2]=1
    spi_bus_write_reg(cs, rfm95w::reg::kModemConfig2, kModemCfg2Default);

    spi_bus_write_reg(cs, rfm95w::reg::kPreambleMsb, 0x00);
    spi_bus_write_reg(cs, rfm95w::reg::kPreambleLsb, 0x08);

    rfm95w_set_tx_power(dev, kDefaultTxPowerDbm);
    spi_bus_write_reg(cs, rfm95w::reg::kSyncWord, kSyncWordDefault);
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
    sleep_ms(kModeSettleMs);
    set_mode(dev, rfm95w::mode::kSleep);
    sleep_ms(kModeSettleMs);

    configure_modem(dev);
    set_mode(dev, rfm95w::mode::kStandby);

    dev->initialized = true;
    return true;
}

// ============================================================================
// Non-Blocking TX (IVP-92)
// ============================================================================

bool rfm95w_send_start(rfm95w_t* dev, const uint8_t* data, uint8_t len) {
    if (!dev->initialized || len == 0 || len > rfm95w::kMaxPayload) {
        return false;
    }

    // Set Standby before writing FIFO
    set_mode(dev, rfm95w::mode::kStandby);

    // Council #4: Set FIFO pointer to TX base before every write
    uint8_t tx_base = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kFifoTxBase);
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFifoAddrPtr, tx_base);

    // Write payload to FIFO
    spi_bus_write_burst(dev->cs_pin, rfm95w::reg::kFifo, data, len);

    // Set payload length
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPayloadLength, len);

    // Clear all IRQ flags before TX
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kIrqFlags, rfm95w::irq::kAll);

    // Map DIO0 to TxDone (bits [7:6] = 01)
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kDioMapping1, 0x40);

    // Record start time for timeout detection in send_poll()
    dev->tx_start_us = time_us_64();

    // Set TX mode — returns immediately, radio transmits autonomously
    set_mode(dev, rfm95w::mode::kTx);

    return true;
}

TxPollResult rfm95w_send_poll(rfm95w_t* dev) {
    if (!dev->initialized) {
        return TxPollResult::kTimeout;
    }

    // Read IRQ flags register (latched — Council C3-R3: not GPIO DIO0)
    uint8_t irq_flags = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kIrqFlags);

    if (irq_flags & rfm95w::irq::kTxDone) {
        // TX complete — clear flags, restore DIO0 mapping, return to Standby
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kIrqFlags, rfm95w::irq::kAll);
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kDioMapping1, 0x00);
        set_mode(dev, rfm95w::mode::kStandby);
        return TxPollResult::kDone;
    }

    // Check timeout
    if ((time_us_64() - dev->tx_start_us) > rfm95w::kTxTimeoutUs) {
        // Timeout — clear flags, restore DIO0 mapping, return to Standby
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kIrqFlags, rfm95w::irq::kAll);
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kDioMapping1, 0x00);
        set_mode(dev, rfm95w::mode::kStandby);
        return TxPollResult::kTimeout;
    }

    return TxPollResult::kBusy;
}

// ============================================================================
// Blocking TX (convenience wrapper — calls send_start + busy-waits send_poll)
// ============================================================================

bool rfm95w_send(rfm95w_t* dev, const uint8_t* data, uint8_t len) {
    if (!rfm95w_send_start(dev, data, len)) {
        return false;
    }

    for (;;) {
        TxPollResult result = rfm95w_send_poll(dev);
        if (result == TxPollResult::kDone) {
            return true;
        }
        if (result == TxPollResult::kTimeout) {
            return false;
        }
        // kBusy — continue polling
    }
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
    dev->last_rssi = static_cast<int16_t>(kRssiOffsetHf + raw_rssi);

    // Record SNR: SNR(dB) = RegPktSnrValue / 4 (signed, 2's complement)
    int8_t raw_snr = static_cast<int8_t>(
        spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kPktSnrValue));
    dev->last_snr = static_cast<int8_t>(raw_snr / kSnrDivisor);

    return nb_bytes;
}

bool rfm95w_available(rfm95w_t* dev) {
    if (!dev->initialized) {
        return false;
    }

    // Check IRQ flags register directly for RxDone.
    // GPIO DIO0 polling is unreliable on some boards (Fruit Jam GPIO5
    // shared with Button3 may have external pull-down clamping DIO0).
    // Register read is authoritative — SX1276 sets RxDone bit regardless
    // of DIO0 pin state.
    uint8_t irq_flags = spi_bus_read_reg(dev->cs_pin,
                                         rfm95w::reg::kIrqFlags);
    return (irq_flags & rfm95w::irq::kRxDone) != 0;
}

bool rfm95w_poll_irq(rfm95w_t* dev) {
    // Council #6: Isolated poll function for future ISR swap.
    // Currently polls GPIO; future version can check a flag set by ISR.
    return gpio_get(dev->irq_pin) != 0;
}

void rfm95w_set_frequency(rfm95w_t* dev, uint32_t freq_hz) {
    // Council #1: Use 64-bit arithmetic — 32-bit overflows at 915 MHz
    // Frf = (freq_hz * 2^19) / F_XOSC, where F_XOSC = 32 MHz
    uint64_t frf = (static_cast<uint64_t>(freq_hz) << kFreqRegShift) / kFxoscHz;

    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFrMsb,
                      static_cast<uint8_t>((frf >> 16) & 0xFF));
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFrMid,
                      static_cast<uint8_t>((frf >> 8) & 0xFF));
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kFrLsb,
                      static_cast<uint8_t>(frf & 0xFF));
}

void rfm95w_set_tx_power(rfm95w_t* dev, int8_t dbm) {
    // Clamp to valid range for PA_BOOST
    if (dbm < kPaBoostMinDbm) { dbm = kPaBoostMinDbm; }
    if (dbm > kPaBoostMaxDbm) { dbm = kPaBoostMaxDbm; }

    if (dbm > kPaDacThreshDbm) {
        // High power mode: enable PA_DAC for +20 dBm
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPaDac, kPaDacHighPower);
        // RegPaConfig: PA_BOOST=1, MaxPower=7, OutputPower = dbm - 5
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPaConfig,
                          static_cast<uint8_t>(kPaBoostBit | kMaxPowerBits | (dbm - kPaOffsetHighPower)));
    } else {
        // Normal mode: disable PA_DAC
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPaDac, kPaDacNormal);
        // RegPaConfig: PA_BOOST=1, MaxPower=7, OutputPower = dbm - 2
        spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kPaConfig,
                          static_cast<uint8_t>(kPaBoostBit | kMaxPowerBits | (dbm - kPaOffsetNormal)));
    }
}

int16_t rfm95w_rssi(const rfm95w_t* dev) {
    return dev->last_rssi;
}

void rfm95w_set_bandwidth(rfm95w_t* dev, uint8_t bw) {
    if (!dev->initialized) { return; }

    // Read current RegModemConfig1, replace BW bits [7:4], preserve CR and header bits
    uint8_t cfg1 = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kModemConfig1);
    cfg1 = static_cast<uint8_t>((cfg1 & kBwLowerNibbleMask) | (bw << 4));
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kModemConfig1, cfg1);
}

void rfm95w_set_spreading_factor(rfm95w_t* dev, uint8_t sf) {
    if (!dev->initialized) { return; }
    if (sf < 6) { sf = 6; }
    if (sf > 12) { sf = 12; }

    // RegModemConfig2[7:4] = SF, preserve CRC and other bits [3:0]
    uint8_t cfg2 = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kModemConfig2);
    cfg2 = static_cast<uint8_t>((cfg2 & 0x0F) | (sf << 4));
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kModemConfig2, cfg2);
}

void rfm95w_set_coding_rate(rfm95w_t* dev, uint8_t cr) {
    if (!dev->initialized) { return; }
    // cr = denominator: 5-8 (CR 4/5 through 4/8)
    if (cr < 5) { cr = 5; }
    if (cr > 8) { cr = 8; }

    // RegModemConfig1[3:1] = CR (1=4/5, 2=4/6, 3=4/7, 4=4/8)
    // Encoding: register value = cr - 4
    uint8_t cr_bits = static_cast<uint8_t>(cr - 4);
    uint8_t cfg1 = spi_bus_read_reg(dev->cs_pin, rfm95w::reg::kModemConfig1);
    cfg1 = static_cast<uint8_t>((cfg1 & 0xF1) | (cr_bits << 1));
    spi_bus_write_reg(dev->cs_pin, rfm95w::reg::kModemConfig1, cfg1);
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
