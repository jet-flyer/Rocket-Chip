/**
 * @file Radio_RFM95W.cpp
 * @brief RFM95W/SX1276 LoRa radio driver implementation
 */

#include "Radio_RFM95W.h"
#include "Timing.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <cstring>
#include <cstdio>
#include <cstdarg>

namespace rocketchip {
namespace hal {

// ============================================================================
// SX1276 Register Definitions
// ============================================================================

namespace {
    // Register addresses
    constexpr uint8_t REG_FIFO                 = 0x00;
    constexpr uint8_t REG_OP_MODE              = 0x01;
    constexpr uint8_t REG_FRF_MSB              = 0x06;
    constexpr uint8_t REG_FRF_MID              = 0x07;
    constexpr uint8_t REG_FRF_LSB              = 0x08;
    constexpr uint8_t REG_PA_CONFIG            = 0x09;
    constexpr uint8_t REG_PA_RAMP              = 0x0A;
    constexpr uint8_t REG_OCP                  = 0x0B;
    constexpr uint8_t REG_LNA                  = 0x0C;
    constexpr uint8_t REG_FIFO_ADDR_PTR        = 0x0D;
    constexpr uint8_t REG_FIFO_TX_BASE_ADDR    = 0x0E;
    constexpr uint8_t REG_FIFO_RX_BASE_ADDR    = 0x0F;
    constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
    constexpr uint8_t REG_IRQ_FLAGS_MASK       = 0x11;
    constexpr uint8_t REG_IRQ_FLAGS            = 0x12;
    constexpr uint8_t REG_RX_NB_BYTES          = 0x13;
    constexpr uint8_t REG_PKT_SNR_VALUE        = 0x19;
    constexpr uint8_t REG_PKT_RSSI_VALUE       = 0x1A;
    constexpr uint8_t REG_MODEM_CONFIG_1       = 0x1D;
    constexpr uint8_t REG_MODEM_CONFIG_2       = 0x1E;
    constexpr uint8_t REG_SYMB_TIMEOUT_LSB     = 0x1F;
    constexpr uint8_t REG_PREAMBLE_MSB         = 0x20;
    constexpr uint8_t REG_PREAMBLE_LSB         = 0x21;
    constexpr uint8_t REG_PAYLOAD_LENGTH       = 0x22;
    constexpr uint8_t REG_MAX_PAYLOAD_LENGTH   = 0x23;
    constexpr uint8_t REG_MODEM_CONFIG_3       = 0x26;
    constexpr uint8_t REG_DETECTION_OPTIMIZE   = 0x31;
    constexpr uint8_t REG_DETECTION_THRESHOLD  = 0x37;
    constexpr uint8_t REG_SYNC_WORD            = 0x39;
    constexpr uint8_t REG_DIO_MAPPING_1        = 0x40;
    constexpr uint8_t REG_VERSION              = 0x42;
    constexpr uint8_t REG_PA_DAC               = 0x4D;

    // Operation modes
    constexpr uint8_t MODE_SLEEP              = 0x00;
    constexpr uint8_t MODE_STANDBY            = 0x01;
    constexpr uint8_t MODE_TX                 = 0x03;
    constexpr uint8_t MODE_RX_CONTINUOUS      = 0x05;
    constexpr uint8_t MODE_RX_SINGLE          = 0x06;
    constexpr uint8_t MODE_LONG_RANGE         = 0x80;  // LoRa mode bit

    // IRQ flags
    constexpr uint8_t IRQ_RX_TIMEOUT          = 0x80;
    constexpr uint8_t IRQ_RX_DONE             = 0x40;
    constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR   = 0x20;
    constexpr uint8_t IRQ_TX_DONE             = 0x08;

    // PA config
    constexpr uint8_t PA_BOOST                = 0x80;

    // Expected chip version
    constexpr uint8_t SX1276_VERSION          = 0x12;

    // Frequency calculation constant (32MHz crystal / 2^19)
    constexpr float FREQ_STEP = 32000000.0f / 524288.0f;  // ~61.035 Hz
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

Radio_RFM95W::Radio_RFM95W()
    : m_cs_pin(DEFAULT_CS_PIN)
    , m_rst_pin(DEFAULT_RST_PIN)
    , m_irq_pin(DEFAULT_IRQ_PIN)
    , m_sck_pin(DEFAULT_SCK_PIN)
    , m_mosi_pin(DEFAULT_MOSI_PIN)
    , m_miso_pin(DEFAULT_MISO_PIN)
    , m_spi(nullptr)
    , m_initialized(false)
    , m_config()
    , m_last_rssi(0)
    , m_last_snr(0)
    , m_packet_count(0)
{
}

Radio_RFM95W::Radio_RFM95W(uint8_t cs_pin, uint8_t rst_pin, uint8_t irq_pin,
                           uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin)
    : m_cs_pin(cs_pin)
    , m_rst_pin(rst_pin)
    , m_irq_pin(irq_pin)
    , m_sck_pin(sck_pin)
    , m_mosi_pin(mosi_pin)
    , m_miso_pin(miso_pin)
    , m_spi(nullptr)
    , m_initialized(false)
    , m_config()
    , m_last_rssi(0)
    , m_last_snr(0)
    , m_packet_count(0)
{
}

Radio_RFM95W::~Radio_RFM95W()
{
    if (m_initialized) {
        sleep();
    }
}

// ============================================================================
// Initialization
// ============================================================================

bool Radio_RFM95W::begin()
{
    return begin(RadioConfig());
}

bool Radio_RFM95W::begin(const RadioConfig& config)
{
    m_config = config;

    // Initialize SPI
    spiBegin();

    // Initialize control pins
    gpio_init(m_cs_pin);
    gpio_set_dir(m_cs_pin, GPIO_OUT);
    gpio_put(m_cs_pin, 1);  // Deselect

    gpio_init(m_rst_pin);
    gpio_set_dir(m_rst_pin, GPIO_OUT);

    gpio_init(m_irq_pin);
    gpio_set_dir(m_irq_pin, GPIO_IN);

    // Reset the radio
    reset();

    // Check chip version
    uint8_t version = spiRead(REG_VERSION);
    if (version != SX1276_VERSION) {
        return false;
    }

    // Put in sleep mode for configuration
    sleep();

    // Set LoRa mode (must be done in sleep mode)
    spiWrite(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);
    Timing::delayMs(10);

    // Configure FIFO pointers
    spiWrite(REG_FIFO_TX_BASE_ADDR, 0x00);
    spiWrite(REG_FIFO_RX_BASE_ADDR, 0x00);

    // Set LNA boost
    spiWrite(REG_LNA, spiRead(REG_LNA) | 0x03);

    // Set auto AGC
    spiWrite(REG_MODEM_CONFIG_3, 0x04);

    // Configure radio parameters
    setFrequency(m_config.frequency_mhz);
    setTxPower(m_config.tx_power_dbm);
    setSpreadingFactor(m_config.spreading_factor);
    setBandwidth(m_config.bandwidth_hz);
    setCodingRate(m_config.coding_rate);

    // Set preamble length (8 symbols)
    spiWrite(REG_PREAMBLE_MSB, 0x00);
    spiWrite(REG_PREAMBLE_LSB, 0x08);

    // Set sync word (0x12 = private, 0x34 = LoRaWAN public)
    spiWrite(REG_SYNC_WORD, 0x12);

    // Enable CRC
    uint8_t mc2 = spiRead(REG_MODEM_CONFIG_2);
    spiWrite(REG_MODEM_CONFIG_2, mc2 | 0x04);

    // Map DIO0 to RxDone/TxDone
    spiWrite(REG_DIO_MAPPING_1, 0x00);

    // Go to standby
    standby();

    m_initialized = true;
    return true;
}

bool Radio_RFM95W::isConnected()
{
    uint8_t version = spiRead(REG_VERSION);
    return (version == SX1276_VERSION);
}

// ============================================================================
// SPI Communication
// ============================================================================

void Radio_RFM95W::spiBegin()
{
    // Use SPI0 by default
    m_spi = spi0;
    spi_inst_t* spi = static_cast<spi_inst_t*>(m_spi);

    // Initialize at 8MHz
    spi_init(spi, 8000000);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(m_sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(m_mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(m_miso_pin, GPIO_FUNC_SPI);
}

uint8_t Radio_RFM95W::spiRead(uint8_t reg)
{
    spi_inst_t* spi = static_cast<spi_inst_t*>(m_spi);

    gpio_put(m_cs_pin, 0);

    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), 0x00 };
    uint8_t rx[2];
    spi_write_read_blocking(spi, tx, rx, 2);

    gpio_put(m_cs_pin, 1);

    return rx[1];
}

void Radio_RFM95W::spiWrite(uint8_t reg, uint8_t value)
{
    spi_inst_t* spi = static_cast<spi_inst_t*>(m_spi);

    gpio_put(m_cs_pin, 0);

    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), value };
    spi_write_blocking(spi, tx, 2);

    gpio_put(m_cs_pin, 1);
}

void Radio_RFM95W::spiBurstRead(uint8_t reg, uint8_t* buffer, size_t length)
{
    spi_inst_t* spi = static_cast<spi_inst_t*>(m_spi);

    gpio_put(m_cs_pin, 0);

    uint8_t cmd = reg & 0x7F;
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, buffer, length);

    gpio_put(m_cs_pin, 1);
}

void Radio_RFM95W::spiBurstWrite(uint8_t reg, const uint8_t* buffer, size_t length)
{
    spi_inst_t* spi = static_cast<spi_inst_t*>(m_spi);

    gpio_put(m_cs_pin, 0);

    uint8_t cmd = reg | 0x80;
    spi_write_blocking(spi, &cmd, 1);
    spi_write_blocking(spi, buffer, length);

    gpio_put(m_cs_pin, 1);
}

// ============================================================================
// Internal Configuration
// ============================================================================

void Radio_RFM95W::reset()
{
    gpio_put(m_rst_pin, 0);
    Timing::delayMs(10);
    gpio_put(m_rst_pin, 1);
    Timing::delayMs(10);
}

void Radio_RFM95W::setMode(uint8_t mode)
{
    spiWrite(REG_OP_MODE, MODE_LONG_RANGE | mode);
}

void Radio_RFM95W::setFrequency(float mhz)
{
    uint32_t frf = static_cast<uint32_t>((mhz * 1000000.0f) / FREQ_STEP);
    spiWrite(REG_FRF_MSB, (frf >> 16) & 0xFF);
    spiWrite(REG_FRF_MID, (frf >> 8) & 0xFF);
    spiWrite(REG_FRF_LSB, frf & 0xFF);
}

void Radio_RFM95W::setTxPower(int8_t dbm)
{
    // Clamp to valid range
    if (dbm < 2) dbm = 2;
    if (dbm > 20) dbm = 20;

    if (dbm > 17) {
        // Use PA_DAC for +20dBm
        spiWrite(REG_PA_DAC, 0x87);  // Enable high power mode
        dbm -= 3;  // Adjust for PA_DAC offset
    } else {
        spiWrite(REG_PA_DAC, 0x84);  // Normal mode
    }

    // PA_BOOST pin, max power setting
    spiWrite(REG_PA_CONFIG, PA_BOOST | (dbm - 2));
}

void Radio_RFM95W::setSpreadingFactor(uint8_t sf)
{
    if (sf < 6) sf = 6;
    if (sf > 12) sf = 12;

    // Detection optimization for SF6
    if (sf == 6) {
        spiWrite(REG_DETECTION_OPTIMIZE, 0xC5);
        spiWrite(REG_DETECTION_THRESHOLD, 0x0C);
    } else {
        spiWrite(REG_DETECTION_OPTIMIZE, 0xC3);
        spiWrite(REG_DETECTION_THRESHOLD, 0x0A);
    }

    uint8_t mc2 = spiRead(REG_MODEM_CONFIG_2);
    spiWrite(REG_MODEM_CONFIG_2, (mc2 & 0x0F) | (sf << 4));
}

void Radio_RFM95W::setBandwidth(uint32_t bw)
{
    uint8_t bwReg;
    switch (bw) {
        case 7800:   bwReg = 0; break;
        case 10400:  bwReg = 1; break;
        case 15600:  bwReg = 2; break;
        case 20800:  bwReg = 3; break;
        case 31250:  bwReg = 4; break;
        case 41700:  bwReg = 5; break;
        case 62500:  bwReg = 6; break;
        case 125000: bwReg = 7; break;
        case 250000: bwReg = 8; break;
        case 500000: bwReg = 9; break;
        default:     bwReg = 7; break;  // Default 125kHz
    }

    uint8_t mc1 = spiRead(REG_MODEM_CONFIG_1);
    spiWrite(REG_MODEM_CONFIG_1, (mc1 & 0x0F) | (bwReg << 4));
}

void Radio_RFM95W::setCodingRate(uint8_t cr)
{
    if (cr < 5) cr = 5;
    if (cr > 8) cr = 8;

    uint8_t mc1 = spiRead(REG_MODEM_CONFIG_1);
    spiWrite(REG_MODEM_CONFIG_1, (mc1 & 0xF1) | ((cr - 4) << 1));
}

bool Radio_RFM95W::waitForIRQ(uint32_t timeout_ms)
{
    uint32_t start = Timing::millis();
    while ((Timing::millis() - start) < timeout_ms) {
        if (gpio_get(m_irq_pin)) {
            return true;
        }
        Timing::delayMs(1);
    }
    return false;
}

// ============================================================================
// Mode Control
// ============================================================================

void Radio_RFM95W::standby()
{
    setMode(MODE_STANDBY);
}

void Radio_RFM95W::sleep()
{
    setMode(MODE_SLEEP);
}

void Radio_RFM95W::startReceive()
{
    // Clear IRQ flags
    spiWrite(REG_IRQ_FLAGS, 0xFF);

    // Set RX base address
    spiWrite(REG_FIFO_ADDR_PTR, 0x00);

    // Enter continuous receive mode
    setMode(MODE_RX_CONTINUOUS);
}

// ============================================================================
// Transmit
// ============================================================================

RadioResult Radio_RFM95W::send(const uint8_t* data, size_t length, uint32_t timeout_ms)
{
    if (!m_initialized) {
        return RadioResult::ERR_NOT_INITIALIZED;
    }

    if (data == nullptr || length == 0) {
        return RadioResult::ERR_INVALID_PARAM;
    }

    if (length > MAX_PACKET_SIZE) {
        return RadioResult::ERR_PACKET_TOO_LONG;
    }

    // Go to standby
    standby();

    // Clear IRQ flags
    spiWrite(REG_IRQ_FLAGS, 0xFF);

    // Set TX base address
    spiWrite(REG_FIFO_ADDR_PTR, 0x00);

    // Write payload to FIFO
    spiBurstWrite(REG_FIFO, data, length);

    // Set payload length
    spiWrite(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(length));

    // Start transmission
    setMode(MODE_TX);

    // Wait for TX done
    if (!waitForIRQ(timeout_ms)) {
        standby();
        return RadioResult::ERR_TIMEOUT;
    }

    // Check TX done flag
    uint8_t irq = spiRead(REG_IRQ_FLAGS);
    spiWrite(REG_IRQ_FLAGS, 0xFF);  // Clear flags

    if (!(irq & IRQ_TX_DONE)) {
        standby();
        return RadioResult::ERR_HARDWARE;
    }

    // Back to standby
    standby();

    m_packet_count++;
    return RadioResult::OK;
}

RadioResult Radio_RFM95W::send(const char* str, uint32_t timeout_ms)
{
    return send(reinterpret_cast<const uint8_t*>(str), strlen(str), timeout_ms);
}

RadioResult Radio_RFM95W::printf(const char* fmt, ...)
{
    char buf[MAX_PACKET_SIZE];

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) {
        return RadioResult::ERR_INVALID_PARAM;
    }

    if (static_cast<size_t>(len) >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }

    return send(reinterpret_cast<uint8_t*>(buf), len);
}

// ============================================================================
// Receive
// ============================================================================

bool Radio_RFM95W::available()
{
    if (!m_initialized) {
        return false;
    }

    uint8_t irq = spiRead(REG_IRQ_FLAGS);
    return (irq & IRQ_RX_DONE) != 0;
}

RadioResult Radio_RFM95W::receive(RadioPacket& packet, uint32_t timeout_ms)
{
    if (!m_initialized) {
        return RadioResult::ERR_NOT_INITIALIZED;
    }

    // Start receive if not already in RX mode
    startReceive();

    // Wait for packet or timeout
    if (timeout_ms > 0) {
        if (!waitForIRQ(timeout_ms)) {
            return RadioResult::ERR_TIMEOUT;
        }
    } else {
        // Non-blocking check
        if (!gpio_get(m_irq_pin)) {
            return RadioResult::ERR_NO_PACKET;
        }
    }

    // Read IRQ flags
    uint8_t irq = spiRead(REG_IRQ_FLAGS);
    spiWrite(REG_IRQ_FLAGS, 0xFF);  // Clear flags

    // Check for CRC error
    if (irq & IRQ_PAYLOAD_CRC_ERROR) {
        startReceive();  // Resume RX
        return RadioResult::ERR_CRC;
    }

    // Check RX done
    if (!(irq & IRQ_RX_DONE)) {
        startReceive();  // Resume RX
        return RadioResult::ERR_NO_PACKET;
    }

    // Get packet length
    packet.length = spiRead(REG_RX_NB_BYTES);

    // Get FIFO pointer
    uint8_t fifoAddr = spiRead(REG_FIFO_RX_CURRENT_ADDR);
    spiWrite(REG_FIFO_ADDR_PTR, fifoAddr);

    // Read packet data
    spiBurstRead(REG_FIFO, packet.data, packet.length);

    // Get signal quality
    packet.rssi = -157 + spiRead(REG_PKT_RSSI_VALUE);
    int8_t snr_raw = static_cast<int8_t>(spiRead(REG_PKT_SNR_VALUE));
    packet.snr = snr_raw / 4;

    // Store for later access
    m_last_rssi = packet.rssi;
    m_last_snr = packet.snr;

    // Resume receive
    startReceive();

    m_packet_count++;
    return RadioResult::OK;
}

} // namespace hal
} // namespace rocketchip
