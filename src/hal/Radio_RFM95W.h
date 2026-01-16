/**
 * @file Radio_RFM95W.h
 * @brief RFM95W/SX1276 LoRa radio driver for RocketChip
 *
 * Simple LoRa driver for wireless debugging and telemetry.
 * Uses SPI communication with the Semtech SX1276 chip.
 *
 * @note Default pins for Feather RP2350 + Radio FeatherWing
 * @see https://www.adafruit.com/product/3231
 */

#ifndef ROCKETCHIP_HAL_RADIO_RFM95W_H
#define ROCKETCHIP_HAL_RADIO_RFM95W_H

#include <cstdint>
#include <cstddef>

namespace rocketchip {
namespace hal {

/**
 * @brief Radio operation result codes
 */
enum class RadioResult : uint8_t {
    OK = 0,
    ERR_TIMEOUT,
    ERR_CRC,
    ERR_NO_PACKET,
    ERR_PACKET_TOO_LONG,
    ERR_NOT_INITIALIZED,
    ERR_INVALID_PARAM,
    ERR_HARDWARE
};

/**
 * @brief Radio configuration
 */
struct RadioConfig {
    float frequency_mhz;    // Center frequency (e.g., 915.0 for US ISM)
    int8_t tx_power_dbm;    // TX power (2-20 dBm)
    uint8_t spreading_factor; // 6-12 (higher = longer range, slower)
    uint32_t bandwidth_hz;  // 125000, 250000, or 500000
    uint8_t coding_rate;    // 5-8 (4/5 to 4/8)

    // Defaults for good range/speed balance
    RadioConfig()
        : frequency_mhz(915.0f)
        , tx_power_dbm(17)
        , spreading_factor(7)
        , bandwidth_hz(125000)
        , coding_rate(5)
    {}
};

/**
 * @brief Received packet info
 */
struct RadioPacket {
    uint8_t data[255];
    uint8_t length;
    int16_t rssi;           // Signal strength in dBm
    int8_t snr;             // Signal-to-noise ratio in dB
};

/**
 * @brief RFM95W/SX1276 LoRa Radio Driver
 *
 * Provides simple TX/RX for wireless serial bridge and telemetry.
 *
 * @code
 * Radio_RFM95W radio;
 *
 * if (radio.begin()) {
 *     // Transmit
 *     radio.send((uint8_t*)"Hello", 5);
 *
 *     // Receive
 *     RadioPacket pkt;
 *     if (radio.receive(pkt, 1000) == RadioResult::OK) {
 *         printf("Got %d bytes, RSSI=%d\n", pkt.length, pkt.rssi);
 *     }
 * }
 * @endcode
 */
class Radio_RFM95W {
public:
    // Max packet size for LoRa mode
    static constexpr size_t MAX_PACKET_SIZE = 255;

    // Default pins for Feather RP2350 + Radio FeatherWing
    // Configured for jumpers: CS="B"(D10), RST="A"(D11), IRQ="D"(D6)
    // On Feather RP2350, Dx pin labels = GPIOx for these pins
    static constexpr uint8_t DEFAULT_CS_PIN   = 10;  // Feather D10 / jumper "B"
    static constexpr uint8_t DEFAULT_RST_PIN  = 11;  // Feather D11 / jumper "A"
    static constexpr uint8_t DEFAULT_IRQ_PIN  = 6;   // Feather D6  / jumper "D" (DIO0)

    // SPI pins (Feather RP2350 SPI0)
    static constexpr uint8_t DEFAULT_SCK_PIN  = 18;  // Feather SCK
    static constexpr uint8_t DEFAULT_MOSI_PIN = 19;  // Feather MO
    static constexpr uint8_t DEFAULT_MISO_PIN = 16;  // Feather MI

    /**
     * @brief Construct radio with default pins
     */
    Radio_RFM95W();

    /**
     * @brief Construct radio with custom pins
     */
    Radio_RFM95W(uint8_t cs_pin, uint8_t rst_pin, uint8_t irq_pin,
                 uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin);

    ~Radio_RFM95W();

    /**
     * @brief Initialize the radio with default config
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Initialize the radio with custom config
     */
    bool begin(const RadioConfig& config);

    /**
     * @brief Check if radio hardware is present
     */
    bool isConnected();

    /**
     * @brief Send a packet
     * @param data Data to send
     * @param length Number of bytes (max 255)
     * @param timeout_ms Timeout for TX complete
     * @return RadioResult::OK on success
     */
    RadioResult send(const uint8_t* data, size_t length, uint32_t timeout_ms = 2000);

    /**
     * @brief Send a string (convenience)
     */
    RadioResult send(const char* str, uint32_t timeout_ms = 2000);

    /**
     * @brief Printf-style send for debug output
     */
    RadioResult printf(const char* fmt, ...);

    /**
     * @brief Check if a packet is available
     */
    bool available();

    /**
     * @brief Receive a packet (blocking with timeout)
     * @param packet Output packet structure
     * @param timeout_ms Timeout in milliseconds (0 = non-blocking check)
     * @return RadioResult::OK if packet received
     */
    RadioResult receive(RadioPacket& packet, uint32_t timeout_ms = 1000);

    /**
     * @brief Put radio in continuous receive mode
     */
    void startReceive();

    /**
     * @brief Put radio in standby/idle mode
     */
    void standby();

    /**
     * @brief Put radio in sleep mode (lowest power)
     */
    void sleep();

    /**
     * @brief Get last RSSI reading
     */
    int16_t getLastRSSI() const { return m_last_rssi; }

    /**
     * @brief Get last SNR reading
     */
    int8_t getLastSNR() const { return m_last_snr; }

    /**
     * @brief Get packet count (TX + RX)
     */
    uint32_t getPacketCount() const { return m_packet_count; }

private:
    // SPI communication
    void spiBegin();
    uint8_t spiRead(uint8_t reg);
    void spiWrite(uint8_t reg, uint8_t value);
    void spiBurstRead(uint8_t reg, uint8_t* buffer, size_t length);
    void spiBurstWrite(uint8_t reg, const uint8_t* buffer, size_t length);

    // Internal helpers
    void reset();
    void setMode(uint8_t mode);
    void setFrequency(float mhz);
    void setTxPower(int8_t dbm);
    void setSpreadingFactor(uint8_t sf);
    void setBandwidth(uint32_t bw);
    void setCodingRate(uint8_t cr);
    bool waitForIRQ(uint32_t timeout_ms);

    // Pins
    uint8_t m_cs_pin;
    uint8_t m_rst_pin;
    uint8_t m_irq_pin;
    uint8_t m_sck_pin;
    uint8_t m_mosi_pin;
    uint8_t m_miso_pin;

    // State
    void* m_spi;            // SPI instance
    bool m_initialized;
    RadioConfig m_config;
    int16_t m_last_rssi;
    int8_t m_last_snr;
    uint32_t m_packet_count;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_RADIO_RFM95W_H
