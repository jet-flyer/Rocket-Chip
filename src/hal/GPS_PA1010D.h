/**
 * @file GPS_PA1010D.h
 * @brief PA1010D GPS module driver for RocketChip
 *
 * I2C driver for the MediaTek MT3333-based PA1010D GPS module.
 * Parses NMEA sentences (GGA, RMC, VTG) to provide position, velocity,
 * and timing data for sensor fusion.
 *
 * @note I2C address 0x10, up to 10Hz update rate
 * @see https://www.adafruit.com/product/4415
 */

#ifndef ROCKETCHIP_HAL_GPS_PA1010D_H
#define ROCKETCHIP_HAL_GPS_PA1010D_H

#include "Bus.h"
#include <cstdint>

namespace rocketchip {
namespace hal {

/**
 * @brief GPS fix quality indicators
 */
enum class GPSFixQuality : uint8_t {
    NO_FIX          = 0,
    GPS_FIX         = 1,  // Standard GPS
    DGPS_FIX        = 2,  // Differential GPS
    PPS_FIX         = 3,  // Precise Positioning Service
    RTK_FIX         = 4,  // Real-Time Kinematic
    RTK_FLOAT       = 5,  // RTK Float
    ESTIMATED       = 6,  // Dead reckoning
    MANUAL_INPUT    = 7,
    SIMULATION      = 8
};

/**
 * @brief GPS fix type (from GSA)
 */
enum class GPSFixType : uint8_t {
    NO_FIX = 1,
    FIX_2D = 2,
    FIX_3D = 3
};

/**
 * @brief GPS time structure
 */
struct GPSTime {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    bool valid;

    GPSTime() : hour(0), minute(0), second(0), millisecond(0),
                day(0), month(0), year(0), valid(false) {}
};

/**
 * @brief GPS position and velocity data
 */
struct GPSData {
    // Position
    double latitude;        // Degrees (positive = N, negative = S)
    double longitude;       // Degrees (positive = E, negative = W)
    float altitude_msl;     // Meters above mean sea level
    float geoid_sep;        // Geoid separation in meters

    // Velocity
    float speed_knots;      // Speed over ground in knots
    float speed_mps;        // Speed over ground in m/s
    float course_deg;       // Course over ground in degrees (true north)

    // Quality
    GPSFixQuality fix_quality;
    GPSFixType fix_type;
    uint8_t satellites;     // Number of satellites in use
    float hdop;             // Horizontal dilution of precision
    float vdop;             // Vertical dilution of precision
    float pdop;             // Position dilution of precision

    // Time
    GPSTime time;

    // Status flags
    bool position_valid;
    bool velocity_valid;
    bool time_valid;

    GPSData() : latitude(0), longitude(0), altitude_msl(0), geoid_sep(0),
                speed_knots(0), speed_mps(0), course_deg(0),
                fix_quality(GPSFixQuality::NO_FIX), fix_type(GPSFixType::NO_FIX),
                satellites(0), hdop(99.99f), vdop(99.99f), pdop(99.99f),
                position_valid(false), velocity_valid(false), time_valid(false) {}
};

/**
 * @brief GPS update rate options
 */
enum class GPSUpdateRate : uint8_t {
    RATE_1HZ  = 1,
    RATE_5HZ  = 5,
    RATE_10HZ = 10
};

/**
 * @brief PA1010D GPS module driver
 *
 * Reads NMEA sentences over I2C and parses position, velocity, and time.
 * Designed for integration with EKF3 sensor fusion.
 *
 * @code
 * I2CBus bus(i2c1, GPS_PA1010D::I2C_ADDR, SDA_PIN, SCL_PIN);
 * GPS_PA1010D gps(&bus);
 *
 * if (gps.begin()) {
 *     gps.setUpdateRate(GPSUpdateRate::RATE_10HZ);
 *
 *     while (true) {
 *         gps.update();  // Call frequently to process incoming data
 *
 *         if (gps.hasNewData()) {
 *             GPSData data = gps.getData();
 *             if (data.position_valid) {
 *                 // Use position for fusion...
 *             }
 *         }
 *     }
 * }
 * @endcode
 */
class GPS_PA1010D {
public:
    static constexpr uint8_t I2C_ADDR = 0x10;
    static constexpr size_t RX_BUFFER_SIZE = 256;
    static constexpr size_t SENTENCE_BUFFER_SIZE = 128;

    /**
     * @brief Construct GPS driver instance
     * @param bus Pointer to initialized I2CBus (GPS uses raw I2C, not register-based)
     */
    explicit GPS_PA1010D(I2CBus* bus);

    /**
     * @brief Initialize the GPS module
     *
     * Configures default NMEA sentence output (GGA, RMC, VTG, GSA).
     *
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Check if GPS is responding
     * @return true if I2C communication successful
     */
    bool isConnected();

    /**
     * @brief Process incoming GPS data
     *
     * Call this frequently (at least 10x the GPS update rate) to read
     * and parse incoming NMEA sentences. Non-blocking.
     *
     * @return true if new complete sentence was parsed
     */
    bool update();

    /**
     * @brief Check if new GPS data is available since last read
     */
    bool hasNewData() const { return m_new_data; }

    /**
     * @brief Get the latest GPS data
     *
     * Clears the new data flag.
     */
    GPSData getData();

    /**
     * @brief Get const reference to latest GPS data without clearing flag
     */
    const GPSData& peekData() const { return m_data; }

    /**
     * @brief Check if we have a valid 3D fix
     */
    bool hasFix() const;

    /**
     * @brief Set GPS update rate
     */
    bool setUpdateRate(GPSUpdateRate rate);

    /**
     * @brief Configure which NMEA sentences to output
     * @param gga Enable GGA (position, altitude)
     * @param rmc Enable RMC (position, velocity, time)
     * @param vtg Enable VTG (velocity, course)
     * @param gsa Enable GSA (DOP values)
     */
    bool configureNMEA(bool gga, bool rmc, bool vtg, bool gsa);

    /**
     * @brief Send raw PMTK command
     * @param cmd Command string without $ prefix or checksum (e.g., "PMTK314,...")
     */
    bool sendCommand(const char* cmd);

    /**
     * @brief Get number of valid fixes received
     */
    uint32_t getFixCount() const { return m_fix_count; }

    /**
     * @brief Get number of sentences parsed
     */
    uint32_t getSentenceCount() const { return m_sentence_count; }

private:
    // I2C read buffer management
    bool readI2CData();
    void processBuffer();

    // NMEA parsing
    bool parseSentence(const char* sentence);
    bool parseGGA(const char* sentence);
    bool parseRMC(const char* sentence);
    bool parseVTG(const char* sentence);
    bool parseGSA(const char* sentence);

    // NMEA helpers
    static bool validateChecksum(const char* sentence);
    static uint8_t calculateChecksum(const char* sentence);
    static double parseLatLon(const char* field, char direction);
    static float parseFloat(const char* field);
    static int parseInt(const char* field);
    static bool getField(const char* sentence, int fieldNum, char* out, size_t maxLen);

    I2CBus* m_bus;
    GPSData m_data;
    bool m_initialized;
    bool m_new_data;

    // Receive buffer for accumulating I2C reads
    char m_rx_buffer[RX_BUFFER_SIZE];
    size_t m_rx_index;

    // Sentence buffer for current NMEA sentence being built
    char m_sentence[SENTENCE_BUFFER_SIZE];
    size_t m_sentence_index;
    bool m_in_sentence;

    // Statistics
    uint32_t m_fix_count;
    uint32_t m_sentence_count;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_GPS_PA1010D_H
