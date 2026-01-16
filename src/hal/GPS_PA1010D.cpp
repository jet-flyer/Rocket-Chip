/**
 * @file GPS_PA1010D.cpp
 * @brief PA1010D GPS module driver implementation
 *
 * I2C communication and NMEA parsing for MT3333-based GPS module.
 */

#include "GPS_PA1010D.h"
#include "Timing.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

namespace rocketchip {
namespace hal {

// PMTK command templates
namespace {
    // Configure NMEA sentence output
    // $PMTK314,GLL,RMC,VTG,GGA,GSA,GSV,...*CS
    // 0=disabled, 1=every fix, 2=every 2nd fix, etc.
    constexpr const char* PMTK_SET_NMEA_OUTPUT = "PMTK314";

    // Set update rate in milliseconds
    constexpr const char* PMTK_SET_RATE = "PMTK220";

    // Knots to m/s conversion
    constexpr float KNOTS_TO_MPS = 0.514444f;
}

// ============================================================================
// Constructor / Initialization
// ============================================================================

GPS_PA1010D::GPS_PA1010D(I2CBus* bus)
    : m_bus(bus)
    , m_data()
    , m_initialized(false)
    , m_new_data(false)
    , m_rx_index(0)
    , m_sentence_index(0)
    , m_in_sentence(false)
    , m_fix_count(0)
    , m_sentence_count(0)
{
    memset(m_rx_buffer, 0, sizeof(m_rx_buffer));
    memset(m_sentence, 0, sizeof(m_sentence));
}

bool GPS_PA1010D::begin()
{
    if (m_bus == nullptr) {
        return false;
    }

    // Initialize the bus
    if (!m_bus->begin()) {
        return false;
    }

    // Give GPS module time to boot
    Timing::delayMs(100);

    // Check if GPS is responding by attempting a read
    if (!isConnected()) {
        return false;
    }

    // Configure NMEA output: enable GGA, RMC, VTG, GSA; disable others
    if (!configureNMEA(true, true, true, true)) {
        // Non-fatal - GPS will use default output
    }

    m_initialized = true;
    return true;
}

bool GPS_PA1010D::isConnected()
{
    // Try to read a few bytes - GPS should always have data available
    uint8_t buf[4];
    BusResult result = m_bus->read(buf, 4);
    return (result == BusResult::OK);
}

// ============================================================================
// Data Reading
// ============================================================================

bool GPS_PA1010D::update()
{
    if (!m_initialized) {
        return false;
    }

    // Read available data from I2C
    if (!readI2CData()) {
        return false;
    }

    // Process buffer and look for complete sentences
    processBuffer();

    return m_new_data;
}

bool GPS_PA1010D::readI2CData()
{
    // Read a chunk of data from GPS
    // PA1010D will return 0x0A bytes when no data is available
    uint8_t buf[32];
    BusResult result = m_bus->read(buf, sizeof(buf));

    if (result != BusResult::OK) {
        return false;
    }

    // Append valid bytes to RX buffer
    for (size_t i = 0; i < sizeof(buf); i++) {
        uint8_t c = buf[i];

        // Skip padding bytes (0x0A when no real data)
        // Only accept printable ASCII and control chars we care about
        if (c == '$' || (c >= ' ' && c <= '~') || c == '\r' || c == '\n') {
            if (m_rx_index < RX_BUFFER_SIZE - 1) {
                m_rx_buffer[m_rx_index++] = static_cast<char>(c);
            }
        }
    }

    return true;
}

void GPS_PA1010D::processBuffer()
{
    // Process characters from RX buffer
    for (size_t i = 0; i < m_rx_index; i++) {
        char c = m_rx_buffer[i];

        if (c == '$') {
            // Start of new sentence
            m_sentence_index = 0;
            m_in_sentence = true;
            m_sentence[m_sentence_index++] = c;
        }
        else if (m_in_sentence) {
            if (c == '\r' || c == '\n') {
                // End of sentence
                if (m_sentence_index > 0) {
                    m_sentence[m_sentence_index] = '\0';

                    // Parse the complete sentence
                    if (parseSentence(m_sentence)) {
                        m_sentence_count++;
                    }
                }
                m_in_sentence = false;
                m_sentence_index = 0;
            }
            else if (m_sentence_index < SENTENCE_BUFFER_SIZE - 1) {
                m_sentence[m_sentence_index++] = c;
            }
            else {
                // Sentence too long, discard
                m_in_sentence = false;
                m_sentence_index = 0;
            }
        }
    }

    // Clear processed data from RX buffer
    m_rx_index = 0;
}

GPSData GPS_PA1010D::getData()
{
    m_new_data = false;
    return m_data;
}

bool GPS_PA1010D::hasFix() const
{
    return m_data.fix_type == GPSFixType::FIX_3D &&
           m_data.fix_quality != GPSFixQuality::NO_FIX;
}

// ============================================================================
// Configuration
// ============================================================================

bool GPS_PA1010D::setUpdateRate(GPSUpdateRate rate)
{
    char cmd[32];
    int period_ms;

    switch (rate) {
        case GPSUpdateRate::RATE_1HZ:  period_ms = 1000; break;
        case GPSUpdateRate::RATE_5HZ:  period_ms = 200; break;
        case GPSUpdateRate::RATE_10HZ: period_ms = 100; break;
        default: period_ms = 1000; break;
    }

    snprintf(cmd, sizeof(cmd), "%s,%d", PMTK_SET_RATE, period_ms);
    return sendCommand(cmd);
}

bool GPS_PA1010D::configureNMEA(bool gga, bool rmc, bool vtg, bool gsa)
{
    char cmd[64];
    // PMTK314: GLL,RMC,VTG,GGA,GSA,GSV,0,0,0,0,0,0,0,0,0,0,0,0,0
    snprintf(cmd, sizeof(cmd), "%s,0,%d,%d,%d,%d,0,0,0,0,0,0,0,0,0,0,0,0,0,0",
             PMTK_SET_NMEA_OUTPUT,
             rmc ? 1 : 0,
             vtg ? 1 : 0,
             gga ? 1 : 0,
             gsa ? 1 : 0);
    return sendCommand(cmd);
}

bool GPS_PA1010D::sendCommand(const char* cmd)
{
    if (m_bus == nullptr || cmd == nullptr) {
        return false;
    }

    // Calculate checksum
    uint8_t checksum = 0;
    for (const char* p = cmd; *p != '\0'; p++) {
        checksum ^= static_cast<uint8_t>(*p);
    }

    // Format complete command: $CMD*XX\r\n
    char buf[96];
    int len = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", cmd, checksum);

    if (len <= 0 || len >= static_cast<int>(sizeof(buf))) {
        return false;
    }

    // Write to GPS via I2C
    BusResult result = m_bus->write(reinterpret_cast<const uint8_t*>(buf),
                                     static_cast<size_t>(len));

    // GPS needs time to process command
    Timing::delayMs(10);

    return (result == BusResult::OK);
}

// ============================================================================
// NMEA Parsing
// ============================================================================

bool GPS_PA1010D::parseSentence(const char* sentence)
{
    if (sentence == nullptr || sentence[0] != '$') {
        return false;
    }

    // Validate checksum
    if (!validateChecksum(sentence)) {
        return false;
    }

    // Determine sentence type (skip $Gx prefix)
    // $GPGGA, $GNGGA, $GLGGA all map to GGA
    const char* type = sentence + 3;  // Skip "$Gx"

    if (strncmp(type, "GGA", 3) == 0) {
        return parseGGA(sentence);
    }
    else if (strncmp(type, "RMC", 3) == 0) {
        return parseRMC(sentence);
    }
    else if (strncmp(type, "VTG", 3) == 0) {
        return parseVTG(sentence);
    }
    else if (strncmp(type, "GSA", 3) == 0) {
        return parseGSA(sentence);
    }

    return false;
}

bool GPS_PA1010D::parseGGA(const char* sentence)
{
    // $xxGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
    // Field: 0    1    2   3   4   5   6       7     8   9  10 11  12  13         14
    char field[32];

    // Field 1: Time (hhmmss.sss)
    if (getField(sentence, 1, field, sizeof(field)) && field[0] != '\0') {
        int time_int = static_cast<int>(parseFloat(field));
        m_data.time.hour = time_int / 10000;
        m_data.time.minute = (time_int / 100) % 100;
        m_data.time.second = time_int % 100;

        const char* dot = strchr(field, '.');
        if (dot) {
            m_data.time.millisecond = static_cast<uint16_t>(atoi(dot + 1));
        }
    }

    // Fields 2-3: Latitude
    char lat_field[16], lat_dir[4];
    if (getField(sentence, 2, lat_field, sizeof(lat_field)) &&
        getField(sentence, 3, lat_dir, sizeof(lat_dir)) &&
        lat_field[0] != '\0') {
        m_data.latitude = parseLatLon(lat_field, lat_dir[0]);
    }

    // Fields 4-5: Longitude
    char lon_field[16], lon_dir[4];
    if (getField(sentence, 4, lon_field, sizeof(lon_field)) &&
        getField(sentence, 5, lon_dir, sizeof(lon_dir)) &&
        lon_field[0] != '\0') {
        m_data.longitude = parseLatLon(lon_field, lon_dir[0]);
    }

    // Field 6: Fix quality
    if (getField(sentence, 6, field, sizeof(field))) {
        int qual = parseInt(field);
        m_data.fix_quality = static_cast<GPSFixQuality>(qual);
        m_data.position_valid = (qual > 0);

        if (m_data.position_valid) {
            m_fix_count++;
            m_new_data = true;
        }
    }

    // Field 7: Number of satellites
    if (getField(sentence, 7, field, sizeof(field))) {
        m_data.satellites = static_cast<uint8_t>(parseInt(field));
    }

    // Field 8: HDOP
    if (getField(sentence, 8, field, sizeof(field)) && field[0] != '\0') {
        m_data.hdop = parseFloat(field);
    }

    // Field 9: Altitude (MSL)
    if (getField(sentence, 9, field, sizeof(field)) && field[0] != '\0') {
        m_data.altitude_msl = parseFloat(field);
    }

    // Field 11: Geoid separation
    if (getField(sentence, 11, field, sizeof(field)) && field[0] != '\0') {
        m_data.geoid_sep = parseFloat(field);
    }

    return true;
}

bool GPS_PA1010D::parseRMC(const char* sentence)
{
    // $xxRMC,time,status,lat,N/S,lon,E/W,spd,cog,date,mv,mvE,posMode,navStatus*cs
    // Field: 0    1    2      3   4   5   6   7   8    9   10  11    12       13
    char field[32];

    // Field 1: Time
    if (getField(sentence, 1, field, sizeof(field)) && field[0] != '\0') {
        int time_int = static_cast<int>(parseFloat(field));
        m_data.time.hour = time_int / 10000;
        m_data.time.minute = (time_int / 100) % 100;
        m_data.time.second = time_int % 100;
    }

    // Field 2: Status (A=active/valid, V=void)
    if (getField(sentence, 2, field, sizeof(field))) {
        m_data.time_valid = (field[0] == 'A');
    }

    // Fields 3-4: Latitude
    char lat_field[16], lat_dir[4];
    if (getField(sentence, 3, lat_field, sizeof(lat_field)) &&
        getField(sentence, 4, lat_dir, sizeof(lat_dir)) &&
        lat_field[0] != '\0') {
        m_data.latitude = parseLatLon(lat_field, lat_dir[0]);
    }

    // Fields 5-6: Longitude
    char lon_field[16], lon_dir[4];
    if (getField(sentence, 5, lon_field, sizeof(lon_field)) &&
        getField(sentence, 6, lon_dir, sizeof(lon_dir)) &&
        lon_field[0] != '\0') {
        m_data.longitude = parseLatLon(lon_field, lon_dir[0]);
    }

    // Field 7: Speed over ground (knots)
    if (getField(sentence, 7, field, sizeof(field)) && field[0] != '\0') {
        m_data.speed_knots = parseFloat(field);
        m_data.speed_mps = m_data.speed_knots * KNOTS_TO_MPS;
        m_data.velocity_valid = true;
        m_new_data = true;
    }

    // Field 8: Course over ground (degrees true)
    if (getField(sentence, 8, field, sizeof(field)) && field[0] != '\0') {
        m_data.course_deg = parseFloat(field);
    }

    // Field 9: Date (ddmmyy)
    if (getField(sentence, 9, field, sizeof(field)) && field[0] != '\0') {
        int date = parseInt(field);
        m_data.time.day = date / 10000;
        m_data.time.month = (date / 100) % 100;
        m_data.time.year = 2000 + (date % 100);
    }

    return true;
}

bool GPS_PA1010D::parseVTG(const char* sentence)
{
    // $xxVTG,cogt,T,cogm,M,knots,N,kph,K,posMode*cs
    // Field: 0    1   2  3   4   5   6  7  8   9
    char field[32];

    // Field 1: Course over ground (true)
    if (getField(sentence, 1, field, sizeof(field)) && field[0] != '\0') {
        m_data.course_deg = parseFloat(field);
    }

    // Field 5: Speed in knots
    if (getField(sentence, 5, field, sizeof(field)) && field[0] != '\0') {
        m_data.speed_knots = parseFloat(field);
        m_data.speed_mps = m_data.speed_knots * KNOTS_TO_MPS;
        m_data.velocity_valid = true;
        m_new_data = true;
    }

    return true;
}

bool GPS_PA1010D::parseGSA(const char* sentence)
{
    // $xxGSA,opMode,navMode,sv1,sv2,...,sv12,PDOP,HDOP,VDOP,systemId*cs
    // Field: 0     1       2      3-14        15   16   17    18
    char field[16];

    // Field 2: Navigation mode (1=no fix, 2=2D, 3=3D)
    if (getField(sentence, 2, field, sizeof(field))) {
        int mode = parseInt(field);
        m_data.fix_type = static_cast<GPSFixType>(mode);
    }

    // Field 15: PDOP
    if (getField(sentence, 15, field, sizeof(field)) && field[0] != '\0') {
        m_data.pdop = parseFloat(field);
    }

    // Field 16: HDOP
    if (getField(sentence, 16, field, sizeof(field)) && field[0] != '\0') {
        m_data.hdop = parseFloat(field);
    }

    // Field 17: VDOP
    if (getField(sentence, 17, field, sizeof(field)) && field[0] != '\0') {
        m_data.vdop = parseFloat(field);
    }

    return true;
}

// ============================================================================
// NMEA Helpers
// ============================================================================

bool GPS_PA1010D::validateChecksum(const char* sentence)
{
    if (sentence == nullptr || sentence[0] != '$') {
        return false;
    }

    // Find checksum marker
    const char* asterisk = strchr(sentence, '*');
    if (asterisk == nullptr) {
        return false;
    }

    // Calculate checksum (XOR of chars between $ and *)
    uint8_t calc_checksum = 0;
    for (const char* p = sentence + 1; p < asterisk; p++) {
        calc_checksum ^= static_cast<uint8_t>(*p);
    }

    // Parse provided checksum (hex digits after *)
    uint8_t provided_checksum = static_cast<uint8_t>(strtol(asterisk + 1, nullptr, 16));

    return (calc_checksum == provided_checksum);
}

uint8_t GPS_PA1010D::calculateChecksum(const char* sentence)
{
    uint8_t checksum = 0;
    for (const char* p = sentence; *p != '\0' && *p != '*'; p++) {
        if (*p != '$') {
            checksum ^= static_cast<uint8_t>(*p);
        }
    }
    return checksum;
}

double GPS_PA1010D::parseLatLon(const char* field, char direction)
{
    if (field == nullptr || field[0] == '\0') {
        return 0.0;
    }

    // Format: DDDMM.MMMMM (longitude) or DDMM.MMMMM (latitude)
    double value = atof(field);

    // Extract degrees and minutes
    int degrees = static_cast<int>(value / 100);
    double minutes = value - (degrees * 100);

    // Convert to decimal degrees
    double decimal = degrees + (minutes / 60.0);

    // Apply direction
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

float GPS_PA1010D::parseFloat(const char* field)
{
    if (field == nullptr || field[0] == '\0') {
        return 0.0f;
    }
    return static_cast<float>(atof(field));
}

int GPS_PA1010D::parseInt(const char* field)
{
    if (field == nullptr || field[0] == '\0') {
        return 0;
    }
    return atoi(field);
}

bool GPS_PA1010D::getField(const char* sentence, int fieldNum, char* out, size_t maxLen)
{
    if (sentence == nullptr || out == nullptr || maxLen == 0) {
        return false;
    }

    out[0] = '\0';

    const char* p = sentence;
    int currentField = 0;

    // Skip to start of first field (after $GPGGA,)
    p = strchr(p, ',');
    if (p == nullptr) {
        return false;
    }
    p++;  // Skip the comma

    // Navigate to requested field
    while (currentField < fieldNum) {
        p = strchr(p, ',');
        if (p == nullptr) {
            return false;
        }
        p++;
        currentField++;
    }

    // Copy field content until next comma, asterisk, or end
    size_t i = 0;
    while (*p != ',' && *p != '*' && *p != '\0' && i < maxLen - 1) {
        out[i++] = *p++;
    }
    out[i] = '\0';

    return true;
}

} // namespace hal
} // namespace rocketchip
