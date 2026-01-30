/**
 * @file GCS.cpp
 * @brief GCS MAVLink implementation for RocketChip
 *
 * Sends MAVLink messages over USB CDC for ground station communication.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#include "GCS.h"
#include <cstring>
#include <cstdio>
#include <pico/stdlib.h>

// Include debug macros if available
#ifdef CONFIG_DEBUG
#include "debug.h"
#else
#define DBG_PRINT(...) ((void)0)
#endif

// ============================================================================
// Global MAVLink System ID (required by convenience functions)
// ============================================================================

mavlink_system_t mavlink_system = {
    MAVLINK_SYS_ID,   // sysid - system ID
    MAVLINK_COMP_ID   // compid - component ID
};

// ============================================================================
// Singleton Instance
// ============================================================================

static GCS s_gcs_instance;

GCS& GCS::get_singleton() {
    return s_gcs_instance;
}

namespace AP {
    GCS& gcs() {
        return GCS::get_singleton();
    }
}

// ============================================================================
// Internal Helper: Send MAVLink Buffer
// ============================================================================

/**
 * @brief Send MAVLink message buffer over USB CDC
 *
 * @param buf Message buffer to send
 * @param len Number of bytes to send
 */
static void send_mavlink_buffer(const uint8_t* buf, uint16_t len) {
    // Send over USB CDC
    // Note: This blocks until all bytes are written or timeout
    for (uint16_t i = 0; i < len; i++) {
        putchar_raw(buf[i]);
    }
}

// ============================================================================
// GCS Implementation
// ============================================================================

GCS::GCS()
    : _initialized(false)
    , _connected(false)
    , _last_heartbeat_ms(0)
    , _mavlink_status{}
    , _seq(0)
{
}

void GCS::init() {
    _initialized = true;
    _connected = false;
    _last_heartbeat_ms = 0;
    _seq = 0;

    // Reset MAVLink status
    memset(&_mavlink_status, 0, sizeof(_mavlink_status));
}

void GCS::send_statustext(MAV_SEVERITY severity, const char* text) {
    if (!_initialized) {
        return;
    }

    // Only send MAVLink binary when GCS is connected
    // Otherwise it shows as garbage in CLI terminal
    if (!_connected) {
        return;
    }

    mavlink_message_t msg;
    char text_buf[51];  // 50 chars + null terminator for safety

    // Copy and truncate text to 50 chars
    strncpy(text_buf, text, 50);
    text_buf[50] = '\0';

    // Pack STATUSTEXT message
    // id=0, chunk_seq=0 means this is a complete single-chunk message
    mavlink_msg_statustext_pack(
        MAVLINK_SYS_ID,
        MAVLINK_COMP_ID,
        &msg,
        severity,
        text_buf,
        0,      // id - 0 for single message
        0       // chunk_seq - 0 for first/only chunk
    );

    // Send to GCS
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    send_mavlink_buffer(buf, len);
}

void GCS::send_statustext_fmt(MAV_SEVERITY severity, const char* fmt, ...) {
    char text_buf[64];  // Slightly larger for formatting, will be truncated

    va_list args;
    va_start(args, fmt);
    vsnprintf(text_buf, sizeof(text_buf), fmt, args);
    va_end(args);

    send_statustext(severity, text_buf);
}

void GCS::send_heartbeat() {
    if (!_initialized) {
        return;
    }

    mavlink_message_t msg;

    // Pack HEARTBEAT message
    mavlink_msg_heartbeat_pack(
        MAVLINK_SYS_ID,
        MAVLINK_COMP_ID,
        &msg,
        MAV_TYPE_ROCKET,                    // type - we're a rocket!
        MAV_AUTOPILOT_INVALID,              // autopilot - not a full autopilot
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, // base_mode
        0,                                  // custom_mode
        MAV_STATE_ACTIVE                    // system_status
    );

    // Send to GCS
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    send_mavlink_buffer(buf, len);

    _last_heartbeat_ms = to_ms_since_boot(get_absolute_time());
}

void GCS::update() {
    if (!_initialized) {
        return;
    }

    // Parse incoming MAVLink messages from USB CDC
    mavlink_message_t msg;
    mavlink_status_t status;

    // Read available bytes (non-blocking)
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        uint8_t byte = static_cast<uint8_t>(c);

        // Parse byte into MAVLink message
        if (mavlink_parse_char(MAVLINK_CHANNEL_USB, byte, &msg, &status)) {
            // Complete message received - dispatch to handler
            handle_message(msg);
        }
    }
}

bool GCS::parse_byte(uint8_t byte) {
    if (!_initialized) {
        return false;
    }

    mavlink_message_t msg;
    mavlink_status_t status;

    // Feed byte to MAVLink parser
    if (mavlink_parse_char(MAVLINK_CHANNEL_USB, byte, &msg, &status)) {
        // Complete message received - dispatch to handler
        handle_message(msg);
        return true;
    }
    return false;
}

void GCS::handle_message(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // Track GCS connection
            _connected = true;
            _last_heartbeat_ms = to_ms_since_boot(get_absolute_time());
            break;

        case MAVLINK_MSG_ID_COMMAND_LONG:
            handle_command_long(msg);
            break;

        case MAVLINK_MSG_ID_COMMAND_ACK:
            handle_command_ack(msg);
            break;

        default:
            // Unknown message - ignore
            break;
    }
}

void GCS::handle_command_long(const mavlink_message_t& msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    MAV_RESULT result = MAV_RESULT_UNSUPPORTED;

    switch (cmd.command) {
        case MAV_CMD_PREFLIGHT_CALIBRATION:
            result = handle_preflight_calibration(cmd);
            break;

        case MAV_CMD_ACCELCAL_VEHICLE_POS:
            // GCS acknowledging vehicle position for accel cal
            result = handle_accelcal_vehicle_pos(cmd);
            break;

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
    }

    // Send COMMAND_ACK response
    send_command_ack(cmd.command, result, msg.sysid, msg.compid);
}

void GCS::handle_command_ack(const mavlink_message_t& msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);

    // Forward to AP_AccelCal if calibration is running
    if (_calibration_callback) {
        _calibration_callback(ack);
    }
}

MAV_RESULT GCS::handle_preflight_calibration(const mavlink_command_long_t& cmd) {
    // param1: Gyro (1 = gyro cal, 3 = gyro temp cal)
    // param2: Magnetometer
    // param3: Ground pressure
    // param4: Radio RC
    // param5: Accel (1 = simple, 2 = 6-position, 4 = simple, 76 = force)
    // param6: Compass motor interference
    // param7: Airspeed

    if (cmd.param1 > 0) {
        // Gyro calibration
        printf("[GCS] Gyro calibration requested (not yet implemented)\n");
        return MAV_RESULT_UNSUPPORTED;
    }

    if (cmd.param2 > 0) {
        // Magnetometer calibration
        printf("[GCS] Compass calibration requested\n");
        if (_compass_cal_start_callback) {
            return _compass_cal_start_callback() ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        }
        return MAV_RESULT_UNSUPPORTED;
    }

    if (cmd.param3 > 0) {
        // Ground pressure calibration (barometer)
        printf("[GCS] Barometer calibration requested\n");
        if (_baro_cal_callback) {
            return _baro_cal_callback() ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        }
        return MAV_RESULT_UNSUPPORTED;
    }

    if (cmd.param5 > 0) {
        // Accelerometer calibration
        int cal_type = static_cast<int>(cmd.param5);
        printf("[GCS] Accel calibration requested (type=%d)\n", cal_type);

        if (cal_type == 1 || cal_type == 4) {
            // Simple 1D calibration
            if (_simple_accel_cal_callback) {
                return _simple_accel_cal_callback();
            }
        } else if (cal_type == 2 || cal_type == 76) {
            // Full 6-position calibration
            if (_accel_cal_start_callback) {
                return _accel_cal_start_callback() ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
            }
        }
        return MAV_RESULT_UNSUPPORTED;
    }

    return MAV_RESULT_UNSUPPORTED;
}

MAV_RESULT GCS::handle_accelcal_vehicle_pos(const mavlink_command_long_t& cmd) {
    // GCS is confirming that vehicle is in the requested position
    int position = static_cast<int>(cmd.param1);
    printf("[GCS] AccelCal position confirmed: %d\n", position);

    if (_accel_cal_pos_callback) {
        return _accel_cal_pos_callback(position) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
    }
    return MAV_RESULT_UNSUPPORTED;
}

void GCS::send_command_ack(uint16_t command, MAV_RESULT result, uint8_t target_sysid, uint8_t target_compid) {
    mavlink_message_t msg;

    mavlink_msg_command_ack_pack(
        MAVLINK_SYS_ID,
        MAVLINK_COMP_ID,
        &msg,
        command,
        result,
        0,              // progress (0 = N/A)
        0,              // result_param2
        target_sysid,
        target_compid
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    send_mavlink_buffer(buf, len);
}

// ============================================================================
// Global Function for GCS_SEND_TEXT Macro
// ============================================================================

void gcs_send_statustext(MAV_SEVERITY severity, const char* fmt, ...) {
    char text_buf[64];

    va_list args;
    va_start(args, fmt);
    vsnprintf(text_buf, sizeof(text_buf), fmt, args);
    va_end(args);

    // Send via MAVLink
    GCS::get_singleton().send_statustext(severity, text_buf);

    // Also output to debug console
    // Prefix with severity level for clarity
    const char* sev_str = "???";
    switch (severity) {
        case MAV_SEVERITY_EMERGENCY: sev_str = "EMERG"; break;
        case MAV_SEVERITY_ALERT:     sev_str = "ALERT"; break;
        case MAV_SEVERITY_CRITICAL:  sev_str = "CRIT";  break;
        case MAV_SEVERITY_ERROR:     sev_str = "ERROR"; break;
        case MAV_SEVERITY_WARNING:   sev_str = "WARN";  break;
        case MAV_SEVERITY_NOTICE:    sev_str = "NOTE";  break;
        case MAV_SEVERITY_INFO:      sev_str = "INFO";  break;
        case MAV_SEVERITY_DEBUG:     sev_str = "DEBUG"; break;
        default: break;
    }

    // Output to debug (if enabled)
#ifdef CONFIG_DEBUG
    DBG_PRINT("[GCS:%s] %s\n", sev_str, text_buf);
#else
    // If debug not enabled, still print to serial for visibility
    printf("[GCS:%s] %s\n", sev_str, text_buf);
#endif
}

// ============================================================================
// GCS_MAVLINK Implementation (for AP_AccelCal compatibility)
// ============================================================================

GCS_MAVLINK* GCS_MAVLINK::_singleton = nullptr;

GCS_MAVLINK* GCS_MAVLINK::get_singleton() {
    if (_singleton == nullptr) {
        _singleton = new GCS_MAVLINK();
    }
    return _singleton;
}

void GCS_MAVLINK::send_text(MAV_SEVERITY severity, const char* fmt, ...) {
    char text_buf[64];

    va_list args;
    va_start(args, fmt);
    vsnprintf(text_buf, sizeof(text_buf), fmt, args);
    va_end(args);

    // Delegate to the main GCS
    GCS::get_singleton().send_statustext(severity, text_buf);
}

void GCS_MAVLINK::send_accelcal_vehicle_position(uint32_t position) {
    // Only send MAVLink binary when GCS is connected
    // Otherwise it shows as garbage in CLI terminal
    if (GCS::get_singleton().is_connected()) {
        // Send COMMAND_LONG with MAV_CMD_ACCELCAL_VEHICLE_POS
        // This tells the GCS what position the vehicle is in (or success/failure)
        mavlink_message_t msg;

        mavlink_msg_command_long_pack(
            MAVLINK_SYS_ID,
            MAVLINK_COMP_ID,
            &msg,
            0,  // target_system (broadcast)
            0,  // target_component (broadcast)
            MAV_CMD_ACCELCAL_VEHICLE_POS,
            0,  // confirmation
            static_cast<float>(position),  // param1 - position
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f  // unused params
        );

        // Send over USB CDC
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        for (uint16_t i = 0; i < len; i++) {
            putchar_raw(buf[i]);
        }
    }

    // Note: No CLI print here - ArduPilot's AP_AccelCal already sends user prompts
    // via _printf() -> send_text() -> gcs_send_statustext() which prints to console.
    // This function is only for MAVLink binary protocol to GCS.
}
