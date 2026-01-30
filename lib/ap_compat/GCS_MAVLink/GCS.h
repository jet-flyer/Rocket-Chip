/**
 * @file GCS.h
 * @brief GCS (Ground Control Station) MAVLink interface for RocketChip
 *
 * Implements GCS_SEND_TEXT macro using real MAVLink STATUSTEXT messages.
 * Messages are sent over USB CDC and also output to debug console.
 *
 * Compatible with QGroundControl, Mission Planner, and other MAVLink GCS.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

#include <cstdint>
#include <cstdarg>
#include <cstdio>


// MAVLink configuration - must be set before including MAVLink headers

// MAVLINK_CRC_EXTRA must be defined before including MAVLink - required for
// consistent function signatures in mavlink_finalize_message_buffer
#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

// Enable MAVLink convenience functions (_send variants)
// Required by AP_Compass_Calibration.cpp for progress messages
#ifndef MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#endif

// ============================================================================
// MAVLink UART Send Implementation (for convenience functions)
// MUST be defined BEFORE including MAVLink headers
// ============================================================================

#include <pico/stdlib.h>
#include <mavlink/mavlink_types.h>  // For mavlink_channel_t, mavlink_system_t

// Global MAVLink system identification (required by convenience functions)
extern mavlink_system_t mavlink_system;

// comm_send_ch: Called by MAVLink convenience functions to send one byte
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t byte) {
    (void)chan;  // Only one channel (USB CDC)
    putchar_raw(byte);
}

// Define the macro that MAVLink expects for the send implementation
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) \
    do { \
        for (uint16_t i = 0; i < (len); i++) { \
            putchar_raw((buf)[i]); \
        } \
    } while(0)

// Include MAVLink ardupilotmega dialect (includes common + ArduPilot-specific enums)
#include <mavlink/ardupilotmega/mavlink.h>

// ============================================================================
// MAVLink Buffer Space Macros (ArduPilot Compatibility)
// ============================================================================

/**
 * @brief Check if TX buffer has space for a MAVLink message
 *
 * ArduPilot's real implementation checks actual TX buffer space.
 * For RocketChip's USB CDC (blocking writes), we always have "space" since
 * writes block until complete. Return true to allow sends to proceed.
 *
 * @param chan MAVLink channel (MAVLINK_COMM_0, etc.)
 * @param id Message type name (e.g., MAG_CAL_PROGRESS)
 */
#define HAVE_PAYLOAD_SPACE(chan, id) (true)

// ============================================================================
// MAVLink System Configuration
// ============================================================================

// System and component IDs (matches deprecated Arduino config pattern)
#ifndef MAVLINK_SYS_ID
#define MAVLINK_SYS_ID      1       // System ID (unique per vehicle)
#endif

#ifndef MAVLINK_COMP_ID
#define MAVLINK_COMP_ID     1       // Component ID (flight controller = 1)
#endif

// MAVLink channel for USB CDC
#define MAVLINK_CHANNEL_USB 0

// ============================================================================
// GCS_SEND_TEXT Macro (ArduPilot Compatible)
// ============================================================================

/**
 * @brief Send a STATUSTEXT message to GCS
 *
 * This macro matches ArduPilot's GCS_SEND_TEXT interface. It:
 * 1. Sends a MAVLink STATUSTEXT message over USB CDC
 * 2. Also outputs to debug console via DBG_PRINT
 *
 * @param severity MAV_SEVERITY value (MAV_SEVERITY_INFO, MAV_SEVERITY_WARNING, etc.)
 * @param fmt Printf-style format string
 * @param ... Format arguments
 *
 * Usage:
 *   GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Compass cal started");
 *   GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Fitness: %.1f", fitness);
 */
#define GCS_SEND_TEXT(severity, fmt, ...) \
    gcs_send_statustext((MAV_SEVERITY)(severity), fmt, ##__VA_ARGS__)

// ============================================================================
// GCS_MAVLINK Class (ArduPilot Compatibility)
// ============================================================================

/**
 * @brief MAVLink channel interface for calibration feedback
 *
 * ArduPilot's AP_AccelCal uses GCS_MAVLINK* to send status messages.
 * This class provides the send_text interface it expects.
 */
class GCS_MAVLINK {
public:
    /**
     * @brief Send text message to GCS
     *
     * @param severity MAV_SEVERITY value
     * @param fmt Printf format string
     * @param ... Format arguments
     */
    void send_text(MAV_SEVERITY severity, const char* fmt, ...);

    /**
     * @brief Send accelerometer calibration vehicle position
     *
     * Used by AP_AccelCal to inform GCS of calibration state.
     *
     * @param position ACCELCAL_VEHICLE_POS enum value
     */
    void send_accelcal_vehicle_position(uint32_t position);

    /**
     * @brief Get MAVLink channel for this link
     *
     * Used by calibration routines to send progress messages.
     * Returns MAVLINK_COMM_0 (USB CDC channel).
     */
    mavlink_channel_t get_chan() const { return MAVLINK_COMM_0; }

    /**
     * @brief Get singleton instance
     */
    static GCS_MAVLINK* get_singleton();

private:
    static GCS_MAVLINK* _singleton;
};

// ============================================================================
// GCS Class
// ============================================================================

/**
 * @brief Ground Control Station interface
 *
 * Provides MAVLink communication with ground stations.
 * Currently implements STATUSTEXT for diagnostic messages.
 */
class GCS {
public:
    /**
     * @brief Get singleton instance
     */
    static GCS& get_singleton();

    /**
     * @brief Initialize GCS subsystem
     * Call after USB CDC is initialized
     */
    void init();

    /**
     * @brief Send STATUSTEXT message
     *
     * @param severity MAV_SEVERITY value
     * @param text Message text (max 50 chars, will be truncated)
     */
    void send_statustext(MAV_SEVERITY severity, const char* text);

    /**
     * @brief Send STATUSTEXT with printf formatting
     *
     * @param severity MAV_SEVERITY value
     * @param fmt Printf format string
     * @param ... Format arguments
     */
    void send_statustext_fmt(MAV_SEVERITY severity, const char* fmt, ...);

    /**
     * @brief Send HEARTBEAT message
     *
     * Should be called at 1Hz to maintain connection with GCS.
     */
    void send_heartbeat();

    /**
     * @brief Check if GCS is connected
     * @return true if heartbeats are being exchanged
     */
    bool is_connected() const { return _connected; }

    /**
     * @brief Update GCS state
     * Call periodically (e.g., 10Hz) to handle incoming messages
     */
    void update();

    /**
     * @brief Parse a single byte as MAVLink
     *
     * Called by input router when MAVLink start byte detected.
     * Feeds byte to MAVLink state machine and handles complete messages.
     *
     * @param byte The byte to parse
     * @return true if a complete message was parsed and handled
     */
    bool parse_byte(uint8_t byte);

    /**
     * @brief Send parameter value to GCS
     * Used by AP_Param when parameters change
     * @note Stub - does nothing for now
     */
    template<typename T>
    void send_parameter_value(const char* name, T type, float value) {
        (void)name; (void)type; (void)value;
    }

    /**
     * @brief Check if parameter setting is allowed
     * @return true (always allow for RocketChip)
     */
    bool get_allow_param_set() const { return true; }

    // ========================================================================
    // Calibration Callbacks
    // Set these to wire up calibration commands to your sensor task
    // ========================================================================

    using SimpleAccelCalCallback = MAV_RESULT (*)();
    using AccelCalStartCallback = bool (*)();
    using AccelCalPosCallback = bool (*)(int position);
    using CompassCalStartCallback = bool (*)();
    using BaroCalCallback = bool (*)();
    using CommandAckCallback = void (*)(const mavlink_command_ack_t& ack);

    void set_simple_accel_cal_callback(SimpleAccelCalCallback cb) { _simple_accel_cal_callback = cb; }
    void set_accel_cal_start_callback(AccelCalStartCallback cb) { _accel_cal_start_callback = cb; }
    void set_accel_cal_pos_callback(AccelCalPosCallback cb) { _accel_cal_pos_callback = cb; }
    void set_compass_cal_start_callback(CompassCalStartCallback cb) { _compass_cal_start_callback = cb; }
    void set_baro_cal_callback(BaroCalCallback cb) { _baro_cal_callback = cb; }
    void set_calibration_callback(CommandAckCallback cb) { _calibration_callback = cb; }

    // Constructor (public for singleton pattern)
    GCS();

    /**
     * @brief Send a command locally (as if from GCS)
     *
     * Used by RC_OS CLI to route commands through the same MAVLink handler
     * path as external GCS commands. This ensures CLI and GCS use identical
     * code paths for calibration, etc.
     *
     * @param command MAV_CMD_* command ID
     * @param p1-p7 Command parameters (meaning depends on command)
     * @return MAV_RESULT indicating success/failure
     */
    MAV_RESULT send_local_command(uint16_t command,
                                   float p1 = 0, float p2 = 0, float p3 = 0,
                                   float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0);

private:
    bool _initialized;
    bool _connected;
    uint32_t _last_heartbeat_ms;

    // MAVLink system status
    mavlink_status_t _mavlink_status;

    // Message sequence number
    uint8_t _seq;

    // Message handlers
    void handle_message(const mavlink_message_t& msg);
    void handle_command_long(const mavlink_message_t& msg);
    void handle_command_ack(const mavlink_message_t& msg);
    MAV_RESULT handle_preflight_calibration(const mavlink_command_long_t& cmd);
    MAV_RESULT handle_accelcal_vehicle_pos(const mavlink_command_long_t& cmd);
    void send_command_ack(uint16_t command, MAV_RESULT result, uint8_t target_sysid, uint8_t target_compid);

    // Calibration callbacks
    SimpleAccelCalCallback _simple_accel_cal_callback = nullptr;
    AccelCalStartCallback _accel_cal_start_callback = nullptr;
    AccelCalPosCallback _accel_cal_pos_callback = nullptr;
    CompassCalStartCallback _compass_cal_start_callback = nullptr;
    BaroCalCallback _baro_cal_callback = nullptr;
    CommandAckCallback _calibration_callback = nullptr;
};

// ============================================================================
// Global Function (for GCS_SEND_TEXT macro)
// ============================================================================

/**
 * @brief Send STATUSTEXT (called by GCS_SEND_TEXT macro)
 *
 * @param severity MAV_SEVERITY value
 * @param fmt Printf format string
 * @param ... Format arguments
 */
void gcs_send_statustext(MAV_SEVERITY severity, const char* fmt, ...);

// Convenience accessor (ArduPilot convention)
namespace AP {
    GCS& gcs();
}

// Global accessor (used by AP_Param macros)
inline GCS& gcs() { return AP::gcs(); }
