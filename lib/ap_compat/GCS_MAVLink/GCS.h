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
// Note: We don't use MAVLINK_USE_CONVENIENCE_FUNCTIONS because it requires
// a global mavlink_system variable. Instead, we use explicit pack functions
// like mavlink_msg_statustext_pack() which don't need it.

// MAVLINK_CRC_EXTRA must be defined before including MAVLink - required for
// consistent function signatures in mavlink_finalize_message_buffer
#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

// Include MAVLink ardupilotmega dialect (includes common + ArduPilot-specific enums)
#include <mavlink/ardupilotmega/mavlink.h>

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

    // Constructor (public for singleton pattern)
    GCS();

private:
    bool _initialized;
    bool _connected;
    uint32_t _last_heartbeat_ms;

    // MAVLink system status
    mavlink_status_t _mavlink_status;

    // Message sequence number
    uint8_t _seq;
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
