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
    // TODO: Parse incoming MAVLink messages for bidirectional communication
    // For now, we only send outgoing messages
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
