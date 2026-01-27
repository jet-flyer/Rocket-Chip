/**
 * @file Util.h
 * @brief AP_HAL::Util base class for ArduPilot compatibility
 *
 * Provides the PersistentData structure and snprintf interface
 * expected by ArduPilot libraries like AP_InternalError.
 */
#pragma once

#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace AP_HAL {

/**
 * @brief Safety switch states
 */
enum safety_state : uint8_t {
    SAFETY_NONE = 0,
    SAFETY_DISARMED = 1,
    SAFETY_ARMED = 2
};

/**
 * @brief Base Util class for ArduPilot compatibility
 *
 * Provides the interface expected by ArduPilot libraries.
 * Platform implementations should inherit from this.
 */
class Util {
public:
    virtual ~Util() = default;

    // ========================================================================
    // Printf functions (used by AP_InternalError::errors_as_string)
    // ========================================================================

    int snprintf(char* str, size_t size, const char* format, ...) {
        va_list ap;
        va_start(ap, format);
        int result = vsnprintf(str, size, format, ap);
        va_end(ap);
        return result;
    }

    int vsnprintf(char* str, size_t size, const char* format, va_list ap) {
        return ::vsnprintf(str, size, format, ap);
    }

    // ========================================================================
    // Arming state (used by various ArduPilot code)
    // ========================================================================

    virtual void set_soft_armed(bool b) {
        soft_armed = b;
    }

    bool get_soft_armed() const {
        return soft_armed;
    }

    // ========================================================================
    // Watchdog support (used by AP_InternalError)
    // ========================================================================

    virtual bool was_watchdog_reset() const { return false; }

    bool was_watchdog_armed() const {
        return was_watchdog_reset() && persistent_data.armed;
    }

    // ========================================================================
    // Safety state
    // ========================================================================

    virtual enum safety_state safety_switch_state() { return SAFETY_NONE; }

    // ========================================================================
    // Persistent Data (survives watchdog reset on some platforms)
    // ========================================================================

    /**
     * @brief Data structure that survives watchdog resets
     *
     * On RP2350, this is stored in RAM and NOT actually persistent
     * across resets. This is a stub for ArduPilot compatibility.
     */
    struct PersistentData {
        float roll_rad, pitch_rad, yaw_rad;
        int32_t home_lat, home_lon, home_alt_cm;
        uint32_t fault_addr;
        uint32_t fault_icsr;
        uint32_t fault_lr;
        uint32_t internal_errors;
        uint16_t internal_error_count;
        uint16_t internal_error_last_line;
        uint32_t spi_count;
        uint32_t i2c_count;
        uint32_t i2c_isr_count;
        uint16_t waypoint_num;
        uint16_t last_mavlink_msgid;
        uint16_t last_mavlink_cmd;
        uint16_t semaphore_line;
        uint16_t fault_line;
        uint8_t fault_type;
        uint8_t fault_thd_prio;
        char thread_name4[4];
        int8_t scheduler_task;
        bool armed;
        enum safety_state safety_state;
        bool boot_to_dfu;
    };

    PersistentData persistent_data{};
    PersistentData last_persistent_data{};

protected:
    bool soft_armed = false;
    uint32_t last_armed_change_ms = 0;
};

}  // namespace AP_HAL
