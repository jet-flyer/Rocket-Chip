/**
 * @file Util.h
 * @brief AP_HAL Util implementation for RP2350
 *
 * Provides system utilities: memory info, system ID, safety state, etc.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include "hwdef.h"
#include <cstdint>
#include <cstring>

// Include AP_HAL_Boards.h first to ensure HAL_PARAM_DEFAULTS_PATH is defined
// before it's used in AP_HAL/Util.h
#include <AP_HAL/AP_HAL_Boards.h>

// AP_HAL base class for inheritance
#include <AP_HAL/Util.h>

namespace RP2350 {

// Use AP_HAL::Util::safety_state enum instead of custom enum
// (SAFETY_NONE, SAFETY_DISARMED, SAFETY_ARMED are defined in base class)


/**
 * @brief System utilities
 *
 * Provides memory information, system identification, and various
 * utility functions required by ArduPilot libraries.
 *
 * Inherits from AP_HAL::Util for ArduPilot compatibility.
 */
class Util : public AP_HAL::Util {
public:
    Util();

    // Prevent copying
    Util(const Util&) = delete;
    Util& operator=(const Util&) = delete;

    // ========================================================================
    // Memory Information
    // ========================================================================

    /**
     * @brief Get available heap memory
     * @return Free heap bytes
     */
    uint32_t available_memory() override;

    /**
     * @brief Get total heap size
     * @return Total heap bytes
     */
    uint32_t total_memory() const;

    /**
     * @brief Get memory usage information string
     * @param buffer Output buffer
     * @param buflen Buffer size
     * @note Not const because it calls available_memory() which is non-const
     */
    void mem_info(char* buffer, uint16_t buflen);

    // ========================================================================
    // System Identification
    // ========================================================================

    /**
     * @brief Get formatted system ID
     *
     * Returns a unique identifier for this board as a hex string.
     *
     * @param buf Output buffer (50 bytes per base class spec)
     * @return true on success
     */
    bool get_system_id(char buf[50]) override;

    /**
     * @brief Get raw system ID bytes
     *
     * Returns the RP2350 unique board ID (8 bytes).
     *
     * @param buf Output buffer
     * @param len On input: buffer size; on output: bytes written
     * @return true on success
     */
    bool get_system_id_unformatted(uint8_t buf[], uint8_t& len) override;

    // ========================================================================
    // Safety State
    // ========================================================================

    /**
     * @brief Get safety switch state
     *
     * RocketChip has no physical safety switch, always returns SAFETY_NONE.
     *
     * @return Safety state
     */
    enum safety_state safety_switch_state() override;

    // ========================================================================
    // RTC (Hardware Real-Time Clock)
    // ========================================================================

    /**
     * @brief Set hardware RTC in UTC microseconds
     *
     * RP2350 has no battery-backed RTC, so this just stores the value in RAM.
     *
     * @param time_utc_usec UTC time in microseconds since epoch
     */
    void set_hw_rtc(uint64_t time_utc_usec) override;

    /**
     * @brief Get hardware RTC in UTC microseconds
     *
     * Returns stored UTC time adjusted by elapsed time since set.
     *
     * @return UTC time in microseconds since epoch (0 if never set)
     */
    uint64_t get_hw_rtc() const override;

    // Note: Arming state methods (get_soft_armed, set_soft_armed, get_last_armed_change)
    // are provided by the base class AP_HAL::Util using protected members

    // ========================================================================
    // System Load
    // ========================================================================

    /**
     * @brief Get system CPU load
     *
     * Returns estimated CPU usage as percentage.
     *
     * @return CPU load 0-100
     */
    uint16_t get_system_load() const;

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /**
     * @brief Get thread information string
     * @param buffer Output buffer
     * @param buflen Buffer size
     */
    void thread_info(char* buffer, uint16_t buflen) const;

    /**
     * @brief Get timer information string
     * @param buffer Output buffer
     * @param buflen Buffer size
     */
    void timer_info(char* buffer, uint16_t buflen) const;

private:
    // RTC time storage (no battery-backed RTC on RP2350)
    uint64_t m_rtc_time_utc_usec;
    uint64_t m_rtc_set_time_us;  // micros() when RTC was set
};

}  // namespace RP2350
