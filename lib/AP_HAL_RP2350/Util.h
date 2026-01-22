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

namespace RP2350 {

/**
 * @brief Safety switch states
 */
enum class SafetyState : uint8_t {
    SAFETY_NONE = 0,      // No safety switch present
    SAFETY_DISARMED = 1,  // Safety engaged, outputs disabled
    SAFETY_ARMED = 2      // Safety disengaged, outputs enabled
};


/**
 * @brief System utilities
 *
 * Provides memory information, system identification, and various
 * utility functions required by ArduPilot libraries.
 *
 * Implements AP_HAL::Util interface.
 */
class Util {
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
    uint32_t available_memory() const;

    /**
     * @brief Get total heap size
     * @return Total heap bytes
     */
    uint32_t total_memory() const;

    /**
     * @brief Get memory usage information string
     * @param buffer Output buffer
     * @param buflen Buffer size
     */
    void mem_info(char* buffer, uint16_t buflen) const;

    // ========================================================================
    // System Identification
    // ========================================================================

    /**
     * @brief Get formatted system ID
     *
     * Returns a unique identifier for this board as a hex string.
     *
     * @param buffer Output buffer (minimum 25 bytes)
     * @param buflen Buffer size
     * @return true on success
     */
    bool get_system_id(char* buffer, uint8_t buflen) const;

    /**
     * @brief Get raw system ID bytes
     *
     * Returns the RP2350 unique board ID (8 bytes).
     *
     * @param buffer Output buffer (minimum 8 bytes)
     * @param buflen Buffer size
     * @return Number of bytes written
     */
    uint8_t get_system_id_unformatted(uint8_t* buffer, uint8_t buflen) const;

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
    SafetyState safety_switch_state() const;

    // ========================================================================
    // Arming State
    // ========================================================================

    /**
     * @brief Get soft-armed state
     *
     * Soft arming is a software flag, not tied to safety switch.
     *
     * @return true if armed
     */
    bool get_soft_armed() const;

    /**
     * @brief Set soft-armed state
     * @param armed New armed state
     */
    void set_soft_armed(bool armed);

    /**
     * @brief Get time of last arming state change
     * @return Milliseconds since boot when armed state last changed
     */
    uint32_t get_last_armed_change_ms() const;

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
    bool m_soft_armed;
    uint32_t m_last_armed_change_ms;
};

}  // namespace RP2350

// ============================================================================
// AP_HAL Compatibility
// ============================================================================

namespace AP_HAL {
    using Util = RP2350::Util;

    // Safety state enum aliases
    constexpr auto SAFETY_NONE = RP2350::SafetyState::SAFETY_NONE;
    constexpr auto SAFETY_DISARMED = RP2350::SafetyState::SAFETY_DISARMED;
    constexpr auto SAFETY_ARMED = RP2350::SafetyState::SAFETY_ARMED;
}
