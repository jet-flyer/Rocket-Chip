/**
 * @file AP_InternalError.h
 * @brief AP_InternalError implementation for RocketChip
 *
 * Provides the AP_InternalError class with error_t enum for ArduPilot code.
 * This matches the ArduPilot error_t bitmask enum structure and provides
 * full error accumulation/tracking for production error reporting.
 *
 * Council Decision (2026-01-21): Use ArduPilot AP_InternalError pattern with:
 * - Error accumulation bitmask
 * - Error count tracking
 * - Integration with debug output macros
 * - Future MAVLink telemetry support (placeholder)
 *
 * @see standards/STANDARDS_DEVIATIONS.md for resolution history
 */

#pragma once

#include <cstdint>
#include "debug.h"

/**
 * @brief Internal error tracking class for RocketChip
 *
 * Accumulates error flags and counts for diagnostic purposes.
 * Thread-safe through atomic operations on error state.
 */
class AP_InternalError {
public:
    // Error type enum - matches ArduPilot's bitmask enum
    enum class error_t : uint32_t {
        logger_mapfailure           = (1U <<  0),
        logger_missing_logstructure = (1U <<  1),
        logger_logwrite_missingfmt  = (1U <<  2),
        logger_too_many_deletions   = (1U <<  3),
        logger_bad_getfilename      = (1U <<  4),
        panic                       = (1U <<  5),
        logger_flushing_without_sem = (1U <<  6),
        logger_bad_current_block    = (1U <<  7),
        logger_blockcount_mismatch  = (1U <<  8),
        logger_dequeue_failure      = (1U <<  9),
        constraining_nan            = (1U << 10),
        watchdog_reset              = (1U << 11),
        iomcu_reset                 = (1U << 12),
        iomcu_fail                  = (1U << 13),
        spi_fail                    = (1U << 14),
        main_loop_stuck             = (1U << 15),
        gcs_bad_missionprotocol_link= (1U << 16),
        bitmask_range               = (1U << 17),
        gcs_offset                  = (1U << 18),
        i2c_isr                     = (1U << 19),
        flow_of_control             = (1U << 20),
        switch_full_sector_recursion= (1U << 21),
        bad_rotation                = (1U << 22),
        stack_overflow              = (1U << 23),
        imu_reset                   = (1U << 24),
        gpio_isr                    = (1U << 25),
        mem_guard                   = (1U << 26),
        dma_fail                    = (1U << 27),
        params_restored             = (1U << 28),
        invalid_arg_or_result       = (1U << 29),
    };

    /**
     * @brief Report an error condition
     * @param e Error type from error_t enum
     *
     * Accumulates the error in the bitmask and increments the error count.
     * Outputs diagnostic message via DBG_ERROR when enabled.
     */
    static void error(error_t e) {
        // Accumulate error in bitmask
        s_errorMask |= static_cast<uint32_t>(e);
        s_errorCount++;

        // Output diagnostic (compile-time guarded)
        DBG_ERROR("[AP_InternalError] Error: 0x%08lx (total: %lu)\n",
                  static_cast<unsigned long>(e),
                  static_cast<unsigned long>(s_errorCount));

        // TODO: Queue for MAVLink STATUS_TEXT when telemetry available
    }

    /**
     * @brief Report an error with source line information
     * @param e Error type from error_t enum
     * @param line Source line number where error occurred
     */
    static void error(error_t e, uint16_t line) {
        s_errorMask |= static_cast<uint32_t>(e);
        s_errorCount++;

        DBG_ERROR("[AP_InternalError] Error: 0x%08lx at line %d (total: %lu)\n",
                  static_cast<unsigned long>(e),
                  line,
                  static_cast<unsigned long>(s_errorCount));
    }

    /**
     * @brief Get accumulated error bitmask
     * @return Bitmask of all errors that have occurred
     */
    static uint32_t errors() {
        return s_errorMask;
    }

    /**
     * @brief Get total error count
     * @return Number of error() calls since boot
     */
    static uint32_t count() {
        return s_errorCount;
    }

    /**
     * @brief Check if any errors have occurred
     * @return true if any errors have been reported
     */
    static bool has_errors() {
        return s_errorMask != 0;
    }

    /**
     * @brief Clear all error state (use with caution)
     *
     * Primarily for test code. Production code should not clear errors.
     */
    static void clear() {
        s_errorMask = 0;
        s_errorCount = 0;
    }

private:
    // Static error state (initialized in AP_HAL_Compat.cpp)
    static uint32_t s_errorMask;
    static uint32_t s_errorCount;
};

// Note: AP::internal_error() is defined in AP_HAL_Compat.h
// to avoid duplicate definitions

// INTERNAL_ERROR macro - calls AP_InternalError::error
#ifndef INTERNAL_ERROR
#define INTERNAL_ERROR(e) AP_InternalError::error(e)
#endif
