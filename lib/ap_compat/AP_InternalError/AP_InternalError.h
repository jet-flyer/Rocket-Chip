/**
 * @file AP_InternalError.h
 * @brief AP_InternalError stub for RocketChip
 *
 * Provides the AP_InternalError class with error_t enum for ArduPilot code.
 * This matches the ArduPilot error_t bitmask enum structure.
 *
 * TODO: Evaluate error handling strategy for RocketChip:
 *   Option A: Fully utilize ArduPilot's AP_InternalError with:
 *     - Error accumulation bitmask
 *     - Error count tracking
 *     - Error-to-string conversion
 *     - Integration with logging/telemetry
 *   Option B: Create RocketChip-specific error system:
 *     - Aligned with DEBUG_OUTPUT.md DBG_ERROR pattern
 *     - Flight state-aware error handling (ERROR state in state machine)
 *     - Integration with NeoPixel status indication
 *     - Simplified error categories for rocket use case
 *   Current: Minimal stub that prints errors - sufficient for calibration
 *   Decision needed before EKF3/fusion integration.
 */

#pragma once

#include <cstdint>
#include <cstdio>

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

    static void error(error_t e) {
        printf("[AP_InternalError] Error: 0x%lx\n", (unsigned long)e);
    }

    // Two-argument version used in some ArduPilot code
    static void error(error_t e, uint16_t line) {
        printf("[AP_InternalError] Error: 0x%lx at line %d\n", (unsigned long)e, line);
    }
};

// Note: AP::internal_error() is defined in AP_HAL_Compat.h
// to avoid duplicate definitions

// INTERNAL_ERROR macro - calls AP_InternalError::error
#ifndef INTERNAL_ERROR
#define INTERNAL_ERROR(e) AP_InternalError::error(e)
#endif
