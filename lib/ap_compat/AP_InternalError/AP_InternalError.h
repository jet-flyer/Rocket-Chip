/**
 * @file AP_InternalError.h
 * @brief AP_InternalError implementation compatible with ArduPilot interface
 *
 * Implements ArduPilot's AP_InternalError interface with:
 * - error_t bitmask enum (matches ArduPilot)
 * - Singleton accessor via AP::internalerror()
 * - errors_as_string() for debugging
 * - error_to_string() for single error names
 *
 * NOTE: Uses ArduPilot's interface but custom implementation because our HAL
 * structure differs (hal.util is a direct member, not a pointer). This is
 * documented in AP_DEPENDENCY_POLICY.md under "Provisional" implementations.
 *
 * @see standards/AP_DEPENDENCY_POLICY.md
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "debug.h"

class AP_InternalError {
public:
    // Error type enum - matches ArduPilot's bitmask enum exactly
    enum class error_t : uint32_t {                           // Hex      Decimal
        logger_mapfailure           = (1U <<  0),  // 0x00001  1
        logger_missing_logstructure = (1U <<  1),  // 0x00002  2
        logger_logwrite_missingfmt  = (1U <<  2),  // 0x00004  4
        logger_too_many_deletions   = (1U <<  3),  // 0x00008  8
        logger_bad_getfilename      = (1U <<  4),  // 0x00010  16
        panic                       = (1U <<  5),  // 0x00020  32
        logger_flushing_without_sem = (1U <<  6),  // 0x00040  64
        logger_bad_current_block    = (1U <<  7),  // 0x00080  128
        logger_blockcount_mismatch  = (1U <<  8),  // 0x00100  256
        logger_dequeue_failure      = (1U <<  9),  // 0x00200  512
        constraining_nan            = (1U << 10),  // 0x00400  1024
        watchdog_reset              = (1U << 11),  // 0x00800  2048
        iomcu_reset                 = (1U << 12),  // 0x01000  4096
        iomcu_fail                  = (1U << 13),  // 0x02000  8192
        spi_fail                    = (1U << 14),  // 0x04000  16384
        main_loop_stuck             = (1U << 15),  // 0x08000  32768
        gcs_bad_missionprotocol_link= (1U << 16),  // 0x10000  65536
        bitmask_range               = (1U << 17),  // 0x20000  131072
        gcs_offset                  = (1U << 18),  // 0x40000  262144
        i2c_isr                     = (1U << 19),  // 0x80000  524288
        flow_of_control             = (1U << 20),  //0x100000  1048576
        switch_full_sector_recursion= (1U << 21),  //0x200000  2097152
        bad_rotation                = (1U << 22),  //0x400000  4194304
        stack_overflow              = (1U << 23),  //0x800000  8388608
        imu_reset                   = (1U << 24),  //0x1000000 16777216
        gpio_isr                    = (1U << 25),  //0x2000000 33554432
        mem_guard                   = (1U << 26),  //0x4000000 67108864
        dma_fail                    = (1U << 27),  //0x8000000 134217728
        params_restored             = (1U << 28),  //0x10000000 268435456
        invalid_arg_or_result       = (1U << 29),  //0x20000000 536870912
        __LAST__                    = (1U << 30),  // used only for sanity check
    };

    static_assert(sizeof(error_t) == 4, "error_t should be 32-bit type");

    /**
     * @brief Report an error condition (ArduPilot-compatible interface)
     */
    void error(const AP_InternalError::error_t e, uint16_t line);

    /**
     * @brief Get last error line number
     */
    uint16_t last_error_line() const { return last_line; }

    /**
     * @brief Fill buffer with comma-separated list of error names
     *
     * Example output: "flow_of_ctrl,spi_fail"
     */
    void errors_as_string(uint8_t* buffer, uint16_t len) const;

    /**
     * @brief Convert single error code to string
     */
    void error_to_string(char* buffer, uint16_t len, error_t error_code) const;

    /**
     * @brief Get total error count
     */
    uint32_t count() const { return total_error_count; }

    /**
     * @brief Get accumulated error bitmask
     */
    uint32_t errors() const { return internal_errors; }

private:
    uint32_t internal_errors = 0;
    uint32_t total_error_count = 0;
    uint16_t last_line = 0;
};

// ArduPilot-compatible singleton accessor
namespace AP {
    AP_InternalError& internalerror();
}

// INTERNAL_ERROR macro - matches ArduPilot's usage
#ifndef __AP_LINE__
#define __AP_LINE__ __LINE__
#endif

#define INTERNAL_ERROR(error_number) \
    AP::internalerror().error(error_number, __AP_LINE__)

// EXPECT_DELAY_MS macro - used in calibration code
#ifndef EXPECT_DELAY_MS
#define EXPECT_DELAY_MS(ms) ((void)0)
#endif
