/**
 * @file AP_InternalError.cpp
 * @brief AP_InternalError implementation
 *
 * Implementation matching ArduPilot's interface.
 * Error string table copied from ArduPilot for compatibility.
 */

#include "AP_InternalError.h"
#include <cmath>

// Singleton instance (lazy init to avoid static constructor issues)
static AP_InternalError* s_instance = nullptr;

// Error string table (copied from ArduPilot for compatibility)
static const char* const error_bit_descriptions[] = {
    "mapfailure",     // logger_mapfailure
    "miss_struct",    // logger_missing_logstructure
    "write_mssfmt",   // logger_logwrite_missingfmt
    "many_deletes",   // logger_too_many_deletions
    "bad_getfile",    // logger_bad_getfilename
    "panic",
    "flush_no_sem",   // logger_flushing_without_sem
    "bad_curr_blk",   // logger_bad_current_block
    "blkcnt_bad",     // logger_blockcount_mismatch
    "dq_failure",     // logger_dequeue_failure
    "cnstring_nan",   // constraining_nan
    "watchdog_rst",   // watchdog_reset
    "iomcu_reset",
    "iomcu_fail",
    "spi_fail",
    "main_loop_stk",  // main_loop_stuck
    "gcs_bad_link",   // gcs_bad_missionprotocol_link
    "bitmask_range",
    "gcs_offset",
    "i2c_isr",
    "flow_of_ctrl",   // flow_of_control
    "sfs_recursion",  // switch_full_sector_recursion
    "bad_rotation",
    "stack_ovrflw",   // stack_overflow
    "imu_reset",
    "gpio_isr",
    "mem_guard",
    "dma_fail",
    "params_restored",
    "invalid arguments",
};

// Verify table size matches enum
static_assert(
    (1U << (sizeof(error_bit_descriptions) / sizeof(error_bit_descriptions[0]))) ==
    static_cast<uint32_t>(AP_InternalError::error_t::__LAST__),
    "error_bit_descriptions size mismatch"
);

void AP_InternalError::error(const AP_InternalError::error_t e, uint16_t line)
{
    internal_errors |= static_cast<uint32_t>(e);
    total_error_count++;
    last_line = line;

    // Debug output
    DBG_ERROR("[AP_InternalError] Error: 0x%08lx at line %u (total: %lu)",
              static_cast<unsigned long>(e),
              static_cast<unsigned int>(line),
              static_cast<unsigned long>(total_error_count));
}

void AP_InternalError::error_to_string(char* buffer, uint16_t len, error_t error_code) const
{
    if (len == 0) return;

    // Find which bit is set (log2)
    uint32_t code = static_cast<uint32_t>(error_code);
    uint32_t bit_index = 0;
    while (code > 1) {
        code >>= 1;
        bit_index++;
    }

    // Bounds check
    constexpr size_t table_size = sizeof(error_bit_descriptions) / sizeof(error_bit_descriptions[0]);
    if (bit_index < table_size) {
        strncpy(buffer, error_bit_descriptions[bit_index], len - 1);
        buffer[len - 1] = '\0';
    } else {
        snprintf(buffer, len, "unknown_%lu", static_cast<unsigned long>(bit_index));
    }
}

void AP_InternalError::errors_as_string(uint8_t* buffer, uint16_t len) const
{
    if (len == 0) return;

    buffer[0] = '\0';
    uint32_t buffer_used = 0;
    const char* format = "%s";  // no comma before first item

    constexpr size_t table_size = sizeof(error_bit_descriptions) / sizeof(error_bit_descriptions[0]);

    for (uint8_t i = 0; i < table_size; i++) {
        if (buffer_used >= len) {
            break;
        }
        if (internal_errors & (1U << i)) {
            int written = snprintf(
                reinterpret_cast<char*>(&buffer[buffer_used]),
                len - buffer_used,
                format,
                error_bit_descriptions[i]
            );
            format = ",%s";  // comma before subsequent items

            if (written < 0) {
                break;
            }
            buffer_used += static_cast<uint32_t>(written);
        }
    }
}

// Singleton accessor
namespace AP {
    AP_InternalError& internalerror() {
        if (!s_instance) {
            s_instance = new AP_InternalError();
        }
        return *s_instance;
    }
}
