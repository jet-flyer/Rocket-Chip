/**
 * @file debug_stream.h
 * @brief Deferred debug output via FreeRTOS Stream Buffer
 *
 * Provides non-blocking debug output that is safe to call from any task,
 * including high-priority real-time tasks. Output is buffered and later
 * flushed to USB CDC by a low-priority task (UITask).
 *
 * Usage:
 *   - Call debug_stream_init() before scheduler starts
 *   - Use dbg_printf() instead of printf() in sensor/calibration code
 *   - Call debug_stream_flush() periodically from UITask
 *
 * This solves the fundamental RTOS issue where printf() from high-priority
 * tasks blocks on USB CDC mutex, causing priority inversion and USB errors.
 */

#ifndef ROCKETCHIP_DEBUG_STREAM_H
#define ROCKETCHIP_DEBUG_STREAM_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize debug stream buffer
 *
 * Must be called before vTaskStartScheduler().
 * Creates the internal FreeRTOS stream buffer.
 */
void debug_stream_init(void);

/**
 * @brief Non-blocking printf alternative
 *
 * Formats message and writes to stream buffer without blocking.
 * Safe to call from any task at any priority, including ISRs.
 *
 * @param fmt Format string (printf-style)
 * @param ... Format arguments
 * @return Number of bytes written, or 0 if buffer full
 *
 * @note If buffer is full, message is silently dropped (no blocking)
 * @note Maximum single message size is 256 bytes
 */
int dbg_printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief Flush buffered output to USB CDC
 *
 * Reads from stream buffer and writes to stdout via printf().
 * Should only be called from UITask when terminal is connected.
 *
 * @note This function MAY block on USB CDC - only call from low-priority task
 */
void debug_stream_flush(void);

/**
 * @brief Check if buffer has pending data
 *
 * @return true if there is data waiting to be flushed
 */
bool debug_stream_has_data(void);

/**
 * @brief Get number of bytes pending in buffer
 *
 * @return Number of bytes waiting to be flushed
 */
size_t debug_stream_pending(void);

#ifdef __cplusplus
}
#endif

#endif // ROCKETCHIP_DEBUG_STREAM_H
