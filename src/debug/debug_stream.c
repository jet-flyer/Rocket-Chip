/**
 * @file debug_stream.c
 * @brief Deferred debug output implementation
 *
 * Uses FreeRTOS Stream Buffer for lock-free message passing between
 * high-priority tasks (producers) and UITask (consumer).
 */

#include "debug_stream.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include <stdio.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================

#define DEBUG_STREAM_SIZE       4096    // 4KB buffer
#define DEBUG_STREAM_TRIGGER    1       // Trigger level (bytes before unblocking reader)
#define DEBUG_MSG_MAX           256     // Maximum single message size

// ============================================================================
// Private State
// ============================================================================

static StreamBufferHandle_t g_debug_stream = NULL;
static bool g_initialized = false;

// ============================================================================
// Public API
// ============================================================================

void debug_stream_init(void) {
    if (g_initialized) {
        return;
    }

    g_debug_stream = xStreamBufferCreate(DEBUG_STREAM_SIZE, DEBUG_STREAM_TRIGGER);
    if (g_debug_stream != NULL) {
        g_initialized = true;
    }
}

int dbg_printf(const char* fmt, ...) {
    if (!g_initialized || g_debug_stream == NULL) {
        return 0;
    }

    // Format message to local buffer
    char buffer[DEBUG_MSG_MAX];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len <= 0) {
        return 0;
    }

    // Clamp to buffer size
    if (len >= (int)sizeof(buffer)) {
        len = sizeof(buffer) - 1;
    }

    // Non-blocking write to stream buffer (timeout = 0)
    size_t written = xStreamBufferSend(g_debug_stream, buffer, (size_t)len, 0);

    return (int)written;
}

void debug_stream_flush(void) {
    if (!g_initialized || g_debug_stream == NULL) {
        return;
    }

    // Read and print in chunks
    char buffer[256];
    size_t bytes_read;

    // Drain the entire buffer
    while ((bytes_read = xStreamBufferReceive(g_debug_stream, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[bytes_read] = '\0';
        // Use fputs instead of printf to avoid format string issues
        fputs(buffer, stdout);
    }

    // Ensure output is flushed to USB
    fflush(stdout);
}

bool debug_stream_has_data(void) {
    if (!g_initialized || g_debug_stream == NULL) {
        return false;
    }

    return !xStreamBufferIsEmpty(g_debug_stream);
}

size_t debug_stream_pending(void) {
    if (!g_initialized || g_debug_stream == NULL) {
        return 0;
    }

    return xStreamBufferBytesAvailable(g_debug_stream);
}
