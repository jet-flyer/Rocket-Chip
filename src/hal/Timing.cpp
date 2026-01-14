/**
 * @file Timing.cpp
 * @brief Time and delay utilities implementation
 *
 * Implements platform-independent timing functions wrapping FreeRTOS
 * and Pico SDK timing APIs.
 *
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#include "Timing.h"

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"

#include <cstdio>

namespace rocketchip {
namespace hal {

// ============================================================================
// Timing class implementation
// ============================================================================

uint64_t Timing::micros() {
    return time_us_64();
}

uint32_t Timing::micros32() {
    return time_us_32();
}

uint64_t Timing::millis() {
    return time_us_64() / 1000ULL;
}

uint32_t Timing::millis32() {
    return static_cast<uint32_t>(time_us_64() / 1000ULL);
}

void Timing::delayMicros(uint32_t us) {
    busy_wait_us_32(us);
}

void Timing::delayMs(uint32_t ms) {
    // Use FreeRTOS delay if scheduler is running
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        // Fall back to busy wait if scheduler not running
        busy_wait_us_32(ms * 1000);
    }
}

uint32_t Timing::elapsedMicros(uint32_t start_us) {
    uint32_t now = micros32();
    // Handles wrap-around correctly due to unsigned subtraction
    return now - start_us;
}

uint32_t Timing::elapsedMs(uint32_t start_ms) {
    uint32_t now = millis32();
    // Handles wrap-around correctly due to unsigned subtraction
    return now - start_ms;
}

bool Timing::intervalElapsed(uint32_t start_us, uint32_t interval_us) {
    return elapsedMicros(start_us) >= interval_us;
}

// ============================================================================
// IntervalTimer class implementation
// ============================================================================

IntervalTimer::IntervalTimer(uint32_t interval_us)
    : m_interval_us(interval_us)
    , m_last_trigger_us(Timing::micros32())
{
}

bool IntervalTimer::ready() {
    if (elapsed()) {
        reset();
        return true;
    }
    return false;
}

bool IntervalTimer::elapsed() const {
    return Timing::elapsedMicros(m_last_trigger_us) >= m_interval_us;
}

void IntervalTimer::reset() {
    m_last_trigger_us = Timing::micros32();
}

void IntervalTimer::setInterval(uint32_t interval_us) {
    m_interval_us = interval_us;
}

uint32_t IntervalTimer::remaining() const {
    uint32_t elapsed_us = Timing::elapsedMicros(m_last_trigger_us);
    if (elapsed_us >= m_interval_us) {
        return 0;
    }
    return m_interval_us - elapsed_us;
}

// ============================================================================
// StopWatch class implementation
// ============================================================================

void StopWatch::start() {
    m_start_us = Timing::micros32();
    m_running = true;
}

uint32_t StopWatch::stop() {
    if (!m_running) {
        return 0;
    }
    m_running = false;
    return Timing::elapsedMicros(m_start_us);
}

uint32_t StopWatch::elapsed() const {
    if (!m_running) {
        return 0;
    }
    return Timing::elapsedMicros(m_start_us);
}

void StopWatch::reset() {
    m_start_us = 0;
    m_running = false;
}

// ============================================================================
// ScopedTimer class implementation
// ============================================================================

ScopedTimer::ScopedTimer(const char* label)
    : m_label(label)
    , m_start_us(Timing::micros32())
{
}

ScopedTimer::~ScopedTimer() {
#ifndef ROCKETCHIP_DISABLE_TIMING_DEBUG
    uint32_t elapsed_us = Timing::elapsedMicros(m_start_us);
    // Output timing result - using printf for debug output
    // In production, this could be replaced with a logging system
    printf("%s: %luus\n", m_label, static_cast<unsigned long>(elapsed_us));
#endif
}

// ============================================================================
// RateStats class implementation
// ============================================================================

RateStats::RateStats()
    : m_last_tick_us(0)
    , m_min_interval_us(UINT32_MAX)
    , m_max_interval_us(0)
    , m_total_interval_us(0)
    , m_count(0)
{
}

void RateStats::tick() {
    uint32_t now = Timing::micros32();

    if (m_count > 0) {
        uint32_t interval = Timing::elapsedMicros(m_last_tick_us);

        if (interval < m_min_interval_us) {
            m_min_interval_us = interval;
        }
        if (interval > m_max_interval_us) {
            m_max_interval_us = interval;
        }
        m_total_interval_us += interval;
    }

    m_last_tick_us = now;
    m_count++;
}

float RateStats::getRate() const {
    if (m_count < 2) {
        return 0.0f;
    }

    // Calculate average interval
    float avg_interval_us = static_cast<float>(m_total_interval_us) /
                            static_cast<float>(m_count - 1);

    if (avg_interval_us == 0.0f) {
        return 0.0f;
    }

    // Convert to Hz (1,000,000 us per second)
    return 1000000.0f / avg_interval_us;
}

void RateStats::reset() {
    m_last_tick_us = 0;
    m_min_interval_us = UINT32_MAX;
    m_max_interval_us = 0;
    m_total_interval_us = 0;
    m_count = 0;
}

} // namespace hal
} // namespace rocketchip
