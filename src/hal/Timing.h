/**
 * @file Timing.h
 * @brief Time and delay utilities
 * 
 * Provides platform-independent timing functions, wrapping FreeRTOS
 * and Pico SDK timing APIs.
 * 
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#ifndef ROCKETCHIP_HAL_TIMING_H
#define ROCKETCHIP_HAL_TIMING_H

#include <cstdint>

namespace rocketchip {
namespace hal {

/**
 * @brief Timing utilities
 * 
 * Provides microsecond and millisecond timing functions.
 * Uses hardware timer for high-resolution timestamps.
 */
class Timing {
public:
    /**
     * @brief Get current time in microseconds
     * 
     * High-resolution timestamp from hardware timer.
     * Wraps approximately every 71 minutes (32-bit) or never (64-bit).
     * 
     * @return Microseconds since boot
     */
    static uint64_t micros();

    /**
     * @brief Get current time in microseconds (32-bit, faster)
     * 
     * Use when 71-minute wrap is acceptable.
     * 
     * @return Microseconds since boot (wrapping)
     */
    static uint32_t micros32();

    /**
     * @brief Get current time in milliseconds
     * @return Milliseconds since boot
     */
    static uint64_t millis();

    /**
     * @brief Get current time in milliseconds (32-bit)
     * 
     * Wraps approximately every 49 days.
     * 
     * @return Milliseconds since boot (wrapping)
     */
    static uint32_t millis32();

    /**
     * @brief Busy-wait delay in microseconds
     * 
     * @warning Blocks the current core. Use sparingly.
     * @note For delays > 1ms, prefer delayMs() which yields to RTOS
     * 
     * @param us Microseconds to delay
     */
    static void delayMicros(uint32_t us);

    /**
     * @brief Delay in milliseconds (RTOS-aware)
     * 
     * Yields to FreeRTOS scheduler, allowing other tasks to run.
     * Actual delay may be slightly longer due to scheduling.
     * 
     * @param ms Milliseconds to delay
     */
    static void delayMs(uint32_t ms);

    /**
     * @brief Calculate elapsed time in microseconds
     * 
     * Handles 32-bit wrap correctly for intervals up to 71 minutes.
     * 
     * @param start_us Start timestamp from micros32()
     * @return Microseconds elapsed since start
     */
    static uint32_t elapsedMicros(uint32_t start_us);

    /**
     * @brief Calculate elapsed time in milliseconds
     * 
     * Handles 32-bit wrap correctly.
     * 
     * @param start_ms Start timestamp from millis32()
     * @return Milliseconds elapsed since start
     */
    static uint32_t elapsedMs(uint32_t start_ms);

    /**
     * @brief Check if interval has elapsed
     * 
     * @param start_us Start timestamp from micros32()
     * @param interval_us Interval in microseconds
     * @return true if interval has elapsed
     */
    static bool intervalElapsed(uint32_t start_us, uint32_t interval_us);

private:
    Timing() = delete;  // Static-only class
};


/**
 * @brief Simple interval timer
 * 
 * Useful for periodic tasks or rate limiting.
 * 
 * @code
 * IntervalTimer sensorTimer(1000);  // 1ms interval
 * 
 * while (true) {
 *     if (sensorTimer.ready()) {
 *         readSensors();
 *     }
 *     // ... other work
 * }
 * @endcode
 */
class IntervalTimer {
public:
    /**
     * @brief Construct interval timer
     * @param interval_us Interval in microseconds
     */
    explicit IntervalTimer(uint32_t interval_us);

    /**
     * @brief Check if interval has elapsed and reset if so
     * @return true if interval elapsed (automatically resets)
     */
    bool ready();

    /**
     * @brief Check if interval has elapsed without resetting
     * @return true if interval elapsed
     */
    bool elapsed() const;

    /**
     * @brief Reset timer to current time
     */
    void reset();

    /**
     * @brief Change interval
     * @param interval_us New interval in microseconds
     */
    void setInterval(uint32_t interval_us);

    /**
     * @brief Get configured interval
     */
    uint32_t getInterval() const { return m_interval_us; }

    /**
     * @brief Get time remaining until next trigger
     * @return Microseconds remaining (0 if already elapsed)
     */
    uint32_t remaining() const;

private:
    uint32_t m_interval_us;
    uint32_t m_last_trigger_us;
};


/**
 * @brief Execution time measurement
 * 
 * RAII-style timer for measuring code execution time.
 * 
 * @code
 * {
 *     ScopedTimer timer("SensorRead");
 *     readAllSensors();
 * }  // Prints "SensorRead: 125us" to debug output
 * 
 * // Or manual use:
 * StopWatch sw;
 * sw.start();
 * doWork();
 * uint32_t elapsed = sw.stop();
 * @endcode
 */
class StopWatch {
public:
    StopWatch() : m_start_us(0), m_running(false) {}

    /**
     * @brief Start the stopwatch
     */
    void start();

    /**
     * @brief Stop and return elapsed time
     * @return Elapsed microseconds
     */
    uint32_t stop();

    /**
     * @brief Get elapsed time without stopping
     * @return Microseconds since start
     */
    uint32_t elapsed() const;

    /**
     * @brief Check if stopwatch is running
     */
    bool isRunning() const { return m_running; }

    /**
     * @brief Reset stopwatch
     */
    void reset();

private:
    uint32_t m_start_us;
    bool m_running;
};


/**
 * @brief RAII scoped timer for performance measurement
 * 
 * Automatically prints elapsed time when scope exits.
 * Compile out in release builds with ROCKETCHIP_DISABLE_TIMING_DEBUG.
 */
class ScopedTimer {
public:
    /**
     * @brief Start timing with label
     * @param label Description printed with timing result
     */
    explicit ScopedTimer(const char* label);

    /**
     * @brief Stop timing and print result
     */
    ~ScopedTimer();

private:
    const char* m_label;
    uint32_t m_start_us;
};


/**
 * @brief Rate statistics tracker
 * 
 * Tracks timing statistics for a periodic operation.
 * Useful for monitoring actual sensor sample rates.
 */
class RateStats {
public:
    RateStats();

    /**
     * @brief Record a sample
     * 
     * Call this each time the operation executes.
     */
    void tick();

    /**
     * @brief Get average rate in Hz
     */
    float getRate() const;

    /**
     * @brief Get minimum interval (microseconds)
     */
    uint32_t getMinInterval() const { return m_min_interval_us; }

    /**
     * @brief Get maximum interval (microseconds)
     */
    uint32_t getMaxInterval() const { return m_max_interval_us; }

    /**
     * @brief Get jitter (max - min interval)
     */
    uint32_t getJitter() const { return m_max_interval_us - m_min_interval_us; }

    /**
     * @brief Get total sample count
     */
    uint32_t getCount() const { return m_count; }

    /**
     * @brief Reset statistics
     */
    void reset();

private:
    uint32_t m_last_tick_us;
    uint32_t m_min_interval_us;
    uint32_t m_max_interval_us;
    uint64_t m_total_interval_us;
    uint32_t m_count;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_TIMING_H
