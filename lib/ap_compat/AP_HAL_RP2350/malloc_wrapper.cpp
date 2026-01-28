/**
 * @file malloc_wrapper.cpp
 * @brief Zero-initializing memory allocation for ArduPilot compatibility
 *
 * ArduPilot code assumes allocated memory is zero-initialized (like ChibiOS).
 * Standard malloc/new does not zero memory.
 *
 * The Pico SDK's pico_malloc already wraps malloc with __wrap_malloc for
 * thread safety and panic on allocation failure. We can't override that.
 *
 * Instead, we override C++ operator new to use calloc (which zeros memory).
 * This ensures all C++ allocations get zeroed memory, fixing issues like
 * Device::_checked.regs being garbage instead of nullptr.
 *
 * See RP2350_FULL_AP_PORT.md PD11 for background.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include <cstdlib>
#include <cstring>
#include <new>

/**
 * @brief Override operator new to zero-initialize memory
 *
 * Uses calloc instead of malloc to ensure memory is zeroed.
 * This matches ArduPilot's expectation from ChibiOS.
 */
void* operator new(std::size_t size) {
    if (size == 0) {
        size = 1;  // Ensure unique pointer for zero-size allocations
    }
    void* ptr = calloc(1, size);
    if (ptr == nullptr) {
        // Note: We can't throw std::bad_alloc in embedded (no exceptions)
        // Let the Pico SDK's malloc panic handler deal with it
        ptr = malloc(size);  // This will panic if OOM
    }
    return ptr;
}

void* operator new[](std::size_t size) {
    return operator new(size);
}

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    if (size == 0) {
        size = 1;
    }
    return calloc(1, size);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    return operator new(size, std::nothrow);
}

void operator delete(void* ptr) noexcept {
    free(ptr);
}

void operator delete[](void* ptr) noexcept {
    free(ptr);
}

void operator delete(void* ptr, std::size_t) noexcept {
    free(ptr);
}

void operator delete[](void* ptr, std::size_t) noexcept {
    free(ptr);
}
