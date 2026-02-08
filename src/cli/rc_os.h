/**
 * @file rc_os.h
 * @brief RocketChip OS - CLI menu system (bare-metal)
 *
 * Provides terminal-based CLI with menu state machine.
 * Adapted from v0.3 FreeRTOS implementation for bare-metal Pico SDK.
 *
 * Key patterns:
 * - Single-key commands (no parsing)
 * - Two-level menu: Main → Calibration
 * - Non-blocking input via getchar_timeout_us(0)
 * - Terminal-connected guard (no USB I/O when disconnected)
 */

#ifndef ROCKETCHIP_RC_OS_H
#define ROCKETCHIP_RC_OS_H

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Menu State
// ============================================================================

typedef enum {
    RC_OS_MENU_MAIN = 0,
    RC_OS_MENU_CALIBRATION,
} rc_os_menu_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize RC_OS CLI system
 *
 * Call once after stdio_init_all() and before main loop.
 */
void rc_os_init(void);

// ============================================================================
// Main Loop Integration
// ============================================================================

/**
 * @brief Process CLI input (call from main loop)
 *
 * This function:
 * - Checks if terminal is connected
 * - Prints banner on first connection
 * - Processes single-key commands
 * - Runs calibration state machines
 *
 * Should be called at ~20Hz (every 50ms) from main loop.
 * Does nothing if terminal not connected.
 *
 * @return true if a command was processed
 */
bool rc_os_update(void);

/**
 * @brief Check if terminal is connected
 */
bool rc_os_is_connected(void);

/**
 * @brief Check if a calibration is currently in progress
 */
bool rc_os_is_calibrating(void);

/**
 * @brief Get current menu mode
 */
rc_os_menu_t rc_os_get_menu(void);

// ============================================================================
// Sensor Status Callback (set by main)
// ============================================================================

/**
 * @brief Function pointer for sensor status printing
 *
 * Set this to your sensor status print function.
 * Called when user presses 's' in main menu.
 */
typedef void (*rc_os_sensor_status_fn)(void);
extern rc_os_sensor_status_fn rc_os_print_sensor_status;

// ============================================================================
// Sensor Availability Flags (set by main)
// ============================================================================

/**
 * @brief Flags indicating which sensors are available for calibration.
 * Set these in main.cpp after sensor initialization.
 */
extern bool rc_os_imu_available;
extern bool rc_os_baro_available;

// ============================================================================
// Accel Read Callback (set by main, used by 6-pos cal)
// ============================================================================

/**
 * @brief Callback to read one accelerometer sample for 6-pos calibration.
 *
 * Set this in main.cpp after IMU initialization.
 * Should block until a fresh sample is available (~10ms at 100Hz).
 * @return true on success
 */
typedef bool (*rc_os_read_accel_fn)(float* ax, float* ay, float* az, float* temp_c);
extern rc_os_read_accel_fn rc_os_read_accel;

// ============================================================================
// I2C Bus Scan Guard (set by main)
// ============================================================================

/**
 * @brief Flag to disable I2C bus scan from CLI.
 *
 * Set to false when Core 1 owns the I2C bus (sensor phase).
 * Prevents 'i' command from corrupting bus while Core 1 reads sensors.
 * Defaults to true (scan allowed).
 */
extern bool rc_os_i2c_scan_allowed;

// ============================================================================
// Unhandled Key Callback (set by main)
// ============================================================================

/**
 * @brief Callback for keys not handled by the main menu.
 *
 * Set this to handle test commands (e.g., 'o', 'w', 'W' for IVP-29/30)
 * without modifying the CLI menu structure. Called with the character code.
 */
typedef void (*rc_os_unhandled_key_fn)(int key);
extern rc_os_unhandled_key_fn rc_os_on_unhandled_key;

// ============================================================================
// Pre/Post Calibration Hooks (set by main)
// ============================================================================

/**
 * @brief Optional hooks called before/after 6-pos accel calibration.
 *
 * Use these to disable the ICM-20948 I2C master during calibration,
 * preventing bank-switching race conditions on rapid accel reads.
 * Set to NULL if no setup/teardown is needed.
 */
typedef void (*rc_os_cal_hook_fn)(void);
extern rc_os_cal_hook_fn rc_os_cal_pre_hook;
extern rc_os_cal_hook_fn rc_os_cal_post_hook;

#endif // ROCKETCHIP_RC_OS_H
