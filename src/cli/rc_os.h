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
// Boot Summary Callback (set by main)
// ============================================================================

/**
 * @brief Function pointer for boot/init summary reprint
 *
 * Set this to your HW validation print function.
 * Called when user presses 'b' in main menu (late-connect reprint).
 * Per DEBUG_OUTPUT.md: "Boot button can trigger result reprint"
 */
typedef void (*rc_os_boot_summary_fn)(void);
extern rc_os_boot_summary_fn rc_os_print_boot_summary;

/**
 * @brief Function pointer for full boot status (banner + HW validation).
 *
 * Called once on first terminal connection. Prints the full boot banner
 * including version, board info, watchdog status, and HW validation.
 * Enables non-blocking USB: firmware runs without terminal, boot output
 * is deferred until a terminal connects.
 */
typedef void (*rc_os_boot_status_fn)(void);
extern rc_os_boot_status_fn rc_os_print_boot_status;

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

/**
 * @brief GPS pause flag for mag calibration.
 *
 * Set to true by cmd_mag_cal() to suppress GPS reads on Core 1.
 * In bypass mode, GPS NMEA streaming (0x10) causes bus contention
 * with AK09916 mag reads (0x0C). Core 1 checks this flag before
 * calling core1_read_gps().
 */
extern volatile bool rc_os_mag_cal_active;

// ============================================================================
// Unhandled Key Callback (set by main)
// ============================================================================

/**
 * @brief Callback for keys not handled by the main menu.
 *
 * Set this to handle additional key commands without modifying
 * the CLI menu structure. Called with the character code.
 */
typedef void (*rc_os_unhandled_key_fn)(int key);
extern rc_os_unhandled_key_fn rc_os_on_unhandled_key;

// ============================================================================
// Mag Read Callback (set by main, used by mag cal)
// ============================================================================

/**
 * @brief Callback to read one magnetometer sample for compass calibration.
 *
 * Reads from seqlock (Core 1 keeps running). Returns false on seqlock
 * failure or mag not valid.
 * @return true on success
 */
typedef bool (*rc_os_read_mag_fn)(float* mx, float* my, float* mz);
extern rc_os_read_mag_fn rc_os_read_mag;

/**
 * @brief Reset mag read staleness counter.
 *
 * Call before starting a new mag cal so the staleness gate doesn't
 * inherit a counter from a previous calibration run.
 */
typedef void (*rc_os_reset_mag_staleness_fn)(void);
extern rc_os_reset_mag_staleness_fn rc_os_reset_mag_staleness;

// ============================================================================
// Pre/Post Calibration Hooks (set by main)
// ============================================================================

/**
 * @brief Optional hooks called before/after calibration sequences.
 *
 * Pre-hook pauses Core 1 sensor reads so Core 0 can take I2C ownership.
 * Post-hook signals calibration reload and resumes Core 1.
 * Set to NULL if no setup/teardown is needed.
 */
typedef void (*rc_os_cal_hook_fn)(void);
extern rc_os_cal_hook_fn rc_os_cal_pre_hook;
extern rc_os_cal_hook_fn rc_os_cal_post_hook;

// ============================================================================
// INTERIM: NeoPixel Calibration Override (Phase M.5)
// Replace with AP_Notify-style status state machine when implemented.
// ============================================================================

/**
 * @brief NeoPixel override mode values (set by CLI during calibration).
 *
 * 0 = off (normal NeoPixel behavior), 1-8 = calibration states.
 * Defined as constexpr in main.cpp. Values listed here for reference:
 *   0=off, 1=gyro, 2=level, 3=baro, 4=accel_wait, 5=accel_sample,
 *   6=mag, 7=success, 8=fail
 */
typedef void (*rc_os_set_cal_neo_fn)(uint8_t mode);
extern rc_os_set_cal_neo_fn rc_os_set_cal_neo;

// ============================================================================
// ESKF Live Output Callback (set by main)
// ============================================================================

/**
 * @brief Function pointer for compact ESKF status line.
 *
 * Called at 1Hz when live ESKF mode is active (user pressed 'e').
 * Should print a single compact line (no trailing newline).
 */
typedef void (*rc_os_eskf_live_fn)(void);
extern rc_os_eskf_live_fn rc_os_print_eskf_live;

// ============================================================================
// Calibration Feed Callback (set by main)
// ============================================================================

/**
 * @brief Wizard polling loop callback.
 *
 * Called from the wizard's blocking loop. Core 1 feeds sensor samples
 * directly (no I2C from Core 0). This callback exists for the wizard's
 * wait loop to call — main.cpp provides a no-op since Core 1 handles feeds.
 */
typedef void (*rc_os_feed_cal_fn)(void);
extern rc_os_feed_cal_fn rc_os_feed_cal;

#endif // ROCKETCHIP_RC_OS_H
