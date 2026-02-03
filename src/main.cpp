/**
 * @file main.cpp
 * @brief RocketChip firmware entry point
 *
 * FreeRTOS SMP application with sensor sampling and calibration CLI.
 * Based on pico-examples/freertos/hello_freertos pattern.
 *
 * Core Assignment:
 * - Core 0: UITask (CLI, printf via USB CDC)
 * - Core 1: SensorTask (I2C sensor sampling)
 *
 * USB is handled by SDK's pico_stdio_usb with IRQ-based tud_task().
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include "rocketchip/config.h"
#include "ws2812_status.h"

// Sensor task and debug stream (C interface)
extern "C" {
#include "sensor_task.h"
#include "calibration_manager.h"
#include "i2c_bus.h"
#include "debug/debug_stream.h"
}

// ============================================================================
// Build Version (update on each significant change)
// ============================================================================

static const char* kBuildVersion = "v0.3.4-stack-fix";

// ============================================================================
// Task Handles
// ============================================================================

static TaskHandle_t g_uiTaskHandle = nullptr;

// ============================================================================
// CLI Menu State
// ============================================================================

enum class MenuMode {
    Main,
    Calibration,
    Calibrating,
    SixPos,
    Wizard,
    WizardStep
};

static MenuMode g_menuMode = MenuMode::Main;

// Wizard state
enum class WizardStep {
    Gyro,
    Level,
    SixPos,
    Save,
    Done
};
static WizardStep g_wizardStep = WizardStep::Gyro;
static bool g_inWizard = false;  // Track if we came from wizard

// ============================================================================
// CLI Helpers
// ============================================================================

static void print_main_menu() {
    printf("\n=== RocketChip %s ===\n", kBuildVersion);
    printf("Commands:\n");
    printf("  h - Help (this menu)\n");
    printf("  s - Sensor status\n");
    printf("  c - Calibration menu\n");
    printf("\n");
}

static void print_cal_menu() {
    printf("\n=== Calibration Menu ===\n");
    printf("  w - Wizard (guided calibration of all sensors)\n");
    printf("  l - Level Cal (quick accel level)\n");
    printf("  a - Accel 6-Pos (full accelerometer)\n");
    printf("  g - Gyro Cal\n");
    printf("  m - Mag Cal (magnetometer/compass)\n");
    printf("  b - Baro Cal (ground pressure)\n");
    printf("  v - View calibration data\n");
    printf("  x - Exit (back to main menu)\n");
    printf("\n");
}

static void print_wizard_step() {
    printf("\n=== Calibration Wizard ===\n");
    switch (g_wizardStep) {
        case WizardStep::Gyro:
            printf("Step 1/4: GYRO CALIBRATION\n");
            printf("Keep device completely still on a flat surface.\n");
            printf("Press ENTER to start, 's' to skip, 'x' to cancel wizard\n");
            break;
        case WizardStep::Level:
            printf("Step 2/4: LEVEL CALIBRATION\n");
            printf("Keep device flat and still (USB port toward you).\n");
            printf("Press ENTER to start, 's' to skip, 'x' to cancel wizard\n");
            break;
        case WizardStep::SixPos:
            printf("Step 3/4: 6-POSITION ACCEL CALIBRATION\n");
            printf("You will place the device in 6 orientations.\n");
            printf("Press ENTER to start, 's' to skip, 'x' to cancel wizard\n");
            break;
        case WizardStep::Save:
            printf("Step 4/4: SAVE CALIBRATION\n");
            printf("Save calibration to flash memory?\n");
            printf("Press ENTER to save, 's' to skip (calibration lost on reboot)\n");
            break;
        case WizardStep::Done:
            printf("\nCalibration wizard complete!\n");
            break;
    }
}

static void wizard_next_step() {
    switch (g_wizardStep) {
        case WizardStep::Gyro:
            g_wizardStep = WizardStep::Level;
            break;
        case WizardStep::Level:
            g_wizardStep = WizardStep::SixPos;
            break;
        case WizardStep::SixPos:
            g_wizardStep = WizardStep::Save;
            break;
        case WizardStep::Save:
            g_wizardStep = WizardStep::Done;
            break;
        case WizardStep::Done:
            break;
    }
}

static void wizard_start_current_step() {
    bool started = false;
    g_inWizard = true;  // Track that we're in wizard mode

    switch (g_wizardStep) {
        case WizardStep::Gyro:
            printf("Starting gyro calibration - keep device STILL...\n");
            ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_CYAN);
            started = sensor_task_start_gyro_cal();
            if (started) {
                g_menuMode = MenuMode::Calibrating;
            } else {
                printf("Failed to start gyro calibration\n");
            }
            break;
        case WizardStep::Level:
            printf("Starting level calibration - keep device FLAT and STILL...\n");
            ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_CYAN);
            started = sensor_task_start_accel_level_cal();
            if (started) {
                g_menuMode = MenuMode::Calibrating;
            } else {
                printf("Failed to start level calibration\n");
            }
            break;
        case WizardStep::SixPos:
            printf("Starting 6-position accelerometer calibration...\n");
            ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_MAGENTA);
            started = sensor_task_start_accel_6pos_cal();
            if (started) {
                g_menuMode = MenuMode::SixPos;
            } else {
                printf("Failed to start 6-pos calibration\n");
            }
            break;
        case WizardStep::Save:
            printf("Saving calibration to flash...\n");
            if (sensor_task_save_calibration()) {
                printf("Calibration saved!\n");
                ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_GREEN);
            } else {
                printf("Failed to save calibration\n");
                ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_RED);
            }
            wizard_next_step();
            print_wizard_step();
            g_inWizard = false;
            break;
        case WizardStep::Done:
            g_inWizard = false;
            g_menuMode = MenuMode::Calibration;
            print_cal_menu();
            break;
    }
}

static void print_sensor_status() {
    sensor_status_t status;
    sensor_task_get_status(&status);

    printf("\n=== Sensor Status ===\n");
    printf("IMU ready: %s (samples: %lu)\n",
           status.imu_ready ? "YES" : "NO",
           (unsigned long)status.imu_sample_count);
    printf("Baro ready: %s (samples: %lu)\n",
           status.baro_ready ? "YES" : "NO",
           (unsigned long)status.baro_sample_count);
    printf("Calibration valid: %s\n",
           status.calibration_valid ? "YES" : "NO");

    if (status.imu_ready) {
        imu_data_t imu;
        if (sensor_task_get_imu(&imu)) {
            printf("\nIMU Data:\n");
            printf("  Accel: X=%.2f Y=%.2f Z=%.2f m/s²\n",
                   imu.accel_x, imu.accel_y, imu.accel_z);
            printf("  Gyro:  X=%.3f Y=%.3f Z=%.3f rad/s\n",
                   imu.gyro_x, imu.gyro_y, imu.gyro_z);
            printf("  Mag:   X=%.1f Y=%.1f Z=%.1f µT\n",
                   imu.mag_x, imu.mag_y, imu.mag_z);
            printf("  Temp:  %.1f °C\n", imu.temperature_c);
        }
    }

    if (status.baro_ready) {
        baro_data_t baro;
        if (sensor_task_get_baro(&baro)) {
            printf("\nBaro Data:\n");
            printf("  Pressure: %.1f Pa (%.2f hPa)\n",
                   baro.pressure_pa, baro.pressure_pa / 100.0f);
            printf("  Altitude: %.2f m AGL\n", baro.altitude_m);
            printf("  Temp:     %.1f °C\n", baro.temperature_c);
        }
    }
    printf("\n");
}

static void print_calibration_data() {
    const calibration_store_t* cal = calibration_manager_get();

    printf("\n=== Current Calibration ===\n");
    printf("Version: %u\n", cal->version);
    printf("Flags: 0x%02lX\n", (unsigned long)cal->cal_flags);

    printf("\nAccel:\n");
    printf("  Offset: X=%.4f Y=%.4f Z=%.4f m/s²\n",
           cal->accel.offset.x, cal->accel.offset.y, cal->accel.offset.z);
    printf("  Scale:  X=%.4f Y=%.4f Z=%.4f\n",
           cal->accel.scale.x, cal->accel.scale.y, cal->accel.scale.z);
    printf("  Status: %s\n",
           (cal->accel.status & CAL_STATUS_ACCEL_6POS) ? "6-pos" :
           (cal->accel.status & CAL_STATUS_LEVEL) ? "level" : "default");

    printf("\nGyro:\n");
    printf("  Bias: X=%.6f Y=%.6f Z=%.6f rad/s\n",
           cal->gyro.bias.x, cal->gyro.bias.y, cal->gyro.bias.z);
    printf("  Status: %s\n",
           (cal->gyro.status & CAL_STATUS_GYRO) ? "calibrated" : "default");

    printf("\nBaro:\n");
    printf("  Ground pressure: %.1f Pa\n", cal->baro.ground_pressure_pa);
    printf("  Status: %s\n",
           (cal->baro.status & CAL_STATUS_BARO) ? "calibrated" : "default");
    printf("\n");
}

// ============================================================================
// UI Task (Core 1)
// ============================================================================

/**
 * @brief User interface task - handles USB CDC serial and calibration CLI
 */
static void UITask(void* pvParameters) {
    (void)pvParameters;

    bool wasConnected = false;
    uint8_t lastProgress = 0;
    int8_t last6PosIdx = -1;

    while (true) {
        bool isConnected = stdio_usb_connected();

        if (!isConnected) {
            // Not connected - blink LED slowly
            wasConnected = false;
            ws2812_set_mode(WS2812_MODE_BREATHE, WS2812_COLOR_BLUE);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Terminal just connected
        if (!wasConnected) {
            wasConnected = true;
            vTaskDelay(pdMS_TO_TICKS(100));

            // Drain any garbage from USB input buffer
            while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {}

            // Set NeoPixel to green (connected)
            ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_GREEN);

            // Print status
            sensor_status_t status;
            sensor_task_get_status(&status);
            const calibration_store_t* cal = calibration_manager_get();

            printf("\n=== RocketChip %s ===\n", kBuildVersion);
            printf("IMU: %s  Baro: %s\n",
                   status.imu_ready ? "OK" : "FAIL",
                   status.baro_ready ? "OK" : "FAIL");

            // Show calibration status detail
            if (status.calibration_valid) {
                printf("Cal: LOADED from flash (");
                bool first = true;
                if (cal->gyro.status & CAL_STATUS_GYRO) {
                    printf("gyro");
                    first = false;
                }
                if (cal->accel.status & CAL_STATUS_ACCEL_6POS) {
                    printf("%s6pos", first ? "" : "+");
                    first = false;
                } else if (cal->accel.status & CAL_STATUS_LEVEL) {
                    printf("%slevel", first ? "" : "+");
                    first = false;
                }
                if (cal->baro.status & CAL_STATUS_BARO) {
                    printf("%sbaro", first ? "" : "+");
                }
                printf(")\n");
            } else {
                printf("Cal: DEFAULT (run 'c' -> 'w' for wizard)\n");
            }
            printf("Press 'h' for help\n\n");

            g_menuMode = MenuMode::Main;
        }

        // Handle calibration-in-progress state
        if (g_menuMode == MenuMode::Calibrating) {
            uint8_t progress = sensor_task_get_cal_progress();
            if (progress != lastProgress) {
                printf("\rProgress: %u%%  ", progress);
                fflush(stdout);
                lastProgress = progress;
            }

            if (!sensor_task_is_calibrating()) {
                // Calibration finished
                cal_result_t result = calibration_get_result();
                printf("\n");
                if (result == CAL_RESULT_OK) {
                    printf("Calibration complete!\n");
                    ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_GREEN);
                } else {
                    printf("Calibration failed: ");
                    switch (result) {
                        case CAL_RESULT_MOTION_DETECTED:
                            printf("motion detected\n");
                            break;
                        case CAL_RESULT_INVALID_DATA:
                            printf("invalid data\n");
                            break;
                        case CAL_RESULT_TIMEOUT:
                            printf("timeout\n");
                            break;
                        default:
                            printf("error %d\n", result);
                            break;
                    }
                    ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_RED);
                }
                // Reset state so we can start another calibration
                calibration_reset_state();

                // Return to wizard if we came from there
                if (g_inWizard) {
                    wizard_next_step();
                    if (g_wizardStep == WizardStep::Done) {
                        printf("\nCalibration wizard complete!\n");
                        g_inWizard = false;
                        g_menuMode = MenuMode::Calibration;
                        print_cal_menu();
                    } else {
                        g_menuMode = MenuMode::Wizard;
                        print_wizard_step();
                    }
                } else {
                    g_menuMode = MenuMode::Calibration;
                    print_cal_menu();
                }
            }
            // Flush debug output during calibration
            debug_stream_flush();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Handle 6-position calibration state
        if (g_menuMode == MenuMode::SixPos) {
            int8_t posIdx = sensor_task_get_6pos_position();

            // Check if position changed
            if (posIdx != last6PosIdx) {
                last6PosIdx = posIdx;
                if (posIdx >= 0) {
                    printf("\n=== Position %d/6: %s ===\n",
                           posIdx + 1,
                           sensor_task_get_6pos_position_name());
                    printf("Place device in position, then press ENTER\n");
                }
            }

            // Show progress if collecting
            uint8_t posProgress = sensor_task_get_6pos_position_progress();
            if (posProgress > 0 && posProgress < 100) {
                printf("\rCollecting: %u%%  ", posProgress);
                fflush(stdout);
            }

            // Check if calibration finished
            if (!sensor_task_is_calibrating()) {
                cal_result_t result = calibration_get_result();
                printf("\n");
                if (result == CAL_RESULT_OK) {
                    printf("6-position calibration complete!\n");
                    print_calibration_data();
                    ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_GREEN);
                } else {
                    printf("6-position calibration failed: ");
                    switch (result) {
                        case CAL_RESULT_MOTION_DETECTED:
                            printf("motion detected\n");
                            break;
                        case CAL_RESULT_INVALID_DATA:
                            printf("fit failed or data invalid\n");
                            break;
                        default:
                            printf("error %d\n", result);
                            break;
                    }
                    ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_RED);
                }
                // Reset state so we can start another calibration
                calibration_reset_state();

                // Return to wizard if we came from there
                if (g_inWizard) {
                    wizard_next_step();
                    if (g_wizardStep == WizardStep::Done) {
                        printf("\nCalibration wizard complete!\n");
                        g_inWizard = false;
                        g_menuMode = MenuMode::Calibration;
                        print_cal_menu();
                    } else {
                        g_menuMode = MenuMode::Wizard;
                        print_wizard_step();
                    }
                } else {
                    g_menuMode = MenuMode::Calibration;
                    print_cal_menu();
                }
            }

            // Check for keypress (ENTER to accept position, 'x' to cancel)
            int c = getchar_timeout_us(0);
            if (c == '\r' || c == '\n') {
                if (sensor_task_accept_6pos_position()) {
                    printf("Collecting samples...\n");
                    ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_YELLOW);
                }
            } else if (c == 'x' || c == 'X') {
                calibration_cancel();
                printf("\n6-position calibration cancelled\n");
                if (g_inWizard) {
                    g_inWizard = false;
                }
                g_menuMode = MenuMode::Calibration;
                print_cal_menu();
            }

            // Flush debug output during 6-pos calibration
            debug_stream_flush();

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Normal menu input handling
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) {
            ws2812_update();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Process command based on menu mode
        if (g_menuMode == MenuMode::Main) {
            switch (c) {
                case 'h':
                case 'H':
                case '?':
                    print_main_menu();
                    break;
                case 's':
                case 'S':
                    print_sensor_status();
                    break;
                case 'c':
                case 'C':
                    g_menuMode = MenuMode::Calibration;
                    print_cal_menu();
                    break;
                default:
                    break;
            }
        } else if (g_menuMode == MenuMode::Wizard) {
            // Handle wizard menu input
            switch (c) {
                case '\r':
                case '\n':
                    wizard_start_current_step();
                    break;
                case 's':
                case 'S':
                    printf("Skipping this step...\n");
                    wizard_next_step();
                    if (g_wizardStep == WizardStep::Done) {
                        printf("\nCalibration wizard complete!\n");
                        g_menuMode = MenuMode::Calibration;
                        print_cal_menu();
                    } else {
                        print_wizard_step();
                    }
                    break;
                case 'x':
                case 'X':
                    printf("Wizard cancelled.\n");
                    g_menuMode = MenuMode::Calibration;
                    print_cal_menu();
                    break;
                default:
                    break;
            }
        } else if (g_menuMode == MenuMode::Calibration) {
            switch (c) {
                case 'w':
                case 'W':
                    g_wizardStep = WizardStep::Gyro;
                    g_menuMode = MenuMode::Wizard;
                    print_wizard_step();
                    break;

                case 'l':
                case 'L':
                    printf("Starting level calibration - keep device FLAT and STILL...\n");
                    ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_CYAN);
                    if (sensor_task_start_accel_level_cal()) {
                        g_menuMode = MenuMode::Calibrating;
                        lastProgress = 0;
                    } else {
                        printf("Failed to start level calibration\n");
                    }
                    break;

                case 'a':
                case 'A':
                    printf("Starting 6-position accelerometer calibration...\n");
                    ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_MAGENTA);
                    if (sensor_task_start_accel_6pos_cal()) {
                        g_menuMode = MenuMode::SixPos;
                        last6PosIdx = -1;
                    } else {
                        printf("Failed to start 6-pos calibration\n");
                    }
                    break;

                case 'g':
                case 'G':
                    printf("Starting gyro calibration - keep device STILL...\n");
                    ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_CYAN);
                    if (sensor_task_start_gyro_cal()) {
                        g_menuMode = MenuMode::Calibrating;
                        lastProgress = 0;
                    } else {
                        printf("Failed to start gyro calibration\n");
                    }
                    break;

                case 'm':
                case 'M':
                    printf("Magnetometer calibration not yet implemented\n");
                    // TODO: Implement compass calibration
                    break;

                case 'b':
                case 'B':
                    printf("Starting baro ground reference calibration...\n");
                    ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_CYAN);
                    if (sensor_task_start_baro_cal()) {
                        g_menuMode = MenuMode::Calibrating;
                        lastProgress = 0;
                    } else {
                        printf("Failed to start baro calibration\n");
                    }
                    break;

                case 'v':
                case 'V':
                    print_calibration_data();
                    break;

                case 'x':
                case 'X':
                    g_menuMode = MenuMode::Main;
                    print_main_menu();
                    break;

                case 'h':
                case 'H':
                case '?':
                    print_cal_menu();
                    break;

                default:
                    break;
            }
        }

        // Flush deferred debug output to USB (safe here - UITask is low priority)
        debug_stream_flush();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// FreeRTOS Hooks
// ============================================================================

extern "C" {

void vApplicationMallocFailedHook(void) {
    ws2812_set_mode(WS2812_MODE_BLINK_FAST, WS2812_COLOR_RED);
    gpio_init(rocketchip::pins::kLedRed);
    gpio_set_dir(rocketchip::pins::kLedRed, GPIO_OUT);

    while (true) {
        ws2812_update();
        gpio_put(rocketchip::pins::kLedRed, 1);
        for (volatile int i = 0; i < 100000; i++) {}
        gpio_put(rocketchip::pins::kLedRed, 0);
        for (volatile int i = 0; i < 100000; i++) {}
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    ws2812_set_mode(WS2812_MODE_BLINK_FAST, WS2812_COLOR_ORANGE);

    gpio_init(rocketchip::pins::kLedRed);
    gpio_set_dir(rocketchip::pins::kLedRed, GPIO_OUT);

    while (true) {
        ws2812_update();
        gpio_put(rocketchip::pins::kLedRed, 1);
        for (volatile int i = 0; i < 500000; i++) {}
        gpio_put(rocketchip::pins::kLedRed, 0);
        for (volatile int i = 0; i < 200000; i++) {}
    }
}

} // extern "C"

// ============================================================================
// Main Entry Point
// ============================================================================

int main() {
    // Initialize stdio (USB CDC) - SDK handles tud_task via IRQ
    stdio_init_all();

    // Initialize NeoPixel status LED
    ws2812_status_init(pio0, rocketchip::pins::kNeoPixel);
    ws2812_set_mode(WS2812_MODE_BREATHE, WS2812_COLOR_BLUE);

    // Initialize sensor task (I2C, sensors, calibration)
    bool sensorsOk = sensor_task_init();

    if (sensorsOk) {
        // Create sensor task (pinned to Core 1)
        sensor_task_create();
        ws2812_set_mode(WS2812_MODE_SOLID, WS2812_COLOR_GREEN);
    } else {
        // Sensor init failed - show red
        ws2812_set_mode(WS2812_MODE_BLINK, WS2812_COLOR_RED);
    }

    // Create UI task
    BaseType_t result = xTaskCreate(
        UITask,
        "UI",
        rocketchip::task::kStackUi,
        nullptr,
        rocketchip::task::kPriorityUi,
        &g_uiTaskHandle
    );
    (void)result;  // Ignore for now

    // Initialize debug stream buffer
    debug_stream_init();

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true) {
        tight_loop_contents();
    }

    return 0;
}
