# CLI Restore Reference

This file contains CLI code recovered from Claude's context after `git checkout src/main.cpp` reverted uncommitted changes.

## Overview

The CLI system had:
1. Menu modes (Main, Calibration)
2. CLITask that waited for sensor init before showing prompts
3. MAVLink routing (detect 0xFE/0xFD start markers)
4. Calibration commands for all sensors
5. Debug output for 6-pos accel cal troubleshooting

## Key Bug Found

**6-pos accel cal immediate cancel**: Terminal sends CR+LF for Enter, causing double input.
**Fix**: Drain buffer after each confirmation:
```cpp
while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
```

---

## Code Fragments from Context

### 1. Includes and Menu State

```cpp
#include <cstdio>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "services/SensorTask.h"
#include "AP_InternalError/AP_InternalError.h"
#include <GCS_MAVLink/GCS.h>

using namespace rocketchip::services;

// Menu State Machine
enum class MenuMode {
    Main,
    Calibration
};

static MenuMode g_menuMode = MenuMode::Main;
```

### 2. Configuration Constants

```cpp
static constexpr uint8_t kLedPin = PICO_DEFAULT_LED_PIN;

static constexpr uint32_t kCliTaskPriority = 1;
static constexpr uint32_t kCliStackSize = configMINIMAL_STACK_SIZE * 4;

// MAVLink start markers
constexpr uint8_t kMavlinkV1Start = 0xFE;
constexpr uint8_t kMavlinkV2Start = 0xFD;
```

### 3. Print Helpers

```cpp
static void printSystemStatus() {
    printf("\n========================================\n");
    printf("  RocketChip System Status\n");
    printf("========================================\n");
    printf("  Target: Adafruit Feather RP2350\n");
    printf("  Phase: 2 (Sensors)\n");
    printf("  Heap free: %u bytes\n", xPortGetFreeHeapSize());
    printf("  Uptime: %llu ms\n", to_ms_since_boot(get_absolute_time()));
    printf("----------------------------------------\n");
    printf("Commands:\n");
    printf("  h - This help\n");
    printf("  s - Sensor status\n");
    printf("  c - Calibration menu\n");
    printf("========================================\n\n");
}

static void printCalibrationMenu() {
    printf("\n========================================\n");
    printf("  Calibration Menu\n");
    printf("========================================\n");
    printf("  w - Full calibration wizard\n");
    printf("  l - Level calibration (quick)\n");
    printf("  a - 6-position accel calibration\n");
    printf("  m - Compass calibration\n");
    printf("  b - Barometer calibration\n");
    printf("  x - Return to main menu\n");
    printf("========================================\n\n");
}
```

### 4. Calibration Wizard

```cpp
static void runCalibrationWizard() {
    printf("\n=== Calibration Wizard ===\n");
    printf("This wizard will guide you through all calibrations.\n\n");

    // Step 1: Level cal
    printf("Step 1: Level Calibration\n");
    printf("Place device FLAT and STILL on a level surface.\n");
    printf("Press ENTER when ready, or 'x' to skip...\n");

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == '\r' || c == '\n') {
            printf("Calibrating...");
            fflush(stdout);
            MAV_RESULT result = SensorTask_SimpleAccelCal();
            if (result == MAV_RESULT_ACCEPTED) {
                printf(" OK!\n\n");
            } else {
                printf(" FAILED (%d)\n\n", result);
            }
            break;
        } else if (c == 'x' || c == 'X') {
            printf("Skipped.\n\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Step 2: Compass cal
    printf("Step 2: Compass Calibration\n");
    printf("Rotate device slowly in all orientations.\n");
    printf("Press ENTER to start, or 'x' to skip...\n");

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == '\r' || c == '\n') {
            if (SensorTask_StartCompassCal()) {
                printf("Rotate device... press 'x' when done.\n");
                while (SensorTask_IsCalibrating()) {
                    int cmd = getchar_timeout_us(0);
                    if (cmd == 'x' || cmd == 'X') {
                        SensorTask_CancelCalibration();
                        printf("Compass cal complete.\n\n");
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
            break;
        } else if (c == 'x' || c == 'X') {
            printf("Skipped.\n\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Step 3: Baro cal
    printf("Step 3: Barometer Calibration\n");
    printf("Keep device still. Press ENTER to start, or 'x' to skip...\n");

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == '\r' || c == '\n') {
            printf("Calibrating...");
            fflush(stdout);
            if (SensorTask_CalibrateBaro()) {
                printf(" OK!\n\n");
            } else {
                printf(" FAILED\n\n");
            }
            break;
        } else if (c == 'x' || c == 'X') {
            printf("Skipped.\n\n");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    printf("=== Wizard Complete ===\n\n");
}
```

### 5. CLITask (Main Function)

```cpp
static void CLITask(void* pvParameters) {
    (void)pvParameters;

    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);

    // Wait for sensor init to complete before prompting
    // This prevents interleaved output between init debug messages and CLI prompts
    while (!SensorTask_IsInitComplete()) {
        // Fast blink while waiting for init
        gpio_put(kLedPin, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_put(kLedPin, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Brief pause after init output
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("\n");  // Blank line after init messages

    // Wait for keypress to show main UI
    uint32_t promptCount = 0;
    while (true) {
        // Slow blink while waiting for user input
        gpio_put(kLedPin, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_put(kLedPin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Print prompt every 2 seconds
        if (promptCount % 10 == 0) {
            printf("\rPress any key to start...   ");
        }
        promptCount++;

        // Check for keypress
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            break;
        }
    }

    // Print startup banner
    printf("\n\n");
    printf("========================================\n");
    printf("  RocketChip Production Firmware\n");
    printf("  Target: Adafruit Feather RP2350\n");
    printf("  Phase: 2 (Sensors)\n");
    printf("  Press 'h' for help\n");
    printf("========================================\n\n");

    // Print initial sensor status
    SensorTask_PrintStatus();

    // Main UI loop
    bool ledState = false;
    uint32_t ledCounter = 0;

    // MAVLink start markers
    constexpr uint8_t kMavlinkV1Start = 0xFE;
    constexpr uint8_t kMavlinkV2Start = 0xFD;

    while (true) {
        // Toggle LED every 500ms (1Hz blink = system running)
        // Loop runs at 50ms, so toggle every 10 iterations
        if (++ledCounter >= 10) {
            ledCounter = 0;
            ledState = !ledState;
            gpio_put(kLedPin, ledState);
        }

        // Check for input (non-blocking) - route to MAVLink or CLI
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            uint8_t byte = static_cast<uint8_t>(c);

            // Check for MAVLink start marker
            if (byte == kMavlinkV1Start || byte == kMavlinkV2Start) {
                // MAVLink message starting - feed to GCS parser
                GCS::get_singleton().parse_byte(byte);

                // Continue reading until message complete or timeout
                for (int i = 0; i < 300; i++) {
                    int next = getchar_timeout_us(1000);  // 1ms timeout
                    if (next == PICO_ERROR_TIMEOUT) {
                        break;
                    }
                    if (GCS::get_singleton().parse_byte(static_cast<uint8_t>(next))) {
                        break;  // Complete message received
                    }
                }
            } else {
                // CLI command - handle based on menu mode
                if (g_menuMode == MenuMode::Main) {
                    // Main menu commands
                    switch (c) {
                        case 'h':
                        case 'H':
                            printSystemStatus();
                            break;
                        case 's':
                        case 'S':
                            SensorTask_PrintStatus();
                            break;
                        case 'c':
                        case 'C':
                            g_menuMode = MenuMode::Calibration;
                            printCalibrationMenu();
                            break;
                        case '\r':
                        case '\n':
                            printf("\n");
                            break;
                    }
                } else if (g_menuMode == MenuMode::Calibration) {
                    // Calibration menu commands
                    switch (c) {
                        case 'w':
                        case 'W':
                            runCalibrationWizard();
                            g_menuMode = MenuMode::Main;
                            break;
                        case 'l':
                        case 'L':
                            printf("\nLevel Cal - keep device FLAT and STILL (~10 sec)...");
                            fflush(stdout);
                            {
                                MAV_RESULT result = SensorTask_SimpleAccelCal();
                                if (result == MAV_RESULT_ACCEPTED) {
                                    printf(" OK!\n");
                                } else {
                                    printf(" FAILED (%d)\n", result);
                                }
                            }
                            g_menuMode = MenuMode::Main;
                            break;
                        case 'a':
                        case 'A':
                            printf("\nStarting 6-position accel calibration...\n");
                            printf("Press ENTER to confirm each position, 'x' to cancel.\n");
                            if (SensorTask_StartAccelCal()) {
                                // Stay in calibration mode until complete
                                while (SensorTask_IsCalibrating()) {
                                    int cmd = getchar_timeout_us(0);
                                    // Debug: show any received character
                                    if (cmd != PICO_ERROR_TIMEOUT) {
                                        printf("[DBG] char=%d (0x%02X '%c')\n", cmd, cmd, (cmd >= 32 && cmd < 127) ? cmd : '?');
                                    }
                                    if (cmd == 'x' || cmd == 'X' || cmd == 27) {
                                        SensorTask_CancelCalibration();
                                        printf("Cancelled.\n");
                                        break;
                                    } else if (cmd == '\r' || cmd == '\n' || cmd == ' ') {
                                        // Position confirmation
                                        // CRITICAL: Drain buffer to handle CR+LF from terminal
                                        while (getchar_timeout_us(1000) != PICO_ERROR_TIMEOUT) {}
                                        // TODO: wire up position confirmation
                                    }
                                    // Keep LED blinking during calibration
                                    if (++ledCounter >= 5) {
                                        ledCounter = 0;
                                        ledState = !ledState;
                                        gpio_put(kLedPin, ledState);
                                    }
                                    vTaskDelay(pdMS_TO_TICKS(50));
                                }
                            }
                            g_menuMode = MenuMode::Main;
                            break;
                        case 'm':
                        case 'M':
                            printf("\nStarting compass calibration...\n");
                            printf("Rotate device slowly. Press 'x' when done.\n");
                            if (SensorTask_StartCompassCal()) {
                                while (SensorTask_IsCalibrating()) {
                                    int cmd = getchar_timeout_us(0);
                                    if (cmd == 'x' || cmd == 'X' || cmd == 27) {
                                        SensorTask_CancelCalibration();
                                        printf("Cancelled.\n");
                                        break;
                                    }
                                    if (++ledCounter >= 5) {
                                        ledCounter = 0;
                                        ledState = !ledState;
                                        gpio_put(kLedPin, ledState);
                                    }
                                    vTaskDelay(pdMS_TO_TICKS(50));
                                }
                            }
                            g_menuMode = MenuMode::Main;
                            break;
                        case 'b':
                        case 'B':
                            printf("\nCalibrating barometer...\n");
                            SensorTask_CalibrateBaro();
                            g_menuMode = MenuMode::Main;
                            break;
                        case 'x':
                        case 'X':
                        case 27:  // ESC
                            if (SensorTask_IsCalibrating()) {
                                SensorTask_CancelCalibration();
                                printf("\nCalibration cancelled.\n");
                            }
                            printf("Returning to main menu.\n");
                            g_menuMode = MenuMode::Main;
                            break;
                        case 'h':
                        case 'H':
                            printCalibrationMenu();
                            break;
                        case '\r':
                        case '\n':
                            // During calibration, confirm position
                            if (SensorTask_IsCalibrating()) {
                                printf("Position confirmed.\n");
                            }
                            break;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### 6. Task Creation in main()

Replace the UITask creation with CLITask:

```cpp
// Create CLI task (handles keypress wait, menus, and calibration)
TaskHandle_t cliHandle;
BaseType_t result = xTaskCreate(
    CLITask,
    "CLI",
    kCliStackSize,
    nullptr,
    kCliTaskPriority,
    &cliHandle
);

if (result == pdPASS) {
    vTaskCoreAffinitySet(cliHandle, (1 << 0));  // Pin to Core 0
}
```

---

## Available SensorTask API Functions

From `src/services/SensorTask.h`:

```cpp
bool SensorTask_Init();
bool SensorTask_Create();
bool SensorTask_GetData(SensorData& data);
void SensorTask_PrintStatus();
bool SensorTask_StartAccelCal();
MAV_RESULT SensorTask_SimpleAccelCal();
bool SensorTask_AccelCalConfirmPosition(int position);
bool SensorTask_StartCompassCal();
bool SensorTask_CalibrateBaro();
bool SensorTask_IsCalibrating();
void SensorTask_CancelCalibration();
bool SensorTask_IsInitComplete();
```

---

## Notes

1. **SensorTask_IsInitComplete()** - Added in SensorTask.cpp, prevents CLI prompt from appearing before sensor init messages complete

2. **GCS MAVLink parsing** - The GCS.cpp was updated to NOT print CLI messages from `send_accelcal_vehicle_position()` since ArduPilot's `_printf` macro already sends user prompts through `send_text()` -> `gcs_send_statustext()`

3. **Level cal timing** - Takes ~20 seconds, which is longer than expected. Added diagnostics to ArduPilot's `simple_accel_cal()` but output didn't appear (DEV_PRINTF may need verification)

4. **Stack size** - CLI task needs at least `configMINIMAL_STACK_SIZE * 4` due to nested function calls and printf buffers
