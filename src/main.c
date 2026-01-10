/*
 * RocketChip FreeRTOS SMP Validation for RP2350
 *
 * This validation test exercises:
 * - Dual-core SMP scheduling with core affinity
 * - Inter-task communication via queues
 * - Priority-based preemption
 * - USB serial output
 * - GPIO LED control
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Configuration */
#define LED_PIN PICO_DEFAULT_LED_PIN
#define SENSOR_SAMPLE_RATE_HZ 1000
#define SENSOR_TASK_PRIORITY 5
#define LOGGER_TASK_PRIORITY 3
#define UI_TASK_PRIORITY 1
#define QUEUE_LENGTH 32
#define SENSOR_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define LOGGER_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define UI_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)

/* Task state */
typedef struct {
    uint32_t sensor_samples;
    uint32_t logger_processed;
    uint32_t queue_depth;
    uint32_t free_heap;
} system_status_t;

static system_status_t g_status = {0};
static QueueHandle_t sensor_queue = NULL;

/*
 * Sensor Task - Simulates 1kHz sensor sampling
 * Runs on Core 1 with high priority
 */
static void sensor_task(void *pvParameters) {
    (void)pvParameters;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1ms = 1kHz

    uint32_t sample = 0;

    while (1) {
        // Simulate sensor reading (just increment counter)
        sample++;

        // Send to logger queue (non-blocking)
        if (xQueueSend(sensor_queue, &sample, 0) == pdTRUE) {
            g_status.sensor_samples++;
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

/*
 * Logger Task - Processes sensor samples from queue
 * Runs on Core 1 with medium priority
 */
static void logger_task(void *pvParameters) {
    (void)pvParameters;

    uint32_t sample;

    while (1) {
        // Wait for sensor data (block indefinitely)
        if (xQueueReceive(sensor_queue, &sample, portMAX_DELAY) == pdTRUE) {
            g_status.logger_processed++;

            // Simulate processing work (could be I2C write, flash storage, etc.)
            // For now just update queue depth
            g_status.queue_depth = uxQueueMessagesWaiting(sensor_queue);
        }
    }
}

/*
 * UI Task - LED blink and status reporting
 * Runs on Core 0 with low priority
 */
static void ui_task(void *pvParameters) {
    (void)pvParameters;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    bool led_state = false;
    uint32_t cycle_count = 0;

    while (1) {
        // Toggle LED every 500ms (2Hz blink rate)
        led_state = !led_state;
        gpio_put(LED_PIN, led_state);

        cycle_count++;

        // Report status every 5 seconds (10 cycles at 500ms each)
        if (cycle_count >= 10) {
            cycle_count = 0;

            g_status.free_heap = xPortGetFreeHeapSize();

            printf("\n=== System Status ===\n");
            printf("Sensor samples:   %lu\n", g_status.sensor_samples);
            printf("Logger processed: %lu\n", g_status.logger_processed);
            printf("Queue depth:      %lu / %d\n", g_status.queue_depth, QUEUE_LENGTH);
            printf("Free heap:        %lu bytes\n", g_status.free_heap);
            printf("====================\n\n");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(void) {
    // Initialize stdio (USB serial)
    stdio_init_all();

    // Wait for USB enumeration (~2 seconds)
    sleep_ms(2000);

    printf("\n\n");
    printf("========================================\n");
    printf("RocketChip FreeRTOS SMP Validation\n");
    printf("Target: RP2350 (Pico 2)\n");
    printf("FreeRTOS: Dual-core SMP\n");
    printf("========================================\n\n");

    // Create queue for sensor->logger communication
    sensor_queue = xQueueCreate(QUEUE_LENGTH, sizeof(uint32_t));
    if (sensor_queue == NULL) {
        printf("ERROR: Failed to create sensor queue\n");
        return 1;
    }

    printf("Starting tasks...\n");

    // Create sensor task (Core 1, priority 5)
    TaskHandle_t sensor_handle;
    BaseType_t result = xTaskCreate(
        sensor_task,
        "Sensor",
        SENSOR_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        &sensor_handle
    );

    if (result != pdPASS) {
        printf("ERROR: Failed to create sensor task\n");
        return 1;
    }
    vTaskCoreAffinitySet(sensor_handle, (1 << 1)); // Pin to Core 1

    // Create logger task (Core 1, priority 3)
    TaskHandle_t logger_handle;
    result = xTaskCreate(
        logger_task,
        "Logger",
        LOGGER_STACK_SIZE,
        NULL,
        LOGGER_TASK_PRIORITY,
        &logger_handle
    );

    if (result != pdPASS) {
        printf("ERROR: Failed to create logger task\n");
        return 1;
    }
    vTaskCoreAffinitySet(logger_handle, (1 << 1)); // Pin to Core 1

    // Create UI task (Core 0, priority 1)
    TaskHandle_t ui_handle;
    result = xTaskCreate(
        ui_task,
        "UI",
        UI_STACK_SIZE,
        NULL,
        UI_TASK_PRIORITY,
        &ui_handle
    );

    if (result != pdPASS) {
        printf("ERROR: Failed to create UI task\n");
        return 1;
    }
    vTaskCoreAffinitySet(ui_handle, (1 << 0)); // Pin to Core 0

    printf("Tasks created. Starting scheduler...\n\n");

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    printf("ERROR: Scheduler returned!\n");

    while (1) {
        tight_loop_contents();
    }

    return 0;
}
