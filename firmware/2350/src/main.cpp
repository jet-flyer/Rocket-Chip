#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

void blink_task(void *pvParameters) {
    const uint led_pin = PICO_DEFAULT_LED_PIN;

    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    while (true) {
        gpio_put(led_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(led_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void print_task(void *pvParameters) {
    while (true) {
        printf("Core %d\n", portGET_CORE_ID());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main() {
    stdio_init_all();

    xTaskCreate(blink_task, "Blink", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    TaskHandle_t printHandle;
    xTaskCreate(print_task, "Print", configMINIMAL_STACK_SIZE, NULL, 1, &printHandle);
    vTaskCoreAffinitySet(printHandle, (1 << 1));

    vTaskStartScheduler();

    while (true) {}
    return 0;
}