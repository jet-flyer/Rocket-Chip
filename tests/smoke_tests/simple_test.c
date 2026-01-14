// Simple LED blink test without FreeRTOS
#include <stdio.h>
#include "pico/stdlib.h"

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n\n========================================\n");
    printf("Simple LED Blink Test\n");
    printf("========================================\n\n");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    printf("LED Pin: %d\n", PICO_DEFAULT_LED_PIN);
    printf("Starting blink...\n\n");

    while (1) {
        printf("LED ON\n");
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);

        printf("LED OFF\n");
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }

    return 0;
}
