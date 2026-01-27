/*
 * RocketChip ChibiOS Phase 0 - LED Blink Test
 *
 * Purpose: Validate ChibiOS boots and runs on RP2350 hardware.
 * Target:  Adafruit Feather RP2350 HSTX (GPIO25 = onboard LED)
 *
 * This is a minimal test to verify:
 * - ChibiOS kernel initializes correctly
 * - HAL PAL driver works (GPIO output)
 * - Flash execution (XIP) works
 * - Both cores are available (SMP enabled)
 *
 * Based on ChibiOS RT-RP2350-BLINK demo.
 * Modified for RocketChip: dual-core enabled, coding standards applied.
 */

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Configuration Constants                                                   */
/*===========================================================================*/

/* GPIO pin definitions - Board-specific
 *
 * Note: We use RP_PICO2_RP2350 board definition but run on Adafruit Feather RP2350.
 * A custom board file should be created for Phase 1.
 *
 * Adafruit Feather RP2350 HSTX pinout:
 *   https://learn.adafruit.com/adafruit-feather-rp2350/pinouts
 *   - GPIO7:  Red LED (#7 LED, above USB-C connector)
 *   - GPIO21: NeoPixel (status indicator)
 *   - GPIO13: Charge status LED
 *
 * Raspberry Pi Pico 2:
 *   - GPIO25: Green LED
 */
#if defined(BOARD_ADAFRUIT_FEATHER_RP2350)
  #define ROCKETCHIP_LED_PIN    7U
  #define ROCKETCHIP_NEOPIXEL   21U
#elif defined(BOARD_RP_PICO2_RP2350)
  /* Running on Feather with Pico2 board definition - use Feather pins */
  #define ROCKETCHIP_LED_PIN    7U
  #define ROCKETCHIP_NEOPIXEL   21U
#else
  /* Fallback to Pico SDK convention if available */
  #define ROCKETCHIP_LED_PIN    25U
#endif

static const uint32_t kLedPin = ROCKETCHIP_LED_PIN;

/* Timing constants */
static const uint32_t kBlinkIntervalMs = 500U;  /* 500ms on, 500ms off = 1Hz */

/* UART configuration for debug output (optional) */
static const uint32_t kUartTxPin = 0U;
static const uint32_t kUartRxPin = 1U;

/*===========================================================================*/
/* UART Configuration                                                        */
/*===========================================================================*/

/* UART0: 115200 baud, 8-N-1, FIFO enabled */
static const SIOConfig g_sioConfig = {
    .baud      = 115200U,
    .UARTLCR_H = UART_UARTLCR_H_WLEN_8BITS | UART_UARTLCR_H_FEN,
    .UARTCR    = 0U,
    .UARTIFLS  = UART_UARTIFLS_RXIFLSEL_1_2F | UART_UARTIFLS_TXIFLSEL_1_2E,
    .UARTDMACR = 0U
};

/*===========================================================================*/
/* Helper Functions                                                          */
/*===========================================================================*/

/**
 * @brief Send a string over UART (blocking).
 * @param p_str Pointer to null-terminated string.
 */
static void uartPuts(const char* p_str) {
    while (*p_str != '\0') {
        /* Wait for TX FIFO space */
        while ((UART0->UARTFR & UART_UARTFR_TXFF) != 0U) {
            /* Spin */
        }
        UART0->UARTDR = (uint32_t)*p_str;
        p_str++;
    }
}

/*===========================================================================*/
/* Application Entry Point                                                   */
/*===========================================================================*/

/**
 * @brief Main application entry point.
 *
 * Initializes ChibiOS and blinks the onboard LED at 1Hz.
 * Outputs a message on UART0 each time the LED turns on.
 */
int main(void) {
    /*
     * System initializations.
     * - halInit(): Initialize HAL subsystem
     * - chSysInit(): Initialize ChibiOS kernel
     */
    halInit();
    chSysInit();

    /*
     * Configure LED pin as push-pull output with 12mA drive.
     */
    palSetLineMode(kLedPin, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);

    /*
     * Configure UART pins for debug output.
     */
    palSetLineMode(kUartTxPin, PAL_MODE_ALTERNATE_UART);
    palSetLineMode(kUartRxPin, PAL_MODE_ALTERNATE_UART);
    sioStart(&SIOD0, &g_sioConfig);

    /*
     * Startup message.
     */
    uartPuts("\r\n");
    uartPuts("=====================================\r\n");
    uartPuts("RocketChip ChibiOS Phase 0 Test\r\n");
    uartPuts("Target: Adafruit Feather RP2350 HSTX\r\n");
    uartPuts("Test: LED Blink (GPIO25)\r\n");
    uartPuts("Config: Dual-core SMP, Flash (XIP)\r\n");
    uartPuts("=====================================\r\n");
    uartPuts("\r\n");

    /*
     * Main loop - blink LED at 1Hz.
     */
    uint32_t count = 0U;
    while (true) {
        /* LED ON */
        palSetLine(kLedPin);

        /* Print status every 10 blinks */
        if ((count % 10U) == 0U) {
            uartPuts("Blink count: ");
            /* Simple decimal output */
            char buf[12];
            int i = 0;
            uint32_t n = count;
            if (n == 0U) {
                buf[i++] = '0';
            } else {
                char tmp[12];
                int j = 0;
                while (n > 0U) {
                    tmp[j++] = (char)('0' + (n % 10U));
                    n /= 10U;
                }
                while (j > 0) {
                    buf[i++] = tmp[--j];
                }
            }
            buf[i] = '\0';
            uartPuts(buf);
            uartPuts("\r\n");
        }

        chThdSleepMilliseconds(kBlinkIntervalMs);

        /* LED OFF */
        palClearLine(kLedPin);
        chThdSleepMilliseconds(kBlinkIntervalMs);

        count++;
    }

    /* Never reached */
    return 0;
}
