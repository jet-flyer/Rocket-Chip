/*
 * RocketChip ChibiOS Phase 0 - UART Debug Test
 *
 * Purpose: Validate UART output via debug probe for debugging.
 * Target:  Adafruit Feather RP2350 HSTX
 *
 * Connect Debug Probe UART pins:
 *   - Probe TX -> Feather RX (GPIO1)  [for receiving commands, optional]
 *   - Probe RX -> Feather TX (GPIO0)  [for debug output]
 *   - Probe GND -> Feather GND
 *
 * The debug probe's UART appears as a second COM port on the host.
 * Open at 115200 baud to see debug output.
 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

/*===========================================================================*/
/* Configuration Constants                                                   */
/*===========================================================================*/

/* GPIO pin definitions - Adafruit Feather RP2350 HSTX */
static const uint32_t kLedPin = 7U;  /* Red LED */

/* UART pins (directly on RP2350 UART0) */
static const uint32_t kUartTxPin = 0U;  /* TX to debug probe RX */
static const uint32_t kUartRxPin = 1U;  /* RX from debug probe TX */

/* Timing constants */
static const uint32_t kBlinkIntervalMs = 500U;

/*===========================================================================*/
/* UART Configuration                                                        */
/*===========================================================================*/

/* SIO (Simple I/O) configuration for UART0 */
static const SIOConfig sio0cfg = {
    .baud = 115200U,
    .UARTLCR_H = UART_UARTLCR_H_WLEN_8BITS | UART_UARTLCR_H_FEN,
    .UARTCR = 0U,
    .UARTIFLS = UART_UARTIFLS_RXIFLSEL_1_8F | UART_UARTIFLS_TXIFLSEL_7_8E,
    .UARTDMACR = 0U
};

/*===========================================================================*/
/* Debug Output Stream                                                       */
/*===========================================================================*/

/* Stream wrapper for chprintf */
static size_t writes(void *ip, const uint8_t *bp, size_t n) {
    (void)ip;
    return sioAsyncWrite(&SIOD0, bp, n);
}

static size_t reads(void *ip, uint8_t *bp, size_t n) {
    (void)ip;
    (void)bp;
    (void)n;
    return 0;  /* Read not implemented for debug output */
}

static msg_t put(void *ip, uint8_t b) {
    (void)ip;
    sioAsyncWrite(&SIOD0, &b, 1);
    return MSG_OK;
}

static msg_t get(void *ip) {
    (void)ip;
    return MSG_TIMEOUT;  /* Not implemented */
}

static const struct BaseSequentialStreamVMT vmt = {
    .instance_offset = 0,
    .write = writes,
    .read = reads,
    .put = put,
    .get = get
};

static BaseSequentialStream debugStream = {
    .vmt = &vmt
};

#define DBG (&debugStream)

/*===========================================================================*/
/* Application Entry Point                                                   */
/*===========================================================================*/

int main(void) {
    /* Initialize ChibiOS HAL and kernel */
    halInit();
    chSysInit();

    /* Configure LED pin */
    palSetLineMode(kLedPin, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);

    /* Configure UART pins for UART0 function (FUNCSEL 2) */
    palSetLineMode(kUartTxPin, PAL_MODE_ALTERNATE_UART);
    palSetLineMode(kUartRxPin, PAL_MODE_ALTERNATE_UART);

    /* Start UART (SIO) driver */
    sioStart(&SIOD0, &sio0cfg);

    /* Banner */
    chprintf(DBG, "\r\n");
    chprintf(DBG, "=====================================\r\n");
    chprintf(DBG, "RocketChip ChibiOS UART Debug Test\r\n");
    chprintf(DBG, "Target: Adafruit Feather RP2350 HSTX\r\n");
    chprintf(DBG, "=====================================\r\n");
    chprintf(DBG, "\r\n");
    chprintf(DBG, "ChibiOS version: %s\r\n", CH_VERSION);
    chprintf(DBG, "Kernel:          %s\r\n", CH_KERNEL_VERSION);
    chprintf(DBG, "System clock:    %lu Hz\r\n", (uint32_t)SystemCoreClock);
    chprintf(DBG, "\r\n");

    /* Main loop - blink LED and output status */
    uint32_t count = 0;
    while (true) {
        /* LED ON */
        palSetLine(kLedPin);

        /* Output status */
        chprintf(DBG, "Blink %lu: LED ON\r\n", count);
        chThdSleepMilliseconds(kBlinkIntervalMs);

        /* LED OFF */
        palClearLine(kLedPin);
        chprintf(DBG, "Blink %lu: LED OFF\r\n", count);
        chThdSleepMilliseconds(kBlinkIntervalMs);

        count++;
    }

    return 0;
}
