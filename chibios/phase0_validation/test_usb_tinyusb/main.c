/*
 * RocketChip ChibiOS Phase 0 - USB CDC TinyUSB Test
 *
 * Purpose: Validate TinyUSB USB CDC integration with ChibiOS on RP2350.
 * Target:  Adafruit Feather RP2350 HSTX
 *
 * This test uses TinyUSB for USB CDC, bypassing the broken ChibiOS USB HAL.
 * Debug output goes to UART0 via the debug probe.
 */

#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "tusb.h"

/*===========================================================================*/
/* Configuration Constants                                                   */
/*===========================================================================*/

static const uint32_t kLedPin = 7U;  /* Red LED on Feather RP2350 */

/* Debug probe UART pins */
static const uint32_t kUartTxPin = 0U;  /* GPIO0 = UART0 TX */
static const uint32_t kUartRxPin = 1U;  /* GPIO1 = UART0 RX */

/* SIO (Simple I/O) configuration for UART0 */
static const SIOConfig sio0cfg = {
    .baud = 115200U,
    .UARTLCR_H = UART_UARTLCR_H_WLEN_8BITS | UART_UARTLCR_H_FEN,
    .UARTCR = 0U,
    .UARTIFLS = UART_UARTIFLS_RXIFLSEL_1_8F | UART_UARTIFLS_TXIFLSEL_7_8E,
    .UARTDMACR = 0U
};

/* External debug counters from pico_stubs.c */
extern volatile uint32_t usb_irq_count;
extern volatile uint32_t usb_handler_null_count;
extern volatile bool usb_irq_enabled;
extern volatile bool usb_handler_registered;

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
    return 0;
}

static msg_t put(void *ip, uint8_t b) {
    (void)ip;
    sioAsyncWrite(&SIOD0, &b, 1);
    return MSG_OK;
}

static msg_t get(void *ip) {
    (void)ip;
    return MSG_TIMEOUT;
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
/* LED Blinker Thread                                                        */
/*===========================================================================*/

static THD_WORKING_AREA(waBlinkThread, 256);
static THD_FUNCTION(BlinkThread, arg) {
    (void)arg;
    chRegSetThreadName("blinker");

    while (true) {
        /* Blink fast when USB connected, slow otherwise */
        systime_t interval = tud_connected() ? TIME_MS2I(250) : TIME_MS2I(500);

        palSetLine(kLedPin);
        chThdSleep(interval);
        palClearLine(kLedPin);
        chThdSleep(interval);
    }
}

/*===========================================================================*/
/* TinyUSB Task Thread                                                       */
/*===========================================================================*/

static THD_WORKING_AREA(waTinyUSBThread, 1024);
static THD_FUNCTION(TinyUSBThread, arg) {
    (void)arg;
    chRegSetThreadName("tinyusb");

    while (true) {
        /* Process TinyUSB events */
        tud_task();
        chThdSleepMicroseconds(100);
    }
}

/*===========================================================================*/
/* CDC Callbacks                                                             */
/*===========================================================================*/

/* Invoked when CDC line state changes */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;

    if (dtr) {
        /* Terminal connected - send welcome message */
        tud_cdc_write_str("\r\n=====================================\r\n");
        tud_cdc_write_str("RocketChip ChibiOS TinyUSB CDC Test\r\n");
        tud_cdc_write_str("Target: Adafruit Feather RP2350 HSTX\r\n");
        tud_cdc_write_str("=====================================\r\n\r\n");
        tud_cdc_write_flush();
    }
}

/* Invoked when CDC receives data */
void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    /* Echo received data */
    while (tud_cdc_available()) {
        uint8_t buf[64];
        uint32_t count = tud_cdc_read(buf, sizeof(buf));
        tud_cdc_write(buf, count);
        tud_cdc_write_flush();
    }
}

/*===========================================================================*/
/* Application Entry Point                                                   */
/*===========================================================================*/

int main(void) {
    /* Initialize ChibiOS HAL and kernel */
    halInit();
    chSysInit();

    /* Configure LED pin first for early visual feedback */
    palSetLineMode(kLedPin, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);
    palSetLine(kLedPin);  /* LED ON = booting */

    /* Configure debug UART (to debug probe) */
    palSetLineMode(kUartTxPin, PAL_MODE_ALTERNATE_UART);
    palSetLineMode(kUartRxPin, PAL_MODE_ALTERNATE_UART);
    sioStart(&SIOD0, &sio0cfg);

    chprintf(DBG, "\r\n=== TinyUSB CDC Debug v1 ===\r\n");

    /* Disable unaligned access trap (Cortex-M33 CCR register)
     * TinyUSB may perform unaligned accesses on USB buffers.
     * Without this, we get HardFault from UNALIGN trap. */
    SCB->CCR &= ~SCB_CCR_UNALIGN_TRP_Msk;
    chprintf(DBG, "Unaligned access trap disabled\r\n");

    /* ChibiOS already initializes PLL_USB to 48MHz in halInit().
     * We just need to reset the USB controller before TinyUSB init. */

    /* Reset and unreset USB controller using ChibiOS HAL functions */
    chprintf(DBG, "Resetting USB controller...\r\n");
    hal_lld_peripheral_reset(RESETS_ALLREG_USBCTRL);
    hal_lld_peripheral_unreset(RESETS_ALLREG_USBCTRL);
    chprintf(DBG, "USB controller reset complete\r\n");

    /* Initialize TinyUSB */
    chprintf(DBG, "Calling tusb_init()...\r\n");
    tusb_init();
    chprintf(DBG, "tusb_init() complete\r\n");
    chprintf(DBG, "  Handler registered: %d\r\n", usb_handler_registered);
    chprintf(DBG, "  IRQ enabled: %d\r\n", usb_irq_enabled);

    palClearLine(kLedPin);  /* LED OFF = init complete */

    /* Create blinker thread */
    chThdCreateStatic(waBlinkThread, sizeof(waBlinkThread),
                      NORMALPRIO, BlinkThread, NULL);

    /* Create TinyUSB task thread */
    chThdCreateStatic(waTinyUSBThread, sizeof(waTinyUSBThread),
                      NORMALPRIO + 1, TinyUSBThread, NULL);

    chprintf(DBG, "Threads created, entering main loop\r\n");

    /* Main loop - output status over UART and CDC */
    uint32_t count = 0;
    char buf[128];

    while (true) {
        chThdSleepMilliseconds(1000);

        /* Debug output to UART (debug probe) */
        chprintf(DBG, "[%lu] IRQs:%lu null:%lu conn:%d mounted:%d\r\n",
                 count, usb_irq_count, usb_handler_null_count,
                 tud_connected(), tud_mounted());

        /* Also read USB registers for debugging every 5 seconds */
        if (count % 5 == 0) {
            volatile uint32_t *usb_base = (volatile uint32_t *)0x50110000;
            chprintf(DBG, "  SIE_STATUS: 0x%08lx  SIE_CTRL: 0x%08lx  INTS: 0x%08lx\r\n",
                     usb_base[0x50/4], usb_base[0x4c/4], usb_base[0x98/4]);
        }

        /* CDC output if connected */
        if (tud_cdc_connected()) {
            int len = chsnprintf(buf, sizeof(buf), "Uptime: %lu seconds\r\n", count);
            tud_cdc_write((uint8_t*)buf, len);
            tud_cdc_write_flush();
        }
        count++;
    }

    return 0;
}
