/*
 * RocketChip ChibiOS Phase 0 - USB CDC Test
 *
 * Purpose: Validate TinyUSB integration with ChibiOS on RP2350.
 * Target:  Adafruit Feather RP2350 HSTX
 *
 * This test provides USB CDC (serial) for debugging alongside ChibiOS.
 * LED blinks to show system is running, USB CDC provides console output.
 */

#include "ch.h"
#include "hal.h"
#include "tusb.h"

/*===========================================================================*/
/* Configuration Constants                                                   */
/*===========================================================================*/

/* GPIO pin definitions - Adafruit Feather RP2350 HSTX */
static const uint32_t kLedPin = 7U;  /* Red LED */

/* Timing constants */
static const uint32_t kBlinkIntervalMs = 500U;
static const uint32_t kUsbPollIntervalMs = 1U;

/*===========================================================================*/
/* USB CDC Output                                                            */
/*===========================================================================*/

/**
 * @brief Write string to USB CDC.
 */
static void cdcPuts(const char* str) {
    if (tud_cdc_connected()) {
        while (*str) {
            tud_cdc_write_char(*str++);
        }
        tud_cdc_write_flush();
    }
}

/**
 * @brief Write formatted number to USB CDC.
 */
static void cdcPutNum(uint32_t num) {
    char buf[12];
    int i = 0;

    if (num == 0) {
        buf[i++] = '0';
    } else {
        char tmp[12];
        int j = 0;
        while (num > 0) {
            tmp[j++] = (char)('0' + (num % 10));
            num /= 10;
        }
        while (j > 0) {
            buf[i++] = tmp[--j];
        }
    }
    buf[i] = '\0';
    cdcPuts(buf);
}

/*===========================================================================*/
/* TinyUSB Callbacks                                                         */
/*===========================================================================*/

/* Invoked when device is mounted */
void tud_mount_cb(void) {
    /* Could change LED pattern here */
}

/* Invoked when device is unmounted */
void tud_umount_cb(void) {
    /* Could change LED pattern here */
}

/* Invoked when CDC line state changes */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;

    if (dtr) {
        /* Terminal connected */
        cdcPuts("\r\n");
        cdcPuts("=====================================\r\n");
        cdcPuts("RocketChip ChibiOS USB CDC Test\r\n");
        cdcPuts("Target: Adafruit Feather RP2350 HSTX\r\n");
        cdcPuts("=====================================\r\n");
        cdcPuts("\r\n");
    }
}

/* Invoked when CDC data is received */
void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    /* Echo received characters back */
    while (tud_cdc_available()) {
        char c = (char)tud_cdc_read_char();
        tud_cdc_write_char(c);
    }
    tud_cdc_write_flush();
}

/*===========================================================================*/
/* USB Thread                                                                */
/*===========================================================================*/

static THD_WORKING_AREA(waUsbThread, 512);
static THD_FUNCTION(UsbThread, arg) {
    (void)arg;
    chRegSetThreadName("usb");

    while (true) {
        tud_task();  /* TinyUSB device task */
        chThdSleepMilliseconds(kUsbPollIntervalMs);
    }
}

/*===========================================================================*/
/* Application Entry Point                                                   */
/*===========================================================================*/

int main(void) {
    /* Initialize ChibiOS HAL and kernel */
    halInit();
    chSysInit();

    /* Configure LED pin */
    palSetLineMode(kLedPin, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);

    /* Initialize TinyUSB */
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
    };
    tusb_init(0, &dev_init);

    /* Create USB thread */
    chThdCreateStatic(waUsbThread, sizeof(waUsbThread),
                      NORMALPRIO + 1, UsbThread, NULL);

    /* Main loop - blink LED and output status */
    uint32_t count = 0;
    while (true) {
        /* LED ON */
        palSetLine(kLedPin);

        /* Output status every 10 blinks */
        if ((count % 10) == 0) {
            cdcPuts("Blink count: ");
            cdcPutNum(count);
            cdcPuts("\r\n");
        }

        chThdSleepMilliseconds(kBlinkIntervalMs);

        /* LED OFF */
        palClearLine(kLedPin);
        chThdSleepMilliseconds(kBlinkIntervalMs);

        count++;
    }

    return 0;
}
