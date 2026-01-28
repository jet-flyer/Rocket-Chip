/*
 * RocketChip ChibiOS Phase 0 - USB CDC Native Test
 *
 * Purpose: Validate ChibiOS native USB CDC on RP2350.
 * Target:  Adafruit Feather RP2350 HSTX
 *
 * This test provides USB CDC (serial) output using ChibiOS's native
 * USB HAL driver, not TinyUSB.
 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"

/*===========================================================================*/
/* Configuration Constants                                                   */
/*===========================================================================*/

static const uint32_t kLedPin = 7U;  /* Red LED on Feather RP2350 */

/*===========================================================================*/
/* LED Blinker Thread                                                        */
/*===========================================================================*/

static THD_WORKING_AREA(waBlinkThread, 256);
static THD_FUNCTION(BlinkThread, arg) {
    (void)arg;
    chRegSetThreadName("blinker");

    while (true) {
        /* Blink fast when USB active, slow otherwise */
        systime_t interval = (USBD1.state == USB_ACTIVE) ? 250 : 500;

        palSetLine(kLedPin);
        chThdSleepMilliseconds(interval);
        palClearLine(kLedPin);
        chThdSleepMilliseconds(interval);
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

    /* Initialize Serial-over-USB CDC driver */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /* Activate USB driver with disconnect/reconnect sequence */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    /* Create blinker thread */
    chThdCreateStatic(waBlinkThread, sizeof(waBlinkThread),
                      NORMALPRIO, BlinkThread, NULL);

    /* Main loop - output status over USB CDC */
    uint32_t count = 0;
    while (true) {
        if (SDU1.config->usbp->state == USB_ACTIVE) {
            chprintf((BaseSequentialStream *)&SDU1,
                     "\r\n=====================================\r\n");
            chprintf((BaseSequentialStream *)&SDU1,
                     "RocketChip ChibiOS USB CDC Test\r\n");
            chprintf((BaseSequentialStream *)&SDU1,
                     "Target: Adafruit Feather RP2350 HSTX\r\n");
            chprintf((BaseSequentialStream *)&SDU1,
                     "=====================================\r\n\r\n");

            /* Output periodic status */
            while (SDU1.config->usbp->state == USB_ACTIVE) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "Blink count: %lu\r\n", count);
                count++;
                chThdSleepMilliseconds(1000);
            }
        }
        chThdSleepMilliseconds(500);
    }

    return 0;
}
