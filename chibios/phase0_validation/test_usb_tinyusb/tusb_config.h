/*
 * TinyUSB Configuration for RocketChip ChibiOS
 *
 * Purpose: Configure TinyUSB for USB CDC on RP2350 with ChibiOS RTOS
 */

#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* Board/MCU Configuration                                                   */
/*===========================================================================*/

/* Use RP2040 portable driver (works for RP2350 too) */
#define CFG_TUSB_MCU                OPT_MCU_RP2040

/* Use bare-metal mode (no RTOS integration) - simpler and more reliable.
 * We just call tud_task() from the main loop. */
#define CFG_TUSB_OS                 OPT_OS_NONE

/* CPU speed for timing calculations */
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE

/* Memory alignment */
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))

/*===========================================================================*/
/* USB Device Configuration                                                  */
/*===========================================================================*/

/* Enable device stack */
#define CFG_TUD_ENABLED             1

/* Endpoint 0 size */
#define CFG_TUD_ENDPOINT0_SIZE      64

/*===========================================================================*/
/* Device Class Configuration                                                */
/*===========================================================================*/

/* CDC Class - serial communication */
#define CFG_TUD_CDC                 1
#define CFG_TUD_CDC_RX_BUFSIZE      256
#define CFG_TUD_CDC_TX_BUFSIZE      256

/* Disable other classes */
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

/*===========================================================================*/
/* RP2040/RP2350 Specific Configuration                                      */
/*===========================================================================*/

/* USB device enumeration fix for RP2040 - also helps RP2350 */
#define PICO_RP2040_USB_DEVICE_ENUMERATION_FIX  1

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H */
