/*
 * TinyUSB Configuration for ChibiOS + RP2350
 *
 * Minimal CDC (serial) device configuration.
 */

#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* Common Configuration                                                      */
/*===========================================================================*/

/* Use RP2040 USB driver (works on RP2350) */
#define CFG_TUSB_MCU                OPT_MCU_RP2040

/* Device mode */
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE

/* Use ChibiOS for memory allocation */
#define CFG_TUSB_OS                 OPT_OS_NONE

/* Memory alignment */
#define CFG_TUSB_MEM_ALIGN          __attribute__((aligned(4)))

/*===========================================================================*/
/* Device Configuration                                                      */
/*===========================================================================*/

/* Enable device stack */
#define CFG_TUD_ENABLED             1

/* Endpoint 0 size */
#define CFG_TUD_ENDPOINT0_SIZE      64

/*===========================================================================*/
/* Class Configuration                                                       */
/*===========================================================================*/

/* CDC - Communications Device Class (serial port) */
#define CFG_TUD_CDC                 1
#define CFG_TUD_CDC_RX_BUFSIZE      256
#define CFG_TUD_CDC_TX_BUFSIZE      256
#define CFG_TUD_CDC_EP_BUFSIZE      64

/* Disable other classes */
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H */
