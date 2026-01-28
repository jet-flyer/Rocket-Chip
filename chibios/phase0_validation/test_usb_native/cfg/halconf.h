/*
    ChibiOS HAL Configuration for USB CDC Test
*/

#ifndef HALCONF_H
#define HALCONF_H

#define _CHIBIOS_HAL_CONF_
#define _CHIBIOS_HAL_CONF_VER_9_0_

#include "mcuconf.h"

/* Core HAL drivers */
#define HAL_USE_PAL                         TRUE
#define HAL_USE_USB                         TRUE
#define HAL_USE_SERIAL_USB                  TRUE

/* Unused HAL drivers */
#define HAL_USE_ADC                         FALSE
#define HAL_USE_CAN                         FALSE
#define HAL_USE_CRY                         FALSE
#define HAL_USE_DAC                         FALSE
#define HAL_USE_EFL                         FALSE
#define HAL_USE_GPT                         FALSE
#define HAL_USE_I2C                         FALSE
#define HAL_USE_I2S                         FALSE
#define HAL_USE_ICU                         FALSE
#define HAL_USE_MAC                         FALSE
#define HAL_USE_MMC_SPI                     FALSE
#define HAL_USE_PWM                         FALSE
#define HAL_USE_RTC                         FALSE
#define HAL_USE_SDC                         FALSE
#define HAL_USE_SERIAL                      FALSE
#define HAL_USE_SIO                         FALSE
#define HAL_USE_SPI                         FALSE
#define HAL_USE_TRNG                        FALSE
#define HAL_USE_UART                        FALSE
#define HAL_USE_WDG                         FALSE
#define HAL_USE_WSPI                        FALSE

#define PAL_USE_CALLBACKS                   FALSE
#define PAL_USE_WAIT                        FALSE

/*===========================================================================*/
/* USB driver related settings.                                              */
/*===========================================================================*/

/**
 * @brief   Enables synchronous APIs.
 * @note    Disabling this option saves both code and data space.
 */
#if !defined(USB_USE_WAIT) || defined(__DOXYGEN__)
#define USB_USE_WAIT                        FALSE
#endif

/*===========================================================================*/
/* Serial USB driver related settings.                                       */
/*===========================================================================*/

/**
 * @brief   Serial over USB buffers size.
 * @details Configuration parameter, the buffer size must be a multiple of
 *          the USB data endpoint maximum packet size.
 * @note    The default is 256 bytes for both the transmission and receive
 *          buffers.
 */
#if !defined(SERIAL_USB_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define SERIAL_USB_BUFFERS_SIZE             256
#endif

/**
 * @brief   Serial over USB number of buffers.
 * @note    The default is 2 buffers.
 */
#if !defined(SERIAL_USB_BUFFERS_NUMBER) || defined(__DOXYGEN__)
#define SERIAL_USB_BUFFERS_NUMBER           2
#endif

#endif /* HALCONF_H */
