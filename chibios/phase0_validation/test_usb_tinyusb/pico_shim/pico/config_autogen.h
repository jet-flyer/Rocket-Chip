/*
 * Pico SDK config_autogen shim for ChibiOS + TinyUSB build
 *
 * This file is normally CMake-generated. We provide minimal defines
 * needed for TinyUSB to compile with ChibiOS.
 */

#ifndef _PICO_CONFIG_AUTOGEN_H
#define _PICO_CONFIG_AUTOGEN_H

/* Target RP2350 (not RP2040) */
#ifndef PICO_RP2350
#define PICO_RP2350 1
#endif
#ifndef PICO_RP2040
#define PICO_RP2040 0
#endif

/* Board configuration */
#define PICO_BOARD "adafruit_feather_rp2350"

/* We're not using CMake build */
#define PICO_NO_CMAKE_CONFIG 1

/* Default clock - 150 MHz for RP2350 */
#ifndef PICO_DEFAULT_CLK_KHZ
#define PICO_DEFAULT_CLK_KHZ 150000
#endif

/* USB settings */
#define PICO_RP2040_USB_DEVICE_ENUMERATION_FIX 1

/* Include board-specific config */
#include "boards/adafruit_feather_rp2350.h"

#endif
