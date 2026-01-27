/*
 * RocketChip ChibiOS Phase 0 - I2C Bus Scan Test
 *
 * Purpose: Scan I2C bus for connected devices using DIRECT GPIO bit-bang
 * Target:  Adafruit Feather RP2350 HSTX
 *
 * NOTE: This does NOT use ChibiOS I2C HAL - it bit-bangs I2C directly
 * via GPIO to avoid issues with the unimplemented hardware driver.
 *
 * Expected devices on STEMMA QT / QWIIC port:
 *   - DPS310 barometer:    0x77
 *   - ISM330DHCX IMU:      0x6A or 0x6B
 *   - LIS3MDL magnetometer: 0x1C or 0x1E
 *
 * Connect Debug Probe UART pins:
 *   - Probe RX -> Feather TX (GPIO0)
 *   - Probe GND -> Feather GND
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

/* I2C pins (STEMMA QT / QWIIC port) */
static const ioline_t kI2cSdaLine = 2U;  /* GPIO2 = SDA */
static const ioline_t kI2cSclLine = 3U;  /* GPIO3 = SCL */

/* I2C scan range */
static const uint8_t kI2cAddrMin = 0x08U;
static const uint8_t kI2cAddrMax = 0x77U;

/*===========================================================================*/
/* UART Configuration                                                        */
/*===========================================================================*/

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
/* Direct GPIO I2C Bit-Bang Implementation                                   */
/*===========================================================================*/

/* Simple busy-wait delay for ~5us (half bit at 100kHz) */
static void i2c_delay(void) {
    /* At 150MHz, ~750 cycles = 5us */
    for (volatile int i = 0; i < 200; i++) {
        __asm__("nop");
    }
}

static void sda_high(void) {
    palSetLine(kI2cSdaLine);
}

static void sda_low(void) {
    palClearLine(kI2cSdaLine);
}

static void scl_high(void) {
    palSetLine(kI2cSclLine);
}

static void scl_low(void) {
    palClearLine(kI2cSclLine);
}

static int sda_read(void) {
    return palReadLine(kI2cSdaLine);
}

static void i2c_start(void) {
    sda_high();
    i2c_delay();
    scl_high();
    i2c_delay();
    sda_low();  /* SDA goes low while SCL is high = START */
    i2c_delay();
    scl_low();
    i2c_delay();
}

static void i2c_stop(void) {
    sda_low();
    i2c_delay();
    scl_high();
    i2c_delay();
    sda_high();  /* SDA goes high while SCL is high = STOP */
    i2c_delay();
}

/* Write a byte, return 1 if ACK received, 0 if NACK */
static int i2c_write_byte(uint8_t byte) {
    int ack;

    /* Send 8 bits, MSB first */
    for (int i = 7; i >= 0; i--) {
        if (byte & (1 << i)) {
            sda_high();
        } else {
            sda_low();
        }
        i2c_delay();
        scl_high();
        i2c_delay();
        scl_low();
        i2c_delay();
    }

    /* Release SDA for ACK */
    sda_high();
    i2c_delay();
    scl_high();
    i2c_delay();

    /* Read ACK (low = ACK, high = NACK) */
    ack = (sda_read() == 0) ? 1 : 0;

    scl_low();
    i2c_delay();

    return ack;
}

/* Probe an I2C address, return 1 if device responds, 0 otherwise */
static int i2c_probe(uint8_t addr) {
    int ack;

    i2c_start();
    ack = i2c_write_byte((addr << 1) | 0);  /* Write address with R/W=0 */
    i2c_stop();

    return ack;
}

/*===========================================================================*/
/* Device Name Lookup                                                        */
/*===========================================================================*/

static const char* get_device_name(uint8_t addr) {
    switch (addr) {
        case 0x1C: return "LIS3MDL (alt)";
        case 0x1E: return "LIS3MDL";
        case 0x68: return "ICM20948 (alt)";
        case 0x69: return "ICM20948";
        case 0x6A: return "ISM330DHCX (alt)";
        case 0x6B: return "ISM330DHCX";
        case 0x76: return "DPS310 (alt)";
        case 0x77: return "DPS310";
        case 0x3C: return "SSD1306 OLED";
        case 0x3D: return "SSD1306 OLED (alt)";
        case 0x48: return "ADS1115";
        case 0x50: return "EEPROM";
        default:   return NULL;
    }
}

/*===========================================================================*/
/* Application Entry Point                                                   */
/*===========================================================================*/

int main(void) {
    uint8_t found_count = 0;
    uint8_t found_addrs[16];

    /* Initialize ChibiOS HAL and kernel */
    halInit();
    chSysInit();

    /* Configure LED pin */
    palSetLineMode(kLedPin, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE12);
    palSetLine(kLedPin);  /* LED ON to show we started */

    /* Configure UART pins */
    palSetLineMode(kUartTxPin, PAL_MODE_ALTERNATE_UART);
    palSetLineMode(kUartRxPin, PAL_MODE_ALTERNATE_UART);

    /* Start UART driver */
    sioStart(&SIOD0, &sio0cfg);

    /* Configure I2C pins as push-pull with pull-ups and input enabled */
    palSetLineMode(kI2cSdaLine, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_PUE | PAL_RP_PAD_IE);
    palSetLineMode(kI2cSclLine, PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_PUE | PAL_RP_PAD_IE);
    sda_high();
    scl_high();

    /* Small delay for bus to stabilize */
    chThdSleepMilliseconds(100);

    /* Banner */
    chprintf(DBG, "\r\n");
    chprintf(DBG, "=====================================\r\n");
    chprintf(DBG, "RocketChip ChibiOS I2C Scan Test\r\n");
    chprintf(DBG, "Target: Adafruit Feather RP2350 HSTX\r\n");
    chprintf(DBG, "=====================================\r\n");
    chprintf(DBG, "\r\n");
    chprintf(DBG, "I2C Driver: Direct GPIO bit-bang\r\n");
    chprintf(DBG, "I2C SDA:    GPIO%lu\r\n", (uint32_t)kI2cSdaLine);
    chprintf(DBG, "I2C SCL:    GPIO%lu\r\n", (uint32_t)kI2cSclLine);
    chprintf(DBG, "\r\n");

    palClearLine(kLedPin);  /* LED OFF before scan */

    /* Scan I2C bus */
    chprintf(DBG, "Scanning I2C bus (0x%02X - 0x%02X)...\r\n", kI2cAddrMin, kI2cAddrMax);
    chprintf(DBG, "\r\n");

    /* Print header */
    chprintf(DBG, "     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

    for (uint8_t row = 0; row < 8; row++) {
        chprintf(DBG, "%02X: ", row << 4);

        for (uint8_t col = 0; col < 16; col++) {
            uint8_t addr = (row << 4) | col;

            if (addr < kI2cAddrMin || addr > kI2cAddrMax) {
                chprintf(DBG, "   ");
            } else {
                palToggleLine(kLedPin);  /* Blink during scan */

                if (i2c_probe(addr)) {
                    chprintf(DBG, "%02X ", addr);
                    if (found_count < 16) {
                        found_addrs[found_count++] = addr;
                    }
                } else {
                    chprintf(DBG, "-- ");
                }
            }
        }
        chprintf(DBG, "\r\n");
    }

    chprintf(DBG, "\r\n");
    chprintf(DBG, "Found %u device(s)\r\n", found_count);
    chprintf(DBG, "\r\n");

    /* List found devices with names */
    if (found_count > 0) {
        chprintf(DBG, "Detected devices:\r\n");
        for (uint8_t i = 0; i < found_count; i++) {
            const char* name = get_device_name(found_addrs[i]);
            if (name) {
                chprintf(DBG, "  0x%02X: %s\r\n", found_addrs[i], name);
            } else {
                chprintf(DBG, "  0x%02X: Unknown device\r\n", found_addrs[i]);
            }
        }
    }

    chprintf(DBG, "\r\n");
    chprintf(DBG, "Scan complete. LED will blink slowly.\r\n");
    chprintf(DBG, "Press reset to scan again.\r\n");

    /* Main loop - blink LED */
    while (true) {
        palSetLine(kLedPin);
        chThdSleepMilliseconds(500);
        palClearLine(kLedPin);
        chThdSleepMilliseconds(500);
    }

    return 0;
}
