// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Rocket Chip Project
//
// Bare-metal I2C probe for Adafruit Fruit Jam (RP2350B).
//
// v3: per Grok triage — deassert GPIO 22 (shared peripheral RESET for
//     both ESP32-C6 WiFi and TLV320DAC3100 audio DAC), match main
//     firmware init sequence, and explicitly probe 0x18 (DAC) as a
//     positive control independent of STEMMA QT wiring.
//
// Docs reference:
//   include/rocketchip/board_fruit_jam.h:77 — "ESP32-C6 WiFi: RESET=GPIO 22"
//   docs/hardware/BOARD_COMPARISON.md:117     — "RESET 22 (shared with audio DAC)"
//   docs/hardware/BOARD_COMPARISON.md:150     — "RESET 22 (shared with ESP32-C6)"
//
// If 0x18 (DAC) ACKs: bus and pins are correct; problem is GPS-specific
// (cable, device state, or address).  If nothing ACKs: either RESET=22
// still not released, or pins wrong, or global bus failure.
//
// Build:
//   cmake -B build -G Ninja -DPICO_BOARD=adafruit_fruit_jam \
//         -Dpicotool_DIR=<picotool>/picotool
//   ninja -C build
// Flash:
//   picotool load -f build/i2c_bare_test.uf2
//
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/resets.h"

#define SDA_PIN       20
#define SCL_PIN       21
#define I2C_INST      i2c0
#define PERIPH_RESET_PIN  22   // Shared ESP32-C6 + TLV320DAC3100 RESET (active-low)
#define I2C_HZ        100000
#define DAC_ADDR      0x18     // TLV320DAC3100 default address
#define GPS_ADDR      0x10     // PA1010D

// --- main-firmware-equivalent init ---------------------------------

static void release_peripheral_reset(void) {
    // GPIO 22 is the shared active-low peripheral RESET for:
    //   - ESP32-C6 WiFi coprocessor  (board_fruit_jam.h:77)
    //   - TLV320DAC3100 audio DAC    (BOARD_COMPARISON.md:117, :150)
    // Without this deassertion, neither device can respond on I2C.
    gpio_init(PERIPH_RESET_PIN);
    gpio_set_dir(PERIPH_RESET_PIN, true);
    gpio_put(PERIPH_RESET_PIN, true);   // drive HIGH = release reset
    sleep_ms(50);                       // datasheet-safe settle time
}

// Mirror of src/drivers/i2c_bus.cpp::i2c_bus_init() as of commit 10d78fe
static void i2c_full_init(void) {
    // 1) Hard-reset the DW_apb_i2c block (LL Entry 37 hardening)
    reset_block_mask(RESETS_RESET_I2C0_BITS);
    unreset_block_mask_wait_blocking(RESETS_RESET_I2C0_BITS);
    i2c_get_hw(I2C_INST)->enable = 0;

    // 2) Pad de-isolation on RP2350B (commit f7c5cce, BOARD_COMPARISON.md:214)
    gpio_set_function(SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_function(SCL_PIN, GPIO_FUNC_SIO);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // 3) 9-clock bus recovery (NXP UM10204 3.1.16)
    gpio_set_dir(SDA_PIN, false);   // SDA as input
    gpio_set_dir(SCL_PIN, true);    // SCL as output
    gpio_put(SCL_PIN, true);
    sleep_us(5);
    for (int i = 0; i < 9; i++) {
        gpio_put(SCL_PIN, false);
        sleep_us(5);
        gpio_put(SCL_PIN, true);
        sleep_us(5);
        if (gpio_get(SDA_PIN)) break;
    }
    // STOP condition
    gpio_set_dir(SDA_PIN, true);
    gpio_put(SDA_PIN, false);
    sleep_us(5);
    gpio_put(SCL_PIN, true);
    sleep_us(5);
    gpio_put(SDA_PIN, true);
    sleep_us(5);

    // 4) Init peripheral
    uint hz = i2c_init(I2C_INST, I2C_HZ);
    printf("  i2c_init returned %u Hz (requested %u)\n", hz, I2C_HZ);

    // 5) Hand pins to I2C peripheral
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// --- scanning ------------------------------------------------------

static bool probe(uint8_t addr) {
    uint8_t b = 0;
    int rc = i2c_read_timeout_us(I2C_INST, addr, &b, 1, false, 2000);
    return rc >= 0;
}

static void scan_all(void) {
    printf("\n--- Full I2C scan ---\n");
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (probe(a)) {
            printf("  0x%02X: ACK", a);
            if (a == DAC_ADDR) printf("  <-- TLV320DAC3100 (positive control)");
            if (a == GPS_ADDR) printf("  <-- PA1010D GPS");
            printf("\n");
            found++;
        }
    }
    printf("  total: %d device(s)\n", found);
}

static void explicit_check(void) {
    printf("\n--- Explicit address check ---\n");
    bool dac = probe(DAC_ADDR);
    bool gps = probe(GPS_ADDR);
    printf("  0x%02X TLV320DAC3100: %s\n", DAC_ADDR, dac ? "ACK" : "NACK");
    printf("  0x%02X PA1010D GPS : %s\n", GPS_ADDR, gps ? "ACK" : "NACK");

    if (dac && !gps) {
        printf("  => bus + pins CORRECT; GPS-specific issue\n");
    } else if (!dac && !gps) {
        printf("  => whole bus dead; suspect RESET (GPIO 22) not released\n");
        printf("     or pin assignment wrong\n");
    }
}

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(100);
    sleep_ms(500);

    printf("\n========================================\n");
    printf(" Fruit Jam I2C probe (Grok-triage v3)\n");
    printf(" Build: %s %s\n", __DATE__, __TIME__);
    printf("========================================\n");

    printf("\nReleasing GPIO %d (shared ESP32-C6 + DAC RESET)...\n",
           PERIPH_RESET_PIN);
    release_peripheral_reset();
    printf("  GPIO %d state after drive-high: %d\n",
           PERIPH_RESET_PIN, gpio_get(PERIPH_RESET_PIN));

    printf("\nInitialising I2C0 (SDA=%d, SCL=%d)...\n", SDA_PIN, SCL_PIN);
    i2c_full_init();
    sleep_ms(100);

    printf("\nPad state after init:\n");
    printf("  SDA=GPIO%d state=%d, SCL=GPIO%d state=%d\n",
           SDA_PIN, gpio_get(SDA_PIN), SCL_PIN, gpio_get(SCL_PIN));

    while (true) {
        scan_all();
        explicit_check();
        printf("\n--- sleeping 5s ---\n");
        sleep_ms(5000);
    }
    return 0;
}
