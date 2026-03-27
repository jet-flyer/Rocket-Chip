// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file psram_init.cpp
 * @brief APS6404L-3SQR PSRAM initialization via QMI CS1
 *
 * Based on SparkFun sparkfun-pico and AudioMorphology/PSRAM (MIT license).
 * Both derived from Arduino-Pico (earlephilhower).
 *
 * Sequence: detect via SPI ID read → enable QPI mode → configure QMI M1
 * timing/format → enable XIP writable for M1.
 *
 * All init functions run from SRAM (__no_inline_not_in_flash_func) because
 * they manipulate QMI registers that control XIP flash execution.
 *
 * IVP-52a: PSRAM Init + Self-Test (Stage 6: Data Logging)
 */

#include "psram_init.h"

#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "pico/flash.h"

#include <cstring>

namespace rc {

// ============================================================================
// Module state
// ============================================================================

static size_t g_psramSize = 0;

// ============================================================================
// PSRAM detection (SPI mode ID read)
// ============================================================================

// APS6404L Known Good Die byte
static constexpr uint8_t kAps6404Kgd = 0x5D;

// APS6404L SPI commands (datasheet Table 4)
static constexpr uint8_t kCmdExitQpi  = 0xF5U;  // Return to SPI mode
static constexpr uint8_t kCmdReadId   = 0x9FU;  // JEDEC Read ID
static constexpr uint8_t kCmdEnterQpi = 0x35U;  // Enter QPI mode

// APS6404L QPI read/write commands (datasheet Table 3)
static constexpr uint8_t kCmdFastReadQuad = 0xEBU;  // Fast Read Quad I/O
static constexpr uint8_t kCmdQuadWrite    = 0x38U;  // Quad Write

// SPI ID read: 1 cmd + 3 addr + 1 dummy + 1 KGD + 1 EID = 7 bytes
static constexpr uint32_t kIdReadLen    = 7;
static constexpr uint32_t kIdKgdIndex   = 5;  // KGD byte position in response
static constexpr uint32_t kIdEidIndex   = 6;  // EID byte position in response

// EID decoding (datasheet Section 2.2)
static constexpr uint32_t kEidSizeShift = 5;     // size_id = eid >> 5
static constexpr uint8_t  kEid8mb       = 0x26;  // EID value for 8MB variant

// Clock dividers for QMI direct mode
static constexpr uint32_t kDetectClkDiv = 30U;  // Slow clock for SPI detection
static constexpr uint32_t kQpiClkDiv    = 10U;  // Clock for QPI enable command

// QPI read format: 6 dummy cycles for Fast Read Quad I/O
static constexpr uint32_t kQpiReadDummyCycles = 6U;

// Frequency threshold for rxdelay adjustment (100 MHz)
static constexpr uint32_t kRxDelayThresholdHz = 100000000U;

static size_t __no_inline_not_in_flash_func(psram_detect)(uint32_t cs_pin) {
    // Assign GPIO to XIP CS1 function
    gpio_set_function(cs_pin, GPIO_FUNC_XIP_CS1);

    uint32_t intr_stash = save_and_disable_interrupts();

    // Enter direct mode with slow clock for detection
    qmi_hw->direct_csr = kDetectClkDiv << QMI_DIRECT_CSR_CLKDIV_LSB |
                          QMI_DIRECT_CSR_EN_BITS;

    // Wait for any prior XIP transfer cooldown
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {}

    // Exit QPI mode in case PSRAM is already in QPI from prior boot
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS |
                         (QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB) |
                         kCmdExitQpi;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {}
    (void)qmi_hw->direct_rx;
    qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;

    // Read device ID via SPI mode (cmd 0x9F, 3 addr bytes, then KGD + EID)
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    uint8_t kgd = 0;
    uint8_t eid = 0;

    for (uint32_t i = 0; i < kIdReadLen; ++i) {
        qmi_hw->direct_tx = (i == 0) ? static_cast<uint32_t>(kCmdReadId) : 0xFFU;
        while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) == 0) {}
        while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {}

        if (i == kIdKgdIndex) {
            kgd = static_cast<uint8_t>(qmi_hw->direct_rx);
        } else if (i == kIdEidIndex) {
            eid = static_cast<uint8_t>(qmi_hw->direct_rx);
        } else {
            (void)qmi_hw->direct_rx;
        }
    }

    // Deassert CS and disable direct mode
    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS |
                             QMI_DIRECT_CSR_EN_BITS);

    size_t psram_size = 0;
    if (kgd == kAps6404Kgd) {
        psram_size = 1024U * 1024U;  // Base 1MB
        uint8_t size_id = eid >> kEidSizeShift;
        if (eid == kEid8mb || size_id == 2) {
            psram_size *= 8;  // 8MB
        } else if (size_id == 0) {
            psram_size *= 2;  // 2MB
        } else if (size_id == 1) {
            psram_size *= 4;  // 4MB
        }
    }

    restore_interrupts(intr_stash);
    return psram_size;
}

// ============================================================================
// QMI M1 timing calculation for APS6404L
// ============================================================================

// APS6404L max freq: 133 MHz (datasheet). Conservative implementations
// use 109 MHz; we follow the reference implementations at 133 MHz.
// Source: APS6404L-3SQR datasheet, SparkFun/AudioMorphology implementations.
static constexpr uint32_t kMaxPsramFreq = 133000000U;

static uint32_t __no_inline_not_in_flash_func(psram_calc_timing)() {
    const uint32_t clock_hz = clock_get_hz(clk_sys);

    uint32_t divisor = (clock_hz + kMaxPsramFreq - 1) / kMaxPsramFreq;
    if (divisor == 1 && clock_hz > kRxDelayThresholdHz) {
        divisor = 2;
    }
    uint32_t rxdelay = divisor;
    if (clock_hz / divisor > kRxDelayThresholdHz) {
        rxdelay += 1;
    }

    // Max select: <= 8us in 64-clock units
    // Min deselect: >= 18ns in clock cycles minus ceil(divisor/2)
    const uint64_t clock_period_fs = 1000000000000000ULL / clock_hz;
    const uint32_t max_select = static_cast<uint32_t>(
        (125ULL * 1000000ULL) / clock_period_fs);  // 8000ns / 64
    const int32_t min_deselect = static_cast<int32_t>(
        (18ULL * 1000000ULL + clock_period_fs - 1) / clock_period_fs) -
        static_cast<int32_t>((divisor + 1) / 2);

    return 1U << QMI_M1_TIMING_COOLDOWN_LSB |
        QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB |
        (static_cast<uint32_t>(max_select) << QMI_M1_TIMING_MAX_SELECT_LSB) |
        (static_cast<uint32_t>(min_deselect > 0 ? min_deselect : 0)
            << QMI_M1_TIMING_MIN_DESELECT_LSB) |
        rxdelay << QMI_M1_TIMING_RXDELAY_LSB |
        divisor << QMI_M1_TIMING_CLKDIV_LSB;
}

// ============================================================================
// QMI M1 configuration for APS6404L QSPI mode
// ============================================================================

static void __no_inline_not_in_flash_func(psram_configure_qmi)() {
    // Enable direct mode with auto-CS for QPI enable command
    qmi_hw->direct_csr = kQpiClkDiv << QMI_DIRECT_CSR_CLKDIV_LSB |
                          QMI_DIRECT_CSR_EN_BITS |
                          QMI_DIRECT_CSR_AUTO_CS1N_BITS;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {}

    // Enable QPI mode on PSRAM
    qmi_hw->direct_tx = QMI_DIRECT_TX_NOPUSH_BITS | kCmdEnterQpi;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {}

    // Set timing register
    qmi_hw->m[1].timing = psram_calc_timing();

    // Read format: QPI with 6 dummy cycles (Fast Read Quad I/O 0xEB)
    qmi_hw->m[1].rfmt =
        QMI_M1_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_PREFIX_WIDTH_LSB |
        QMI_M1_RFMT_ADDR_WIDTH_VALUE_Q   << QMI_M1_RFMT_ADDR_WIDTH_LSB |
        QMI_M1_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_RFMT_SUFFIX_WIDTH_LSB |
        QMI_M1_RFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M1_RFMT_DUMMY_WIDTH_LSB |
        QMI_M1_RFMT_DATA_WIDTH_VALUE_Q   << QMI_M1_RFMT_DATA_WIDTH_LSB |
        QMI_M1_RFMT_PREFIX_LEN_VALUE_8   << QMI_M1_RFMT_PREFIX_LEN_LSB |
        kQpiReadDummyCycles << QMI_M1_RFMT_DUMMY_LEN_LSB;
    qmi_hw->m[1].rcmd = kCmdFastReadQuad;

    // Write format: QPI (Quad Write 0x38, no dummy cycles)
    qmi_hw->m[1].wfmt =
        QMI_M1_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_PREFIX_WIDTH_LSB |
        QMI_M1_WFMT_ADDR_WIDTH_VALUE_Q   << QMI_M1_WFMT_ADDR_WIDTH_LSB |
        QMI_M1_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M1_WFMT_SUFFIX_WIDTH_LSB |
        QMI_M1_WFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M1_WFMT_DUMMY_WIDTH_LSB |
        QMI_M1_WFMT_DATA_WIDTH_VALUE_Q   << QMI_M1_WFMT_DATA_WIDTH_LSB |
        QMI_M1_WFMT_PREFIX_LEN_VALUE_8   << QMI_M1_WFMT_PREFIX_LEN_LSB;
    qmi_hw->m[1].wcmd = kCmdQuadWrite;

    // Disable direct mode (return to XIP)
    qmi_hw->direct_csr = 0;

    // Enable writes to PSRAM via XIP
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);
}

// ============================================================================
// Public API
// ============================================================================

size_t __no_inline_not_in_flash_func(psram_init)(uint32_t cs_pin) {
    g_psramSize = psram_detect(cs_pin);
    if (g_psramSize == 0) {
        return 0;
    }

    psram_configure_qmi();
    return g_psramSize;
}

bool psram_self_test(size_t size) {
    if (size == 0) {
        return false;
    }

    // Test 3 addresses: start, middle, end
    volatile uint32_t* base =
        reinterpret_cast<volatile uint32_t*>(kPsramCachedBase);

    static constexpr uint32_t kPattern1 = 0xDEADBEEFU;
    static constexpr uint32_t kPattern2 = 0xCAFEBABEU;
    static constexpr uint32_t kPattern3 = 0x12345678U;

    const uint32_t offsets[] = {
        0,                                      // First word
        (static_cast<uint32_t>(size) / 2) / 4,  // Middle (word index)
        (static_cast<uint32_t>(size) - 4) / 4   // Last word
    };
    const uint32_t patterns[] = {kPattern1, kPattern2, kPattern3};

    // Write patterns
    for (uint32_t i = 0; i < 3; ++i) {
        base[offsets[i]] = patterns[i];
    }

    // Read back and verify
    for (uint32_t i = 0; i < 3; ++i) {
        if (base[offsets[i]] != patterns[i]) {
            return false;
        }
    }

    // Clear test data
    for (uint32_t i = 0; i < 3; ++i) {
        base[offsets[i]] = 0;
    }

    return true;
}

uint8_t* psram_base_ptr() {
    if (g_psramSize == 0) {
        return nullptr;
    }
    return reinterpret_cast<uint8_t*>(kPsramCachedBase);
}

uint8_t* psram_uncached_base_ptr() {
    if (g_psramSize == 0) {
        return nullptr;
    }
    return reinterpret_cast<uint8_t*>(kPsramUncachedBase);
}

size_t psram_get_size() {
    return g_psramSize;
}

// ============================================================================
// Flash-safe PSRAM integrity test (council req. #2 hard gate)
// ============================================================================

// Write a known pattern to PSRAM, perform a flash erase+program cycle,
// then verify PSRAM data is byte-for-byte intact. This validates the SDK's
// QMI M1 save/restore in flash_safe_execute() for our case (CS1 not in
// FLASH_DEVINFO → timing/rcmd/rfmt restored, wfmt/wcmd untouched).

static constexpr uint32_t kFlashTestOffset = 0x7FB000U;  // Last sector before flight table
static constexpr uint32_t kFlashSafeTimeoutMs = 1000U;
static constexpr uint32_t kPsramTestSize = 256U;  // Bytes to test
static constexpr uint32_t kPsramTestPattern = 0xA5B6C7D8U;

struct flash_erase_params {
    uint32_t offset;
    size_t len;
};

static void do_flash_erase(void* param) {
    auto* p = static_cast<flash_erase_params*>(param);
    flash_range_erase(p->offset, p->len);
}

bool psram_flash_safe_test() {
    if (g_psramSize == 0) {
        return false;
    }

    // Use uncached alias to avoid cache coherency issues during test
    volatile uint32_t* psram =
        reinterpret_cast<volatile uint32_t*>(kPsramUncachedBase);

    // Step 1: Write known pattern to PSRAM (first 256 bytes via uncached)
    for (uint32_t i = 0; i < kPsramTestSize / 4; ++i) {
        psram[i] = kPsramTestPattern ^ i;
    }

    // Step 2: Verify pattern before flash op (sanity check)
    for (uint32_t i = 0; i < kPsramTestSize / 4; ++i) {
        if (psram[i] != (kPsramTestPattern ^ i)) {
            return false;  // PSRAM write/read failed even before flash op
        }
    }

    // Step 3: Perform flash_safe_execute() erase (this is the real test)
    flash_erase_params params = {
        .offset = kFlashTestOffset,
        .len = FLASH_SECTOR_SIZE
    };
    int result = flash_safe_execute(do_flash_erase, &params, kFlashSafeTimeoutMs);
    if (result != PICO_OK) {
        return false;  // flash_safe_execute itself failed
    }

    // Step 4: Read back PSRAM and verify byte-for-byte
    for (uint32_t i = 0; i < kPsramTestSize / 4; ++i) {
        if (psram[i] != (kPsramTestPattern ^ i)) {
            return false;  // PSRAM data corrupted by flash operation
        }
    }

    // Step 5: Clean up — zero the test area
    for (uint32_t i = 0; i < kPsramTestSize / 4; ++i) {
        psram[i] = 0;
    }

    return true;
}

} // namespace rc
