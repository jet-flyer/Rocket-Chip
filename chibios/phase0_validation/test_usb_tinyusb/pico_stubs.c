/*
 * Pico SDK stubs for ChibiOS + TinyUSB integration
 *
 * TinyUSB's RP2040 driver expects various Pico SDK runtime functions.
 * Rather than pull in the entire SDK runtime (which conflicts with ChibiOS),
 * we provide minimal stubs.
 *
 * NOTE: We provide ALL Pico SDK stubs here to avoid conflicts with ChibiOS.
 * Do not include sync.c, sync_spin_lock.c, or platform.c from Pico SDK.
 */

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Spin Lock stubs - TinyUSB uses these for thread safety                    */
/*===========================================================================*/

/* Hardware spin locks are at SIO base + 0x100 (RP2350 memory map) */
#define RP2350_SIO_BASE 0xD0000000
#define SPINLOCK_BASE (RP2350_SIO_BASE + 0x100)

/* Claim a spin lock - just return the lock number, we use ChibiOS for sync */
uint32_t spin_lock_claim_unused(bool required) {
    (void)required;
    /* Return lock 0 - TinyUSB typically only needs one */
    return 0;
}

/* Unclaim a spin lock */
void spin_lock_unclaim(uint32_t lock_num) {
    (void)lock_num;
}

/* Get spin lock instance - return pointer to hardware lock register */
volatile uint32_t *spin_lock_instance(uint32_t lock_num) {
    return (volatile uint32_t *)(SPINLOCK_BASE + lock_num * 4);
}

/* Block until spin lock acquired, return previous interrupt state */
uint32_t spin_lock_blocking(volatile uint32_t *lock) {
    uint32_t save = chSysGetStatusAndLockX();
    while (*lock == 0) {
        /* Spin - hardware lock returns 0 if already held */
    }
    return save;
}

/* Release spin lock and restore interrupt state */
void spin_unlock(volatile uint32_t *lock, uint32_t saved_irq) {
    *lock = 0;  /* Write any value to release */
    chSysRestoreStatusX(saved_irq);
}

/* Check if spin lock is claimed */
bool is_spin_locked(volatile uint32_t *lock) {
    /* Read the lock - returns non-zero if we can acquire it */
    uint32_t val = *lock;
    if (val) {
        *lock = 0;  /* Release immediately */
        return false;  /* Was not locked */
    }
    return true;  /* Was locked */
}

/* Software spin locks array - TinyUSB may reference this */
uint8_t _sw_spin_locks[32];

/* IRQ handler type for Pico SDK */
typedef void (*irq_handler_t)(void);

/*===========================================================================*/
/* IRQ stubs - TinyUSB needs these for USB interrupt handling               */
/*===========================================================================*/

/* Static handler storage for USB IRQ */
static irq_handler_t usb_irq_handler = NULL;

/* Debug counters for USB IRQ tracking */
volatile uint32_t usb_irq_count = 0;
volatile uint32_t usb_handler_null_count = 0;
volatile bool usb_irq_enabled = false;
volatile bool usb_handler_registered = false;

/* USB IRQ number differs between RP2040 (5) and RP2350 (14) */
#if PICO_RP2350
#define USB_IRQ_NUM 14
#else
#define USB_IRQ_NUM 5
#endif

/* Add a shared IRQ handler */
void irq_add_shared_handler(unsigned int num, irq_handler_t handler,
                            uint8_t order_priority) {
    (void)order_priority;
    if (num == USB_IRQ_NUM) {  /* USBCTRL_IRQ */
        usb_irq_handler = handler;
        usb_handler_registered = true;
    }
}

/* Enable/disable IRQ */
void irq_set_enabled(unsigned int num, bool enabled) {
    if (num == USB_IRQ_NUM) {  /* USBCTRL_IRQ */
        usb_irq_enabled = enabled;
        if (enabled) {
            /* Use priority 2 - same as SYSTICK and timers per mcuconf.h.
             * This is within the kernel-managed range and works with
             * OSAL_IRQ_HANDLER pattern. */
            nvicEnableVector(USBCTRL_IRQn, 2);
        } else {
            nvicDisableVector(USBCTRL_IRQn);
        }
    }
}

/* USB IRQ handler that calls the TinyUSB handler
 * ChibiOS RP2350 uses Vector78 for USB IRQ (IRQ 14) */
OSAL_IRQ_HANDLER(Vector78) {
    OSAL_IRQ_PROLOGUE();
    usb_irq_count++;
    if (usb_irq_handler != NULL) {
        usb_irq_handler();
    } else {
        usb_handler_null_count++;
    }
    OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Panic and assertion stubs                                                 */
/*===========================================================================*/

/* Panic function - called on critical errors */
void __attribute__((noreturn)) panic(const char *fmt, ...) {
    (void)fmt;
    /* In ChibiOS context, could call chSysHalt() or similar */
    /* For now, just loop forever */
    while (1) {
        __asm volatile("nop");
    }
}

/* Hard assertion failure - called when hard_assert fails */
void __attribute__((noreturn)) hard_assertion_failure(void) {
    while (1) {
        __asm volatile("nop");
    }
}

/* Unhandled user IRQ - called for unexpected interrupts */
void __attribute__((noreturn)) __unhandled_user_irq(void) {
    while (1) {
        __asm volatile("nop");
    }
}

/* USB device enumeration fix (RP2040 errata workaround) */
/* On RP2350, this might not be needed but the code references it */
void rp2040_usb_device_enumeration_fix(void) {
    /* Empty - RP2350 may not need this fix */
    /* The original fix involves SOF packet timing adjustments */
}

/* Runtime initialization stubs - we use ChibiOS init instead */
void runtime_init(void) {
    /* Empty - ChibiOS handles initialization */
}

/* Note: __atomic_fetch_add_4 is provided by GCC as a builtin */
