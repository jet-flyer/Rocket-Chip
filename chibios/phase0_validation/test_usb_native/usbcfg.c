/*
 * USB CDC Configuration
 * RocketChip ChibiOS Phase 0 - USB Native Test
 *
 * Based on ChibiOS USB CDC test examples
 */

#include "hal.h"
#include "usbcfg.h"

/*===========================================================================*/
/* USB CDC Driver Instance                                                   */
/*===========================================================================*/

SerialUSBDriver SDU1;

/*===========================================================================*/
/* Endpoint Definitions                                                      */
/*===========================================================================*/

#define USB_DATA_REQUEST_EP         1
#define USB_DATA_AVAILABLE_EP       1
#define USB_INTERRUPT_REQUEST_EP    2

/*===========================================================================*/
/* USB Device Descriptor                                                     */
/*===========================================================================*/

static const uint8_t vcom_device_descriptor_data[18] = {
    USB_DESC_DEVICE(
        0x0200,         /* bcdUSB (USB 2.0) */
        0x02,           /* bDeviceClass (CDC) */
        0x00,           /* bDeviceSubClass */
        0x00,           /* bDeviceProtocol */
        0x40,           /* bMaxPacketSize (64 bytes) */
        0x2E8A,         /* idVendor (Raspberry Pi) */
        0x000A,         /* idProduct (Pico SDK CDC) */
        0x0100,         /* bcdDevice */
        1,              /* iManufacturer */
        2,              /* iProduct */
        3,              /* iSerialNumber */
        1)              /* bNumConfigurations */
};

static const USBDescriptor vcom_device_descriptor = {
    sizeof(vcom_device_descriptor_data),
    vcom_device_descriptor_data
};

/*===========================================================================*/
/* USB Configuration Descriptor                                              */
/*===========================================================================*/

static const uint8_t vcom_configuration_descriptor_data[67] = {
    /* Configuration Descriptor */
    USB_DESC_CONFIGURATION(
        67,             /* wTotalLength */
        0x02,           /* bNumInterfaces */
        0x01,           /* bConfigurationValue */
        0,              /* iConfiguration */
        0xC0,           /* bmAttributes (self powered) */
        50),            /* bMaxPower (100mA) */

    /* Interface Descriptor (CDC Control) */
    USB_DESC_INTERFACE(
        0x00,           /* bInterfaceNumber */
        0x00,           /* bAlternateSetting */
        0x01,           /* bNumEndpoints */
        0x02,           /* bInterfaceClass (CDC) */
        0x02,           /* bInterfaceSubClass (ACM) */
        0x01,           /* bInterfaceProtocol (AT commands) */
        0),             /* iInterface */

    /* Header Functional Descriptor */
    USB_DESC_BYTE(5),
    USB_DESC_BYTE(0x24),    /* CS_INTERFACE */
    USB_DESC_BYTE(0x00),    /* Header */
    USB_DESC_BCD(0x0110),   /* CDC 1.1 */

    /* Call Management Functional Descriptor */
    USB_DESC_BYTE(5),
    USB_DESC_BYTE(0x24),    /* CS_INTERFACE */
    USB_DESC_BYTE(0x01),    /* Call Management */
    USB_DESC_BYTE(0x00),    /* bmCapabilities */
    USB_DESC_BYTE(0x01),    /* bDataInterface */

    /* ACM Functional Descriptor */
    USB_DESC_BYTE(4),
    USB_DESC_BYTE(0x24),    /* CS_INTERFACE */
    USB_DESC_BYTE(0x02),    /* ACM */
    USB_DESC_BYTE(0x02),    /* bmCapabilities */

    /* Union Functional Descriptor */
    USB_DESC_BYTE(5),
    USB_DESC_BYTE(0x24),    /* CS_INTERFACE */
    USB_DESC_BYTE(0x06),    /* Union */
    USB_DESC_BYTE(0x00),    /* bMasterInterface */
    USB_DESC_BYTE(0x01),    /* bSlaveInterface0 */

    /* Endpoint Descriptor (Interrupt IN) */
    USB_DESC_ENDPOINT(
        USB_INTERRUPT_REQUEST_EP | 0x80,
        0x03,           /* bmAttributes (Interrupt) */
        0x0008,         /* wMaxPacketSize */
        0xFF),          /* bInterval */

    /* Interface Descriptor (CDC Data) */
    USB_DESC_INTERFACE(
        0x01,           /* bInterfaceNumber */
        0x00,           /* bAlternateSetting */
        0x02,           /* bNumEndpoints */
        0x0A,           /* bInterfaceClass (CDC Data) */
        0x00,           /* bInterfaceSubClass */
        0x00,           /* bInterfaceProtocol */
        0x00),          /* iInterface */

    /* Endpoint Descriptor (Bulk OUT) */
    USB_DESC_ENDPOINT(
        USB_DATA_AVAILABLE_EP,
        0x02,           /* bmAttributes (Bulk) */
        0x0040,         /* wMaxPacketSize (64) */
        0x00),          /* bInterval */

    /* Endpoint Descriptor (Bulk IN) */
    USB_DESC_ENDPOINT(
        USB_DATA_REQUEST_EP | 0x80,
        0x02,           /* bmAttributes (Bulk) */
        0x0040,         /* wMaxPacketSize (64) */
        0x00)           /* bInterval */
};

static const USBDescriptor vcom_configuration_descriptor = {
    sizeof(vcom_configuration_descriptor_data),
    vcom_configuration_descriptor_data
};

/*===========================================================================*/
/* USB String Descriptors                                                    */
/*===========================================================================*/

static const uint8_t vcom_string0[] = {
    USB_DESC_BYTE(4),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    USB_DESC_WORD(0x0409)   /* English (US) */
};

static const uint8_t vcom_string1[] = {
    USB_DESC_BYTE(22),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    'R', 0, 'o', 0, 'c', 0, 'k', 0, 'e', 0, 't', 0, 'C', 0, 'h', 0,
    'i', 0, 'p', 0
};

static const uint8_t vcom_string2[] = {
    USB_DESC_BYTE(36),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, ' ', 0,
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'C', 0, 'D', 0, 'C', 0, ' ', 0,
    ' ', 0
};

static const uint8_t vcom_string3[] = {
    USB_DESC_BYTE(8),
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),
    '0', 0, '0', 0, '1', 0
};

static const USBDescriptor vcom_strings[] = {
    {sizeof(vcom_string0), vcom_string0},
    {sizeof(vcom_string1), vcom_string1},
    {sizeof(vcom_string2), vcom_string2},
    {sizeof(vcom_string3), vcom_string3}
};

/*===========================================================================*/
/* USB Callbacks                                                             */
/*===========================================================================*/

static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {
    (void)usbp;
    (void)lang;

    switch (dtype) {
    case USB_DESCRIPTOR_DEVICE:
        return &vcom_device_descriptor;
    case USB_DESCRIPTOR_CONFIGURATION:
        return &vcom_configuration_descriptor;
    case USB_DESCRIPTOR_STRING:
        if (dindex < 4) {
            return &vcom_strings[dindex];
        }
        break;
    }
    return NULL;
}

/*===========================================================================*/
/* Endpoint States and Configurations                                        */
/*===========================================================================*/

static USBInEndpointState ep1instate;
static USBOutEndpointState ep1outstate;

static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,
    sduDataTransmitted,
    sduDataReceived,
    0x0040,
    0x0040,
    &ep1instate,
    &ep1outstate
};

static USBInEndpointState ep2instate;

static const USBEndpointConfig ep2config = {
    USB_EP_MODE_TYPE_INTR,
    NULL,
    sduInterruptTransmitted,
    NULL,
    0x0010,
    0x0000,
    &ep2instate,
    NULL
};

/*===========================================================================*/
/* USB Event Handlers                                                        */
/*===========================================================================*/

static void usb_event(USBDriver *usbp, usbevent_t event) {
    switch (event) {
    case USB_EVENT_ADDRESS:
        return;

    case USB_EVENT_CONFIGURED:
        chSysLockFromISR();
        usbInitEndpointI(usbp, USB_DATA_REQUEST_EP, &ep1config);
        usbInitEndpointI(usbp, USB_INTERRUPT_REQUEST_EP, &ep2config);
        sduConfigureHookI(&SDU1);
        chSysUnlockFromISR();
        return;

    case USB_EVENT_RESET:
    case USB_EVENT_UNCONFIGURED:
    case USB_EVENT_SUSPEND:
        chSysLockFromISR();
        sduSuspendHookI(&SDU1);
        chSysUnlockFromISR();
        return;

    case USB_EVENT_WAKEUP:
        chSysLockFromISR();
        sduWakeupHookI(&SDU1);
        chSysUnlockFromISR();
        return;

    case USB_EVENT_STALLED:
        return;
    }
}

static void sof_handler(USBDriver *usbp) {
    (void)usbp;
    osalSysLockFromISR();
    sduSOFHookI(&SDU1);
    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* USB Driver Configuration                                                  */
/*===========================================================================*/

const USBConfig usbcfg = {
    usb_event,
    get_descriptor,
    sduRequestsHook,
    sof_handler
};

SerialUSBConfig serusbcfg = {
    &USBD1,
    USB_DATA_REQUEST_EP,
    USB_DATA_AVAILABLE_EP,
    USB_INTERRUPT_REQUEST_EP
};
