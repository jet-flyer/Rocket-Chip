/*
 * USB Descriptors for TinyUSB CDC Device
 *
 * Minimal CDC (serial port) descriptors for RocketChip ChibiOS.
 */

#include "tusb.h"

/*===========================================================================*/
/* Device Descriptor                                                         */
/*===========================================================================*/

#define USB_VID     0x2E8A  /* Raspberry Pi */
#define USB_PID     0x000A  /* Pico SDK CDC */
#define USB_BCD     0x0200  /* USB 2.0 */

static const tusb_desc_device_t kDeviceDescriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

const uint8_t* tud_descriptor_device_cb(void) {
    return (const uint8_t*)&kDeviceDescriptor;
}

/*===========================================================================*/
/* Configuration Descriptor                                                  */
/*===========================================================================*/

/* Interface numbers */
enum {
    kItfCdc = 0,
    kItfCdcData,
    kItfTotal
};

/* Endpoint addresses */
#define kEpCdcNotify    0x81
#define kEpCdcOut       0x02
#define kEpCdcIn        0x82

/* Total configuration length */
#define kConfigTotalLen (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t kConfigDescriptor[] = {
    /* Configuration descriptor */
    TUD_CONFIG_DESCRIPTOR(1, kItfTotal, 0, kConfigTotalLen, 0, 100),

    /* CDC descriptor */
    TUD_CDC_DESCRIPTOR(kItfCdc, 4, kEpCdcNotify, 8, kEpCdcOut, kEpCdcIn, 64),
};

const uint8_t* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return kConfigDescriptor;
}

/*===========================================================================*/
/* String Descriptors                                                        */
/*===========================================================================*/

/* String index enum */
enum {
    kStrLangId = 0,
    kStrManufacturer,
    kStrProduct,
    kStrSerial,
    kStrCdc
};

/* String descriptors */
static const char* const kStringDescriptors[] = {
    [kStrLangId]      = (const char[]){0x09, 0x04},  /* English (US) */
    [kStrManufacturer] = "RocketChip",
    [kStrProduct]      = "ChibiOS USB CDC",
    [kStrSerial]       = "001",
    [kStrCdc]          = "CDC Serial",
};

const uint16_t* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    static uint16_t descStr[32];
    uint8_t len;

    if (index == 0) {
        /* Language ID */
        descStr[1] = 0x0409;  /* English (US) */
        len = 1;
    } else {
        if (index >= sizeof(kStringDescriptors) / sizeof(kStringDescriptors[0])) {
            return NULL;
        }
        const char* str = kStringDescriptors[index];
        len = 0;
        while (str[len] && len < 31) {
            descStr[1 + len] = str[len];
            len++;
        }
    }

    /* First byte: length (including header), second byte: string type */
    descStr[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));

    return descStr;
}
