/**
 ******************************************************************************
 * @file    usbd_desc.c
 * @brief   USB Device & Configuration descriptors for a USB-MIDI class device.
 *
 * @details Descriptor layout follows the USB MIDI 1.0 class specification:
 *   - Device Descriptor (18 B)
 *   - Configuration Descriptor containing:
 *       Audio Control interface (AC Header)
 *       MIDI Streaming interface (MS Header, IN/OUT Jacks, Bulk endpoints)
 *   - String Descriptors (LangID, Manufacturer, Product, Serial)
 *   - Device Qualifier (for HS hosts — returns empty/stall)
 ******************************************************************************
 */

#include "usbd_desc.h"

// Helpers
#define LOBYTE(x) ((uint8_t)((x)&0x00FFU))
#define HIBYTE(x) ((uint8_t)(((x)&0xFF00U) >> 8))

// Audio / MIDI Class Constants
#define USB_AUDIO_CLASS       0x01U
#define USB_AUDIO_SUBCLASS_AC 0x01U
#define USB_AUDIO_SUBCLASS_MS 0x03U
#define USB_CS_INTERFACE      0x24U
#define USB_CS_ENDPOINT       0x25U

/* AC Header subtypes */
#define AC_HEADER 0x01U

/* MS Header / Jack subtypes */
#define MS_HEADER     0x01U
#define MIDI_IN_JACK  0x02U
#define MIDI_OUT_JACK 0x03U
#define MS_GENERAL    0x01U

/* Jack types */
#define JACK_EMBEDDED 0x01U
#define JACK_EXTERNAL 0x02U

/* Jack IDs */
#define JACK_ID_IN_EMB  0x01U
#define JACK_ID_IN_EXT  0x02U
#define JACK_ID_OUT_EMB 0x03U
#define JACK_ID_OUT_EXT 0x04U

// Device Descriptor (18 bytes)
static const uint8_t USBD_DeviceDesc[18] = {
    18,   /* bLength              */
    0x01, /* bDescriptorType      */
    0x00,
    0x02, /* bcdUSB  = 2.00       */
    0x00, /* bDeviceClass  (per interface) */
    0x00, /* bDeviceSubClass      */
    0x00, /* bDeviceProtocol      */
    64,   /* bMaxPacketSize0      */
    LOBYTE(USBD_VID),
    HIBYTE(USBD_VID), /* idVendor  */
    LOBYTE(USBD_PID),
    HIBYTE(USBD_PID), /* idProduct */
    0x00,
    0x01, /* bcdDevice = 1.00     */
    0x01, /* iManufacturer        */
    0x02, /* iProduct             */
    0x03, /* iSerialNumber        */
    0x01  /* bNumConfigurations   */
};

/**
 *  Configuration Descriptor (total 101 bytes)
 *
 *  Config(9) + AC_IF(9) + AC_Hdr(9) + MS_IF(9) +
 *  MS_Hdr(7) + InJackEmb(6) + InJackExt(6) + OutJackEmb(9) + OutJackExt(9) +
 *  BulkOUT(9) + CS_BulkOUT(5) + BulkIN(9) + CS_BulkIN(5)
 *  = 101
 */
#define CONFIG_TOTAL_LEN 101U

static const uint8_t USBD_ConfigDesc[CONFIG_TOTAL_LEN] = {
    /* ---- Configuration Descriptor (9) ------------------------------------ */
    0x09,                     /* bLength              */
    0x02,                     /* bDescriptorType      */
    LOBYTE(CONFIG_TOTAL_LEN), /* wTotalLength lo      */
    HIBYTE(CONFIG_TOTAL_LEN), /* wTotalLength hi      */
    0x02,                     /* bNumInterfaces       */
    0x01,                     /* bConfigurationValue  */
    0x00,                     /* iConfiguration       */
    0x80,                     /* bmAttributes (bus-powered) */
    0x32,                     /* bMaxPower = 100 mA   */

    /* ---- Interface 0: Audio Control (9) ---------------------------------- */
    0x09,                  /* bLength              */
    0x04,                  /* bDescriptorType      */
    0x00,                  /* bInterfaceNumber     */
    0x00,                  /* bAlternateSetting    */
    0x00,                  /* bNumEndpoints        */
    USB_AUDIO_CLASS,       /* bInterfaceClass      */
    USB_AUDIO_SUBCLASS_AC, /* bInterfaceSubClass   */
    0x00,                  /* bInterfaceProtocol   */
    0x00,                  /* iInterface           */

    /* ---- AC Header Descriptor (9) ---------------------------------------- */
    0x09,             /* bLength              */
    USB_CS_INTERFACE, /* bDescriptorType      */
    AC_HEADER,        /* bDescriptorSubtype   */
    0x00, 0x01,       /* bcdADC = 1.00        */
    0x09, 0x00,       /* wTotalLength = 9     */
    0x01,             /* bInCollection = 1    */
    0x01,             /* baInterfaceNr(1) = 1 */

    /* ---- Interface 1: MIDI Streaming (9) --------------------------------- */
    0x09,                  /* bLength              */
    0x04,                  /* bDescriptorType      */
    0x01,                  /* bInterfaceNumber     */
    0x00,                  /* bAlternateSetting    */
    0x02,                  /* bNumEndpoints        */
    USB_AUDIO_CLASS,       /* bInterfaceClass      */
    USB_AUDIO_SUBCLASS_MS, /* bInterfaceSubClass   */
    0x00,                  /* bInterfaceProtocol   */
    0x00,                  /* iInterface           */

    /* ---- MS Header Descriptor (7) ---------------------------------------- */
    0x07,             /* bLength              */
    USB_CS_INTERFACE, /* bDescriptorType      */
    MS_HEADER,        /* bDescriptorSubtype   */
    0x00, 0x01,       /* bcdMSC = 1.00        */
    /* wTotalLength = 7+6+6+9+9+5+5 = 47 */
    0x2F, 0x00,

    /* ---- MIDI IN Jack (Embedded) (6) ------------------------------------- */
    0x06,             /* bLength              */
    USB_CS_INTERFACE, /* bDescriptorType      */
    MIDI_IN_JACK,     /* bDescriptorSubtype   */
    JACK_EMBEDDED,    /* bJackType            */
    JACK_ID_IN_EMB,   /* bJackID              */
    0x00,             /* iJack                */

    /* ---- MIDI IN Jack (External) (6) ------------------------------------- */
    0x06,             /* bLength              */
    USB_CS_INTERFACE, /* bDescriptorType      */
    MIDI_IN_JACK,     /* bDescriptorSubtype   */
    JACK_EXTERNAL,    /* bJackType            */
    JACK_ID_IN_EXT,   /* bJackID              */
    0x00,             /* iJack                */

    /* ---- MIDI OUT Jack (Embedded) (9) ------------------------------------ */
    0x09,             /* bLength              */
    USB_CS_INTERFACE, /* bDescriptorType      */
    MIDI_OUT_JACK,    /* bDescriptorSubtype   */
    JACK_EMBEDDED,    /* bJackType            */
    JACK_ID_OUT_EMB,  /* bJackID              */
    0x01,             /* bNrInputPins         */
    JACK_ID_IN_EXT,   /* BaSourceID(1)        */
    0x01,             /* BaSourcePin(1)       */
    0x00,             /* iJack                */

    /* ---- MIDI OUT Jack (External) (9) ------------------------------------ */
    0x09,             /* bLength              */
    USB_CS_INTERFACE, /* bDescriptorType      */
    MIDI_OUT_JACK,    /* bDescriptorSubtype   */
    JACK_EXTERNAL,    /* bJackType            */
    JACK_ID_OUT_EXT,  /* bJackID              */
    0x01,             /* bNrInputPins         */
    JACK_ID_IN_EMB,   /* BaSourceID(1)        */
    0x01,             /* BaSourcePin(1)       */
    0x00,             /* iJack                */

    /* ---- Bulk OUT Endpoint Standard (9) ---------------------------------- */
    0x09,                     /* bLength (audio class = 9) */
    0x05,                     /* bDescriptorType      */
    MIDI_OUT_EP,              /* bEndpointAddress     */
    0x02,                     /* bmAttributes = Bulk  */
    LOBYTE(MIDI_PACKET_SIZE), /* wMaxPacketSize lo    */
    HIBYTE(MIDI_PACKET_SIZE), /* wMaxPacketSize hi    */
    0x00,                     /* bInterval            */
    0x00,                     /* bRefresh             */
    0x00,                     /* bSynchAddress        */

    /* ---- Class-Specific MS Bulk OUT Endpoint (5) ------------------------- */
    0x05,            /* bLength              */
    USB_CS_ENDPOINT, /* bDescriptorType      */
    MS_GENERAL,      /* bDescriptorSubtype   */
    0x01,            /* bNumEmbMIDIJack      */
    JACK_ID_IN_EMB,  /* BaAssocJackID(1)     */

    /* ---- Bulk IN Endpoint Standard (9) ----------------------------------- */
    0x09,                     /* bLength (audio class = 9) */
    0x05,                     /* bDescriptorType      */
    MIDI_IN_EP,               /* bEndpointAddress     */
    0x02,                     /* bmAttributes = Bulk  */
    LOBYTE(MIDI_PACKET_SIZE), /* wMaxPacketSize lo    */
    HIBYTE(MIDI_PACKET_SIZE), /* wMaxPacketSize hi    */
    0x00,                     /* bInterval            */
    0x00,                     /* bRefresh             */
    0x00,                     /* bSynchAddress        */

    /* ---- Class-Specific MS Bulk IN Endpoint (5) -------------------------- */
    0x05,            /* bLength              */
    USB_CS_ENDPOINT, /* bDescriptorType      */
    MS_GENERAL,      /* bDescriptorSubtype   */
    0x01,            /* bNumEmbMIDIJack      */
    JACK_ID_OUT_EMB  /* BaAssocJackID(1)     */
};

/* Index 0: Supported Language (US English) */
static const uint8_t USBD_StringLangID[4] = {
    0x04, 0x03, /* bLength, bDescriptorType */
    0x09, 0x04  /* wLANGID = 0x0409 (English – United States) */
};

/* Index 1: Manufacturer */
static const uint8_t USBD_StringMfg[] = {
    22,  0x03, /* bLength, bDescriptorType */
    'R', 0,    'a', 0, 'd', 0, 'a', 0, 'r', 0,
    '2', 0,    '0', 0, '0', 0, '0', 0, ' ', 0};

/* Index 2: Product */
static const uint8_t USBD_StringProduct[] = {
    34,  0x03, 'P', 0, 'r', 0, 'o', 0, 'g', 0, 't', 0, 'o', 0, 'm', 0, 'a', 0,
    't', 0,    'a', 0, ' ', 0, 'M', 0, 'I', 0, 'D', 0, 'I', 0, ' ', 0};

/* Index 3: Serial Number */
static const uint8_t USBD_StringSerial[] = {
    18, 0x03, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '1', 0};

/* Device Qualifier — returned to HS hosts, we only support FS */
static const uint8_t USBD_DevQualifier[10] = {
    10,         /* bLength            */
    0x06,       /* bDescriptorType    */
    0x00, 0x02, /* bcdUSB = 2.00      */
    0x00,       /* bDeviceClass       */
    0x00,       /* bDeviceSubClass    */
    0x00,       /* bDeviceProtocol    */
    64,         /* bMaxPacketSize0    */
    0x01,       /* bNumConfigurations */
    0x00        /* bReserved          */
};

#include <trice.h>

const uint8_t *USBD_GetDeviceDescriptor(uint16_t *len) {
  *len = (uint16_t)sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

const uint8_t *USBD_GetConfigDescriptor(uint16_t *len) {
  *len = CONFIG_TOTAL_LEN;
  return USBD_ConfigDesc;
}

const uint8_t *USBD_GetStringDescriptor(uint8_t index, uint16_t *len) {
  const uint8_t *desc = (void *)0;

  switch (index) {
  case 0:
    desc = USBD_StringLangID;
    *len = sizeof(USBD_StringLangID);
    break;
  case 1:
    desc = USBD_StringMfg;
    *len = sizeof(USBD_StringMfg);
    break;
  case 2:
    desc = USBD_StringProduct;
    *len = sizeof(USBD_StringProduct);
    break;
  case 3:
    desc = USBD_StringSerial;
    *len = sizeof(USBD_StringSerial);
    break;
  default:
    *len = 0;
    break;
  }
  return desc;
}

const uint8_t *USBD_GetDeviceQualifier(uint16_t *len) {
  *len = (uint16_t)sizeof(USBD_DevQualifier);
  return USBD_DevQualifier;
}
