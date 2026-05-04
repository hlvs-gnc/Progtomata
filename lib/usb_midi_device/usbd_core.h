/**
  ******************************************************************************
  * @file    usbd_core.h
  * @brief   USB Device Core — types, state machine, and API for EP0 control
  *          transfers and standard device requests.
  ******************************************************************************
  */

#ifndef USBD_CORE_H
#define USBD_CORE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_usb_otg_fs.h"
#include <stdint.h>

// USB Standard Request Codes

#define USB_REQ_GET_STATUS          0x00U
#define USB_REQ_CLEAR_FEATURE       0x01U
#define USB_REQ_SET_FEATURE         0x03U
#define USB_REQ_SET_ADDRESS         0x05U
#define USB_REQ_GET_DESCRIPTOR      0x06U
#define USB_REQ_SET_DESCRIPTOR      0x07U
#define USB_REQ_GET_CONFIGURATION   0x08U
#define USB_REQ_SET_CONFIGURATION   0x09U
#define USB_REQ_GET_INTERFACE       0x0AU
#define USB_REQ_SET_INTERFACE       0x0BU

// Descriptor Types

#define USB_DESC_TYPE_DEVICE        0x01U
#define USB_DESC_TYPE_CONFIGURATION 0x02U
#define USB_DESC_TYPE_STRING        0x03U
#define USB_DESC_TYPE_INTERFACE     0x04U
#define USB_DESC_TYPE_ENDPOINT      0x05U
#define USB_DESC_TYPE_QUALIFIER     0x06U

// bmRequestType Masks

#define USB_REQ_DIR_MASK            0x80U
#define USB_REQ_DIR_IN              0x80U
#define USB_REQ_DIR_OUT             0x00U
#define USB_REQ_TYPE_MASK           0x60U
#define USB_REQ_TYPE_STANDARD       0x00U
#define USB_REQ_TYPE_CLASS          0x20U
#define USB_REQ_TYPE_VENDOR         0x40U
#define USB_REQ_RECIPIENT_MASK      0x1FU
#define USB_REQ_RECIPIENT_DEVICE    0x00U
#define USB_REQ_RECIPIENT_INTERFACE 0x01U
#define USB_REQ_RECIPIENT_ENDPOINT  0x02U

// Device State

typedef enum
{
  USBD_STATE_DEFAULT    = 0,
  USBD_STATE_ADDRESSED  = 1,
  USBD_STATE_CONFIGURED = 2,
  USBD_STATE_SUSPENDED  = 3
} USBD_StateTypeDef;

// EP0 Control Transfer State

typedef enum
{
  USBD_EP0_IDLE       = 0,
  USBD_EP0_SETUP      = 1,
  USBD_EP0_DATA_IN    = 2,
  USBD_EP0_DATA_OUT   = 3,
  USBD_EP0_STATUS_IN  = 4,
  USBD_EP0_STATUS_OUT = 5,
  USBD_EP0_STALL      = 6
} USBD_EP0StateTypeDef;

// SETUP Packet

typedef struct
{
  uint8_t  bmRequest;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} USB_SetupReqTypeDef;

// USB Device Handle

typedef struct
{
  USBD_StateTypeDef       dev_state;
  USBD_EP0StateTypeDef    ep0_state;

  uint8_t                 dev_address;     /* Pending address (applied after STATUS IN) */
  uint8_t                 dev_config;      /* Current configuration value               */

  USB_SetupReqTypeDef     request;         /* Parsed SETUP packet                       */
  uint8_t                 setup_buf[8];    /* Raw SETUP data from FIFO                  */

  USB_OTG_EPTypeDef       in_ep[USB_OTG_FS_MAX_EP_NUM];
  USB_OTG_EPTypeDef       out_ep[USB_OTG_FS_MAX_EP_NUM];

  uint8_t                *ep0_tx_ptr;      /* Data pointer for EP0 IN multi-packet xfer */
  uint32_t                ep0_tx_rem;      /* Remaining bytes for EP0 IN transfer       */
  uint32_t                ep0_tx_total;    /* Total length requested by host             */
} USBD_HandleTypeDef;

// Public API

/**
 * @brief Initialise the USB device: core init, device init, set default state.
 */
void USBD_Init(USBD_HandleTypeDef *hdev);

/**
 * @brief Main USB IRQ handler — call from OTG_FS_IRQHandler().
 */
void USBD_IRQHandler(USBD_HandleTypeDef *hdev);

/**
 * @brief Send data on EP0 IN (control read transfer).
 */
void USBD_CtlSendData(USBD_HandleTypeDef *hdev, uint8_t *buf, uint16_t len);

/**
 * @brief Send zero-length STATUS IN packet on EP0.
 */
void USBD_CtlSendStatus(USBD_HandleTypeDef *hdev);

/**
 * @brief Stall both EP0 IN and OUT (protocol error response).
 */
void USBD_CtlError(USBD_HandleTypeDef *hdev);

#ifdef __cplusplus
}
#endif

#endif /* USBD_CORE_H */
