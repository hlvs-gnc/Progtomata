/**
  ******************************************************************************
  * @file    usbd_core.c
  * @brief   USB Device Core — IRQ handler, EP0 state machine, standard
  *          device requests (GET_DESCRIPTOR, SET_ADDRESS, SET_CONFIGURATION).
  *
  * @details This module sits between the low-level SPL OTG FS driver
  *          (stm32f4xx_usb_otg_fs) and the MIDI class (usbd_midi).
  *          It handles enumeration and EP0 control transfers.
  ******************************************************************************
  */

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_midi.h"
#include "stm32f4xx_usb_otg_fs.h"
#include <string.h>
#include <trice.h>


// Forward declarations (private helpers)

static void USBD_Reset(USBD_HandleTypeDef *hdev);
static void USBD_EnumDone(USBD_HandleTypeDef *hdev);
static void USBD_RxFLVL(USBD_HandleTypeDef *hdev);
static void USBD_OEPInt(USBD_HandleTypeDef *hdev);
static void USBD_IEPInt(USBD_HandleTypeDef *hdev);
static void USBD_ProcessSetup(USBD_HandleTypeDef *hdev);
static void USBD_StdDevReq(USBD_HandleTypeDef *hdev);
static void USBD_StdItfReq(USBD_HandleTypeDef *hdev);
static void USBD_StdEPReq(USBD_HandleTypeDef *hdev);
static void USBD_EP0_InHandler(USBD_HandleTypeDef *hdev);
static void USBD_EP0_OutHandler(USBD_HandleTypeDef *hdev);
static void USBD_WriteEmptyTxFifo(USBD_HandleTypeDef *hdev, uint8_t epnum);

// Private config structure (static, used for core/dev init)


static USB_OTG_DevInitTypeDef usb_cfg =
{
  .dev_endpoints       = 4U,
  .speed               = USB_OTG_SPEED_FULL,
  .ep0_mps             = 64U,
  .phy_itface          = 0U,
  .Sof_enable          = 0U,
  .low_power_enable    = 0U,
  .vbus_sensing_enable = 0U,
  .use_dedicated_ep1   = 0U
};

void USBD_Init(USBD_HandleTypeDef *hdev)
{
  memset(hdev, 0, sizeof(*hdev));

  /* Set EP0 defaults */
  hdev->in_ep[0].num       = 0U;
  hdev->in_ep[0].is_in     = 1U;
  hdev->in_ep[0].type      = USB_EP_TYPE_CTRL;
  hdev->in_ep[0].maxpacket = 64U;
  hdev->in_ep[0].tx_fifo_num = 0U;

  hdev->out_ep[0].num      = 0U;
  hdev->out_ep[0].is_in    = 0U;
  hdev->out_ep[0].type     = USB_EP_TYPE_CTRL;
  hdev->out_ep[0].maxpacket = 64U;

  hdev->dev_state = USBD_STATE_DEFAULT;
  hdev->ep0_state = USBD_EP0_IDLE;

  /* Low-level core & device init */
  USB_OTG_FS_CoreInit(&usb_cfg);
  USB_OTG_FS_DevInit(&usb_cfg);

  /* Disconnect first to give the host a clean D+ edge */
  USB_OTG_FS_DevDisconnect();

  /* Pull-up D+ to signal attachment */
  USB_OTG_FS_DevConnect();
}

// IRQ Handler — called from OTG_FS_IRQHandler()
void USBD_IRQHandler(USBD_HandleTypeDef *hdev)
{
  uint32_t gintsts = USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;

  if (gintsts == 0U) { return; }


  // SOF (Start of Frame)
  if (gintsts & USB_OTG_GINTSTS_SOF)
  {
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_SOF;
  }

  // RxFIFO non-empty
  if (gintsts & USB_OTG_GINTSTS_RXFLVL)
  {
    USBD_RxFLVL(hdev);
  }

  // OUT endpoint interrupt
  if (gintsts & USB_OTG_GINTSTS_OEPINT)
  {
    USBD_OEPInt(hdev);
  }

  // IN endpoint interrupt
  if (gintsts & USB_OTG_GINTSTS_IEPINT)
  {
    USBD_IEPInt(hdev);
  }

  // USB Reset
  if (gintsts & USB_OTG_GINTSTS_USBRST)
  {
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBRST;
    USBD_Reset(hdev);
  }

  // Enumeration done
  if (gintsts & USB_OTG_GINTSTS_ENUMDNE)
  {
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
    USBD_EnumDone(hdev);
  }

  // Suspend
  if (gintsts & USB_OTG_GINTSTS_USBSUSP)
  {
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    if (hdev->dev_state == USBD_STATE_CONFIGURED)
    {
      hdev->dev_state = USBD_STATE_SUSPENDED;
    }
  }

  // Wakeup
  if (gintsts & USB_OTG_GINTSTS_WKUINT)
  {
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    /* Restore previous state if suspended */
  }

}

void USBD_CtlSendData(USBD_HandleTypeDef *hdev, uint8_t *buf, uint16_t len)
{
  hdev->ep0_tx_ptr   = buf;
  hdev->ep0_tx_rem   = len;
  hdev->ep0_tx_total = len;
  hdev->ep0_state    = USBD_EP0_DATA_IN;

  /* Start EP0 IN with first packet (up to 64 bytes) */
  hdev->in_ep[0].xfer_buff = buf;
  hdev->in_ep[0].xfer_len  = len;
  USB_OTG_FS_EP0StartXfer(&hdev->in_ep[0]);
}

void USBD_CtlSendStatus(USBD_HandleTypeDef *hdev)
{
  hdev->ep0_state = USBD_EP0_STATUS_IN;

  /* Send ZLP on EP0 IN */
  hdev->in_ep[0].xfer_buff = (void *)0;
  hdev->in_ep[0].xfer_len  = 0U;
  USB_OTG_FS_EP0StartXfer(&hdev->in_ep[0]);
}

void USBD_CtlError(USBD_HandleTypeDef *hdev)
{
  USB_OTG_FS_EPSetStall(&hdev->in_ep[0]);
  USB_OTG_FS_EPSetStall(&hdev->out_ep[0]);
}

/**
 * @brief Handle USB bus reset.
 *
 * Per RM0090: set NAK on all OUT EPs, flush FIFOs, reset address.
 * Do NOT arm EP0 yet — that happens in USBD_EnumDone after the
 * PHY completes speed detection.
 */
static void USBD_Reset(USBD_HandleTypeDef *hdev)
{
  /* De-init MIDI class if previously configured */
  if (hdev->dev_state == USBD_STATE_CONFIGURED)
  {
    USBD_MIDI_DeInit(hdev);
  }

  /* Clear remote wakeup signaling */
  USB_OTG_FS_DEV->DCTL &= ~USB_OTG_DCTL_RWUSIG;

  /* Flush all Tx FIFOs */
  USB_OTG_FS_FlushTxFIFO(0x10U);

  /* Clear endpoint interrupt flags (0xFB7FU matches HAL), deactivate and NAK all EPs.
   * EP0: Just set NAK (no EPDIS per HAL), but clear USBAEP to force reconfiguration.
   * Other EPs: Disable if enabled, then NAK. */
  for (uint32_t i = 0U; i < 4U; i++)
  {
    /* IN endpoints */
    USB_OTG_FS_INEP(i)->DIEPINT = 0xFB7FU;
    if (i == 0U)
    {
      /* EP0 IN: HAL preserves USBAEP via |= SNAK (EP0 is always active) */
      USB_OTG_FS_INEP(i)->DIEPCTL |= USB_OTG_DEPCTL_SNAK;
    }
    else
    {
      /* Other IN EPs: Disable if enabled, then NAK */
      if (USB_OTG_FS_INEP(i)->DIEPCTL & USB_OTG_DEPCTL_EPENA)
      {
        USB_OTG_FS_INEP(i)->DIEPCTL = USB_OTG_DEPCTL_EPDIS | USB_OTG_DEPCTL_SNAK;
      }
      else
      {
        USB_OTG_FS_INEP(i)->DIEPCTL = 0U;
      }
    }

    /* OUT endpoints */
    USB_OTG_FS_OUTEP(i)->DOEPINT = 0xFB7FU;
    if (i == 0U)
    {
      /* EP0 OUT: HAL preserves USBAEP via |= SNAK (EP0 is always active) */
      USB_OTG_FS_OUTEP(i)->DOEPCTL |= USB_OTG_DEPCTL_SNAK;
    }
    else
    {
      /* Other OUT EPs: Disable if enabled, then NAK */
      if (USB_OTG_FS_OUTEP(i)->DOEPCTL & USB_OTG_DEPCTL_EPENA)
      {
        USB_OTG_FS_OUTEP(i)->DOEPCTL = USB_OTG_DEPCTL_EPDIS | USB_OTG_DEPCTL_SNAK;
      }
      else
      {
        USB_OTG_FS_OUTEP(i)->DOEPCTL = 0U;
      }
    }
  }

  /* Enable EP0 IN + OUT interrupt bits in DAINTMSK */
  USB_OTG_FS_DEV->DAINTMSK = (1U << 16) | (1U << 0);

  /* Re-apply endpoint interrupt masks */
  USB_OTG_FS_DEV->DOEPMSK = USB_OTG_DEPINT_XFRC | USB_OTG_DEPINT_STUP;
  USB_OTG_FS_DEV->DIEPMSK = USB_OTG_DEPINT_XFRC | USB_OTG_DEPINT_TOC;

  /* Reset device address */
  USB_OTG_FS_SetDevAddress(0U);
  hdev->dev_address = 0U;
  hdev->dev_config  = 0U;
  hdev->dev_state   = USBD_STATE_DEFAULT;
  hdev->ep0_state   = USBD_EP0_IDLE;

  /* Prepare EP0 OUT transfer size — do NOT enable (CNAK/EPENA) yet */
  USB_OTG_FS_EP0_OutStart();
}

/**
 * @brief Handle enumeration done (speed detection complete).
 *
 * Now it is safe to activate EP0 and arm it for the first SETUP packet.
 */
static void USBD_EnumDone(USBD_HandleTypeDef *hdev)
{
  /* Set USB turnaround time (TRDT = 6 for HCLK >= 32 MHz) */
  USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
  USB_OTG_FS->GUSBCFG |= (6U << USB_OTG_GUSBCFG_TRDT_POS);

  /* Match HAL's USB_ActivateSetup(): only clear EP0 IN MPSIZ field
   * (bits [1:0]=00 encodes 64 bytes for EP0). Do NOT touch EP0 OUT,
   * do NOT call EP0_OutStart (already done in RESET handler), do NOT
   * call EPOpen, do NOT set CNAK/EPENA. SETUP packets arrive via the
   * STUPCNT mechanism programmed in DOEPTSIZ during RESET. */
  USB_OTG_FS_INEP(0)->DIEPCTL &= ~USB_OTG_DEPCTL_MPSIZ;
}

/**
 * @brief Handle RXFLVL interrupt — read data from the shared RxFIFO.
 */
static void USBD_RxFLVL(USBD_HandleTypeDef *hdev)
{
  /* Mask RXFLVL to prevent re-entry while reading FIFO */
  USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTSTS_RXFLVL;

  uint32_t grxstsp = USB_OTG_FS->GRXSTSP;
  uint8_t  epnum   = (uint8_t)(grxstsp & USB_OTG_GRXSTSP_EPNUM);
  uint16_t bcnt    = (uint16_t)((grxstsp & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_POS);
  uint8_t  pktsts  = (uint8_t)((grxstsp & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_POS);


  switch (pktsts)
  {
  case STS_SETUP_UPDT:
    /* Read 8-byte SETUP data from FIFO */
    USB_OTG_FS_ReadPacket(hdev->setup_buf, 8U);
    break;

  case STS_DATA_UPDT:
    if (bcnt > 0U)
    {
      if (epnum < USB_OTG_FS_MAX_EP_NUM &&
          hdev->out_ep[epnum].xfer_buff != (void *)0)
      {
        USB_OTG_FS_ReadPacket(hdev->out_ep[epnum].xfer_buff + hdev->out_ep[epnum].xfer_count,
                              bcnt);
        hdev->out_ep[epnum].xfer_count += bcnt;
      }
      else
      {
        /* Drain unhandled data into a scratch buffer so the FIFO doesn't jam. */
        static uint8_t rx_drain[64];
        uint16_t remaining = bcnt;
        while (remaining > 0U)
        {
          uint16_t chunk = (remaining > sizeof(rx_drain)) ? sizeof(rx_drain) : remaining;
          USB_OTG_FS_ReadPacket(rx_drain, chunk);
          remaining -= chunk;
        }
      }
    }
    break;

  case STS_XFER_COMP:
  case STS_SETUP_COMP:
  case STS_GOUT_NAK:
  default:
    break;
  }

  /* Unmask RXFLVL */
  USB_OTG_FS->GINTMSK |= USB_OTG_GINTSTS_RXFLVL;
}

/**
 * @brief Handle OUT endpoint interrupts.
 */
static void USBD_OEPInt(USBD_HandleTypeDef *hdev)
{
  uint32_t ep_intr = USB_OTG_FS_ReadDevAllOutEpItr();

  // EP0 OUT
  if (ep_intr & 0x01U)
  {
    uint32_t epint = USB_OTG_FS_OUTEP(0)->DOEPINT & USB_OTG_FS_DEV->DOEPMSK;

    /* Clear ALL pending flags upfront to prevent interrupt storms */
    USB_OTG_FS_OUTEP(0)->DOEPINT = USB_OTG_FS_OUTEP(0)->DOEPINT;

    /* SETUP phase done — handle first, skip XFRC if SETUP was processed */
    if (epint & USB_OTG_DEPINT_STUP)
    {
      USBD_ProcessSetup(hdev);
    }
    else if (epint & USB_OTG_DEPINT_XFRC)
    {
      USBD_EP0_OutHandler(hdev);
    }
  }

  // EP1 OUT (MIDI Bulk)
  if (ep_intr & 0x02U)
  {
    uint32_t ep1int = USB_OTG_FS_OUTEP(1)->DOEPINT;
    USB_OTG_FS_OUTEP(1)->DOEPINT = ep1int;  /* Clear all pending flags */

    if (ep1int & USB_OTG_DEPINT_XFRC)
    {
      USBD_MIDI_DataOut(hdev);
    }
  }
}

/**
 * @brief Handle IN endpoint interrupts.
 */
static void USBD_IEPInt(USBD_HandleTypeDef *hdev)
{
  uint32_t ep_intr = USB_OTG_FS_ReadDevAllInEpItr();

  // EP0 IN
  if (ep_intr & 0x01U)
  {
    uint32_t epint = USB_OTG_FS_INEP(0)->DIEPINT;
    uint32_t empmsk = USB_OTG_FS_DEV->DIEPEMPMSK;

    /* Clear ALL pending w1c flags upfront to prevent interrupt storms.
       TXFE (bit 7) is read-only/status and unaffected by this write. */
    USB_OTG_FS_INEP(0)->DIEPINT = epint;

    /* TxFIFO empty — write data into FIFO */
    if ((empmsk & 0x01U) && (epint & USB_OTG_DEPINT_TXFE))
    {
      USBD_WriteEmptyTxFifo(hdev, 0U);
    }

    /* Transfer complete */
    if (epint & USB_OTG_DEPINT_XFRC)
    {
      USB_OTG_FS_DEV->DIEPEMPMSK &= ~0x01U;
      USBD_EP0_InHandler(hdev);
    }
  }

  // EP1 IN — MIDI Bulk IN transfer complete
  if (ep_intr & 0x02U)
  {
    uint32_t epint = USB_OTG_FS_INEP(1)->DIEPINT;
    USB_OTG_FS_INEP(1)->DIEPINT = epint;  /* Clear all pending flags */
  }
}

static void USBD_ProcessSetup(USBD_HandleTypeDef *hdev)
{
  /* Parse the 8-byte SETUP data into structured form */
  hdev->request.bmRequest = hdev->setup_buf[0];
  hdev->request.bRequest  = hdev->setup_buf[1];
  hdev->request.wValue    = (uint16_t)(hdev->setup_buf[2] | ((uint16_t)hdev->setup_buf[3] << 8));
  hdev->request.wIndex    = (uint16_t)(hdev->setup_buf[4] | ((uint16_t)hdev->setup_buf[5] << 8));
  hdev->request.wLength   = (uint16_t)(hdev->setup_buf[6] | ((uint16_t)hdev->setup_buf[7] << 8));

  hdev->ep0_state     = USBD_EP0_SETUP;
  hdev->ep0_tx_rem    = 0U;

  uint8_t req_type = hdev->request.bmRequest & USB_REQ_TYPE_MASK;
  uint8_t recipient = hdev->request.bmRequest & USB_REQ_RECIPIENT_MASK;

  if (req_type == USB_REQ_TYPE_STANDARD)
  {
    switch (recipient)
    {
    case USB_REQ_RECIPIENT_DEVICE:
      USBD_StdDevReq(hdev);
      break;

    case USB_REQ_RECIPIENT_INTERFACE:
      USBD_StdItfReq(hdev);
      break;

    case USB_REQ_RECIPIENT_ENDPOINT:
      USBD_StdEPReq(hdev);
      break;

    default:
      USBD_CtlError(hdev);
      break;
    }
  }
  else
  {
    /* Class / Vendor requests — stall (MIDI doesn't need class-specific EP0 requests) */
    USBD_CtlError(hdev);
  }
}

/**
 * @brief Handle USB Standard Device Requests on EP0.
 */
static void USBD_StdDevReq(USBD_HandleTypeDef *hdev)
{
  const uint8_t *desc;
  uint16_t desc_len = 0U;
  USB_SetupReqTypeDef *req = &hdev->request;
  static uint8_t status_buf[2] = {0x00, 0x00};

  switch (req->bRequest)
  {
  // GET_DESCRIPTOR
  case USB_REQ_GET_DESCRIPTOR:
  {
    uint8_t desc_type  = (uint8_t)(req->wValue >> 8);
    uint8_t desc_index = (uint8_t)(req->wValue & 0xFFU);

    switch (desc_type)
    {
    case USB_DESC_TYPE_DEVICE:
      desc = USBD_GetDeviceDescriptor(&desc_len);
      break;

    case USB_DESC_TYPE_CONFIGURATION:
      desc = USBD_GetConfigDescriptor(&desc_len);
      break;

    case USB_DESC_TYPE_STRING:
      desc = USBD_GetStringDescriptor(desc_index, &desc_len);
      break;

    case USB_DESC_TYPE_QUALIFIER:
      /* Full-speed-only device: qualifier is unsupported and must stall. */
      USBD_CtlError(hdev);
      return;

    default:
      USBD_CtlError(hdev);
      return;
    }

    if (desc == (void *)0 || desc_len == 0U)
    {
      USBD_CtlError(hdev);
      return;
    }

    /* Clamp to wLength */
    if (desc_len > req->wLength)
    {
      desc_len = req->wLength;
    }

    USBD_CtlSendData(hdev, (uint8_t *)desc, desc_len);
    break;
  }

  // SET_ADDRESS
  case USB_REQ_SET_ADDRESS:
  {
    uint8_t addr = (uint8_t)(req->wValue & 0x7FU);

    if ((req->wIndex != 0U) || (req->wLength != 0U) || (req->wValue >= 128U) ||
        (hdev->dev_state == USBD_STATE_CONFIGURED))
    {
      USBD_CtlError(hdev);
      return;
    }

    hdev->dev_address = addr;
    hdev->dev_state   = (addr != 0U) ? USBD_STATE_ADDRESSED : USBD_STATE_DEFAULT;

    /* Per RM0090 (OTG_FS_DCFG): the application MUST program the device
     * address into DCFG BEFORE sending the STATUS IN ZLP for SET_ADDRESS.
     * Otherwise the host's next request at the new address is not answered. */
    USB_OTG_FS_SetDevAddress(addr);

    /* Send STATUS IN (ZLP) */
    USBD_CtlSendStatus(hdev);
    break;
  }

  // SET_CONFIGURATION 
  case USB_REQ_SET_CONFIGURATION:
  {
    uint8_t cfgval = (uint8_t)(req->wValue & 0xFFU);

    if ((req->wIndex != 0U) || (req->wLength != 0U) || (cfgval > 1U))
    {
      USBD_CtlError(hdev);
      return;
    }

    if (cfgval == 1U)
    {
      hdev->dev_config = cfgval;
      hdev->dev_state  = USBD_STATE_CONFIGURED;

      /* Initialise MIDI class (open endpoints) */
      USBD_MIDI_Init(hdev);
      TRICE(ID(3837), "USB: Device CONFIGURED - MIDI endpoints open\n");
      TRICE(ID(2684), "USB: EP1 OUT DOEPCTL=%08x DOEPTSIZ=%08x DAINTMSK=%08x\n",
            (uint32_t)USB_OTG_FS_OUTEP(1)->DOEPCTL,
            (uint32_t)USB_OTG_FS_OUTEP(1)->DOEPTSIZ,
            (uint32_t)USB_OTG_FS_DEV->DAINTMSK);
    }
    else
    {
      /* Deconfiguration */
      if (hdev->dev_state == USBD_STATE_CONFIGURED)
      {
        USBD_MIDI_DeInit(hdev);
      }
      hdev->dev_config = 0U;
      hdev->dev_state  = USBD_STATE_ADDRESSED;
    }

    USBD_CtlSendStatus(hdev);
    break;
  }

  // GET_CONFIGURATION 
  case USB_REQ_GET_CONFIGURATION:
  {
    if (req->wLength != 1U)
    {
      USBD_CtlError(hdev);
      return;
    }

    if (hdev->dev_state == USBD_STATE_CONFIGURED)
    {
      USBD_CtlSendData(hdev, &hdev->dev_config, 1U);
    }
    else
    {
      static uint8_t default_cfg = 0U;
      USBD_CtlSendData(hdev, &default_cfg, 1U);
    }
    break;
  }

  // GET_STATUS
  case USB_REQ_GET_STATUS:
  {
    if (req->wLength != 2U)
    {
      USBD_CtlError(hdev);
      return;
    }

    USBD_CtlSendData(hdev, status_buf, 2U);
    break;
  }

  // GET_INTERFACE / SET_INTERFACE
  case USB_REQ_GET_INTERFACE:
  {
    USBD_CtlError(hdev);
    break;
  }
  case USB_REQ_SET_INTERFACE:
  {
    USBD_CtlError(hdev);
    break;
  }

  default:
    USBD_CtlError(hdev);
    break;
  }
}

static void USBD_StdItfReq(USBD_HandleTypeDef *hdev)
{
  USB_SetupReqTypeDef *req = &hdev->request;
  static uint8_t alt_setting = 0U;
  static uint8_t status_buf[2] = {0x00, 0x00};

  switch (req->bRequest)
  {
  case USB_REQ_GET_INTERFACE:
    if ((req->wLength == 1U) &&
        (hdev->dev_state == USBD_STATE_ADDRESSED || hdev->dev_state == USBD_STATE_CONFIGURED))
    {
      USBD_CtlSendData(hdev, &alt_setting, 1U);
    }
    else
    {
      USBD_CtlError(hdev);
    }
    break;

  case USB_REQ_SET_INTERFACE:
    if ((req->wLength == 0U) && (hdev->dev_state == USBD_STATE_CONFIGURED))
    {
      USBD_CtlSendStatus(hdev);
    }
    else
    {
      USBD_CtlError(hdev);
    }
    break;

  case USB_REQ_GET_STATUS:
    if (req->wLength == 2U)
    {
      USBD_CtlSendData(hdev, status_buf, 2U);
    }
    else
    {
      USBD_CtlError(hdev);
    }
    break;

  default:
    USBD_CtlError(hdev);
    break;
  }
}

static void USBD_StdEPReq(USBD_HandleTypeDef *hdev)
{
  USB_SetupReqTypeDef *req = &hdev->request;
  uint8_t ep_addr = (uint8_t)(req->wIndex & 0xFFU);
  static uint8_t ep_status[2];

  switch (req->bRequest)
  {
  case USB_REQ_GET_STATUS:
    if (req->wLength != 2U)
    {
      USBD_CtlError(hdev);
      return;
    }

    if ((hdev->dev_state == USBD_STATE_ADDRESSED || hdev->dev_state == USBD_STATE_CONFIGURED) &&
        ((ep_addr == 0x00U) || (ep_addr == 0x80U)))
    {
      ep_status[0] = 0U;
      ep_status[1] = 0U;
      USBD_CtlSendData(hdev, ep_status, 2U);
      return;
    }

    if (hdev->dev_state == USBD_STATE_CONFIGURED)
    {
      uint8_t epnum = ep_addr & 0x7FU;
      if (epnum < USB_OTG_FS_MAX_EP_NUM)
      {
        uint32_t stall = 0U;
        if ((ep_addr & 0x80U) != 0U)
        {
          stall = USB_OTG_FS_INEP(epnum)->DIEPCTL & USB_OTG_DEPCTL_STALL;
        }
        else
        {
          stall = USB_OTG_FS_OUTEP(epnum)->DOEPCTL & USB_OTG_DEPCTL_STALL;
        }

        ep_status[0] = (stall != 0U) ? 1U : 0U;
        ep_status[1] = 0U;
        USBD_CtlSendData(hdev, ep_status, 2U);
        return;
      }
    }

    USBD_CtlError(hdev);
    break;

  default:
    USBD_CtlError(hdev);
    break;
  }
}

/**
 * @brief EP0 IN transfer complete handler.
 */
static void USBD_EP0_InHandler(USBD_HandleTypeDef *hdev)
{
  if (hdev->ep0_state == USBD_EP0_DATA_IN)
  {
    if (hdev->ep0_tx_rem > 0U)
    {
      /* Multi-packet: start HW transfer for the next chunk.
         WriteEmptyTxFifo already advanced ep0_tx_ptr / ep0_tx_rem
         for the chunk just sent, so ep0_tx_ptr now points to the
         start of the next unsent data. */
      uint32_t len = hdev->ep0_tx_rem;
      if (len > 64U) { len = 64U; }

      hdev->in_ep[0].xfer_buff = hdev->ep0_tx_ptr;
      hdev->in_ep[0].xfer_len  = len;
      USB_OTG_FS_EP0StartXfer(&hdev->in_ep[0]);
    }
    else
    {
      /* All data sent — prepare for STATUS OUT from host */
      hdev->ep0_state = USBD_EP0_STATUS_OUT;

      /* Arm EP0 OUT to receive ZLP */
      hdev->out_ep[0].xfer_len = 0U;
      USB_OTG_FS_EP0StartXfer(&hdev->out_ep[0]);
    }
  }
  else if (hdev->ep0_state == USBD_EP0_STATUS_IN)
  {
    /* STATUS IN sent — transfer done */
    hdev->ep0_state = USBD_EP0_IDLE;

    /* Re-arm EP0 OUT for next SETUP via STUPCNT mechanism only.
     * Do NOT set EPENA/CNAK on EP0 OUT here — the SETUP path uses
     * the STUPCNT counter in DOEPTSIZ, which EP0_OutStart() programs. */
    USB_OTG_FS_EP0_OutStart();
  }
}

/**
 * @brief EP0 OUT transfer complete handler.
 */
static void USBD_EP0_OutHandler(USBD_HandleTypeDef *hdev)
{
  if (hdev->ep0_state == USBD_EP0_STATUS_OUT)
  {
    /* STATUS OUT received — control transfer complete */
    hdev->ep0_state = USBD_EP0_IDLE;

    /* Re-arm EP0 OUT for next SETUP via STUPCNT mechanism only.
     * Do NOT set EPENA/CNAK — SETUP arrives through STUPCNT in DOEPTSIZ. */
    USB_OTG_FS_EP0_OutStart();
  }
  else if (hdev->ep0_state == USBD_EP0_DATA_OUT)
  {
    /* Data OUT complete — send STATUS IN (not typical for MIDI, but complete) */
    hdev->ep0_state = USBD_EP0_STATUS_IN;
    USBD_CtlSendStatus(hdev);
  }
}

/**
 * @brief Write data into the TxFIFO when it signals empty.
 *
 * Writes up to one max-packet (64 B) into the FIFO and then ALWAYS
 * disables DIEPEMPMSK.  For multi-packet EP0 transfers the next
 * chunk is kicked off by EP0_InHandler when XFRC fires.
 */
static void USBD_WriteEmptyTxFifo(USBD_HandleTypeDef *hdev, uint8_t epnum)
{
  uint32_t len = hdev->ep0_tx_rem;

  if (len > 64U) { len = 64U; }

  if (len > 0U)
  {
    USB_OTG_FS_WritePacket(hdev->ep0_tx_ptr, epnum, (uint16_t)len);
    hdev->ep0_tx_ptr += len;
    hdev->ep0_tx_rem -= len;
  }

  /* Always disable after writing one packet — EP0_InHandler will
     re-enable when it starts the next packet via EP0StartXfer. */
  USB_OTG_FS_DEV->DIEPEMPMSK &= ~(1U << epnum);
}
