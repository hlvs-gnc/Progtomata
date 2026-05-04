/**
  ******************************************************************************
  * @file    stm32f4xx_usb_otg_fs.c
  * @author  Radar2000
  * @version V1.0.0
  * @date    20-April-2026
  * @brief   USB OTG Full-Speed peripheral driver (SPL style, Device mode).
  *
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB OTG FS peripheral:
  *           + Core initialization and reset
  *           + Device mode initialization
  *           + Endpoint open / close / transfer
  *           + FIFO configuration and flush
  *           + Packet read / write helpers
  *
  ******************************************************************************
  * @attention
  *
  * Register-level driver built on top of the CMSIS USB_OTG_GlobalTypeDef,
  * USB_OTG_DeviceTypeDef, USB_OTG_INEndpointTypeDef, and
  * USB_OTG_OUTEndpointTypeDef structures defined in stm32f4xx.h.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_usb_otg_fs.h"
#include "stm32f4xx_rcc.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup USB_OTG_FS
  * @brief USB OTG FS driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** Timeout loop count for core reset / FIFO flush */
#define USB_OTG_TIMEOUT   200000U

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup USB_OTG_FS_Private_Functions
  * @{
  */

/* ======================== Core Functions =================================== */

/**
  * @brief  Perform a soft reset of the USB OTG FS core.
  * @retval None
  */
void USB_OTG_FS_CoreReset(void)
{
  volatile uint32_t count = 0U;

  /* Wait for AHB master IDLE state */
  do {
    if (++count > USB_OTG_TIMEOUT) { return; }
  } while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  count = 0U;

  /* Core soft reset */
  USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

  do {
    if (++count > USB_OTG_TIMEOUT) { return; }
  } while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0U);
}

/**
  * @brief  Initialize the USB OTG FS core registers.
  * @param  cfg: pointer to USB_OTG_DevInitTypeDef configuration structure
  * @retval None
  */
void USB_OTG_FS_CoreInit(USB_OTG_DevInitTypeDef *cfg)
{
  /* Select Full-Speed embedded PHY */
  USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

  /* Reset the core after PHY selection */
  USB_OTG_FS_CoreReset();

  /* Activate the USB transceiver (power-down deactivation) */
  USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN;

  if (cfg->Sof_enable)
  {
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_SOFOUTEN;
  }

  /* Force Device mode and poll until mode switch is confirmed.
   * HAL reference: USB_SetCurrentMode with polling on GINTSTS.CMOD. */
  USB_OTG_FS->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
  USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

  {
    /* HAL idiom: ALWAYS wait at least 1 ms before checking, then loop up to 50ms.
     * RM0090: mode-change effective after at least 25 ms of OTG block settling. */
    volatile uint32_t ms = 0U;
    do
    {
      /* ~1ms busy wait at 168 MHz */
      for (volatile uint32_t i = 0; i < 42000U; i++) { __NOP(); }
      ms++;
    } while (((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD) != 0U) && (ms < 50U));

    /* Extra safety wait: ensure full settling even if CMOD cleared early */
    for (volatile uint32_t i = 0; i < 42000U * 50U; i++) { __NOP(); }
  }

  /* Set USB turnaround time (TRDT = 6 for HCLK >= 32 MHz, embedded FS PHY).
   * HAL sets this in USB_CoreInit BEFORE USB_DevInit; doing it here ensures
   * the IN-token timing is correct from the very first enumeration attempt. */
  USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
  USB_OTG_FS->GUSBCFG |= (6U << USB_OTG_GUSBCFG_TRDT_POS);
}

/**
  * @brief  Initialize the USB OTG FS core for Device mode.
  * @param  cfg: pointer to USB_OTG_DevInitTypeDef configuration structure
  * @retval None
  */
void USB_OTG_FS_DevInit(USB_OTG_DevInitTypeDef *cfg)
{
  uint32_t i;

  /* Zero all DIEPTXF entries (HAL reference clears these first) */
  for (i = 0U; i < 15U; i++)
  {
    USB_OTG_FS->DIEPTXF[i] = 0U;
  }

  /* VBUS sensing setup for STM32F407:
   * Disable HW VBUS sensing; force VBUS valid internally.
   * Note: GOTGCTL BVALOEN/BVALOVAL are RESERVED on F407 (don't touch). */
  if (cfg->vbus_sensing_enable == 0U)
  {
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
  }
  else
  {
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
  }

  /* Restart the PHY clock */
  *USB_OTG_FS_PCGCCTL = 0U;

  /* Device configuration: Full-Speed */
  USB_OTG_FS_DEV->DCFG |= USB_OTG_DCFG_DSPD; /* DSPD = 3 → Full-Speed */

  /* Flush all Tx FIFOs */
  USB_OTG_FS_FlushTxFIFO(0x10U); /* 0x10 = flush all TX FIFOs */

  /* Flush RxFIFO */
  USB_OTG_FS_FlushRxFIFO();

  /* Clear all pending device interrupt masks */
  USB_OTG_FS_DEV->DIEPMSK  = 0U;
  USB_OTG_FS_DEV->DOEPMSK  = 0U;
  USB_OTG_FS_DEV->DAINT    = 0xFFFFFFFFU;
  USB_OTG_FS_DEV->DAINTMSK = 0U;

  /* Initialize IN endpoints — EP0 uses SNAK only, others use EPDIS|SNAK */
  for (i = 0U; i < cfg->dev_endpoints; i++)
  {
    if (USB_OTG_FS_INEP(i)->DIEPCTL & USB_OTG_DEPCTL_EPENA)
    {
      if (i == 0U)
      {
        USB_OTG_FS_INEP(i)->DIEPCTL = USB_OTG_DEPCTL_SNAK;
      }
      else
      {
        USB_OTG_FS_INEP(i)->DIEPCTL = USB_OTG_DEPCTL_EPDIS | USB_OTG_DEPCTL_SNAK;
      }
    }
    else
    {
      USB_OTG_FS_INEP(i)->DIEPCTL = 0U;
    }

    USB_OTG_FS_INEP(i)->DIEPTSIZ = 0U;
    USB_OTG_FS_INEP(i)->DIEPINT  = 0xFB7FU;
  }

  /* Initialize OUT endpoints — same EP0 exception */
  for (i = 0U; i < cfg->dev_endpoints; i++)
  {
    if (USB_OTG_FS_OUTEP(i)->DOEPCTL & USB_OTG_DEPCTL_EPENA)
    {
      if (i == 0U)
      {
        USB_OTG_FS_OUTEP(i)->DOEPCTL = USB_OTG_DEPCTL_SNAK;
      }
      else
      {
        USB_OTG_FS_OUTEP(i)->DOEPCTL = USB_OTG_DEPCTL_EPDIS | USB_OTG_DEPCTL_SNAK;
      }
    }
    else
    {
      USB_OTG_FS_OUTEP(i)->DOEPCTL = 0U;
    }

    USB_OTG_FS_OUTEP(i)->DOEPTSIZ = 0U;
    USB_OTG_FS_OUTEP(i)->DOEPINT  = 0xFB7FU;
  }

  /* Disable TxFIFO underrun masking (HAL reference does this) */
  USB_OTG_FS_DEV->DIEPMSK &= ~(1U << 8); /* TXFURM bit */

  USB_OTG_FS_DEV->DIEPEMPMSK = 0U;

  /* Setup FIFO sizes: RxFIFO = 128 words, EP0 Tx = 64 words, EP1 Tx = 128 words */
  USB_OTG_FS_SetRxFIFO(128U);
  USB_OTG_FS_SetTxFIFO(0U, 64U);
  USB_OTG_FS_SetTxFIFO(1U, 128U);

  /* Disable all interrupts first */
  USB_OTG_FS->GINTMSK = 0U;

  /* Clear all pending global interrupts */
  USB_OTG_FS->GINTSTS = 0xBFFFFFFFU;

  /* Enable Device mode endpoint interrupt masks */
  USB_OTG_FS_DEV->DIEPMSK |= USB_OTG_DEPINT_XFRC
                            | USB_OTG_DEPINT_TOC;

  USB_OTG_FS_DEV->DOEPMSK |= USB_OTG_DEPINT_XFRC
                            | USB_OTG_DEPINT_STUP;

  /* Enable core interrupts for Device mode */
  USB_OTG_FS->GINTMSK = USB_OTG_GINTSTS_USBRST
                       | USB_OTG_GINTSTS_ENUMDNE
                       | USB_OTG_GINTSTS_IEPINT
                       | USB_OTG_GINTSTS_OEPINT
                       | USB_OTG_GINTSTS_USBSUSP
                       | USB_OTG_GINTSTS_RXFLVL
                       | USB_OTG_GINTSTS_WKUINT;

  /* Enable global interrupt */
  USB_OTG_FS_EnableGlobalInt();
}

/**
  * @brief  Enable the USB OTG FS global interrupt in GAHBCFG.
  * @retval None
  */
void USB_OTG_FS_EnableGlobalInt(void)
{
  USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINTMSK;
}

/**
  * @brief  Disable the USB OTG FS global interrupt in GAHBCFG.
  * @retval None
  */
void USB_OTG_FS_DisableGlobalInt(void)
{
  USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_GINTMSK;
}

/**
  * @brief  Set the USB device address.
  * @param  address: device address (0..127)
  * @retval None
  */
void USB_OTG_FS_SetDevAddress(uint8_t address)
{
  USB_OTG_FS_DEV->DCFG &= ~USB_OTG_DCFG_DAD;
  USB_OTG_FS_DEV->DCFG |= ((uint32_t)address << USB_OTG_DCFG_DAD_POS)
                           & USB_OTG_DCFG_DAD;
}

/**
  * @brief  Connect the USB device (remove soft-disconnect).
  * @retval None
  */
void USB_OTG_FS_DevConnect(void)
{
  /* Ensure PHY clock is ungated before modifying device registers */
  *USB_OTG_FS_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

  USB_OTG_FS_DEV->DCTL &= ~USB_OTG_DCTL_SDIS;

  /* Wait ~5ms for host to detect D+ pull-up.
   * At 168 MHz, ~4 cycles/iteration → 210,000 iters ≈ 5 ms */
  for (volatile uint32_t i = 0; i < 210000U; i++) { __NOP(); }
}

/**
  * @brief  Disconnect the USB device (assert soft-disconnect).
  * @retval None
  */
void USB_OTG_FS_DevDisconnect(void)
{
  USB_OTG_FS_DEV->DCTL |= USB_OTG_DCTL_SDIS;

  /* Wait ~5ms for host to detect disconnect */
  for (volatile uint32_t i = 0; i < 210000U; i++) { __NOP(); }
}


/* ======================== FIFO Functions =================================== */

/**
  * @brief  Set Receive FIFO depth (in 32-bit words).
  * @param  size: FIFO depth in words
  * @retval None
  */
void USB_OTG_FS_SetRxFIFO(uint16_t size)
{
  USB_OTG_FS->GRXFSIZ = size;
}

/**
  * @brief  Set Transmit FIFO depth for a given FIFO number.
  * @param  fifo: FIFO number (0 = EP0, 1..3 = EPx)
  * @param  size: FIFO depth in 32-bit words
  * @retval None
  */
void USB_OTG_FS_SetTxFIFO(uint8_t fifo, uint16_t size)
{
  uint32_t tx_offset;
  uint8_t i;

  /* Calculate FIFO start address from GRXFSIZ + sum of previous TxFIFOs */
  tx_offset = USB_OTG_FS->GRXFSIZ;

  if (fifo == 0U)
  {
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | tx_offset;
  }
  else
  {
    tx_offset += (USB_OTG_FS->DIEPTXF0_HNPTXFSIZ >> 16);
    for (i = 0U; i < (fifo - 1U); i++)
    {
      tx_offset += (USB_OTG_FS->DIEPTXF[i] >> 16);
    }
    USB_OTG_FS->DIEPTXF[fifo - 1U] = ((uint32_t)size << 16) | tx_offset;
  }
}

/**
  * @brief  Flush a Transmit FIFO.
  * @param  fifo_num: FIFO number (0..3, or 0x10 for all)
  * @retval None
  */
void USB_OTG_FS_FlushTxFIFO(uint8_t fifo_num)
{
  volatile uint32_t count = 0U;

  /* Wait for AHB master IDLE state */
  do {
    if (++count > USB_OTG_TIMEOUT) { return; }
  } while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  count = 0U;
  USB_OTG_FS->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH |
                          ((uint32_t)fifo_num << USB_OTG_GRSTCTL_TXFNUM_POS));

  do {
    if (++count > USB_OTG_TIMEOUT) { return; }
  } while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0U);
}

/**
  * @brief  Flush the entire Receive FIFO.
  * @retval None
  */
void USB_OTG_FS_FlushRxFIFO(void)
{
  volatile uint32_t count = 0U;

  /* Wait for AHB master IDLE state */
  do {
    if (++count > USB_OTG_TIMEOUT) { return; }
  } while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  count = 0U;
  USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

  do {
    if (++count > USB_OTG_TIMEOUT) { return; }
  } while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) != 0U);
}


/* ======================== Endpoint Functions ================================ */

/**
  * @brief  Open and configure an endpoint.
  * @param  ep: pointer to endpoint descriptor
  * @retval None
  */
void USB_OTG_FS_EPOpen(USB_OTG_EPTypeDef *ep)
{
  uint32_t mpsiz = ep->maxpacket & USB_OTG_DEPCTL_MPSIZ;

  /* EP0 uses encoded MPSIZ values in DEPCTL: 64-byte packets are encoded as 0. */
  if (ep->num == 0U)
  {
    switch (ep->maxpacket)
    {
    case 64U: mpsiz = 0U; break;
    case 32U: mpsiz = 1U; break;
    case 16U: mpsiz = 2U; break;
    case 8U:  mpsiz = 3U; break;
    default:  mpsiz = 0U; break;
    }
  }

  if (ep->is_in)
  {
    /* Enable endpoint interrupt */
    USB_OTG_FS_DEV->DAINTMSK |= (1U << ep->num);

    /* If the endpoint is not already active, configure it */
    if (!(USB_OTG_FS_INEP(ep->num)->DIEPCTL & USB_OTG_DEPCTL_USBAEP))
    {
      USB_OTG_FS_INEP(ep->num)->DIEPCTL |=
        mpsiz |
        ((uint32_t)ep->type << USB_OTG_DEPCTL_EPTYP_POS) |
        ((uint32_t)ep->tx_fifo_num << USB_OTG_DEPCTL_TXFNUM_POS) |
        USB_OTG_DEPCTL_SD0PID |
        USB_OTG_DEPCTL_USBAEP;
    }
  }
  else /* OUT endpoint */
  {
    /* Enable endpoint interrupt */
    USB_OTG_FS_DEV->DAINTMSK |= (1U << (ep->num + 16U));

    if (!(USB_OTG_FS_OUTEP(ep->num)->DOEPCTL & USB_OTG_DEPCTL_USBAEP))
    {
      USB_OTG_FS_OUTEP(ep->num)->DOEPCTL |=
        mpsiz |
        ((uint32_t)ep->type << USB_OTG_DEPCTL_EPTYP_POS) |
        USB_OTG_DEPCTL_SD0PID |
        USB_OTG_DEPCTL_USBAEP;
    }
  }
}

/**
  * @brief  Deactivate an endpoint.
  * @param  ep: pointer to endpoint descriptor
  * @retval None
  */
void USB_OTG_FS_EPClose(USB_OTG_EPTypeDef *ep)
{
  if (ep->is_in)
  {
    USB_OTG_FS_DEV->DAINTMSK &= ~(1U << ep->num);
    USB_OTG_FS_INEP(ep->num)->DIEPCTL &= ~USB_OTG_DEPCTL_USBAEP;
  }
  else
  {
    USB_OTG_FS_DEV->DAINTMSK &= ~(1U << (ep->num + 16U));
    USB_OTG_FS_OUTEP(ep->num)->DOEPCTL &= ~USB_OTG_DEPCTL_USBAEP;
  }
}

/**
  * @brief  Start a transfer on a non-zero endpoint.
  * @param  ep: pointer to endpoint descriptor (num, is_in, xfer_len, maxpacket set)
  * @retval None
  */
void USB_OTG_FS_EPStartXfer(USB_OTG_EPTypeDef *ep)
{
  if (ep->is_in)
  {
    /* Program transfer size and packet count */
    uint32_t pktcnt;

    if (ep->xfer_len == 0U)
    {
      pktcnt = 1U;
    }
    else
    {
      pktcnt = (ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket;
    }

    USB_OTG_FS_INEP(ep->num)->DIEPTSIZ =
      (pktcnt << USB_OTG_DEPTSIZ_PKTCNT_POS) |
      (ep->xfer_len & USB_OTG_DEPTSIZ_XFRSIZ);

    /* Enable endpoint and clear NAK */
    USB_OTG_FS_INEP(ep->num)->DIEPCTL |=
      USB_OTG_DEPCTL_CNAK | USB_OTG_DEPCTL_EPENA;

    /* If transfer length > 0, enable TxFIFO empty interrupt for this EP */
    if (ep->xfer_len > 0U)
    {
      USB_OTG_FS_DEV->DIEPEMPMSK |= (1U << ep->num);
    }
  }
  else /* OUT endpoint */
  {
    uint32_t pktcnt;

    if (ep->xfer_len == 0U)
    {
      pktcnt = 1U;
      ep->xfer_len = ep->maxpacket;
    }
    else
    {
      pktcnt = (ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket;
    }

    USB_OTG_FS_OUTEP(ep->num)->DOEPTSIZ =
      (pktcnt << USB_OTG_DEPTSIZ_PKTCNT_POS) |
      (ep->xfer_len & USB_OTG_DEPTSIZ_XFRSIZ);

    /* Enable endpoint and clear NAK */
    USB_OTG_FS_OUTEP(ep->num)->DOEPCTL |=
      USB_OTG_DEPCTL_CNAK | USB_OTG_DEPCTL_EPENA;
  }
}

/**
  * @brief  Start a transfer on endpoint 0 (special handling for EP0 sizes).
  * @param  ep: pointer to endpoint descriptor
  * @retval None
  */
void USB_OTG_FS_EP0StartXfer(USB_OTG_EPTypeDef *ep)
{
  if (ep->is_in)
  {
    uint32_t len = ep->xfer_len;

    /* EP0 IN: max packet is limited per transfer */
    if (len > ep->maxpacket)
    {
      len = ep->maxpacket;
    }

    USB_OTG_FS_INEP(0)->DIEPTSIZ =
      (1U << USB_OTG_DEPTSIZ_PKTCNT_POS) |
      (len & USB_OTG_DEPTSIZ_XFRSIZ);

    USB_OTG_FS_INEP(0)->DIEPCTL |=
      USB_OTG_DEPCTL_CNAK | USB_OTG_DEPCTL_EPENA;

    if (len > 0U)
    {
      USB_OTG_FS_DEV->DIEPEMPMSK |= (1U << 0);
    }
  }
  else /* OUT */
  {
    uint32_t len = ep->xfer_len;

    /* EP0 OUT data/status phase is not a SETUP phase.
     * Only USB_OTG_FS_EP0_OutStart() should program STUPCNT.
     * For a status OUT ZLP, the core still expects XFRSIZ=maxpacket. */
    if (len == 0U)
    {
      len = ep->maxpacket;
    }
    else if (len > ep->maxpacket)
    {
      len = ep->maxpacket;
    }

    USB_OTG_FS_OUTEP(0)->DOEPTSIZ =
      (1U << USB_OTG_DEPTSIZ_PKTCNT_POS) |
      (len & USB_OTG_DEPTSIZ_XFRSIZ);

    USB_OTG_FS_OUTEP(0)->DOEPCTL |=
      USB_OTG_DEPCTL_CNAK | USB_OTG_DEPCTL_EPENA;
  }
}

/**
  * @brief  Set the STALL condition on an endpoint.
  * @param  ep: pointer to endpoint descriptor
  * @retval None
  */
void USB_OTG_FS_EPSetStall(USB_OTG_EPTypeDef *ep)
{
  if (ep->is_in)
  {
    /* If the endpoint is already enabled, disable it first */
    if (USB_OTG_FS_INEP(ep->num)->DIEPCTL & USB_OTG_DEPCTL_EPENA)
    {
      USB_OTG_FS_INEP(ep->num)->DIEPCTL |= USB_OTG_DEPCTL_EPDIS;
    }
    USB_OTG_FS_INEP(ep->num)->DIEPCTL |= USB_OTG_DEPCTL_STALL;
  }
  else
  {
    USB_OTG_FS_OUTEP(ep->num)->DOEPCTL |= USB_OTG_DEPCTL_STALL;
  }
}

/**
  * @brief  Clear the STALL condition on an endpoint.
  * @param  ep: pointer to endpoint descriptor
  * @retval None
  */
void USB_OTG_FS_EPClearStall(USB_OTG_EPTypeDef *ep)
{
  if (ep->is_in)
  {
    USB_OTG_FS_INEP(ep->num)->DIEPCTL &= ~USB_OTG_DEPCTL_STALL;

    /* Reset data toggle for interrupt and bulk endpoints */
    if (ep->type == USB_EP_TYPE_INTR || ep->type == USB_EP_TYPE_BULK)
    {
      USB_OTG_FS_INEP(ep->num)->DIEPCTL |= USB_OTG_DEPCTL_SD0PID;
    }
  }
  else
  {
    USB_OTG_FS_OUTEP(ep->num)->DOEPCTL &= ~USB_OTG_DEPCTL_STALL;

    if (ep->type == USB_EP_TYPE_INTR || ep->type == USB_EP_TYPE_BULK)
    {
      USB_OTG_FS_OUTEP(ep->num)->DOEPCTL |= USB_OTG_DEPCTL_SD0PID;
    }
  }
}

/**
  * @brief  Read the Device All OUT Endpoints Interrupt register.
  * @retval Active OUT endpoint interrupt bits (masked)
  */
uint32_t USB_OTG_FS_ReadDevAllOutEpItr(void)
{
  return (USB_OTG_FS_DEV->DAINT & USB_OTG_FS_DEV->DAINTMSK) >> 16;
}

/**
  * @brief  Read the Device All IN Endpoints Interrupt register.
  * @retval Active IN endpoint interrupt bits (masked)
  */
uint32_t USB_OTG_FS_ReadDevAllInEpItr(void)
{
  return (USB_OTG_FS_DEV->DAINT & USB_OTG_FS_DEV->DAINTMSK) & 0xFFFFU;
}

/**
  * @brief  Prepare EP0 to receive the next SETUP packet.
  * @retval None
  */
void USB_OTG_FS_EP0_OutStart(void)
{
  /* HAL reference: DOEPTSIZ = PKTCNT(1) | XFRSIZ(3*8=24) | STUPCNT(3)
   * XFRSIZ = 24 allows 3 back-to-back 8-byte SETUP packets.
   * For non-DMA mode, do NOT set CNAK/EPENA here — the caller does that. */
  USB_OTG_FS_OUTEP(0)->DOEPTSIZ = 0U;
  USB_OTG_FS_OUTEP(0)->DOEPTSIZ |= (1U << USB_OTG_DEPTSIZ_PKTCNT_POS);
  USB_OTG_FS_OUTEP(0)->DOEPTSIZ |= (3U * 8U);  /* 24 bytes for 3 SETUP packets */
  USB_OTG_FS_OUTEP(0)->DOEPTSIZ |= (3U << USB_OTG_DEPTSIZ_STUPCNT_POS);
}


/* ======================== Packet I/O Functions ============================= */

/**
  * @brief  Write a packet into the Tx FIFO of the given endpoint.
  * @param  src: source buffer pointer
  * @param  ep_num: endpoint number
  * @param  len: number of bytes to write
  * @retval None
  */
void USB_OTG_FS_WritePacket(uint8_t *src, uint8_t ep_num, uint16_t len)
{
  uint32_t word_count = ((uint32_t)len + 3U) / 4U;
  __IO uint32_t *fifo = USB_OTG_FS_FIFO(ep_num);
  uint32_t i;

  for (i = 0U; i < word_count; i++)
  {
    uint32_t remaining = len - (i * 4U);
    uint32_t word = 0U;

    if (remaining >= 4U)
    {
      word  = (uint32_t)src[0];
      word |= (uint32_t)src[1] << 8;
      word |= (uint32_t)src[2] << 16;
      word |= (uint32_t)src[3] << 24;
      src += 4U;
    }
    else
    {
      for (uint32_t j = 0U; j < remaining; j++)
      {
        word |= (uint32_t)src[j] << (8U * j);
      }
    }

    *fifo = word;
  }
}

/**
  * @brief  Read a packet from the Rx FIFO.
  * @param  dest: destination buffer pointer
  * @param  len: number of bytes to read
  * @retval None
  */
void USB_OTG_FS_ReadPacket(uint8_t *dest, uint16_t len)
{
  uint32_t word_count = ((uint32_t)len + 3U) / 4U;
  __IO uint32_t *fifo = USB_OTG_FS_FIFO(0U);
  uint32_t i;

  for (i = 0U; i < word_count; i++)
  {
    uint32_t word = *fifo;

    /* Safe unaligned store — only write valid bytes in last word */
    uint32_t remaining = len - (i * 4U);
    if (remaining >= 4U)
    {
      dest[0] = (uint8_t)(word & 0xFFU);
      dest[1] = (uint8_t)((word >> 8)  & 0xFFU);
      dest[2] = (uint8_t)((word >> 16) & 0xFFU);
      dest[3] = (uint8_t)((word >> 24) & 0xFFU);
      dest += 4U;
    }
    else
    {
      for (uint32_t j = 0U; j < remaining; j++)
      {
        dest[j] = (uint8_t)((word >> (8U * j)) & 0xFFU);
      }
    }
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
