/**
  ******************************************************************************
  * @file    stm32f4xx_usb_otg_fs.h
  * @author  Radar2000
  * @version V1.0.0
  * @date    20-April-2026
  * @brief   This file contains all the functions prototypes for the USB OTG FS
  *          firmware library (SPL-style driver for Device mode).
  ******************************************************************************
  * @attention
  *
  * This driver provides a Standard Peripheral Library style abstraction over
  * the STM32F4xx USB OTG Full-Speed peripheral, operating in Device mode.
  * Register definitions come from the CMSIS stm32f4xx.h header.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_USB_OTG_FS_H
#define __STM32F4xx_USB_OTG_FS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup USB_OTG_FS
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup USB_OTG_FS_Exported_Types
  * @{
  */

/**
  * @brief  USB OTG FS Device Speed
  */
typedef enum
{
  USB_OTG_SPEED_FULL = 3  /*!< Full-Speed (USB OTG FS only supports FS) */
} USB_OTG_SpeedTypeDef;

/**
  * @brief  USB Endpoint type
  */
typedef enum
{
  USB_EP_TYPE_CTRL = 0,    /*!< Control endpoint       */
  USB_EP_TYPE_ISOC = 1,    /*!< Isochronous endpoint   */
  USB_EP_TYPE_BULK = 2,    /*!< Bulk endpoint          */
  USB_EP_TYPE_INTR = 3     /*!< Interrupt endpoint     */
} USB_OTG_EPTypeTypeDef;

/**
  * @brief  USB Device Init Structure definition (SPL style)
  */
typedef struct
{
  uint32_t dev_endpoints;           /*!< Number of device endpoints (1..4 for FS) */
  uint32_t speed;                   /*!< Device speed (USB_OTG_SPEED_FULL)        */
  uint32_t ep0_mps;                 /*!< Endpoint 0 max packet size (8,16,32,64)  */
  uint32_t phy_itface;              /*!< PHY interface: 0 = embedded FS PHY       */
  uint32_t Sof_enable;              /*!< Enable Start-of-Frame interrupt          */
  uint32_t low_power_enable;        /*!< Enable low-power mode                    */
  uint32_t vbus_sensing_enable;     /*!< Enable VBUS sensing                      */
  uint32_t use_dedicated_ep1;       /*!< Enable dedicated EP1 interrupt           */
} USB_OTG_DevInitTypeDef;

/**
  * @brief  USB Endpoint descriptor (runtime state)
  */
typedef struct
{
  uint8_t  num;            /*!< Endpoint number (0..3)                   */
  uint8_t  is_in;          /*!< Direction: 1 = IN, 0 = OUT              */
  uint8_t  is_stall;       /*!< Stall condition flag                    */
  uint8_t  type;           /*!< Endpoint type (USB_EP_TYPE_xxx)         */
  uint16_t maxpacket;      /*!< Max packet size                         */
  uint8_t  data_pid_start; /*!< Initial data PID (0 or 1)               */
  uint8_t  *xfer_buff;     /*!< Pointer to transfer buffer              */
  uint32_t xfer_len;       /*!< Total transfer length                   */
  uint32_t xfer_count;     /*!< Transfer count (bytes transferred)      */
  uint32_t tx_fifo_num;    /*!< TX FIFO number (IN endpoints only)      */
} USB_OTG_EPTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup USB_OTG_FS_Exported_Constants
  * @{
  */

/** @defgroup USB_OTG_FS_Core
  * @{
  */
#define USB_OTG_FS_MAX_EP_NUM        4U   /*!< Max endpoints for OTG FS (0..3)   */
#define USB_OTG_FS_MAX_PACKET_SIZE   64U  /*!< Max packet size for FS            */
#define USB_OTG_FS_TOTAL_FIFO_SIZE   1280U /*!< Total FIFO RAM in bytes (320x32b) */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_GAHBCFG_bits
  * @{
  */
#define USB_OTG_GAHBCFG_GINTMSK     ((uint32_t)0x00000001)  /*!< Global interrupt mask       */
#define USB_OTG_GAHBCFG_TXFELVL     ((uint32_t)0x00000080)  /*!< TxFIFO empty level          */
#define USB_OTG_GAHBCFG_PTXFELVL    ((uint32_t)0x00000100)  /*!< Periodic TxFIFO empty level */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_GUSBCFG_bits
  * @{
  */
#define USB_OTG_GUSBCFG_PHYSEL       ((uint32_t)0x00000040)  /*!< Full-Speed serial transceiver select */
#define USB_OTG_GUSBCFG_FDMOD        ((uint32_t)0x40000000)  /*!< Force device mode           */
#define USB_OTG_GUSBCFG_FHMOD        ((uint32_t)0x20000000)  /*!< Force host mode             */
#define USB_OTG_GUSBCFG_TRDT         ((uint32_t)0x00003C00)  /*!< USB turnaround time         */
#define USB_OTG_GUSBCFG_TRDT_POS     10U
/**
  * @}
  */

/** @defgroup USB_OTG_FS_GRSTCTL_bits
  * @{
  */
#define USB_OTG_GRSTCTL_CSRST        ((uint32_t)0x00000001)  /*!< Core soft reset             */
#define USB_OTG_GRSTCTL_RXFFLSH      ((uint32_t)0x00000010)  /*!< RxFIFO flush                */
#define USB_OTG_GRSTCTL_TXFFLSH      ((uint32_t)0x00000020)  /*!< TxFIFO flush                */
#define USB_OTG_GRSTCTL_TXFNUM       ((uint32_t)0x000007C0)  /*!< TxFIFO number               */
#define USB_OTG_GRSTCTL_TXFNUM_POS   6U
#define USB_OTG_GRSTCTL_AHBIDL       ((uint32_t)0x80000000)  /*!< AHB master idle             */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_GINTSTS_bits  (core interrupt status/mask)
  * @{
  */
#define USB_OTG_GINTSTS_CMOD         ((uint32_t)0x00000001)  /*!< Current mode of operation   */
#define USB_OTG_GINTSTS_MMIS         ((uint32_t)0x00000002)  /*!< Mode mismatch interrupt     */
#define USB_OTG_GINTSTS_OTGINT       ((uint32_t)0x00000004)  /*!< OTG interrupt               */
#define USB_OTG_GINTSTS_SOF          ((uint32_t)0x00000008)  /*!< Start of frame              */
#define USB_OTG_GINTSTS_RXFLVL       ((uint32_t)0x00000010)  /*!< RxFIFO non-empty            */
#define USB_OTG_GINTSTS_USBSUSP      ((uint32_t)0x00000800)  /*!< USB suspend                 */
#define USB_OTG_GINTSTS_USBRST       ((uint32_t)0x00001000)  /*!< USB reset                   */
#define USB_OTG_GINTSTS_ENUMDNE      ((uint32_t)0x00002000)  /*!< Enumeration done            */
#define USB_OTG_GINTSTS_ISOODRP      ((uint32_t)0x00004000)  /*!< Isochronous OUT pkt dropped */
#define USB_OTG_GINTSTS_EOPF         ((uint32_t)0x00008000)  /*!< End of periodic frame       */
#define USB_OTG_GINTSTS_IEPINT       ((uint32_t)0x00040000)  /*!< IN endpoint interrupt       */
#define USB_OTG_GINTSTS_OEPINT       ((uint32_t)0x00080000)  /*!< OUT endpoint interrupt      */
#define USB_OTG_GINTSTS_IISOIXFR     ((uint32_t)0x00100000)  /*!< Incomplete isoc IN xfer     */
#define USB_OTG_GINTSTS_IPXFR        ((uint32_t)0x00200000)  /*!< Incomplete periodic xfer    */
#define USB_OTG_GINTSTS_SRQINT       ((uint32_t)0x40000000)  /*!< Session request/new session */
#define USB_OTG_GINTSTS_WKUINT       ((uint32_t)0x80000000)  /*!< Resume/remote wakeup        */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_GCCFG_bits
  * @{
  */
#define USB_OTG_GCCFG_PWRDWN         ((uint32_t)0x00010000)  /*!< Power down                  */
#define USB_OTG_GCCFG_VBUSASEN       ((uint32_t)0x00040000)  /*!< VBUS A-device sensing enable*/
#define USB_OTG_GCCFG_VBUSBSEN       ((uint32_t)0x00080000)  /*!< VBUS B-device sensing enable*/
#define USB_OTG_GCCFG_SOFOUTEN       ((uint32_t)0x00100000)  /*!< SOF output enable           */
#define USB_OTG_GCCFG_NOVBUSSENS     ((uint32_t)0x00200000)  /*!< VBUS sensing disable        */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_DCFG_bits
  * @{
  */
#define USB_OTG_DCFG_DSPD            ((uint32_t)0x00000003)  /*!< Device speed                */
#define USB_OTG_DCFG_NZLSOHSK        ((uint32_t)0x00000004)  /*!< Non-zero-length status OUT  */
#define USB_OTG_DCFG_DAD             ((uint32_t)0x000007F0)  /*!< Device address              */
#define USB_OTG_DCFG_DAD_POS         4U
#define USB_OTG_DCFG_PFIVL           ((uint32_t)0x00001800)  /*!< Periodic frame interval     */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_DCTL_bits
  * @{
  */
#define USB_OTG_DCTL_RWUSIG          ((uint32_t)0x00000001)  /*!< Remote wakeup signaling     */
#define USB_OTG_DCTL_SDIS            ((uint32_t)0x00000002)  /*!< Soft disconnect             */
#define USB_OTG_DCTL_GINSTS          ((uint32_t)0x00000004)  /*!< Global IN NAK status        */
#define USB_OTG_DCTL_GONSTS          ((uint32_t)0x00000008)  /*!< Global OUT NAK status       */
#define USB_OTG_DCTL_SGINAK          ((uint32_t)0x00000080)  /*!< Set global IN NAK           */
#define USB_OTG_DCTL_CGINAK          ((uint32_t)0x00000100)  /*!< Clear global IN NAK         */
#define USB_OTG_DCTL_SGONAK          ((uint32_t)0x00000200)  /*!< Set global OUT NAK          */
#define USB_OTG_DCTL_CGONAK          ((uint32_t)0x00000400)  /*!< Clear global OUT NAK        */
#define USB_OTG_DCTL_POPRGDNE        ((uint32_t)0x00000800)  /*!< Power-on programming done   */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_DSTS_bits
  * @{
  */
#define USB_OTG_DSTS_SUSPSTS         ((uint32_t)0x00000001)  /*!< Suspend status              */
#define USB_OTG_DSTS_ENUMSPD         ((uint32_t)0x00000006)  /*!< Enumerated speed            */
#define USB_OTG_DSTS_ENUMSPD_POS     1U
#define USB_OTG_DSTS_FNSOF           ((uint32_t)0x003FFF00)  /*!< Frame number of received SOF*/
/**
  * @}
  */

/** @defgroup USB_OTG_FS_DIEPCTLx_DOEPCTLx_bits
  * @{
  */
#define USB_OTG_DEPCTL_MPSIZ         ((uint32_t)0x000007FF)  /*!< Max packet size             */
#define USB_OTG_DEPCTL_USBAEP        ((uint32_t)0x00008000)  /*!< USB active endpoint         */
#define USB_OTG_DEPCTL_NAKSTS        ((uint32_t)0x00020000)  /*!< NAK status                  */
#define USB_OTG_DEPCTL_EPTYP         ((uint32_t)0x000C0000)  /*!< Endpoint type               */
#define USB_OTG_DEPCTL_EPTYP_POS     18U
#define USB_OTG_DEPCTL_STALL         ((uint32_t)0x00200000)  /*!< STALL handshake             */
#define USB_OTG_DEPCTL_TXFNUM        ((uint32_t)0x03C00000)  /*!< TxFIFO number (IN eps)      */
#define USB_OTG_DEPCTL_TXFNUM_POS    22U
#define USB_OTG_DEPCTL_CNAK          ((uint32_t)0x04000000)  /*!< Clear NAK                   */
#define USB_OTG_DEPCTL_SNAK          ((uint32_t)0x08000000)  /*!< Set NAK                     */
#define USB_OTG_DEPCTL_SD0PID        ((uint32_t)0x10000000)  /*!< Set DATA0 PID               */
#define USB_OTG_DEPCTL_SD1PID        ((uint32_t)0x10000000)  /*!< Set DATA1 PID (alias)       */
#define USB_OTG_DEPCTL_SODDFRM       ((uint32_t)0x20000000)  /*!< Set odd frame               */
#define USB_OTG_DEPCTL_EPDIS         ((uint32_t)0x40000000)  /*!< Endpoint disable            */
#define USB_OTG_DEPCTL_EPENA         ((uint32_t)0x80000000)  /*!< Endpoint enable             */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_DIEPINTx_DOEPINTx_bits
  * @{
  */
#define USB_OTG_DEPINT_XFRC          ((uint32_t)0x00000001)  /*!< Transfer completed          */
#define USB_OTG_DEPINT_EPDISD        ((uint32_t)0x00000002)  /*!< Endpoint disabled           */
#define USB_OTG_DEPINT_TOC           ((uint32_t)0x00000008)  /*!< Timeout condition (IN)      */
#define USB_OTG_DEPINT_ITTXFE        ((uint32_t)0x00000010)  /*!< IN token when TxFIFO empty  */
#define USB_OTG_DEPINT_INEPNE        ((uint32_t)0x00000040)  /*!< IN endpoint NAK effective   */
#define USB_OTG_DEPINT_TXFE          ((uint32_t)0x00000080)  /*!< Transmit FIFO empty         */
#define USB_OTG_DEPINT_STUP          ((uint32_t)0x00000008)  /*!< SETUP phase done (OUT EP0)  */
#define USB_OTG_DEPINT_OTEPDIS       ((uint32_t)0x00000010)  /*!< OUT token when EP disabled  */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_DIEPTSIZx_DOEPTSIZx_bits
  * @{
  */
#define USB_OTG_DEPTSIZ_XFRSIZ       ((uint32_t)0x0007FFFF)  /*!< Transfer size               */
#define USB_OTG_DEPTSIZ_PKTCNT       ((uint32_t)0x1FF80000)  /*!< Packet count                */
#define USB_OTG_DEPTSIZ_PKTCNT_POS   19U
#define USB_OTG_DEPTSIZ_STUPCNT      ((uint32_t)0x60000000)  /*!< SETUP packet count (EP0 OUT)*/
#define USB_OTG_DEPTSIZ_STUPCNT_POS  29U
/**
  * @}
  */

/** @defgroup USB_OTG_FS_PCGCCTL_bits
  * @{
  */
#define USB_OTG_PCGCCTL_STOPCLK      ((uint32_t)0x00000001)  /*!< Stop PHY clock              */
#define USB_OTG_PCGCCTL_GATECLK      ((uint32_t)0x00000002)  /*!< Gate HCLK                   */
/**
  * @}
  */

/** @defgroup USB_OTG_FS_GRXSTSP_bits  (Receive status pop register)
  * @{
  */
#define USB_OTG_GRXSTSP_EPNUM        ((uint32_t)0x0000000F)  /*!< Endpoint number             */
#define USB_OTG_GRXSTSP_BCNT         ((uint32_t)0x00007FF0)  /*!< Byte count                  */
#define USB_OTG_GRXSTSP_BCNT_POS     4U
#define USB_OTG_GRXSTSP_DPID         ((uint32_t)0x00018000)  /*!< Data PID                    */
#define USB_OTG_GRXSTSP_PKTSTS       ((uint32_t)0x001E0000)  /*!< Packet status               */
#define USB_OTG_GRXSTSP_PKTSTS_POS   17U

/** GRXSTSP packet status values */
#define STS_GOUT_NAK                  1U  /*!< Global OUT NAK              */
#define STS_DATA_UPDT                 2U  /*!< OUT data packet received    */
#define STS_XFER_COMP                 3U  /*!< OUT transfer completed      */
#define STS_SETUP_COMP                4U  /*!< SETUP transaction completed */
#define STS_SETUP_UPDT                6U  /*!< SETUP data packet received  */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/

/** @defgroup USB_OTG_FS_Exported_Macros
  * @{
  */

/** @brief  Access USB OTG FS Device registers */
#define USB_OTG_FS_DEV      ((USB_OTG_DeviceTypeDef *)        \
  ((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))

/** @brief  Access USB OTG FS IN endpoint registers */
#define USB_OTG_FS_INEP(ep) ((USB_OTG_INEndpointTypeDef *)    \
  ((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + \
   ((ep) * USB_OTG_EP_REG_SIZE)))

/** @brief  Access USB OTG FS OUT endpoint registers */
#define USB_OTG_FS_OUTEP(ep) ((USB_OTG_OUTEndpointTypeDef *)  \
  ((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + \
   ((ep) * USB_OTG_EP_REG_SIZE)))

/** @brief  Access USB OTG FS endpoint FIFO (push/pop via 32-bit writes/reads) */
#define USB_OTG_FS_FIFO(ep) ((__IO uint32_t *)                 \
  ((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE +     \
   ((ep) * USB_OTG_FIFO_SIZE)))

/** @brief  Access USB OTG FS PCGCCTL register */
#define USB_OTG_FS_PCGCCTL  ((__IO uint32_t *)                 \
  ((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup USB_OTG_FS_Exported_Functions
  * @{
  */

/** @defgroup USB_OTG_FS_Core_Functions
  * @{
  */
void USB_OTG_FS_CoreInit(USB_OTG_DevInitTypeDef *cfg);
void USB_OTG_FS_CoreReset(void);
void USB_OTG_FS_DevInit(USB_OTG_DevInitTypeDef *cfg);
void USB_OTG_FS_EnableGlobalInt(void);
void USB_OTG_FS_DisableGlobalInt(void);
void USB_OTG_FS_SetDevAddress(uint8_t address);
void USB_OTG_FS_DevConnect(void);
void USB_OTG_FS_DevDisconnect(void);
/**
  * @}
  */

/** @defgroup USB_OTG_FS_FIFO_Functions
  * @{
  */
void USB_OTG_FS_SetRxFIFO(uint16_t size);
void USB_OTG_FS_SetTxFIFO(uint8_t fifo, uint16_t size);
void USB_OTG_FS_FlushTxFIFO(uint8_t fifo_num);
void USB_OTG_FS_FlushRxFIFO(void);
/**
  * @}
  */

/** @defgroup USB_OTG_FS_EP_Functions
  * @{
  */
void USB_OTG_FS_EPOpen(USB_OTG_EPTypeDef *ep);
void USB_OTG_FS_EPClose(USB_OTG_EPTypeDef *ep);
void USB_OTG_FS_EPStartXfer(USB_OTG_EPTypeDef *ep);
void USB_OTG_FS_EP0StartXfer(USB_OTG_EPTypeDef *ep);
void USB_OTG_FS_EPSetStall(USB_OTG_EPTypeDef *ep);
void USB_OTG_FS_EPClearStall(USB_OTG_EPTypeDef *ep);
uint32_t USB_OTG_FS_ReadDevAllOutEpItr(void);
uint32_t USB_OTG_FS_ReadDevAllInEpItr(void);
void USB_OTG_FS_EP0_OutStart(void);
void USB_OTG_FS_WritePacket(uint8_t *src, uint8_t ep_num, uint16_t len);
void USB_OTG_FS_ReadPacket(uint8_t *dest, uint16_t len);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_USB_OTG_FS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
