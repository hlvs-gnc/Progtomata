/**
 ******************************************************************************
 * @file    usbd_midi.c
 * @brief   USB MIDI Streaming class implementation.
 *
 * @details Handles Bulk OUT/IN endpoint lifecycle and incoming MIDI data.
 *          Received 4-byte USB-MIDI event packets are pushed into a lock-free
 *          ring buffer; the application-layer FreeRTOS task consumes them.
 ******************************************************************************
 */

#include "usbd_midi.h"
#include "usbd_desc.h"
#include "stm32f4xx_usb_otg_fs.h"
#include <string.h>

// Private data
static MidiRingBuf midi_ring;                 /* ISR→task ring buffer */
static uint8_t midi_rx_buf[MIDI_PACKET_SIZE]; /* EP1 OUT receive buf  */

// Weak callback (override in application)
__attribute__((weak)) void USBD_MIDI_RxReadyCallback(void) {
  /* Default: no-op.  Application overrides to signal FreeRTOS semaphore. */
}

void USBD_MIDI_Init(USBD_HandleTypeDef *hdev) {
  midi_ring_init(&midi_ring);

  /* ---- Open Bulk OUT endpoint (EP1 OUT) ---- */
  hdev->out_ep[1].num = 1U;
  hdev->out_ep[1].is_in = 0U;
  hdev->out_ep[1].type = USB_EP_TYPE_BULK;
  hdev->out_ep[1].maxpacket = MIDI_PACKET_SIZE;
  hdev->out_ep[1].xfer_buff = midi_rx_buf;
  hdev->out_ep[1].xfer_len = MIDI_PACKET_SIZE;
  hdev->out_ep[1].xfer_count = 0U;
  USB_OTG_FS_EPOpen(&hdev->out_ep[1]);

  /* ---- Open Bulk IN endpoint (EP1 IN) ---- */
  hdev->in_ep[1].num = 1U;
  hdev->in_ep[1].is_in = 1U;
  hdev->in_ep[1].type = USB_EP_TYPE_BULK;
  hdev->in_ep[1].maxpacket = MIDI_PACKET_SIZE;
  hdev->in_ep[1].tx_fifo_num = 1U;
  hdev->in_ep[1].xfer_buff = (void *)0;
  hdev->in_ep[1].xfer_len = 0U;
  hdev->in_ep[1].xfer_count = 0U;
  USB_OTG_FS_EPOpen(&hdev->in_ep[1]);

  /* ---- Arm EP1 OUT to receive the first packet ---- */
  hdev->out_ep[1].xfer_buff = midi_rx_buf;
  hdev->out_ep[1].xfer_len = MIDI_PACKET_SIZE;
  hdev->out_ep[1].xfer_count = 0U;
  USB_OTG_FS_EPStartXfer(&hdev->out_ep[1]);
}

void USBD_MIDI_DeInit(USBD_HandleTypeDef *hdev) {
  USB_OTG_FS_EPClose(&hdev->out_ep[1]);
  USB_OTG_FS_EPClose(&hdev->in_ep[1]);
}

void USBD_MIDI_DataOut(USBD_HandleTypeDef *hdev) {
  uint32_t rx_count = hdev->out_ep[1].xfer_count;
  uint32_t n_events = rx_count / 4U; /* Each USB-MIDI event = 4 bytes */

  for (uint32_t i = 0U; i < n_events; i++) {
    midi_ring_push(&midi_ring, &midi_rx_buf[i * 4U]);
  }

  /* Signal application layer */
  if (n_events > 0U) {
    USBD_MIDI_RxReadyCallback();
  }

  /* Re-arm EP1 OUT for the next transfer */
  hdev->out_ep[1].xfer_buff = midi_rx_buf;
  hdev->out_ep[1].xfer_len = MIDI_PACKET_SIZE;
  hdev->out_ep[1].xfer_count = 0U;
  USB_OTG_FS_EPStartXfer(&hdev->out_ep[1]);
}

MidiRingBuf *USBD_MIDI_GetRingBuf(void) {
  return &midi_ring;
}
