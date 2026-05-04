/**
 ******************************************************************************
 * @file    usbd_midi.h
 * @brief   USB MIDI Streaming class — endpoint management & data reception.
 ******************************************************************************
 */

#ifndef USBD_MIDI_H
#define USBD_MIDI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_core.h"
#include "midi_ringbuf.h"

/**
 * @brief Initialise MIDI class: open Bulk OUT and Bulk IN endpoints,
 *        arm EP1 OUT for the first transfer.
 */
void USBD_MIDI_Init(USBD_HandleTypeDef *hdev);

/**
 * @brief De-initialise MIDI class: close endpoints.
 */
void USBD_MIDI_DeInit(USBD_HandleTypeDef *hdev);

/**
 * @brief Called from the IRQ handler when Bulk OUT transfer completes (EP1).
 *        Parses 4-byte USB-MIDI event packets and pushes them to ring buffer.
 */
void USBD_MIDI_DataOut(USBD_HandleTypeDef *hdev);

/**
 * @brief  Returns the ring buffer pointer for the MIDI task to consume.
 */
MidiRingBuf *USBD_MIDI_GetRingBuf(void);

/**
 * @brief  Weak callback invoked from ISR context when new MIDI data arrives.
 *         Override in application to signal FreeRTOS semaphore.
 */
void USBD_MIDI_RxReadyCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* USBD_MIDI_H */
