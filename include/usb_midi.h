/**
 ******************************************************************************
 * @file    usb_midi.h
 * @brief   Application-level USB MIDI interface — init, FreeRTOS task, and
 *          MIDI note/CC callbacks for the Progtomata audio engine.
 ******************************************************************************
 */

#ifndef USB_MIDI_H
#define USB_MIDI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  Initialise GPIO (PA11/PA12), NVIC, and the USB device stack.
 *         Call once from main() before the scheduler starts.
 */
void USB_MIDI_Init(void);

/**
 * @brief  FreeRTOS task that blocks on a semaphore and processes
 *         incoming USB-MIDI event packets from the ring buffer.
 */
void vMidiDeviceTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* USB_MIDI_H */
