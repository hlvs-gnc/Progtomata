/**
  ******************************************************************************
  * @file    usbd_desc.h
  * @brief   USB device & configuration descriptors for a USB-MIDI device.
  ******************************************************************************
  */

#ifndef USBD_DESC_H
#define USBD_DESC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

// VID / PID (configurable)
#define USBD_VID                    0x1209U   /* pid.codes open-source VID    */
#define USBD_PID                    0x0001U   /* Test PID — replace for prod  */

// MIDI endpoint addresses
#define MIDI_OUT_EP                 0x01U     /* Bulk OUT (host → device)     */
#define MIDI_IN_EP                  0x81U     /* Bulk IN  (device → host)     */
#define MIDI_PACKET_SIZE            64U       /* Max packet for FS bulk       */

// Descriptor access
const uint8_t *USBD_GetDeviceDescriptor(uint16_t *len);
const uint8_t *USBD_GetConfigDescriptor(uint16_t *len);
const uint8_t *USBD_GetStringDescriptor(uint8_t index, uint16_t *len);
const uint8_t *USBD_GetDeviceQualifier(uint16_t *len);

#ifdef __cplusplus
}
#endif

#endif /* USBD_DESC_H */
