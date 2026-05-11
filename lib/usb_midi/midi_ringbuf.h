/**
 ******************************************************************************
 * @file    midi_ringbuf.h
 * @brief   Lock-free single-producer/single-consumer ring buffer for USB-MIDI
 *          event packets (4 bytes each).
 *
 * @details Producer: OTG_FS_IRQHandler (ISR context)
 *          Consumer: vMidiTask (FreeRTOS task context)
 *          No mutex required — head written only by ISR, tail only by task.
 ******************************************************************************
 */

#ifndef MIDI_RINGBUF_H
#define MIDI_RINGBUF_H

#include <stdint.h>

#define MIDI_RING_SIZE 32U /* Must be power of 2 */
#define MIDI_RING_MASK (MIDI_RING_SIZE - 1U)

typedef struct {
  uint8_t buf[MIDI_RING_SIZE][4];
  volatile uint32_t head; /* Written by producer (ISR)  */
  volatile uint32_t tail; /* Written by consumer (task) */
} MidiRingBuf;

static inline void midi_ring_init(MidiRingBuf *rb) {
  rb->head = 0U;
  rb->tail = 0U;
}

static inline uint32_t midi_ring_count(const MidiRingBuf *rb) {
  return (rb->head - rb->tail) & MIDI_RING_MASK;
}

static inline int midi_ring_is_empty(const MidiRingBuf *rb) {
  return rb->head == rb->tail;
}

static inline int midi_ring_is_full(const MidiRingBuf *rb) {
  return ((rb->head + 1U) & MIDI_RING_MASK) == rb->tail;
}

/**
 * @brief Push a 4-byte MIDI event packet (ISR context).
 * @retval 0 on success, -1 if full.
 */
static inline int midi_ring_push(MidiRingBuf *rb, const uint8_t pkt[4]) {
  if (midi_ring_is_full(rb)) {
    return -1;
  }
  uint32_t h = rb->head;
  rb->buf[h][0] = pkt[0];
  rb->buf[h][1] = pkt[1];
  rb->buf[h][2] = pkt[2];
  rb->buf[h][3] = pkt[3];
  __asm volatile(
      "dmb" ::
          : "memory"); /* Ensure data is written before head advances */
  rb->head = (h + 1U) & MIDI_RING_MASK;
  return 0;
}

/**
 * @brief Pop a 4-byte MIDI event packet (task context).
 * @retval 0 on success, -1 if empty.
 */
static inline int midi_ring_pop(MidiRingBuf *rb, uint8_t pkt[4]) {
  if (midi_ring_is_empty(rb)) {
    return -1;
  }
  uint32_t t = rb->tail;
  pkt[0] = rb->buf[t][0];
  pkt[1] = rb->buf[t][1];
  pkt[2] = rb->buf[t][2];
  pkt[3] = rb->buf[t][3];
  __asm volatile("dmb" ::
                     : "memory"); /* Ensure data is read before tail advances */
  rb->tail = (t + 1U) & MIDI_RING_MASK;
  return 0;
}

#endif /* MIDI_RINGBUF_H */
