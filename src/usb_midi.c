/**
  ******************************************************************************
  * @file    usb_midi.c
  * @brief   Application-layer USB MIDI glue for the Progtomata system.
  *
  * @details Provides:
  *  - GPIO / RCC / NVIC initialisation for USB OTG FS (PA11, PA12)
  *  - OTG_FS_IRQHandler (calls into the USB device core, then yields)
  *  - USBD_MIDI_RxReadyCallback override (signals FreeRTOS semaphore)
  *  - vMidiTask — consumes ring buffer and maps MIDI events to audio engine
  ******************************************************************************
  */

#include "usb_midi.h"
#include "usbd_core.h"
#include "usbd_midi.h"
#include "midi_ringbuf.h"

#include <progtomata_system.h>
#include <trice.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

/* Semaphore defined in tasks.h (included only by main.c) */
extern SemaphoreHandle_t  xMidiSemaphoreHandle;
extern StaticSemaphore_t  xMidiSemaphoreStatic;

/** Global USB device handle */
static USBD_HandleTypeDef hUSBD;

/** Context-switch flag shared between IRQ and callback */
static BaseType_t xMidiHigherPrioTaskWoken;

static void USB_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio;

  /* Enable clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);

  /* PA9 = OTG_FS_VBUS */
  gpio.GPIO_Pin   = GPIO_Pin_9;
  gpio.GPIO_Mode  = GPIO_Mode_IN;
  gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);

  /* PA10 = OTG_FS_ID, PA11 = OTG_FS_DM, PA12 = OTG_FS_DP */
  gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  gpio.GPIO_Mode  = GPIO_Mode_AF;
  gpio.GPIO_Speed = GPIO_Speed_100MHz;  /* USB FS requires fast GPIO for 12 Mbps */
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_OTG_FS);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG_FS);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG_FS);

  /* NVIC: OTG_FS_IRQn (IRQ 67).
   * Use CMSIS NVIC_SetPriority directly: SPL's NVIC_Init reads AIRCR PRIGROUP
   * (which is 0 at reset on Cortex-M4, all 4 bits are preemption) and computes
   * a bogus shift that ends up writing priority 0 — higher than
   * configMAX_SYSCALL_INTERRUPT_PRIORITY, which conflicts with the FreeRTOS API
   * priority assertion on the first ISR-context queue/semaphore call.
   * CMSIS NVIC_SetPriority correctly shifts by (8 - __NVIC_PRIO_BITS). */
  NVIC_SetPriority(OTG_FS_IRQn, 6U);  /* numerically > 5 = below syscall ceiling */
  NVIC_EnableIRQ(OTG_FS_IRQn);
}

void OTG_FS_IRQHandler(void)
{
  xMidiHigherPrioTaskWoken = pdFALSE;

  USBD_IRQHandler(&hUSBD);

  portYIELD_FROM_ISR(xMidiHigherPrioTaskWoken);
}

void USBD_MIDI_RxReadyCallback(void)
{
  xSemaphoreGiveFromISR(xMidiSemaphoreHandle, &xMidiHigherPrioTaskWoken);
}

void USB_MIDI_Init(void)
{
  USB_GPIO_Init();
  USBD_Init(&hUSBD);
}

void vMidiTask(void *pvParameters)
{
  (void)pvParameters;

  MidiRingBuf *ring = USBD_MIDI_GetRingBuf();
  uint8_t pkt[4];

  for (;;)
  {
    /* Block until the USB ISR signals new MIDI data */
    xSemaphoreTake(xMidiSemaphoreHandle, portMAX_DELAY);

    /* Drain all available packets */
    while (midi_ring_pop(ring, pkt) == 0)
    {
      /* Verify USB MIDI data exchange — log every received packet.
       * pkt[0] = USB-MIDI header (cable + code-index)
       * pkt[1] = MIDI status byte
       * pkt[2] = data1 (e.g. note number)
       * pkt[3] = data2 (e.g. velocity)                                       */
      TRICE(ID(5642), "MIDI rx: %02x %02x %02x %02x\n",
            pkt[0], pkt[1], pkt[2], pkt[3]);
    }
  }
}
