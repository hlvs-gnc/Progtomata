/**
 * @file hooks.c
 *
 * @brief FreeRTOS Hook Functions for STM32F4 Application.
 *
 * @details
 * Hook functions required by FreeRTOS for system-level event handling,
 * including memory allocation failures, stack overflows, idle and timer tasks.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#include <hooks.h>
#include <stm32f4xx.h>
#include <triceUart.h>

/* Emergency busy-wait UART print for fault handlers (no scheduler / no trice
 * deferred). */
static void fault_emit(const char *tag) {
  /* Send a recognizable raw ASCII marker on USART2 (trice port). */
  const char *p = "\r\n*** FAULT: ";
  while (*p) {
    triceTransmitData8UartA((uint8_t)*p++);
  }
  while (*tag) {
    triceTransmitData8UartA((uint8_t)*tag++);
  }
  triceTransmitData8UartA('\r');
  triceTransmitData8UartA('\n');
  for (;;) {
  }
}

void HardFault_Handler(void) {
  fault_emit("HARD");
}
void BusFault_Handler(void) {
  fault_emit("BUS");
}
void UsageFault_Handler(void) {
  fault_emit("USAGE");
}
void MemManage_Handler(void) {
  fault_emit("MEM");
}

#if (vAppTickHook == 1)
void vApplicationTickHook(void) {}
#endif

void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  fault_emit("MALLOC");
}

#if (vAppIdleHook == 1)
void vApplicationIdleHook(void) {}
#endif

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  fault_emit(pcTaskName ? pcTaskName : "STACK");
}

StaticTask_t xIdleTaskTCB CCM_RAM;
StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE] CCM_RAM;

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

StaticTask_t xTimerTaskTCB CCM_RAM;
StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] CCM_RAM;

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
