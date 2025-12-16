/**
 * @file hooks.c
 *
 * @brief FreeRTOS Hook Functions for STM32F4 Application.
 *
 * This file implements the hook functions required by FreeRTOS for system-level
 * event handling, including memory allocation failures, stack overflows, idle
 * tasks, and timer tasks.
 *
 * @details
 * - Implements required FreeRTOS hook functions.
 * - Static memory allocation for idle and timer tasks using Core Coupled Memory
 * (CCM).
 * 
 * Dependencies:
 * - STM32F4xx Standard Peripheral Library
 * - FreeRTOS Kernel
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#include <hooks.h>

#if (vAppTickHook == 1)
void vApplicationTickHook(void) {}
#endif

void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
}

#if (vAppIdleHook == 1)
void vApplicationIdleHook(void) {}
#endif

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
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
