/**
 * @file hooks.h
 *
 * @brief FreeRTOS Hook Function Declarations for STM32F4 Application.
 *
 * This header file declares hook functions and memory allocations required 
 * by FreeRTOS for system-level operations, such as idle tasks, timer tasks, 
 * and error handling. It supports static memory allocation using Core Coupled 
 * Memory (CCM) for optimized performance on STM32F4 devices.
 *
 * @details
 * - Provides declarations for FreeRTOS hooks including:
 *   - Tick Hook
 *   - Memory Allocation Failure Hook
 *   - Idle Hook
 *   - Stack Overflow Hook
 * - Implements static memory allocation for Idle and Timer tasks using CCM.
 * - Ensures compatibility with FreeRTOS static memory management configuration.
 *
 * Features:
 * - Static memory management with Core Coupled Memory (CCM) for deterministic behavior.
 * - Hook function declarations for robust error handling and debugging.
 * - Optimized for STM32F4-based embedded systems.
 *
 * Dependencies:
 * - STM32F4xx Standard Peripheral Library
 * - FreeRTOS Kernel
 *
 * Hardware:
 * - Board: STM32F4-Discovery or compatible STM32F4 boards.
 *
 * Configuration Requirements:
 * - Enable static memory allocation in FreeRTOSConfig.h:
 *   - configSUPPORT_STATIC_ALLOCATION = 1
 *   - configUSE_IDLE_HOOK = 1
 *   - configUSE_TICK_HOOK = 1
 *   - configCHECK_FOR_STACK_OVERFLOW = 2
 *   - configUSE_TIMERS = 1
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons 
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef HOOKS_H_
#define HOOKS_H_

#include <FreeRTOS.h>
#include <task.h>

// Macro to use CCM (Core Coupled Memory) in STM32F4
#define CCM_RAM __attribute__((section(".ccmram")))

#define vAppTickHook 0
#define vAppIdleHook 0

// Hook function prototypes
void vApplicationTickHook(void);
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName);
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize);

// Memory allocation for Idle Task
extern StaticTask_t xIdleTaskTCB CCM_RAM;
extern StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE] CCM_RAM;

// Memory allocation for Timer Task
extern StaticTask_t xTimerTaskTCB CCM_RAM;
extern StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] CCM_RAM;

#endif  // HOOKS_H_
