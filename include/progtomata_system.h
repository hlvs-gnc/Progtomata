
/**
 * @file progtomata_system.h
 *
 * @brief Header file for system-wide definitions and task configurations.
 *
 * @details This file contains macro definitions, constants, and memory
 * allocation for task stacks and control blocks, using CCM (Core Coupled Memory)
 * for STM32F4 microcontrollers.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons 
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef PROGTOMATA_SYSTEM_H_
#define PROGTOMATA_SYSTEM_H_

/// @brief Macro to use CCM (Core Coupled Memory) in STM32F4
#define CCM_RAM __attribute__((section(".ccmram")))

/// @brief Stack size for the button task in bytes
#define BUTTON_TASK_STACK_SIZE 256

/// @brief Stack size for the blink task in bytes
#define BLINK_TASK_STACK_SIZE 256

/// @brief Initial step size for delay increment and initial delay
/// value in milliseconds
static uint32_t kBlinkStep = 10, kBlinkDelay = 50;

/// @brief Minimum delay for LED blinking in milliseconds
const uint32_t MIN_DELAY = 10;
/// @brief Maximum delay for LED blinking in milliseconds
const uint32_t MAX_DELAY = 250;

/// @brief Stack memory allocation for the button task stored in CCM
StackType_t buttonTaskStack[BUTTON_TASK_STACK_SIZE] CCM_RAM;
/// @brief Task control block (TCB) for the button task stored in CCM
StaticTask_t buttonTaskBuffer CCM_RAM;

/// @brief Stack memory allocation for the blink task stored in CCM
StackType_t blinkTaskStack[BLINK_TASK_STACK_SIZE] CCM_RAM;
/// @brief Task control block (TCB) for the blink task stored in CCM
StaticTask_t blinkTaskBuffer CCM_RAM;

#endif  // PROGTOMATA_SYSTEM_H_
