/**
 * @file tasks.h
 *
 * @brief 
 *
 * @details 
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef TASKS_H_
#define TASKS_H_

#include <hooks.h>
#include <semphr.h>

// Task stack sizes
/// @brief Stack size for the sample button task in bytes
#define SAMPLE_TASK_STACK_SIZE 64

/// @brief Stack size for the step button task in bytes
#define STEP_TASK_STACK_SIZE 64

/// @brief Stack size for the blink task in bytes
#define BLINK_TASK_STACK_SIZE 64

/// @brief Stack size for the animation task in bytes
#define ANIMATION_TASK_STACK_SIZE 64

// Task priority levels
/// @brief Priority level for the sample button task (2 = medium)
#define SAMPLE_TASK_PRIORITY 2

/// @brief Priority level for the step button task (2 = medium)
#define STEP_TASK_PRIORITY 2

/// @brief Priority level for the blink task (1 = low)
#define BLINK_TASK_PRIORITY 1

/// @brief Priority level for the animation task (1 = low)
#define ANIMATION_TASK_PRIORITY 1

/// @brief Binary semaphore for button events
StaticSemaphore_t xButtonSemaphoreStatic;

/// @brief Binary semaphore handle for button events
SemaphoreHandle_t xButtonSemaphoreHandle;

// --- Sample buttons Task ---
/// @brief Stack memory allocation for the sample button task stored in CCM
StackType_t sampleButtonTaskStack[SAMPLE_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the sample button task stored in CCM
StaticTask_t sampleButtonTaskBuffer CCM_RAM;

/// @brief Handle for the sample button task
TaskHandle_t sampleButtonTaskHandle;

// --- Step buttons Task ---
/// @brief Stack memory allocation for the stepbutton task stored in CCM
StackType_t stepButtonTaskStack[STEP_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the step button task stored in CCM
StaticTask_t stepButtonTaskBuffer CCM_RAM;

/// @brief Handle for the step button task
TaskHandle_t stepButtonTaskHandle;

// --- Blink Task ---
/// @brief Stack memory allocation for the blink task stored in CCM
StackType_t blinkTaskStack[BLINK_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the blink task stored in CCM
StaticTask_t blinkTaskBuffer CCM_RAM;

/// @brief Handle for the blink task
TaskHandle_t blinkTaskHandle;

// --- Animation Task ---
/// @brief Stack memory allocation for the playback task stored in CCM
StackType_t animationTaskStack[ANIMATION_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the playback task stored in CCM
StaticTask_t animationTaskBuffer CCM_RAM;

/// @brief Handle for the playback task
TaskHandle_t animationTaskHandle;

#endif // TASKS_H_
