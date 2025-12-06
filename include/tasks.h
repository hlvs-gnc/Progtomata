/**
 * @file tasks.h
 *
 * @brief FreeRTOS task definitions and configurations for the Progtomata system
 *
 * @details This header defines all FreeRTOS tasks used in the system
 * It includes task stack sizes, priority levels, task handles, and
 * semaphore definitions for inter-task synchronization.
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

// Task stack sizes (increased for Tonbandgerät trace hooks)
/// @brief Stack size for the sample button task in bytes
#define SAMPLE_TASK_STACK_SIZE 128

/// @brief Stack size for the step button task in bytes
#define STEP_TASK_STACK_SIZE 128

/// @brief Stack size for the LED task in bytes
#define LEDBLINK_TASK_STACK_SIZE 128

/// @brief Stack size for the animation task in bytes
#define ANIMATION_TASK_STACK_SIZE 128

/// @brief Stack size for the waveform visualization task in bytes
#define WAVEFORM_TASK_STACK_SIZE 128

// Task priority levels
/// @brief Priority level for the sample button task (2 = medium)
#define SAMPLE_TASK_PRIORITY 2

/// @brief Priority level for the step button task (2 = medium)
#define STEP_TASK_PRIORITY 2

/// @brief Priority level for the LED task (1 = low)
#define LEDBLINK_TASK_PRIORITY 1

/// @brief Priority level for the waveform visualization task (1 = low)
#define WAVEFORM_TASK_PRIORITY 1

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

// --- LED Task ---
/// @brief Stack memory allocation for the LED task stored in CCM
StackType_t ledBlinkTaskStack[LEDBLINK_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the LED task stored in CCM
StaticTask_t ledBlinkTaskBuffer CCM_RAM;

/// @brief Handle for the LED task
TaskHandle_t ledBlinkTaskHandle;

// --- Animation Task ---
/// @brief Stack memory allocation for the playback task stored in CCM
StackType_t animationTaskStack[ANIMATION_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the playback task stored in CCM
StaticTask_t animationTaskBuffer CCM_RAM;

// --- Waveform Task ---
/// @brief Stack memory allocation for the waveform task stored in CCM
StackType_t waveformTaskStack[WAVEFORM_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the waveform task stored in CCM
StaticTask_t waveformTaskBuffer CCM_RAM;

/// @brief Handle for the waveform task
TaskHandle_t waveformTaskHandle;

/**
 * @brief
 *
 * Monitors the state of the user button connected to GPIOA Pin 0.
 * If a button press is detected, it resets the LED delay to its
 * minimum value. Designed as a FreeRTOS task that runs continuously.
 *
 * @param[in] p Pointer to task parameters (unused).
 */
void vButtonSampleTask(void *p);

/**
 * @brief
 *
 * @param[in] p Pointer to task parameters (unused).
 */
void vButtonStepTask(void *pvParameters);

/**
 * @brief Controls LED blinking behavior with variable delay.
 *
 * Cycles through four LEDs connected to GPIOD Pins 12, 13, 14, and 15.
 * Adjusts the blinking delay dynamically within defined limits.
 * Runs as a FreeRTOS task, demonstrating time-based scheduling.
 *
 * @param[in] p Pointer to task parameters (unused).
 */
void vLedBlinkTask(void *p);

/**
 * @brief Task to visualize audio waveform on the OLED display.
 *
 * This task runs continuously and displays a real-time waveform
 * visualization of the audio playback buffer on the OLED screen.
 * Updates at approximately 10 Hz to match audio buffer refresh rate.
 *
 * @param[in] pvParameters Pointer to task parameters (unused).
 */
void vWaveformTask(void *pvParameters);

#endif // TASKS_H_
