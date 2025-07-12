/**
 * @file core.h
 *
 * @brief Core header file for the functional task instances.
 *
 * @details This file provides the core definitions, macros, and function
 * declarations for the FreeRTOS task instances. It includes the necessary
 * header files, defines the application constants, and declares the functions
 * used in the application.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef CORE_H_
#define CORE_H_

#include <hooks.h>
#include <semphr.h>
#include <stdint.h>

// Mono Audio samples
#include <kick_22050_mono.h>
#include <openhat_22050_mono.h>

#define NUM_SAMPLES 2
#define NUM_STEPS 4
#define STEP_DURATION (BUFFERSIZE / NUM_STEPS)
#define CLIP16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

/// @brief Number of samples in kick mono audio file
#define SOUNDSIZE1 (6918)

/// @brief Number of samples in open hat mono audio file
#define SOUNDSIZE2 (9882)

/// @brief Audio sample identifiers
typedef enum { SAMPLE1 = 0, SAMPLE2 } audioSampleId;

/// @brief Current step index
static uint8_t playHeadStep = 0;
/// @brief Samples already rendered in current beat-step
static uint32_t stepPos = 0;
/// @brief Grid column currently playing
static uint8_t stepIndex = 0;

/// @brief Sample button state - for N samples
static uint16_t sampleButtonState = 0;
/// @brief Step button's state - for N steps
static uint16_t stepButtonState = 0;

/// @brief Binary semaphore for button events
StaticSemaphore_t xButtonSemaphoreStatic;

/// @brief Binary semaphore handle for button events
SemaphoreHandle_t xButtonSemaphoreHandle;

/// @brief Array of audio sample data pointers
static const int16_t *const sampleData[NUM_SAMPLES] = {kick_22050_mono,
                                                       openhat_22050_mono};

static const uint32_t sampleLen[NUM_SAMPLES] = {SOUNDSIZE1, SOUNDSIZE2};

// Task stack sizes
/// @brief Stack size for the sample button task in bytes
#define SAMPLE_BUTTON_TASK_STACK_SIZE 128

/// @brief Stack size for the step button task in bytes
#define STEP_BUTTON_TASK_STACK_SIZE 128

/// @brief Stack size for the blink task in bytes
#define BLINK_TASK_STACK_SIZE 128

/// @brief Stack size for the modify task in bytes
#define MODIFYBUFFER_TASK_STACK_SIZE 512

/// @brief Stack size for the sequencer task in bytes
#define SEQUENCER_TASK_STACK_SIZE 256

/// @brief Stack size for the animation task in bytes
#define ANIMATION_TASK_STACK_SIZE 256

// Task priority levels
/// @brief Priority level for the sample button task (2 = medium)
#define SAMPLE_BUTTON_TASK_PRIORITY 2

/// @brief Priority level for the step button task (2 = medium)
#define STEP_BUTTON_TASK_PRIORITY 2

/// @brief Priority level for the blink task (1 = low)
#define BLINK_TASK_PRIORITY 1

/// @brief Priority level for the playback task (3 = high)
#define PLAYBACK_TASK_PRIORITY 3

/// @brief Priority level for the modify task (4 = very high)
#define MODIFYBUFFER_TASK_PRIORITY 4

/// @brief Priority level for the sequencer task (4 = very high)
#define SEQUENCER_TASK_PRIORITY 4

/// @brief Priority level for the animation task (4 = very high)
#define ANIMATION_TASK_PRIORITY 1

/// @brief Initial step size for delay increment and initial delay
/// value in milliseconds
static uint32_t kBlinkStep = 10;
static uint32_t kBlinkDelay = 50;

/// @brief Minimum delay for LED blinking in milliseconds
extern const uint32_t MIN_BLINK_DELAY;
/// @brief Maximum delay for LED blinking in milliseconds
extern const uint32_t MAX_BLINK_DELAY;

// --- Sample buttons Task ---
/// @brief Stack memory allocation for the sample button task stored in CCM
StackType_t sampleButtonTaskStack[SAMPLE_BUTTON_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the sample button task stored in CCM
StaticTask_t sampleButtonTaskBuffer CCM_RAM;

/// @brief Handle for the sample button task
TaskHandle_t sampleButtonTaskHandle;

// --- Step buttons Task ---
/// @brief Stack memory allocation for the stepbutton task stored in CCM
StackType_t stepButtonTaskStack[STEP_BUTTON_TASK_STACK_SIZE] CCM_RAM;

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

#endif // CORE_H_
