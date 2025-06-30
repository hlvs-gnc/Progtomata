
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

#include <stdint.h>
#include <semphr.h>

/// @brief Macro to use CCM (Core Coupled Memory) in STM32F4
#define CCM_RAM __attribute__((section(".ccmram")))

// Task stack sizes
/// @brief Stack size for the sample button task in bytes
#define SAMPLE_BUTTON_TASK_STACK_SIZE 128

/// @brief Stack size for the step button task in bytes
#define STEP_BUTTON_TASK_STACK_SIZE 128

/// @brief Stack size for the blink task in bytes
#define BLINK_TASK_STACK_SIZE 128

/// @brief Stack size for the playback task in bytes
#define PLAYBACK_TASK_STACK_SIZE 256

/// @brief Stack size for the modify task in bytes
#define MODIFYBUFFER_TASK_STACK_SIZE 512

/// @brief Stack size for the sequencer task in bytes
#define SEQUENCER_TASK_STACK_SIZE 256

/// @brief Stack size for the display task in bytes
#define DISPLAY_TASK_STACK_SIZE 256

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

/// @brief Priority level for the display task (4 = very high)
#define DISPLAY_TASK_PRIORITY 1

/// @brief Priority level for the animation task (4 = very high)
#define ANIMATION_TASK_PRIORITY 1

/// @brief Initial step size for delay increment and initial delay
/// value in milliseconds
static uint32_t kBlinkStep = 10;
static uint32_t kBlinkDelay = 50;


/// @brief Minimum delay for LED blinking in milliseconds
const uint32_t MIN_BLINK_DELAY = 10;
/// @brief Maximum delay for LED blinking in milliseconds
const uint32_t MAX_BLINK_DELAY = 250;


/// @brief Number of samples in kick mono audio file
#define SOUNDSIZE1 (6918)

/// @brief Number of samples in kick stereo audio file
#define SOUNDSIZE2 (17700)

/// @brief Number of samples in open hat mono audio file
#define SOUNDSIZE3 (9882)

/// @brief Size of the audio playback buffer in bytes
#define BUFFERSIZE (32768)

/// @brief Buffer for storing audio data for playback
int16_t playbackBuffer[BUFFERSIZE] = {0};


/// @brief Static semaphore buffer for playback control
StaticSemaphore_t xSemaphorePlaybackStatic;

/// @brief Binary semaphore handle used to signal playback
SemaphoreHandle_t xSemaphorePlayback;

/// @brief Static semaphore buffer for buffer modification
StaticSemaphore_t xSemaphoreModifyBufferStatic;

/// @brief Binary semaphore handle used to signal buffer modification
SemaphoreHandle_t xSemaphoreModifyBuffer;

/// @brief Static semaphore for OLED events
StaticSemaphore_t xSemaphoreOledStatic;

/// @brief Binary semaphore buffer for buffer modification
SemaphoreHandle_t xSemaphoreOled;

/// @brief Binary semaphore for button events
StaticSemaphore_t xButtonSemaphore;

/// @brief Binary semaphore handle for button events
SemaphoreHandle_t xButtonSemaphoreHandle;



// Tempo variables
/// @brief Maximum playback task delay (<2^32)
static const uint16_t MAX_PB_DELAY = 1000;

/// @brief Minimum playback task delay (>=1)
static const uint16_t MIN_PB_DELAY = 500;

/// @brief Interval delay for the playback task
uint16_t playback_delay = 1250;

/// @brief Counts when the OS has no task to execute
uint64_t u64IdleTicksCnt = 0;

/// @brief Counts OS ticks (default = 1000Hz)
uint64_t tickTime = 0;


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


// --- Playback Task ---
/// @brief Stack memory allocation for the playback task stored in CCM
StackType_t playbackTaskStack[PLAYBACK_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the playback task stored in CCM
StaticTask_t playbackTaskBuffer CCM_RAM;

/// @brief Handle for the playback task
TaskHandle_t playbackTaskHandle;


// --- Modify Buffer Task ---
/// @brief Stack memory allocation for the modify task stored in CCM
StackType_t modifyBufferTaskStack[MODIFYBUFFER_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the modify task stored in CCM
StaticTask_t modifyBufferTaskBuffer CCM_RAM;

/// @brief Handle for the modify buffer task
TaskHandle_t modifyBufferTaskHandle;


// --- Sequencer Task ---
/// @brief Stack memory allocation for the modify task stored in CCM
StackType_t sequencerTaskStack[SEQUENCER_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the modify task stored in CCM
StaticTask_t sequencerTaskBuffer CCM_RAM;

/// @brief Handle for the modify buffer task
TaskHandle_t sequencerTaskHandle;

/**
 * @brief
 *
 * Monitors the state of the user button connected to GPIOA Pin 0.
 * If a button press is detected, it resets the blink delay to its
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
void vButtonStepTask(void *p);

/**
 * @brief Controls LED blinking behavior with variable delay.
 *
 * Cycles through four LEDs connected to GPIOD Pins 12, 13, 14, and 15.
 * Adjusts the blinking delay dynamically within defined limits.
 * Runs as a FreeRTOS task, demonstrating time-based scheduling.
 *
 * @param[in] p Pointer to task parameters (unused).
 */
void vBlinkTask(void *p);

/**
 * @brief Loads sound into the playback buffer
 *
 * Loads sound into the array, applies effects to it if they are selected,
 * and displays active effect on LCD screen after modifying playback buffer
 *
 * @param[in] pvparameters Pointer to task parameters (unused).
 */
void vModifyBufferTask(void *pvparameters);

/**
 * @brief Triggers audio playback with a variable delay.
 *
 * Runs as a FreeRTOS task that waits for a binary semaphore to be given.
 * When the semaphore is given, it plays a 16-bit signed audio sample with
 * a defined sample rate and waits for a variable delay before playing the
 * sample again. This task is designed to be triggered by the button task
 * when a button press is detected.
 *
 * @param[in] pvparameters Pointer to task parameters (unused).
 */
void vPlaybackTask(void *pvparameters);

/**
 * @brief Triggers playback of sound samples in a sequence.
 *
 * This task is responsible for triggering the playback of sound samples
 * in a predefined sequence. It waits for a binary semaphore to be given,
 * a triggering the playback task to play the next sound sample
 * in the sequence. The sequence is defined by the button state,
 * which is monitored by the button task. If the button state
 * changes, the sequence is updated accordingly.
 *
 * @param[in] pvparameters Pointer to task parameters (unused).
 */
void vSequencerTask(void *pvparameters);

/**
 * @brief Configures the user button GPIO (PA0) as an input.
 *
 * Initializes GPIO settings, enabling input mode without pull-up or
 * pull-down resistors. Prepares the pin to detect user button presses.
 */
void config_userbutton(void);

/**
 * @brief Initializes GPIO pins connected to LEDs.
 *
 * Prepares GPIOD Pins 12, 13, 14, and 15 for output mode. Ensures
 * proper configuration for controlling LED states.
 */
void leds_init(void);

/**
 * @brief Configures the system clock to 168 MHz for STM32F4.
 * 
 * This function enables the power interface clock, configures voltage 
 * regulator settings, and uses the High Speed External (HSE) oscillator 
 * as the clock source. It sets the AHB, APB1, and APB2 prescalers and 
 * configures the main PLL to achieve a 168 MHz system clock frequency.
 * It also configures the Flash memory latency and enables the prefetch buffer.
 */
void SystemClock_Config(void);

#endif  // PROGTOMATA_SYSTEM_H_
