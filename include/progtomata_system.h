
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

/// @brief Stack size for the playback task in bytes
#define PLAYBACK_TASK_STACK_SIZE 512

/// @brief Stack size for the modify task in bytes
#define MODIFY_TASK_STACK_SIZE 512

/// @brief Initial step size for delay increment and initial delay
/// value in milliseconds
static uint32_t kBlinkStep = 10, kBlinkDelay = 50;

/// @brief Minimum delay for LED blinking in milliseconds
const uint32_t MIN_DELAY = 10;
/// @brief Maximum delay for LED blinking in milliseconds
const uint32_t MAX_DELAY = 250;

/// @brief Number of samples in kick audio file
#define SOUNDSIZE1 (6918)

/// @brief Number of samples in open hat audio file
#define SOUNDSIZE2 (9882)

/// @brief Number of samples in open hat audio file
#define SOUNDSIZE3 (17700)

/// @brief Size of the audio playback buffer in bytes
#define BUFFERSIZE (32768)

// NOTE: ignore for now
// uint16_t playbackBuffer[BUFFERSIZE] = {0};

/// @brief Declare a StaticSemaphore_t variable
StaticSemaphore_t xSemaphoreBuffer;

/// @brief Binary semaphore for playback sequence
SemaphoreHandle_t xSemaphorePlayback;

/// @brief Interval delay in milliseconds for the playback task
uint16_t playback_delay = 500;

/**
 * @brief Handles button press events to adjust LED blink delay.
 *
 * Monitors the state of the user button connected to GPIOA Pin 0.
 * If a button press is detected, it resets the blink delay to its
 * minimum value. Designed as a FreeRTOS task that runs continuously.
 *
 * @param[in] p Pointer to task parameters (unused).
 */
void vButtonTask(void *p);

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

/// @brief Stack memory allocation for the button task stored in CCM
StackType_t buttonTaskStack[BUTTON_TASK_STACK_SIZE] CCM_RAM;
/// @brief Task control block (TCB) for the button task stored in CCM
StaticTask_t buttonTaskBuffer CCM_RAM;

TaskHandle_t buttonTaskHandle;

/// @brief Stack memory allocation for the blink task stored in CCM
StackType_t blinkTaskStack[BLINK_TASK_STACK_SIZE] CCM_RAM;
/// @brief Task control block (TCB) for the blink task stored in CCM
StaticTask_t blinkTaskBuffer CCM_RAM;
TaskHandle_t blinkTaskHandle;

/// @brief Stack memory allocation for the playback task stored in CCM
StackType_t playbackTaskStack[PLAYBACK_TASK_STACK_SIZE] CCM_RAM;
/// @brief Task control block (TCB) for the playback task stored in CCM
StaticTask_t playbackTaskBuffer CCM_RAM;
TaskHandle_t playbackTaskHandle;

/// @brief Stack memory allocation for the modify task stored in CCM
StackType_t modifyTaskStack[MODIFY_TASK_STACK_SIZE] CCM_RAM;
/// @brief Task control block (TCB) for the modify task stored in CCM
StaticTask_t modifyTaskBuffer CCM_RAM;
TaskHandle_t modifyTaskHandle;

#endif  // PROGTOMATA_SYSTEM_H_
