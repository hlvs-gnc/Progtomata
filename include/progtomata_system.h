/**
 * @file progtomata_system.h
 *
 * @brief Header file for system-wide definitions and task configurations.
 *
 * @details This file contains macro definitions, constants, and memory
 * allocation for task stacks and control blocks, using CCM (Core Coupled
 * Memory) for STM32F4 microcontrollers.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef PROGTOMATA_SYSTEM_H_
#define PROGTOMATA_SYSTEM_H_

/**
 * @brief
 *
 * Monitors the state of the user button connected to GPIOA Pin 0.
 * If a button press is detected, it resets the blink delay to its
 * minimum value. Designed as a FreeRTOS task that runs continuously.
 *
 * @param[in] p Pointer to task parameters (unused).
 */
static void vButtonSampleTask(void *p);

/**
 * @brief
 *
 * @param[in] p Pointer to task parameters (unused).
 */
static void vButtonStepTask(void *pvParameters);

/**
 * @brief Controls LED blinking behavior with variable delay.
 *
 * Cycles through four LEDs connected to GPIOD Pins 12, 13, 14, and 15.
 * Adjusts the blinking delay dynamically within defined limits.
 * Runs as a FreeRTOS task, demonstrating time-based scheduling.
 *
 * @param[in] p Pointer to task parameters (unused).
 */
static void vBlinkTask(void *p);

/**
 * @brief Task to animate the OLED display.
 *
 * This task runs continuously and redraws the OLED display with an
 * animation. The animation is a bouncing ball that moves within the
 * display boundaries.
 *
 * @param[in] pvParameters Pointer to task parameters (unused).
 */
static void vOledAnimationTask(void *pvParameters);

/**
 * @brief Configures the system clock to 168 MHz for STM32F4.
 *
 * This function enables the power interface clock, configures voltage
 * regulator settings, and uses the High Speed External (HSE) oscillator
 * as the clock source. It sets the AHB, APB1, and APB2 prescalers and
 * configures the main PLL to achieve a 168 MHz system clock frequency.
 * It also configures the Flash memory latency and enables the prefetch buffer.
 */
static void SystemClock_Config(void);

#endif // PROGTOMATA_SYSTEM_H_
