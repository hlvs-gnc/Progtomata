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

#if (BUILD_EMBEDDED == 1)
// STD Peripheral Library
#include <stm32f4xx.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_flash.h>
#include <stm32f4xx_pwr.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_tim.h>

// STM32F4 Discovery
#include <stm32f4_discovery.h>
#include <stm32f4_discovery_audio_codec.h>
#endif

/**
 * @brief Configures the system clock to 168 MHz for STM32F4.
 *
 * This function enables the power interface clock, configures voltage
 * regulator settings, and uses the High Speed External (HSE) oscillator
 * as the clock source. It sets the AHB, APB1, and APB2 prescalers and
 * configures the main PLL to achieve a 168 MHz system clock frequency.
 * It also configures the Flash memory latency and enables the prefetch buffer.
 */
void systemClock_config(void);

/**
 * @brief Configures the user button GPIO (PA0) as an input.
 *
 * Initializes GPIO settings, enabling input mode without pull-up or
 * pull-down resistors. Prepares the pin to detect user button presses.
 */
void userButton_config(void);

/**
 * @brief Initializes GPIO pins connected to LEDs.
 *
 * Prepares GPIOD Pins 12, 13, 14, and 15 for output mode. Ensures
 * proper configuration for controlling LED states.
 */
void boardLeds_config(void);

#endif // PROGTOMATA_SYSTEM_H_
