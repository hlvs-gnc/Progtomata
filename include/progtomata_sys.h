/**
 * @file progtomata_sys.h
 *
 * @brief Header file for system-wide definitions.
 *
 * @details This file contains the function declaration for system clock
 * configuration, interface buttons and LEDs.
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

#endif // PROGTOMATA_SYSTEM_H_
