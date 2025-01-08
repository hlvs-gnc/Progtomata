/**
 * @file uart_driver.h
 *
 * @brief STM32F4 USART1 Driver for UART Communication.
 *
 * This header provides initialization and basic transmit functions for
 * UART communication using USART1 on the STM32F4 microcontroller.
 *
 * @details
 * - Initializes USART1 with configurable baud rate and settings.
 * - Provides functions for sending single characters and strings.
 * - Uses GPIOA pins (PA9 for TX, PA10 for RX) for USART1 communication.
 * - Designed for compatibility with STM32F4-Discovery Board.
 *
 * Features:
 * - UART initialization with 115200 baud rate.
 * - Functions for transmitting characters and strings.
 * - Suitable for debugging and general communication tasks.
 *
 * Dependencies:
 * - STM32F4xx Standard Peripheral Library
 *
 * Hardware:
 * - Board: STM32F4-Discovery
 * - USART1 TX: PA9
 * - USART1 RX: PA10
 *
 * @note Ensure that the STM32F4 peripheral drivers and RCC configurations are
 *       correctly enabled before compiling the project.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons 
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 */

#ifndef LIB_DRIVERS_UART_UART_DRIVER_H_
#define LIB_DRIVERS_UART_UART_DRIVER_H_

#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>

/**
 * @brief Initializes USART1 for UART communication.
 *
 * Configures GPIO pins, USART1 settings, and enables the USART peripheral.
 * Uses PA9 (TX) and PA10 (RX).
 */
void uart_init(void);

/**
 * @brief Transmits a single character over USART1.
 *
 * @param c Character to send.
 */
void uart_send_char(char c);

/**
 * @brief Transmits a string over USART1.
 *
 * @param str Pointer to the null-terminated string to send.
 */
void uart_send_string(const char *str);

#endif /* LIB_DRIVERS_UART_UART_DRIVER_H_ */
