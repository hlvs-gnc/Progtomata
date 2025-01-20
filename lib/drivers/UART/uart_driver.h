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

#include <stdarg.h> // For va_list, va_start, va_end

#define UART_PRINTF_BUFFER_SIZE 128

typedef enum {
  PRINT_SIGNED_DEC,
  PRINT_UNSIGNED_DEC,
  PRINT_HEX,
  PRINT_FLOAT
} PrintFormat_t;

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

/**
 * @brief Convert an integer (signed or unsigned) to a string and send it over UART.
 *        Handles signed decimal, unsigned decimal, and hexadecimal.
 *
 * @param value  The integer value to print (pass as long for convenience).
 * @param format The format specifier (PRINT_SIGNED_DEC, PRINT_UNSIGNED_DEC, PRINT_HEX).
 */
static void uart_print_type(double value, PrintFormat_t format);

/**
 * @brief Transmits a formatted string over USART2.
 *
 * This function formats a string using the provided format specifier and arguments,
 * then sends the resulting string over USART2.
 *
 * @param format The format string (like printf).
 * @param ... The variable arguments to format.
 */
void uart_print(const char *format, ...);

#endif /* LIB_DRIVERS_UART_UART_DRIVER_H_ */
