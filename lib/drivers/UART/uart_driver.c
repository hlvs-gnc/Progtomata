/**
 * @file uart_driver.c
 *
 * @brief STM32F4 USART2 Driver Implementation.
 *
 * This file provides the implementation for initializing and utilizing USART2 
 * for UART communication on the STM32F4 microcontroller. It supports sending 
 * single characters and strings through the UART interface.
 *
 * Features:
 * - Initializes USART2 with a baud rate of 115200.
 * - Configures GPIO pins PA9 (TX) and PA10 (RX) for UART communication.
 * - Provides functions for transmitting characters and strings.
 *
 * Dependencies:
 * - STM32F4xx Standard Peripheral Library
 * - GPIO and USART drivers
 *
 * Hardware:
 * - Board: STM32F4-Discovery
 * - USART2 TX: PA2
 * - USART2 RX: PA3
 *
 * @note Ensure proper RCC configurations and GPIO initialization before use.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons 
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 */

#include "uart_driver.h"

/**
 * @brief Initializes USART2 for UART communication.
 *
 * Configures GPIOA pins (PA9, PA10) for USART2 and sets the baud rate to 115200.
 * Enables USART2 peripheral for data transmission and reception.
 */
void uart_init(void) {
  // GPIO Configuration
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIOA clock (if using PA2 and PA3)
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // Configure USART2_TX (PA2)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,
                   GPIO_AF_USART2);  // Alternate function mapping

  // Configure USART2_RX (PA3)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,
                   GPIO_AF_USART2);  // Alternate function mapping

  // Enable USART2 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // USART2 Configuration
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  USART_Init(USART2, &USART_InitStructure);

  // Enable USART2
  USART_Cmd(USART2, ENABLE);
}

/**
 * @brief Transmits a single character over USART2.
 *
 * @param c Character to send.
 */
void uart_send_char(char c) {
  // Wait until transmit buffer is empty
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
  USART_SendData(USART2, c);
}

/**
 * @brief Transmits a string over USART2.
 *
 * @param str Pointer to the null-terminated string to send.
 */
void uart_send_string(const char *str) {
  while (*str) {
    uart_send_char(*str++);
  }
}
