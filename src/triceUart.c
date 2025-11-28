/**
 * @file triceUart.c
 *
 * @brief This file contains the definitions and configuration for UART
 * communication used by the Trice library.
 *
 * @details
 *   - Provides initialization and handling routines for UART peripheral.
 *   - Supports sending and receiving TRICE data over UART.
 *   - Designed for integration with FreeRTOS and STM32F4 platform.
 */

#include <triceUart.h>

#if TRICE_DEFERRED_UARTA == 1

uint32_t triceTxDataRegisterEmptyUartA(void)
{
  // Return 1 if TXE is SET, meaning transmit data register is empty:
  return (USART_GetFlagStatus(TRICE_UARTA, USART_FLAG_TXE) == SET);
}

void triceTransmitData8UartA(uint8_t v)
{
  // Wait until transmit buffer is empty
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
  }
  USART_SendData(TRICE_UARTA, v);
}

void triceEnableTxEmptyInterruptUartA(void)
{
  USART_ITConfig(TRICE_UARTA, USART_IT_TXE, ENABLE);
}

/*************  ✨ Windsurf Command ⭐  *************/
/*******  7714aea5-8f77-48ad-97ff-e7824ca6e46d  *******/
void triceDisableTxEmptyInterruptUartA(void)
{
  USART_ITConfig(TRICE_UARTA, USART_IT_TXE, DISABLE);
}

#endif // #if TRICE_DEFERRED_UARTA == 1

#if TRICE_DEFERRED_UARTB == 1

uint32_t triceTxDataRegisterEmptyUartB(void)
{
  // Return 1 if TXE is SET, meaning transmit data register is empty:
  return (USART_GetFlagStatus(TRICE_UARTB, USART_FLAG_TXE) == SET);
}

void triceTransmitData8UartB(uint8_t v)
{
  USART_SendData(TRICE_UARTB, v);
  ToggleOpticalFeedbackLED();
}

void triceEnableTxEmptyInterruptUartB(void)
{
  USART_ITConfig(TRICE_UARTB, USART_IT_TXE, ENABLE);
}

void triceDisableTxEmptyInterruptUartB(void)
{
  USART_ITConfig(TRICE_UARTB, USART_IT_TXE, DISABLE);
}

#endif // #if TRICE_DEFERRED_UARTB == 1
