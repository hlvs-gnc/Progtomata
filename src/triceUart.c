/*!
 * \file triceUart.h
 ******************************************************************************/

#include <triceUart.h>

#if TRICE_DEFERRED_UARTA == 1


uint32_t triceTxDataRegisterEmptyUartA(void) {
  // Return 1 if TXE is SET, meaning transmit data register is empty:
  return (USART_GetFlagStatus(TRICE_UARTA, USART_FLAG_TXE) == SET);
}

void triceTransmitData8UartA(uint8_t v) {
  // Wait until transmit buffer is empty
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
  }
  USART_SendData(TRICE_UARTA, v);
}

void triceEnableTxEmptyInterruptUartA(void) {
  USART_ITConfig(TRICE_UARTA, USART_IT_TXE, ENABLE);
}

void triceDisableTxEmptyInterruptUartA(void) {
  USART_ITConfig(TRICE_UARTA, USART_IT_TXE, DISABLE);
}

#endif  // #if TRICE_DEFERRED_UARTA == 1

#if TRICE_DEFERRED_UARTB == 1

uint32_t triceTxDataRegisterEmptyUartB(void) {
  // Return 1 if TXE is SET, meaning transmit data register is empty:
  return (USART_GetFlagStatus(TRICE_UARTB, USART_FLAG_TXE) == SET);
}

void triceTransmitData8UartB(uint8_t v) {
  USART_SendData(TRICE_UARTB, v);
  ToggleOpticalFeedbackLED();
}

void triceEnableTxEmptyInterruptUartB(void) {
  USART_ITConfig(TRICE_UARTB, USART_IT_TXE, ENABLE);
}

void triceDisableTxEmptyInterruptUartB(void) {
  USART_ITConfig(TRICE_UARTB, USART_IT_TXE, DISABLE);
}

#endif  // #if TRICE_DEFERRED_UARTB == 1
