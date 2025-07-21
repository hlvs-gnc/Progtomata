/*
 * \file triceUart.h
 * \brief Deferred-mode UART adapter for Trice using STM32 SPL drivers
 *
 * In your project, define TRICE_DEFERRED_UARTA or TRICE_DEFERRED_UARTB as 1 to
 * enable the corresponding UART functionality. Also define TRICE_UARTA (e.g.,
 * USART1) and/or TRICE_UARTB (e.g., USART2) to match your hardware setup.
 */

#ifndef CONFIG_TRICEUART_H_
#define CONFIG_TRICEUART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <triceConfig.h>

// SPL includes for STM32F407
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>

/**
 * @brief Toggle an LED for feedback (optional).
 *
 * Modify this to match your actual LED GPIO pin and port.
 */
void ToggleOpticalFeedbackLED(void);

#if TRICE_DEFERRED_UARTA == 1

/**
 * @brief Check if a new byte can be written into the UARTA transmit register.
 * @return Non-zero if the TX register is empty, 0 if it is not.
 */
uint32_t triceTxDataRegisterEmptyUartA(void);

/**
 * @brief Write a byte into the UARTA transmit register.
 * @param v Byte to transmit
 */
void triceTransmitData8UartA(uint8_t v);

/**
 * @brief Enable the "TX data register empty" interrupt for UARTA.
 */
void triceEnableTxEmptyInterruptUartA(void);

/**
 * @brief Disable the "TX data register empty" interrupt for UARTA.
 */
void triceDisableTxEmptyInterruptUartA(void);

#endif // TRICE_DEFERRED_UARTA

#if TRICE_DEFERRED_UARTB == 1

/**
 * @brief Check if a new byte can be written into the UARTB transmit register.
 * @return Non-zero if the TX register is empty, 0 if it is not.
 */
TRICE_INLINE uint32_t triceTxDataRegisterEmptyUartB(void);

/**
 * @brief Write a byte into the UARTB transmit register.
 * @param v Byte to transmit
 */
TRICE_INLINE void triceTransmitData8UartB(uint8_t v);

/**
 * @brief Enable the "TX data register empty" interrupt for UARTB.
 */
TRICE_INLINE void triceEnableTxEmptyInterruptUartB(void);

/**
 * @brief Disable the "TX data register empty" interrupt for UARTB.
 */
TRICE_INLINE void triceDisableTxEmptyInterruptUartB(void);

#endif // TRICE_DEFERRED_UARTB

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_TRICEUART_H_ */
