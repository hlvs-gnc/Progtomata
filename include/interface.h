#ifndef INTERFACE_H_
#define INTERFACE_H_

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"

/**
 * @defgroup Interface_Pins Interface Pin Definitions
 * @brief Definition of pin numbers used by the interface functions.
 *
 * The definitions are based on the STM32F4DISCOVERY board pinout.
 * @{
 */
#define Encoder1_Port GPIOA     /*!< GPIO port for encoder 1 */
#define Encoder1_Q0 GPIO_Pin_8  /*!< Pin number for encoder 1 Q0 */
#define Encoder1_Q1 GPIO_Pin_9  /*!< Pin number for encoder 1 Q1 */
#define Encoder1_Q2 GPIO_Pin_15 /*!< Pin number for encoder 1 Q2 */
// #define Encoder1_EN GPIO_Pin_8 /*!< Pin number for encoder 1 enable */

#define Shifter1_Port GPIOB      /*!< GPIO port for shifter 1 */
#define Shifter1_DS GPIO_Pin_14  /*!< Pin number for shifter 1 data */
#define Shifter1_CLK GPIO_Pin_15 /*!< Pin number for shifter 1 clock */
/**
 * @}
 */
#define STEPIDLE_VALUE 0

/**
 * @brief Initializes all the GPIO pins being used.
 * They connect to the encoders (buttons) and shift registers (LEDs).
 */
void Interface_Init();

/**
 * @brief Determines if any button from the 4-step playback grid has been
 * pressed.
 * @return Non-zero if a button has been pressed, 0 otherwise.
 */
// int Interface_ButtonStepPressed();

/**
 * @brief Determines which button from the 4-step grid has been pressed.
 * @return The code of the pressed button or 0xFF if no button is pressed.
 */
uint8_t Interface_ReadButtonStep();

// void Interface_SetRegister(uint8_t bits);

// uint8_t Interface_ReadFxButton();

#endif //  INTERFACE_H_
