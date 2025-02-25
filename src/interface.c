/**
 * Controls communication with encoders and shift registers
 * Used to read buttons and write to LEDs
 */

#include "interface.h"

// @TODO: check
/**
Initialise all the GPIO pins being used
They connect to the encoders (buttons) and shift registers (LEDs)
*/
void Interface_Init() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;

  // Initialize encoder
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_InitStruct.GPIO_Pin = Encoder1_Q0 | Encoder1_Q1 | Encoder1_Q2;
  GPIO_Init(Encoder1_Port, &GPIO_InitStruct);

  // Initialize shifter
  /* GPIO_InitStruct.GPIO_Pin = Shifter1_DS | Shifter1_CLK;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_Init(Shifter1_Port, &GPIO_InitStruct);*/
}

// @TODO: implement
/**
 * Write to the LEDs by writing to the shift register
 * Instead of using an SPI peripheral, a GPIO pin is manually toggled to
 * simulate the clock line
 *
void Interface_SetFunctionLED(uint8_t bits) {
  int i = 0;
  uint8_t mask = 0b10000000;

  for (; i < 8; i++) {
    if (mask & bits) {
      GPIO_SetBits(Shifter1_Port, Shifter1_DS);
    } else {
      GPIO_ResetBits(Shifter1_Port, Shifter1_DS);
    }

    mask = mask >> 1;

    GPIO_SetBits(Shifter1_Port, Shifter1_CLK);
    GPIO_ResetBits(Shifter1_Port, Shifter1_CLK);
  }

  GPIO_SetBits(Shifter1_Port, Shifter1_CLK);
  GPIO_ResetBits(Shifter1_Port, Shifter1_CLK);
}*/

// @TODO: implement
/**
 * Write to the LEDs by writing to the shift register
 * Instead of using an SPI peripheral, a GPIO pin is manually toggled to
 * simulate the clock line
 *
 * This function is used for the second shift register as pins allocations are
 * different on each shift register
 *
void Interface_SetRegister(uint8_t bits) {
  int i = 0;
  uint8_t mask = 0b10000000;

  for (; i < 8; i++) {
    if (mask & bits) {
      GPIO_SetBits(Shifter1_Port, Shifter1_DS);
    } else {
      GPIO_ResetBits(Shifter1_Port, Shifter1_DS);
    }

    mask = mask >> 1;

    GPIO_SetBits(Shifter1_Port, Shifter1_CLK);
    GPIO_ResetBits(Shifter1_Port, Shifter1_CLK);
  }
}*/

// @TODO: implement
/**
 * Read an effect button
 *
uint8_t Interface_ReadFxButton() {
  uint8_t button_pressed = 0;
  if (GPIO_ReadInputDataBit(Encoder3_Port, Encoder3_Q0) == 1) {
    button_pressed = button_pressed | 0b001;
  }
  if (GPIO_ReadInputDataBit(Encoder3_Port, Encoder3_Q1) == 1) {
    button_pressed = button_pressed | 0b010;
  }
  if (GPIO_ReadInputDataBit(Encoder3_Port, Encoder3_Q2) == 1) {
    button_pressed = button_pressed | 0b100;
  }

  return button_pressed;
}
*/

// @TODO: implement
/**
Determine which button from the 4-step grid has been pressed
*/
uint8_t Interface_ReadButtonStep() {
  uint8_t button_code = 0;

  if (GPIO_ReadInputDataBit(Encoder1_Port, Encoder1_Q0) != 0x00) {
    button_code |= 0b001;  // O0
  }
  if (GPIO_ReadInputDataBit(Encoder1_Port, Encoder1_Q1) != 0x00) {
    button_code |= 0b010;  // O1
  }
  if (GPIO_ReadInputDataBit(Encoder1_Port, Encoder1_Q2) != 0x00) {
    button_code |= 0b100;  // O2
  }

  // If no valid button is detected, return STEPIDLE_VALUE
  return (button_code == 0) ? STEPIDLE_VALUE : button_code;
}