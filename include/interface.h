#ifndef interface
#define interface

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"

#define Encoder1_Port GPIOD
#define Encoder1_Q0 GPIO_Pin_3
#define Encoder1_Q1 GPIO_Pin_7
#define Encoder1_Q2 GPIO_Pin_5
#define Encoder1_EN GPIO_Pin_6

#define Shifter1_Port GPIOC
#define Shifter1_DS GPIO_Pin_14
#define Shifter1_CLK GPIO_Pin_15

void Interface_Init();
void Interface_SetRegister2(uint8_t bits);
void Interface_SetRegister3(uint8_t bits);

uint8_t Interface_ReadFunctionButton();
int Interface_ButtonPadPressed();
uint8_t Interface_ReadButtonPad();

#endif
