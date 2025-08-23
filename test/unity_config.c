#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include <stdint.h>

#ifndef UNITY_USART_BAUD
#define UNITY_USART_BAUD 115200
#endif

static void usart2_init(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  GPIO_InitTypeDef gpio;
  GPIO_StructInit(&gpio);
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;
  gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // PA2 TX, PA3 RX
  GPIO_Init(GPIOA, &gpio);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  USART_InitTypeDef us;
  USART_StructInit(&us);
  us.USART_BaudRate = UNITY_USART_BAUD;
  us.USART_WordLength = USART_WordLength_8b;
  us.USART_StopBits = USART_StopBits_1;
  us.USART_Parity = USART_Parity_No;
  us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  us.USART_Mode = USART_Mode_Tx; // only TX needed
  USART_Init(USART2, &us);
  USART_Cmd(USART2, ENABLE);
}

void unityOutputStart(void) {
  static int inited = 0;
  if (!inited) {
    usart2_init();
    inited = 1;
  }
}

void unityOutputChar(int c) {
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
  }
  USART_SendData(USART2, (uint16_t)(c & 0xFF));
}

void unityOutputFlush(void) {
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
  }
}

void unityOutputComplete(void) {

}
