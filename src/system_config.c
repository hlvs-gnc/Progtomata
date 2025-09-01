#include <progtomata_system.h>

void systemClock_config(void) {
  // Enable the power interface clock and configure voltage regulator
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_MainRegulatorModeConfig(PWR_Regulator_Voltage_Scale1);

  // Enable HSE
  RCC_HSEConfig(RCC_HSE_ON);
  while (RCC_WaitForHSEStartUp() == ERROR)
    ;

  // Configure Flash wait states
  FLASH_SetLatency(FLASH_Latency_1);
  FLASH_PrefetchBufferCmd(ENABLE);

  // Configure AHB, APB1, and APB2 prescalers
  // AHB @ 168 MHz, APB2 @ 84 MHz, APB1 @ 42 MHz
  RCC_HCLKConfig(RCC_SYSCLK_Div1); // AHB = SYSCLK/1 = 168 MHz
  RCC_PCLK2Config(RCC_HCLK_Div2);  // APB2 = AHB/2 = 84 MHz
  RCC_PCLK1Config(RCC_HCLK_Div4);  // APB1 = AHB/4 = 42 MHz

  // Set up main PLL to 168 MHz system clock
  // (HSE=8 MHz, VCO=336 MHz, /2 => 168 MHz, /7 => 48 MHz USB)
  RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
  RCC_PLLCmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    ;

  // Select the main PLL as system clock source
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  while (RCC_GetSYSCLKSource() != 0x08)
    ; // 0x08 = PLL used as sysclk

  // Configure the I2S PLL for a proper I2S clock
  RCC_PLLI2SConfig(271, 2);
  RCC_PLLI2SCmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY) == RESET)
    ;

  // Update the SystemCoreClock global variable
  SystemCoreClockUpdate();

  // Enable peripheral clocks
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
}

void config_userButton(void) {
  // Enable clock for GPIOD
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Declare a variable of type struct GPIO_InitTypeDef
  GPIO_InitTypeDef GPIO_InitStructure;

  // Set pin mode to input
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;

  // Select pin PA0 only
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;

  // Set no internal pull-up or pull-down resistor
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  // Initialize PA0 pins by passing port name and address of PushButton struct
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure PD1 and PD2 (new buttons) as input with internal pull-up
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // Enable internal pull-up
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void leds_init(void) {
  // Initialize board LEDs
  STM_EVAL_LEDInit(LED3);

  STM_EVAL_LEDInit(LED4);

  STM_EVAL_LEDInit(LED5);

  STM_EVAL_LEDInit(LED6);

  // External LEDs
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
                                GPIO_Pin_15 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
