#include <stdint.h>

/* Mock-visible flags */
int mock_rcc_ahb1_called = 0;
int mock_rcc_apb1_called = 0;
int mock_hse_requested = 0;
int mock_hse_ready = 1;
int mock_pll_ready = 1;
int mock_systemcoreclock_update_called = 0;
int mock_gpio_init_called = 0;

/* Minimal mock implementations used by systemClock_config and gpio init */

void RCC_AHB1PeriphClockCmd(uint32_t Periph, int NewState) {
  (void)Periph;
  (void)NewState;
  mock_rcc_ahb1_called = 1;
}

void RCC_APB1PeriphClockCmd(uint32_t Periph, int NewState) {
  (void)Periph;
  (void)NewState;
  mock_rcc_apb1_called = 1;
}

void RCC_HSEConfig(int state) {
  (void)state;
  mock_hse_requested = 1;
}
int RCC_WaitForHSEStartUp(void) {
  return mock_hse_ready ? 0 : 1;
} /* 0 == success */

void FLASH_SetLatency(int latency) { (void)latency; }
void FLASH_PrefetchBufferCmd(int state) { (void)state; }

void RCC_HCLKConfig(int val) { (void)val; }
void RCC_PCLK2Config(int val) { (void)val; }
void RCC_PCLK1Config(int val) { (void)val; }

void RCC_PLLConfig(int src, int m, int n, int p, int q) {
  (void)src;
  (void)m;
  (void)n;
  (void)p;
  (void)q;
}
void RCC_PLLCmd(int state) {
  (void)state;
  mock_pll_ready = 1;
}
int RCC_GetFlagStatus(int flag) {
  (void)flag;
  return 1;
} /* non-zero -> ready */
void RCC_SYSCLKConfig(int src) { (void)src; }
int RCC_GetSYSCLKSource(void) { return 0x08; } /* 0x08 == PLL used as sysclk */

void RCC_PLLI2SConfig(int val, int div) {
  (void)val;
  (void)div;
}
void RCC_PLLI2SCmd(int state) { (void)state; }

void SystemCoreClockUpdate(void) { mock_systemcoreclock_update_called = 1; }

/* Minimal GPIO mock used by config_userbutton / leds_init */
typedef struct {
  uint32_t GPIO_Pin;
  uint32_t GPIO_Mode;
  uint32_t GPIO_OType;
  uint32_t GPIO_Speed;
  uint32_t GPIO_PuPd;
} GPIO_InitTypeDef;

void RCC_AHB1PeriphClockCmd(uint32_t Periph, int NewState); /* declared above */

void GPIO_Init(void *GPIOx, GPIO_InitTypeDef *init) {
  (void)GPIOx;
  (void)init;
  mock_gpio_init_called = 1;
}

/* Dummy definitions used by code but not exercised in these tests */
void PWR_MainRegulatorModeConfig(int mode) { (void)mode; }
void GPIO_SetBits(void *a, uint32_t b) {
  (void)a;
  (void)b;
}
void GPIO_ResetBits(void *a, uint32_t b) {
  (void)a;
  (void)b;
}
int GPIO_ReadInputDataBit(void *a, uint32_t b) {
  (void)a;
  (void)b;
  return 0;
}


void systemClock_config(void) {
  // empty implementation
}