#include "unity.h"

/* mocks flags */
extern int mock_rcc_ahb1_called;
extern int mock_rcc_apb1_called;
extern int mock_hse_requested;
extern int mock_systemcoreclock_update_called;
extern int mock_gpio_init_called;

/* functions from main.c (made non-static above) */
void SystemClock_Config(void);
void config_userbutton(void);
void leds_init(void);

void setUp(void) {
  mock_rcc_ahb1_called = 0;
  mock_rcc_apb1_called = 0;
  mock_hse_requested = 0;
  mock_systemcoreclock_update_called = 0;
  mock_gpio_init_called = 0;
}

void tearDown(void) {}

/* Test that SystemClock_Config calls RCC and SystemCoreClockUpdate (via mocks) */
void test_SystemClock_Config_invokes_rcc_and_core_update(void) {
  SystemClock_Config();
  TEST_ASSERT_EQUAL_INT(1, mock_hse_requested);
  TEST_ASSERT_EQUAL_INT(1, mock_rcc_ahb1_called);
  TEST_ASSERT_EQUAL_INT(1, mock_systemcoreclock_update_called);
}

/* Test that button and LED init call GPIO_Init (mock) */
void test_config_userbutton_and_leds_init_call_gpio_init(void) {
  config_userbutton();
  TEST_ASSERT_EQUAL_INT(1, mock_gpio_init_called);

  /* reset and test leds_init */
  mock_gpio_init_called = 0;
  leds_init();
  TEST_ASSERT_EQUAL_INT(1, mock_gpio_init_called);
}
