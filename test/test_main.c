/**
 * @file test_main.c
 *
 * @brief Unit test runner for Progtomata system using Unity framework.
 *
 * @details This file contains the main entry point and test setup for running
 * unit tests on the project using the Unity test framework. It replicates
 * minimal system timing requirements and provides simple implementations for
 * required hooks and delays to allow tests to be executed in a
 * simulated environment.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 */

#include <progtomata_system.h>
#include <unity.h>

// Replicate system timing variables
static volatile uint64_t u64IdleTicksCnt = 0;

static volatile uint64_t tickTime = 0;

static void delay_ms(uint32_t ms)
{
  // crude delay @ ~16MHz HSI
  for (uint32_t i = 0; i < ms * 12000; ++i) {
    __asm__("nop");
  }
}

void setUp(void)
{
  systemClock_config();
}

void tearDown(void)
{
  // clean stuff up here
}

void test_function_dummy(void)
{
  // test dummy stuff
}

int runUnityTests(void)
{
  delay_ms(1000);
  UNITY_BEGIN();
  RUN_TEST(test_function_dummy);
  int rc = UNITY_END();

  return rc;
}

int main(void)
{
  return runUnityTests();
}

void vApplicationTickHook(void)
{
  ++tickTime;
}

void vApplicationIdleHook(void)
{
  ++u64IdleTicksCnt;
}
