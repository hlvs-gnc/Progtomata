#include <progtomata_system.h>
#include <unity.h>

// Replicate system timing variables
static volatile uint64_t u64IdleTicksCnt = 0;

static volatile uint64_t tickTime = 0;

static void delay_ms(uint32_t ms) {
  // crude delay @ ~16MHz HSI
  for (uint32_t i = 0; i < ms * 12000; ++i) {
    __asm__("nop");
  }
}

void setUp(void) { systemClock_config(); }

void tearDown(void) {
  // clean stuff up here
}

void test_function_dummy(void) {
  // test dummy stuff
}

int runUnityTests(void) {
  delay_ms(1000);
  UNITY_BEGIN();
  RUN_TEST(test_function_dummy);
  int rc = UNITY_END();

  return rc;
}

int main(void) { return runUnityTests(); }

void vApplicationTickHook(void) { ++tickTime; }

void vApplicationIdleHook(void) { ++u64IdleTicksCnt; }
