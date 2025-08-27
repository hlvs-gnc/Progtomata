#include "unity.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_function_should_doX(void) {
  // test stuff
}

void test_function_should_doY(void) {
  // more test stuff
}

int runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_function_should_doX);
  RUN_TEST(test_function_should_doY);
  return UNITY_END();
}

int main(void) { return runUnityTests(); }
