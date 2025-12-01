/**
 * @file unity_config.c
 *
 * @brief Unity configuration for host/unit-test environment.
 *
 * @details Provides Unity functions that route test data
 * through the UART driver when running tests. This keeps test output
 * consistent with embedded runtime logging while initializes the peripheral
 * and sets the respective flag to 1.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 */

#include <stdio.h>

#if BUILD_EMBEDDED
#include <uart_driver.h>
static int initialized = 0;
#endif

void unityOutputStart(void) {
#if BUILD_EMBEDDED
  if (!initialized) {
    uart_init();
    initialized = 1;
  }
#endif
}

void unityOutputChar(int c) {
#if BUILD_EMBEDDED
  uart_send_char((char)c);
#else
  putchar(c);
#endif
}

void unityOutputFlush(void) {
#if !BUILD_EMBEDDED
  fflush(stdout);
#endif
}

void unityOutputComplete(void) {}
