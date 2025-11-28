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

#include <stdint.h>
#include <uart_driver.h>

static int initialized = 0;

void unityOutputStart(void) {
  if (!initialized) {
    uart_init();
    initialized = 1;
  }
}

void unityOutputChar(int c) {
  uart_send_char((char)c);
}

void unityOutputFlush(void) {}

void unityOutputComplete(void) {}
