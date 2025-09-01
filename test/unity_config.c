#include <stdint.h>
#include <uart_driver.h>

void unityOutputStart(void) {
  static int initialized = 0;
  if (!initialized) {
    uart_init();
    initialized = 1;
  }
}

void unityOutputChar(int c) { uart_send_char((char)c); }

void unityOutputFlush(void) {}

void unityOutputComplete(void) {}
