#include <stdint.h>
#include <uart_driver.h>

static int initialized = 0;

void unityOutputStart(void) {
  if (!initialized) {
    uart_init();
    initialized = 1;
  }
}

void unityOutputChar(int c) { uart_send_char((char)c); }

void unityOutputFlush(void) {}

void unityOutputComplete(void) {}
