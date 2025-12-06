#include "tband_config.h"
#include <stm32f4xx.h>

// Timestamp using DWT cycle counter
uint64_t traceport_timestamp(void) {
  return DWT->CYCCNT;
}
