/**
 * @file tband_port.c
 *
 * @brief Tonbandgerat tracing library port
 *
 * @details Implements the timestamp function using the
 * ARM Cortex-M4 DWT (Data Watchpoint and Trace) cycle counter
 * for high-resolution timing.
 *
 */

#include "tband_config.h"
#include <stm32f4xx.h>

// Timestamp using DWT cycle counter
uint64_t traceport_timestamp(void) {
  return DWT->CYCCNT;
}
