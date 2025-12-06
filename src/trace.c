/**
 * @file trace.c
 * @brief This file contains the definitions of user-provided functions and
 * function pointers for use with the Trice library in deferred mode.
 *
 * @details
 *   - A FreeRTOS task is created to periodically invoke \c TriceTransfer().
 *   - \c TraceInit() calls \c TriceInit(), creates the task, and ensures it
 * only runs once.
 */

#include <hooks.h>
#include <trace.h>

#ifdef TRACE_TONBANDGERAT
#include "tband_config.h"
#include "tband.h"
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stm32f4_discovery.h>  // For LED debug
#endif

/**
 * @brief Interval (in FreeRTOS ticks) between each TriceTransfer call.
 */
#define TRICE_TASK_INTERVAL (100)

/**
 * @brief Stack size and priority for the Trice task.
 * @note  Adjust as needed for your system.
 */
#define TRACE_TASK_STACK_SIZE 512  // Increased for Tonbandgerät
#define TRACE_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)

/// @brief Stack memory allocation for the button task stored in CCM
StackType_t traceTaskStack[TRACE_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the button task stored in CCM
StaticTask_t traceTaskBuffer CCM_RAM;

static void DWT_Init(void) {
  // Enable trace and debug block
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Reset the cycle counter
  DWT->CYCCNT = 0;

  // Enable the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

#ifdef LOG_TRICE
/// @brief Flag to ensure TraceInit() is only called once.
static bool g_isTraceInitialized = false;

/**
 * @brief The FreeRTOS task that periodically invokes TriceTransfer().
 *
 * @param pvParameters Not used.
 */
static void vTriceTask(void *pvParameters) {
  (void)pvParameters; // Unused parameter

  for (;;) {
    // Wait for the given interval
    vTaskDelay(TRICE_TASK_INTERVAL);

    // Every loop, call TriceTransfer() to handle deferred output
    TriceTransfer();
  }
}
#endif

void TraceInit(void) {
  DWT_Init();

#ifdef LOG_TRICE
  if (!g_isTraceInitialized) {
    // Create the FreeRTOS task for periodic TriceTransfer
    xTaskCreateStatic(vTriceTask, "TriceTask", TRACE_TASK_STACK_SIZE, NULL,
                      TRACE_TASK_PRIORITY, traceTaskStack, &traceTaskBuffer);

    g_isTraceInitialized = true;
  }
#endif
}

#ifdef TRACE_TONBANDGERAT
// Call this AFTER vTaskStartScheduler() from a task
void TraceStart(void) {
  static bool started = false;
  if (!started) {
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for system to stabilize
    
    // Tell Tonbandgerät scheduler has started
    tband_freertos_scheduler_started();
    
    // Gather and buffer system metadata (task names, etc.)
    tband_gather_system_metadata();
    
    // 3. Start streaming - sends metadata buffer
    // After this, tband_submit_to_backend is called from FreeRTOS hooks
    // which calls traceport_stream_data() with trace events
    int ret = tband_start_streaming();
    
    if (ret == 0) {
      // Success - turn on LED3 to show streaming is active
      STM_EVAL_LEDOn(LED3);
    }
    
    started = true;
  }
}

// Stream data handler - shares UART2 with Trice
// Tonbandgerät uses COBS framing (0x00 byte delimiters)
// This is called DIRECTLY from FreeRTOS trace hooks (ISR context)
bool traceport_stream_data(const uint8_t *buf, size_t len) {
  if (!buf || len == 0) {
    return false;
  }
  
  // Write to UART2 with timeout to prevent blocking
  for (size_t i = 0; i < len; i++) {
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
    };
  
    USART_SendData(USART2, buf[i]);
  }
  
  return false;  // Return false = no data dropped
}
#endif
