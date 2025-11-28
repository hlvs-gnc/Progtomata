/**
 * @file trace.c
 * @brief This file contains the definitions of user-provided functions and
 * function pointers for use with the Trice library in deferred mode.
 *
 * @details
 *   - A FreeRTOS task is created to periodically invoke \c TriceTransfer()
 *     (here, every 10,000 clock ticks).
 *   - \c TraceInit() calls \c TriceInit(), creates the task, and ensures it
 * only runs once.
 *   - This code demonstrates how to open a file (\c trices.raw) in append mode
 * (creating it if it doesn’t exist) and write TRICE data to it.
 */

#include <hooks.h>
#include <trace.h>

/**
 * @brief Interval (in FreeRTOS ticks) between each TriceTransfer call.
 */
#define TRICE_TASK_INTERVAL (100)

/**
 * @brief Stack size and priority for the Trice task.
 * @note  Adjust as needed for your system.
 */
#define TRACE_TASK_STACK_SIZE 256
#define TRACE_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)

/// @brief Stack memory allocation for the button task stored in CCM
StackType_t traceTaskStack[TRACE_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the button task stored in CCM
StaticTask_t traceTaskBuffer CCM_RAM;

/**
 * @brief Flag to ensure TraceInit() is only called once.
 */
static bool g_isTraceInitialized = false;

static void DWT_Init(void)
{
  // Enable trace and debug block
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Reset the cycle counter
  DWT->CYCCNT = 0;

  // Enable the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief The FreeRTOS task that periodically invokes TriceTransfer().
 *
 * @param pvParameters Not used.
 */
static void vTriceTask(void *pvParameters)
{
  (void)pvParameters; // Unused parameter

  for (;;) {
    // Wait for the given interval
    vTaskDelay(TRICE_TASK_INTERVAL);

    // Every loop, call TriceTransfer() to handle deferred output
    TriceTransfer();
  }
}

void TraceInit(void)
{
  DWT_Init();

  if (!g_isTraceInitialized) {
    // Create the FreeRTOS task for periodic TriceTransfer
    xTaskCreateStatic(vTriceTask, "TriceTask", TRACE_TASK_STACK_SIZE, NULL,
                      TRACE_TASK_PRIORITY, traceTaskStack, &traceTaskBuffer);

    g_isTraceInitialized = true;
  }
}
