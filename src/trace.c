/*!
 * \file trace.c
 * \brief This file contains the definitions of user-provided functions and
 * function pointers for use with the Trice library in deferred mode.
 *
 * \details
 *   - A FreeRTOS task is created to periodically invoke \c TriceTransfer()
 *     (here, every 10,000 clock ticks).
 *   - \c TraceInit() calls \c TriceInit(), creates the task, and ensures it
 * only runs once.
 *   - This code demonstrates how to open a file (\c trices.raw) in append mode
 * (creating it if it doesn’t exist) and write TRICE data to it.
 */

#include <trace.h>

/*!
 * \brief Interval (in FreeRTOS ticks) between each TriceTransfer call.
 */
#define TRICE_TASK_INTERVAL (10)

/*!
 * \brief Stack size and priority for the Trice task.
 * \note  Adjust as needed for your system.
 */
#define TRICE_TASK_STACK_SIZE 512
#define TRICE_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

/// @brief Macro to use CCM (Core Coupled Memory) in STM32F4
#define CCM_RAM __attribute__((section(".ccmram")))

/// @brief Stack size for the button task in bytes
#define TRACE_TASK_STACK_SIZE 256

/// @brief Stack memory allocation for the button task stored in CCM
StackType_t traceTaskStack[TRACE_TASK_STACK_SIZE] CCM_RAM;

/// @brief Task control block (TCB) for the button task stored in CCM
StaticTask_t traceTaskBuffer CCM_RAM;

/*!
 * \brief Flag to ensure TraceInit() is only called once.
 */
static bool g_isTraceInitialized = false;

uint16_t TimeSample16(void) {
  return (uint16_t)(TIM2->CNT & 0xFFFF);
}

uint32_t TimeSample32(void) {
  return (uint32_t)(TIM2->CNT);
}

#ifndef TriceStamp16
//! Use SysTick->VAL as a 16-bit timestamp
#define TriceStamp16 (uint16_t)(TimeSample16())  // Lower 16 bits of SysTick->VAL
#endif

#ifndef TriceStamp32
//! Use SysTick->VAL as a 32-bit timestamp (extended with SysTick->CTRL)
#define TriceStamp32 (uint32_t)(TimeSample32())  // Combine SysTick->VAL and CTRL
#endif

/*!
 * \brief The FreeRTOS task that periodically invokes TriceTransfer().
 *
 * \param pvParameters Not used in this example.
 */
static void vTriceTask(void* pvParameters) {
  (void) pvParameters;  // Unused parameter

  for (;;) {
    // Wait for the given interval
    vTaskDelay(TRICE_TASK_INTERVAL);
 
    // Every loop, call TriceTransfer() to handle deferred output
    TriceTransfer();
  }
}

/*!
 * \brief TraceInit
 *
 * \details
 *   - Initializes the Trice library via TriceInit().
 *   - Creates a FreeRTOS task that periodically calls TriceTransfer().
 *   - Ensures it only happens once.
 */
void TraceInit(void) {
  if (!g_isTraceInitialized) {
    // Create the FreeRTOS task for periodic TriceTransfer
    xTaskCreateStatic(vTriceTask, "TriceTask", TRICE_TASK_STACK_SIZE, NULL, 1,
                      traceTaskStack, &traceTaskBuffer);

    g_isTraceInitialized = true;
  }
}
