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

#include <FreeRTOS.h>
#include <stdbool.h>
#include <stdio.h>
#include <task.h>
#include <trace.h>

/*!
 * \brief Interval (in FreeRTOS ticks) between each TriceTransfer call.
 * \note  10,000 ticks at 1kHz tick rate is ~10 seconds, adjust for your needs.
 */
#define TRICE_TASK_INTERVAL (10000)

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
 * \brief Global file pointer for "trices.raw". NULL until we open it.
 */
static FILE* g_logFilePtr = NULL;

/*!
 * \brief Flag to ensure TraceInit() is only called once.
 */
static bool g_isTraceInitialized = false;

/*!
 * \brief The FreeRTOS task that periodically invokes TriceTransfer().
 *
 * \param pvParameters Not used in this example.
 */
static void vTriceTask(void* pvParameters) {
  (void)pvParameters;  // Unused parameter

  for (;;) {
    // Every loop, call TriceTransfer() to handle deferred output
    TriceTransfer();

    // Wait for the given interval
    vTaskDelay(TRICE_TASK_INTERVAL);
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
    // Initialize the Trice system
    TriceInit();

    // Create the FreeRTOS task for periodic TriceTransfer
    xTaskCreateStatic(vTriceTask, "TriceTask", TRICE_TASK_STACK_SIZE, NULL, 1,
                    traceTaskStack, &traceTaskBuffer);

    g_isTraceInitialized = true;
  }
}

/*!
 * \brief The actual function that writes TRICE data (32-bit words) in deferred
 * mode.
 *
 * \param[in] enc   Pointer to an array of 32-bit words to write.
 * \param[in] count Number of 32-bit words in \p enc.
 *
 * \note  On a microcontroller without a native file system, you might replace
 * this with writing to a UART, or storing in memory, etc.
 */
static void NonBlockingDeferredWrite32AuxImpl(const uint32_t* enc,
                                              unsigned count) {
  // If the file is not yet open, open it in append mode (creates if not
  // existing).
  if (g_logFilePtr == NULL) {
    g_logFilePtr = fopen("trices.raw", "ab");
    if (g_logFilePtr == NULL) {
      // Could not open file; handle error as needed (log, assert, etc.)
      return;
    }
  }

  // Write 'count' 32-bit words
  size_t written = fwrite(enc, sizeof(uint32_t), count, g_logFilePtr);
  if (written < count) {
    // Handle partial write or error
    // e.g., log or handle in an application-specific way
  }

  // Flush so data is physically written; might remove for performance reasons
  fflush(g_logFilePtr);
}

/*!
 * \brief A global function pointer expected by the Trice library.
 *
 * \details We assign it to point to our implementation
 *          (\c NonBlockingDeferredWrite32AuxImpl).
 */
Write32AuxiliaryFn_t UserNonBlockingDeferredWrite32AuxiliaryFn = NonBlockingDeferredWrite32AuxImpl;
