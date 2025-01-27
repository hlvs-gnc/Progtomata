/*!
 * \file trace.h
 * \brief Public header for the trace module.
 *
 * \details
 *   This header declares the \c TraceInit() function for setting up Trice
 *   in deferred mode and creating the associated FreeRTOS task.
 */

#ifndef TRACE_H_
#define TRACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx.h>

#include <FreeRTOS.h>
#include <task.h>

#include <trice.h>

/*!
 * \brief Initialize the trace system.
 *
 * \details
 *   - Calls \c TriceInit() to initialize the Trice library.
 *   - Creates a FreeRTOS task that periodically calls \c TriceTransfer().
 *   - Ensures it is only initialized once.
 */
void TraceInit(void);

#ifdef __cplusplus
}
#endif

#endif // TRACE_H_
