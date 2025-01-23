/*!
 * \file trace.h
 * \brief Public header for the trace module.
 *
 * \details
 *   This header declares the \c TraceInit() function for setting up Trice
 *   in deferred mode and creating the associated FreeRTOS task.
 */

#ifndef TRACE_H
#define TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

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

#endif // TRACE_H
