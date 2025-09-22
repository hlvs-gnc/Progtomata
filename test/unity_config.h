/**
 * @file unity_config.h
 *
 * @brief Unity configuration header for Progtomata unit tests.
 *
 * @details Declares the Unity hook functions and maps Unity's output
 * macros to project-specific implementations that route test output.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

/**
 * @brief Initialize the Unity output backend.
 *
 * Called once by Unity before test output begins. Implementations should
 * perform any one-time initialization required to send characters.
 * Must be safe to call multiple times (implementations should be idempotent).
 */
void unityOutputStart();


/**
 * @brief Emit a single character to the Unity output backend.
 *
 * Unity invokes this for each character of its textual output.
 *
 * @param c Integer value of the character to send (fits in unsigned char).
 */
void unityOutputChar(char);

/**
 * @brief Flush any buffered output to the transport.
 *
 * Called by Unity when it needs to ensure all output has been emitted.
 * Implementations may be no-ops if output is unbuffered.
 */
void unityOutputFlush();

/**
 * @brief Finalize Unity output after test run completes.
 *
 * Called once when Unity completes; implementations may use this to
 * perform cleanup or to emit trailing markers.
 */
void unityOutputComplete();

#define UNITY_OUTPUT_START() unityOutputStart()
#define UNITY_OUTPUT_CHAR(c) unityOutputChar(c)
#define UNITY_OUTPUT_FLUSH() unityOutputFlush()
#define UNITY_OUTPUT_COMPLETE() unityOutputComplete()

#ifdef __cplusplus
}
#endif
