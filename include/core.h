/**
 * @file core.h
 *
 * @brief Core header file for the functional task instances.
 *
 * @details This file provides the core definitions, macros, and function
 * declarations for the FreeRTOS task instances. It includes the necessary
 * header files, defines the application constants and data elements
 * that are used in the application.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef CORE_H_
#define CORE_H_

#include <stdint.h>

// Mono Audio samples
#include <kick_22050_mono.h>
#include <kick_aether.h>
#include <openhat_22050_mono.h>

/// @brief Size of the audio playback buffer in bytes
#define BUFFERSIZE (2048)

/// @brief Number of audio samples available in the sequencer
#define NUM_SAMPLES 2

/// @brief Number of steps in the sequencer pattern
#define NUM_STEPS 4

/// @brief Macro to clamp a 32-bit value to 16-bit signed range
#define CLIP16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

/// @brief Number of samples in kick mono audio file
#define SOUNDSIZE1 (6918)

/// @brief Number of samples in open hat mono audio file
#define SOUNDSIZE2 (9882)

/// @brief Number of samples in open hat mono audio file
#define SOUNDSIZE3 (12959)

/// @brief I²S sample-rate
#define SAMPLE_RATE I2S_AudioFreq_22k

/// @brief Tempo limits
#define MIN_BPM 40U
#define MAX_BPM 200U

/// @brief Audio sample identifiers
typedef enum { SAMPLE1 = 0, SAMPLE2 } audioSampleId;

/// @brief Step grid to hold sample triggers for each step
uint8_t stepGrid[NUM_SAMPLES][NUM_STEPS] = {0};

/// @brief Buffer for storing audio data for playback
int16_t playbackBuffer[BUFFERSIZE] = {0};

/// @brief current tempo
static volatile uint16_t currBpm;

/// @brief Samples per sequencer step
static volatile uint32_t nbrSamplesStep;

/// @brief Counts when the OS has no task to execute
volatile uint64_t u64IdleTicksCnt = 0;

/// @brief Counts OS ticks (default = 1000Hz)
volatile uint64_t tickTime = 0;

/// @brief Interval between BPM ticks in ms
static uint32_t bpmInterval = 0;

/// @brief Currently selected sample button index (0 to NUM_SAMPLES-1)
static uint16_t sampleButton = 0;

/// @brief Temporary storage for samples per step calculation
uint32_t stepSamples = 0;

/// @brief Current step index (volatile - accessed from interrupt)
static volatile uint8_t playHeadStep = 0;

/// @brief Grid column currently playing (volatile - modified in interrupt)
static volatile uint8_t stepIndex = 0;

/// @brief Samples already rendered in current beat-step
static uint32_t stepPos = 0;

/// @brief Remaining samples left to render in current step
uint32_t leftInStep = 0;

/// @brief Size of current audio chunk being rendered
uint32_t chunk = 0;

/// @brief Number of samples remaining to render in current buffer half
uint32_t toRender = 0;

/// @brief Array of audio sample data pointers
static const int16_t *const sampleData[NUM_SAMPLES] = {kick_aether_mono,
                                                       openhat_mono};

/// @brief Array of sample lengths in bytes
static const uint32_t sampleLen[NUM_SAMPLES] = {SOUNDSIZE3, SOUNDSIZE2};

#endif // CORE_H_
