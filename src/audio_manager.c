/**
 * @file audio_manager.c
 * @brief Audio playback manager with smooth transitions and silence generation
 * 
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 * 
 */

#include <audio_manager.h>

#include <stm32f4_discovery_audio_codec.h>
#include <semphr.h>
#include <task.h>

// Audio state structure
typedef struct {
  uint16_t *currentBuffer;
  uint32_t currentSize;
  uint32_t currentPosition;
  uint8_t isPlaying;
} AudioState_t;

// Double buffering for smooth playback
#define AUDIO_BUFFER_SIZE 8192
static uint16_t audioBuffer1[AUDIO_BUFFER_SIZE];
static uint16_t audioBuffer2[AUDIO_BUFFER_SIZE];
static uint16_t *activeBuffer = audioBuffer1;
static uint16_t *backBuffer = audioBuffer2;

// Silence buffer for smooth transitions
#define SILENCE_BUFFER_SIZE 2048
static uint16_t silenceBuffer[SILENCE_BUFFER_SIZE] = {0};

// Audio state
static AudioState_t audioState = {0};
StaticSemaphore_t xAudioMutex;
static SemaphoreHandle_t audioMutex;

/**
 * @brief Initialize the audio manager
 */
void AudioManager_Init(void) {
  // Create mutex for thread safety
  audioMutex = xSemaphoreCreateBinaryStatic(&xAudioMutex);

  // Initialize silence buffer with DC offset for CS43L22
  // The CS43L22 expects signed 16-bit data with 0x8000 as the center value
  for (int i = 0; i < SILENCE_BUFFER_SIZE; i++) {
    silenceBuffer[i] = 0x8000; // Center value for silence
  }

  // Initialize audio state
  audioState.isPlaying = 0;
  audioState.currentBuffer = NULL;
  audioState.currentSize = 0;
  audioState.currentPosition = 0;
}

/**
 * @brief Fill buffer with audio data
 * @param destBuffer Destination buffer
 * @param size Number of samples to fill
 * @return 1 if audio is still playing, 0 if finished
 */
static uint8_t FillAudioBuffer(uint16_t *destBuffer, uint32_t size) {
  uint32_t i;
  uint32_t samplesToFill = size;
  uint32_t samplesRemaining;

  if (!audioState.isPlaying || !audioState.currentBuffer) {
    // Fill with silence
    for (i = 0; i < size; i++) {
      destBuffer[i] = 0x8000;
    }
    return 0;
  }

  samplesRemaining = (audioState.currentSize - audioState.currentPosition) / 2;

  if (samplesRemaining < samplesToFill) {
    samplesToFill = samplesRemaining;
  }

  // Copy audio data
  memcpy(destBuffer, &audioState.currentBuffer[audioState.currentPosition / 2],
         samplesToFill * 2);

  // Fill remaining buffer with silence if needed
  if (samplesToFill < size) {
    for (i = samplesToFill; i < size; i++) {
      destBuffer[i] = 0x8000;
    }
  }

  audioState.currentPosition += samplesToFill * 2;

  return audioState.isPlaying;
}

/**
 * @brief DMA half-transfer callback - fill the first half of the buffer
 */
void AudioManager_HalfTransferCallback(void) {
  if (xSemaphoreTake(audioMutex, 0) == pdTRUE) {
    FillAudioBuffer(backBuffer, AUDIO_BUFFER_SIZE / 2);
    xSemaphoreGive(audioMutex);
  }
}

/**
 * @brief DMA transfer complete callback - fill the second half of the buffer
 */
void AudioManager_TransferCompleteCallback(void) {
  if (xSemaphoreTake(audioMutex, 0) == pdTRUE) {
    FillAudioBuffer(backBuffer + AUDIO_BUFFER_SIZE / 2, AUDIO_BUFFER_SIZE / 2);

    // Swap buffers
    uint16_t *temp = activeBuffer;
    activeBuffer = backBuffer;
    backBuffer = temp;

    xSemaphoreGive(audioMutex);
  }
}

/**
 * @brief Start playing an audio sample with smooth transition
 * @param buffer Pointer to audio data
 * @param size Size of audio data in bytes
 */
void AudioManager_PlaySample(uint16_t *buffer, uint32_t size) {
  if (xSemaphoreTake(audioMutex, portMAX_DELAY) == pdTRUE) {

    // Setup new playback
    audioState.currentBuffer = buffer;
    audioState.currentSize = size;
    audioState.currentPosition = 0;
    audioState.isPlaying = 1;

    // Prepare initial buffer
    FillAudioBuffer(activeBuffer, AUDIO_BUFFER_SIZE);

    // Start DMA with double buffering
    EVAL_AUDIO_Play(activeBuffer, AUDIO_BUFFER_SIZE * 2);

    xSemaphoreGive(audioMutex);
  }
}

/**
 * @brief Stop audio playback
 */
void AudioManager_Stop(void) {
  if (xSemaphoreTake(audioMutex, portMAX_DELAY) == pdTRUE) {
    xSemaphoreGive(audioMutex);
  }
}

/**
 * @brief Check if audio is currently playing
 * @return 1 if playing, 0 if not
 */
uint8_t AudioManager_IsPlaying(void) {
  uint8_t playing = 0;
  if (xSemaphoreTake(audioMutex, portMAX_DELAY) == pdTRUE) {
    playing = audioState.isPlaying;
    xSemaphoreGive(audioMutex);
  }
  return playing;
}
