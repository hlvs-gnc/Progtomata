/**
 * @file audio_manager.h
 * @brief Audio playback manager interface
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 */

#ifndef AUDIO_MANAGER_H_
#define AUDIO_MANAGER_H_

#include <FreeRTOS.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief Initialize the audio manager
 */
void AudioManager_Init(void);

/**
 * @brief Start playing an audio sample with smooth transition
 * @param buffer Pointer to audio data
 * @param size Size of audio data in bytes
 */
void AudioManager_PlaySample(uint16_t *buffer, uint32_t size);

/**
 * @brief Stop audio playback with smooth fade-out
 */
void AudioManager_Stop(void);

/**
 * @brief Check if audio is currently playing
 * @return 1 if playing, 0 if not
 */
uint8_t AudioManager_IsPlaying(void);

/**
 * @brief DMA half-transfer callback
 * @note This should be called from EVAL_AUDIO_HalfTransfer_CallBack
 */
void AudioManager_HalfTransferCallback(void);

/**
 * @brief DMA transfer complete callback
 * @note This should be called from EVAL_AUDIO_TransferComplete_CallBack
 */
void AudioManager_TransferCompleteCallback(void);

#endif /* AUDIO_MANAGER_H_ */
