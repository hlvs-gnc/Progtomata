/**
 * @file main.c
 *
 * @brief Audio sequencer & synthesizer project main source code file.
 *
 * @details This file implements an embedded application for the system.
 * Implementation of tasks (sequencer, audio playback, effect chain)
 * and interface functionalities (LED, button pannel, display).
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

// System libraries
#include <math.h>
#include <stdbool.h>

// System definition/ configuration
#include <core.h>
#include <progtomata_system.h>
#include <tasks.h>

// Real-time operating system
#include <FreeRTOS.h>
#include <hooks.h>
#include <semphr.h>
#include <task.h>

// Displays
#include <interface.h>
#include <lcd.h>
#include <oled.h>

// Communication drivers
#include <uart_driver.h>

// Information logging
#include <trace.h>

static void sequencer_setBpm(uint16_t bpm) {
  if (bpm < MIN_BPM) {
    bpm = MIN_BPM;
  }
  if (bpm > MAX_BPM) {
    bpm = MAX_BPM;
  }

  currBpm = bpm;
  nbrSamplesStep = (SAMPLE_RATE * 60U) / bpm;
  bpmInterval = 60000 / currBpm;
}

static void mixSegment(uint32_t dst, uint32_t cnt, uint32_t ofs) {
  // Cache stepIndex to avoid race condition during rendering
  uint8_t currentStepIdx = stepIndex;

  // Initialize buffer to silence
  for (uint32_t i = 0; i < cnt; ++i) {
    playbackBuffer[dst + i] = 0;
  }

  // Mix in samples that are triggered at current step
  for (uint8_t s = 0; s < NUM_SAMPLES; ++s) {
    if (!stepGrid[s][currentStepIdx]) {
      continue;
    }

    // Bounds check to prevent buffer overflow
    if (ofs >= sampleLen[s]) {
      continue;
    }

    const int16_t *src = sampleData[s] + ofs;
    uint32_t len = sampleLen[s] - ofs;
    if (len > cnt) {
      len = cnt;
    }

    // Mix with gain reduction (>>1 = /2 for headroom)
    for (uint32_t i = 0; i < len; ++i) {
      int32_t v = playbackBuffer[dst + i] + (src[i] >> 1);
      playbackBuffer[dst + i] = CLIP16(v);
    }
  }
}

static void renderHalf(uint32_t base) {
  toRender = BUFFERSIZE / 2;

  while (toRender) {
    stepSamples = nbrSamplesStep;
    leftInStep = stepSamples - stepPos;
    chunk = (leftInStep < toRender) ? leftInStep : toRender;

    mixSegment(base + (BUFFERSIZE / 2 - toRender), chunk, stepPos);

    stepPos += chunk;
    toRender -= chunk;

    if (stepPos == stepSamples) {
      stepPos = 0;
      stepIndex = (stepIndex + 1) % NUM_STEPS;
    }
  }
}

int main(void) {
  SystemInit();
  systemClock_config();

  // Initialize OLED display
  OLED_Init();

  // Clear the display buffer
  OLED_Clear();

  // Text width: 16 characters * 6 pixels = 96 pixels
  // Starting X position: (132 - 96) / 2 = 18
  OLED_DrawString(18, 16, "*PROGTOMATA2000*");

  // Update the display to show the content
  OLED_UpdateScreen();

  // Initialize LCD display
  LCD_Init();

  LCD_GotoXY(0, 0);
  LCD_WriteString("****************");
  LCD_GotoXY(1, 0);
  LCD_WriteString("*PROGTOMATA2000*");

  // Initialize system interface
  userButton_config();
  boardLeds_config();
  interface_init();

  // Initialize UART driver
  uart_init();

  // Initialize trace
  TraceInit();

  // Create the semaphore before any button processing
  xButtonSemaphoreHandle =
      xSemaphoreCreateBinaryStatic(&xButtonSemaphoreStatic);
  if (xButtonSemaphoreHandle == NULL) {
#ifdef LOG_TRICE
    TRice(iD(3926), "error: Button semaphore creation failed\n");
#endif
  }

  EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);

  if (EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 85, SAMPLE_RATE) != 0) {
#ifdef LOG_TRICE
    TRice(iD(4708), "msg: Audio codec initialization failed\n");
#endif
  }

#ifdef LOG_TRICE
  TRice(iD(3720), "msg: Audio setup complete\n");

  // Set master tempo
  sequencer_setBpm(130);
#endif

  // Start audio playback
  if (EVAL_AUDIO_Play((uint16_t *)playbackBuffer, BUFFERSIZE) != 0) {
#ifdef LOG_TRICE
    TRice(iD(3630), "error: Failed to start audio playback\n");
#endif
  }

  sampleButtonTaskHandle = xTaskCreateStatic(
      vButtonSampleTask, "SampleButtonTask", SAMPLE_TASK_STACK_SIZE, NULL,
      SAMPLE_TASK_PRIORITY, sampleButtonTaskStack, &sampleButtonTaskBuffer);

  stepButtonTaskHandle = xTaskCreateStatic(
      vButtonStepTask, "StepButtonTask", STEP_TASK_STACK_SIZE, NULL,
      STEP_TASK_PRIORITY, stepButtonTaskStack, &stepButtonTaskBuffer);

  blinkTaskHandle =
      xTaskCreateStatic(vBlinkTask, "BlinkTask", BLINK_TASK_STACK_SIZE, NULL,
                        BLINK_TASK_PRIORITY, blinkTaskStack, &blinkTaskBuffer);

  waveformTaskHandle = xTaskCreateStatic(
      vWaveformTask, "WaveformTask", WAVEFORM_TASK_STACK_SIZE, NULL,
      WAVEFORM_TASK_PRIORITY, waveformTaskStack, &waveformTaskBuffer);

#ifdef LOG_TRICE
  TRice(iD(1106), "info: 🐛 PROGTOMATA2000 System initialized\n");
#endif

  vTaskStartScheduler(); // This shall never return

  for (;;) {
  }
}

void vButtonSampleTask(void *p) {
  uint8_t prevStatePD1 = Bit_RESET;
  uint8_t prevStatePD2 = Bit_RESET;

  while (1) {
    // Read current states
    uint8_t currentStatePD1 = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1);
    uint8_t currentStatePD2 = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2);

    //  PD1 => PD5
    if (currentStatePD1 == Bit_RESET) {
      // Button is pressed
      GPIO_SetBits(GPIOD, GPIO_Pin_5);
    } else {
      // Button is not pressed
      GPIO_ResetBits(GPIOD, GPIO_Pin_5);
    }

    // Handle PD1 (Trigger Playback for Sound 1)
    if (currentStatePD1 == Bit_RESET && prevStatePD1 == Bit_SET) {
      sampleButton = 0x0000;
#ifdef LOG_TRICE
      TRice(iD(5828), "Sample 0 (kick)\n");
#endif
      // Turn on external LED on PD5 to indicate Button 1 action
      GPIO_SetBits(GPIOD, GPIO_Pin_5);
    }
    prevStatePD1 = currentStatePD1;

    // PD2 => PD6
    if (currentStatePD2 == Bit_RESET) {
      GPIO_SetBits(GPIOD, GPIO_Pin_6);
    } else {
      GPIO_ResetBits(GPIOD, GPIO_Pin_6);
    }

    // Handle PD2 (Trigger Playback for Sound 2)
    if (currentStatePD2 == Bit_RESET && prevStatePD2 == Bit_SET) {
      sampleButton = 0x0001;
#ifdef LOG_TRICE
      TRice(iD(4156), "Sample 1 (hi-hat) \n");
#endif
      // Turn on external LED on PD6 to indicate Button 2 action
      GPIO_SetBits(GPIOD, GPIO_Pin_6);
    }
    prevStatePD2 = currentStatePD2;

    // Debounce delay
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Delete the task if it ever exits
  vTaskDelete(NULL);
}

void vButtonStepTask(void *pvParameters) {
  static uint16_t stepButton = 0;
  static uint8_t wasPressed = 0;

  // Initially, give the semaphore so the task doesn't block immediately
  xSemaphoreGive(xButtonSemaphoreHandle);

  for (;;) {
    // Wait a short time for the event semaphore
    if (xSemaphoreTake(xButtonSemaphoreHandle, pdMS_TO_TICKS(30)) == pdTRUE) {
      // Poll the button value
      stepButton = interface_readButtonStep();

      // Process an event transitioning from idle to a valid state (0-3)
      if ((stepButton != STEPIDLE_VALUE) && (wasPressed == 0)) {
        if (stepButton <= NUM_STEPS && sampleButton < NUM_SAMPLES) {
          stepGrid[sampleButton][stepButton - 1] =
              1 - stepGrid[sampleButton][stepButton - 1];
#ifdef LOG_TRICE
          TRice(iD(7398), "info: Select step %d | sample %d | state %d \n",
                stepButton, sampleButton,
                stepGrid[sampleButton][stepButton - 1]);
#endif
        }
        wasPressed = 1; // Mark that this press has been processed
      }
      // When the button returns to idle, clear the flag
      if (stepButton == STEPIDLE_VALUE) {
        wasPressed = 0;
      }
    }

    // Debounce delay
    vTaskDelay(pdMS_TO_TICKS(10));

    // Re-give the semaphore to trigger the next poll cycle
    xSemaphoreGive(xButtonSemaphoreHandle);
  }
}

void vBlinkTask(void *p) {
  const Led_TypeDef leds[NUM_STEPS] = {LED3, LED4, LED6, LED5};
  uint8_t lastStepIndex = 0xFF; // Invalid initial value to force first update

  while (1) {
    // Get current step from the sequencer
    uint8_t currentStep = playHeadStep;

    // Check if step changed
    if (currentStep != lastStepIndex) {
      // Turn off all LEDs first
      for (uint8_t i = 0; i < NUM_STEPS; ++i) {
        STM_EVAL_LEDOff(leds[i]);
      }

      // Check if current step has any samples triggered
      bool currentStepHasSample = false;
      for (uint8_t s = 0; s < NUM_SAMPLES; ++s) {
        if (stepGrid[s][currentStep]) {
          currentStepHasSample = true;
          break;
        }
      }

      // Turn on LED for current step only if it has samples
      if (currentStepHasSample) {
        STM_EVAL_LEDOn(leds[currentStep]);
      }

      lastStepIndex = currentStep;

#ifdef BLINK
      TRice(iD(6506), "att:🐁 LED step: %d, active: %d\n", currentStep,
            currentStepHasSample);
#endif
    }

    // Poll frequently to catch step changes
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  vTaskDelete(NULL);
}

void vWaveformTask(void *pvParameters) {
  (void)pvParameters;

  // Small delay to allow system initialization
  vTaskDelay(pdMS_TO_TICKS(100));

  while (1) {
    // Clear display
    OLED_Clear();

    // Draw waveform from playback buffer
    // Display width: 128 pixels, use 126 to leave margins
    OLED_DrawWaveform(playbackBuffer, BUFFERSIZE, 1, 126);

    // Update screen with new content
    OLED_UpdateScreen();

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  vTaskDelete(NULL);
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void) {
#ifdef LOG_TRICE
  TRice(iD(6204), "GetSample\n");
#endif
  return 1;
}

void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size) {
  // Render audio for first half of buffer
  renderHalf(0);

  // Update play head position (atomic read of volatile)
  playHeadStep = stepIndex;

#ifdef LOG_AUDIO_BUFFER
  TRice(iD(4918), "HalfTransfer. pBuffer: %x; Size: %d\n", pBuffer, Size);
#endif
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size) {
  // Render audio for second half of buffer
  renderHalf(BUFFERSIZE / 2);

  // Update play head position (atomic read of volatile)
  playHeadStep = stepIndex;

#ifdef LOG_AUDIO_BUFFER
  TRice(iD(7634), "TransferComplete. pBuffer: %x; Size: %d\n", pBuffer, Size);
#endif
}

void EVAL_AUDIO_Error_CallBack(void *pData, int32_t errorType) {
#ifdef LOG_TRICE
  TRice(iD(5387), "error: Error. pData: %x type: %x \n", pData, errorType);
#endif
}

uint32_t Codec_TIMEOUT_UserCallback(void) {
#ifdef LOG_TRICE
  TRice(iD(1479), "Codec_TIMEOUT_User\n");
#endif
  return 1;
}

void vApplicationTickHook(void) { ++tickTime; }

void vApplicationIdleHook(void) { ++u64IdleTicksCnt; }
