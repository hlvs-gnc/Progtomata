/**
 * @file main.c
 *
 * @brief STM32F4 FreeRTOS LED Control and Button Handling Application.
 *
 * This file implements an embedded application using FreeRTOS on an STM32F4
 * microcontroller. The application controls LEDs and responds to user button
 * presses, demonstrating task scheduling and semaphore usage.
 *
 * @details
 * - **Button Task**: Detects button presses and resets the LED blink delay to
 *   the minimum value.
 * - **Blink Task**: Cycles through LEDs with variable delay, adjusting the
 *   delay dynamically.
 * - Uses FreeRTOS static task creation and Core Coupled Memory (CCM) for
 *   optimized performance.
 * - Implements basic input handling and GPIO initialization for STM32F4.
 *
 * Features:
 * - LED sequencing with dynamic delay adjustment.
 * - Button press detection to modify LED behavior.
 * - Static task allocation with CCM for optimized memory usage.
 * - Real-time operation using FreeRTOS.
 *
 * Dependencies:
 * - STM32F4xx Standard Peripheral Library
 * - STM32F4-Discovery Board Support Package (BSP)
 * - FreeRTOS Kernel
 *
 * Hardware:
 * - Board: STM32F4-Discovery
 * - LEDs: Connected to GPIOD (Pins 12, 13, 14, 15)
 * - User Button: Connected to GPIOA (Pin 0)
 *
 * @note Ensure that FreeRTOS and STM32 peripheral drivers are correctly
 *       configured before compiling the project.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 */

// System libraries
#include <math.h>
#include <stdbool.h>

// Real-time operating system
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

// STD Library
#include <stm32f4xx.h>
#include <stm32f4xx_tim.h>

// STM32F4 Discovery
#include <stm32f4_discovery.h>
#include <stm32f4_discovery_audio_codec.h>

// Peripherals
#include <lcd.h>

// Drivers
#include <uart_driver.h>

// System definitions
#include <progtomata_system.h>

// Information logging
#include <trace.h>

// Audio samples
#include <fxTom.h>
#include <snare.h>
#include <kick.h>
#include <hihat.h>

#define SOUNDSIZE1 (6044)    // size of snare
#define SOUNDSIZE2 (12701)  // size of fxTom
#define SOUNDSIZE3 (12690)  // size of kick
#define SOUNDSIZE4 (14736)  // size of hihat
// TOTAL SIZE: 92,342
#define BUFFERSIZE (32768)

int16_t playbackBuffer[BUFFERSIZE];

uint64_t u64IdleTicksCnt = 0;  // Counts when the OS has no task to execute.
uint64_t tickTime = 0;         // Counts OS ticks (default = 1000Hz).

// Declare a StaticSemaphore_t variable
StaticSemaphore_t xSemaphoreBuffer;

// Create the binary semaphore
SemaphoreHandle_t xSemaphore;

xSemaphoreHandle xSemLCD = NULL;

/**
 * @brief Handles button press events to adjust LED blink delay.
 *
 * Monitors the state of the user button connected to GPIOA Pin 0.
 * If a button press is detected, it resets the blink delay to its
 * minimum value. Designed as a FreeRTOS task that runs continuously.
 *
 * @param[in] p Pointer to task parameters (unused).
 */
void vButtonTask(void *p);

/**
 * @brief Controls LED blinking behavior with variable delay.
 *
 * Cycles through four LEDs connected to GPIOD Pins 12, 13, 14, and 15.
 * Adjusts the blinking delay dynamically within defined limits.
 * Runs as a FreeRTOS task, demonstrating time-based scheduling.
 *
 * @param[in] p Pointer to task parameters (unused).
 */
void vBlinkTask(void *p);

void vPlaybackTask(void *pvparameters);
void vModifyBuffer(void *pvparameters);

uint16_t playback_delay = 500;  // interval delay in ms
uint8_t beats = 3;

// Indicates whether a sound is playing (1) or not (0)
uint8_t status = 0;

/**
 * @brief Configures the user button GPIO (PA0) as an input.
 *
 * Initializes GPIO settings, enabling input mode without pull-up or
 * pull-down resistors. Prepares the pin to detect user button presses.
 */
void config_userbutton(void);

/**
 * @brief Initializes GPIO pins connected to LEDs.
 *
 * Prepares GPIOD Pins 12, 13, 14, and 15 for output mode. Ensures
 * proper configuration for controlling LED states.
 */
void leds_init(void);

void SystemClock_Config(void);

/**
 * @brief Configure the system clock to 168 MHz (for STM32F4)
 */
void SystemClock_Config(void) {
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);

  // Wait for HSE to be ready
  while (RCC_WaitForHSEStartUp() == ERROR);

  // PLL config: HSE, VCO 336MHz
  RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);  
  RCC_PLLCmd(ENABLE);

  // Wait for PLL to be ready
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  // Update clock speed values
  SystemCoreClockUpdate();

  // Enable clocks for peripherals
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
}

int main(void) {
  SystemInit();
  SystemClock_Config();

  config_userbutton();
  leds_init();
  uart_init();

  LCD_Init();

  LCD_GotoXY(0, 0);
  LCD_WriteString("****************");
  LCD_GotoXY(1, 0);
  LCD_WriteString("*PROGTOMATA2000*");

  TraceInit();

  TRice(iD(7167), "info: 🐛 PROGTOMATA2000 System initialized\n");

  xSemaphore = xSemaphoreCreateBinaryStatic(&xSemaphoreBuffer);
  if (xSemaphore == NULL) {
    // Handle error: Semaphore creation failed
    TRice(iD(2542), "error: Semaphore creation failed\n");
  }

  EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);
  if (EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 90, 22050) != 0) {
    TRice(iD(7317), "msg: Audio codec initialization failed\n");
  }

  TRice(iD(5254), "msg: Audio setup complete\n");

  // Create button task
  buttonTaskHandle =
      xTaskCreateStatic(vButtonTask, "ButtonTask", BUTTON_TASK_STACK_SIZE, NULL,
                        1, buttonTaskStack, &buttonTaskBuffer);

  // Create blink task
  blinkTaskHandle =
      xTaskCreateStatic(vBlinkTask, "BlinkTask", BLINK_TASK_STACK_SIZE, NULL, 1,
                        blinkTaskStack, &blinkTaskBuffer);

  playbackTaskHandle =
      xTaskCreateStatic(vPlaybackTask, "PlayTask", PLAYBACK_TASK_STACK_SIZE,
                        NULL, 1, playbackTaskStack, &playbackTaskBuffer);

  vTaskStartScheduler();  // This shall never return

  for (;;) {
  }
}

void vButtonTask(void *p) {
  uint8_t prevStatePA0 = Bit_RESET;  // Previous state for PA0
  uint8_t prevStatePD1 = Bit_RESET;  // Previous state for PD1
  uint8_t prevStatePD2 = Bit_RESET;  // Previous state for PD2

  while (1) {
    // Read current states
    uint8_t currentStatePA0 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
    uint8_t currentStatePD1 = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1);
    uint8_t currentStatePD2 = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2);

    // Handle PA0 (onboard button)
    if (currentStatePA0 == Bit_SET && prevStatePA0 == Bit_RESET) {
      GPIO_SetBits(GPIOD,
                   GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
      kBlinkDelay = MIN_DELAY;
      kBlinkStep = MIN_DELAY;
      TRice(iD(5676), "Onboard button pressed\n");
      TRice(iD(4528), "Reset blink to minimum delay\n");
    }
    prevStatePA0 = currentStatePA0;

    // Handle PD1 (Trigger Playback for Sound 1)
    if (currentStatePD1 == Bit_RESET && prevStatePD1 == Bit_SET) {
      // Trigger playback task for Sound 1
      int i = 0;
      for (; i < BUFFERSIZE; ++i) {
        if (i < SOUNDSIZE1) {
          playbackBuffer[i] = (int16_t) (snare[i]- 32768);
        } else {
          playbackBuffer[i] = 0;
        }
      }

      xSemaphoreGive(xSemaphore);    // Signal playback task
      TRice(iD(1234), "Button 1 pressed: Triggering playback for Sound 1\n");
    }
    prevStatePD1 = currentStatePD1;

    // Handle PD2 (Trigger Playback for Sound 2)
    if (currentStatePD2 == Bit_RESET && prevStatePD2 == Bit_SET) {
      // Trigger playback task for Sound 2
      int i = 0;
      for (; i < BUFFERSIZE; ++i) {
        if (i < SOUNDSIZE2) {
          playbackBuffer[i] = (int16_t) (fxTom[i]- 32768);
        } else {
          playbackBuffer[i] = 0;
        }
      }
      xSemaphoreGive(xSemaphore);    // Signal playback task
      TRice(iD(5678), "Button 2 pressed: Triggering playback for Sound 2\n");
    }
    prevStatePD2 = currentStatePD2;

    // Add a debounce delay
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // Delete the task if it ever exits
  vTaskDelete(NULL);
}

void vBlinkTask(void *p) {
  while (1) {
    STM_EVAL_LEDOn(LED3);
    vTaskDelay(kBlinkDelay);

    STM_EVAL_LEDOff(LED3);

    STM_EVAL_LEDOn(LED4);
    vTaskDelay(kBlinkDelay);

    STM_EVAL_LEDOff(LED4);

    STM_EVAL_LEDOn(LED6);
    vTaskDelay(kBlinkDelay);

    STM_EVAL_LEDOff(LED6);

    STM_EVAL_LEDOn(LED5);
    vTaskDelay(kBlinkDelay);

    STM_EVAL_LEDOff(LED5);

    // Adjust delay
    kBlinkDelay += kBlinkStep;

    if (kBlinkDelay >= MAX_DELAY) {
      kBlinkStep -= MIN_DELAY;  // Reverse step direction

    } else if (kBlinkDelay < MIN_DELAY || kBlinkStep == 0) {
      kBlinkDelay = MIN_DELAY;
      kBlinkStep = MIN_DELAY;
    }
    TRice(iD(2089), "att:🐁 Blink LEDs cycle: blinkStep=%d; blinkDelay=%d\n",
          kBlinkStep, kBlinkDelay);
  }

  vTaskDelete(NULL);
}

/*
 * Plays sound in a loop
 * "delay" determine the tempo - how often it plays
 * A semaphore is used for synchronisation so that you can only play when the
 * previous sound has completed The "playbackComplete" callback has the other
 * end of the semaphore
 */
void vPlaybackTask(void *pvparameters) {
  for (;;) {
    while (xSemaphoreTake(xSemaphore, (portTickType)254) == pdFALSE) {
    };

    EVAL_AUDIO_Play((uint16_t *)(playbackBuffer), 16384);
    status = 1;
    vTaskDelay(playback_delay / portTICK_RATE_MS);
  }
}

void config_userbutton(void) {
  // Enable clock for GPIOD
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Declare a variable of type struct GPIO_InitTypeDef
  GPIO_InitTypeDef GPIO_InitStructure;

  // Set pin mode to input
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;

  // Select pin PA0 only
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;

  // Set no internal pull-up or pull-down resistor
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  // Initialize PA0 pins by passing port name and address of PushButton struct
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure PD1 and PD2 (new buttons) as input with internal pull-up
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  // Enable internal pull-up
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void leds_init(void) {
  // Initialize board LEDs
  STM_EVAL_LEDInit(LED3);

  STM_EVAL_LEDInit(LED4);

  STM_EVAL_LEDInit(LED5);

  STM_EVAL_LEDInit(LED6);

  // External LEDs
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
                                GPIO_Pin_15 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void) {
  TRice(iD(1975), "EVAL_AUDIO_GetSampleCallBack\n");
  return 1;
}

/*
 * Called when buffer has been played out
 * Releases semaphore and wakes up task which modifies the buffer
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size) {
  status = 0;
  TRice(iD(3989), "EVAL_AUDIO_TransferComplete_CallBack\n");
}

void EVAL_AUDIO_Error_CallBack(void *pData) {
  TRice(iD(6126), "EVAL_AUDIO_Error_CallBack\n");
}

uint32_t Codec_TIMEOUT_UserCallback(void) {
  TRice(iD(4868), "Codec_TIMEOUT_UserCallback\n");
  return 1;
}
