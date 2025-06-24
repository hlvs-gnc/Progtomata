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
#include <stm32f4xx_flash.h>
#include <stm32f4xx_pwr.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_tim.h>

// STM32F4 Discovery
#include <stm32f4_discovery.h>
#include <stm32f4_discovery_audio_codec.h>

// Interface
#include <interface.h>

// Peripherals
#include <lcd.h>
#include <oled.h>

// Drivers
#include <uart_driver.h>

// System definitions
#include <progtomata_system.h>

// Information logging
#include <trace.h>

// FreeRTOS Hook functions
#include <hooks.h>

// Mono Audio samples
#include <kick_22050_mono.h>
#include <openhat_22050_mono.h>

// Stereo Audio samples
#include <kick_44100_stereo.h>

#define NUM_SAMPLES 2
#define NUM_STEPS 4

// Step grid to hold sample triggers for each step
// stepGrid[step][sample] = 1 if sample should play at this step
uint8_t stepGrid[NUM_SAMPLES][NUM_STEPS] = {0};

// Current step index
static uint8_t currentStep = 0;

// N Buttons' states - each bit indicate respective button's ON/OFF state
static uint16_t sampleButtonState = 0;

static uint16_t stepButtonState = 0;

// Indicates whether a sound is playing (1) or not (0)
static uint8_t status = 0;

/**
 * @brief Configure the system clock to 168 MHz (for STM32F4)
 */
void SystemClock_Config(void) {
  // Enable the power interface clock and configure voltage regulator
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_MainRegulatorModeConfig(PWR_Regulator_Voltage_Scale1);

  // Enable HSE
  RCC_HSEConfig(RCC_HSE_ON);
  while (RCC_WaitForHSEStartUp() == ERROR);

  // Configure Flash wait states
  FLASH_SetLatency(FLASH_Latency_1);
  FLASH_PrefetchBufferCmd(ENABLE);

  // Configure AHB, APB1, and APB2 prescalers
  // AHB @ 168 MHz, APB2 @ 84 MHz, APB1 @ 42 MHz
  RCC_HCLKConfig(RCC_SYSCLK_Div1);  // AHB = SYSCLK/1 = 168 MHz
  RCC_PCLK2Config(RCC_HCLK_Div2);   // APB2 = AHB/2 = 84 MHz
  RCC_PCLK1Config(RCC_HCLK_Div4);   // APB1 = AHB/4 = 42 MHz

  // Set up main PLL to 168 MHz system clock
  // (HSE=8 MHz, VCO=336 MHz, /2 => 168 MHz, /7 => 48 MHz USB)
  RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
  RCC_PLLCmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

  // Select the main PLL as system clock source
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  while (RCC_GetSYSCLKSource() != 0x08);  // 0x08 = PLL used as sysclk

  // Configure the I2S PLL for a proper I2S clock
  RCC_PLLI2SConfig(271, 2);
  RCC_PLLI2SCmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY) == RESET);

  // Update the SystemCoreClock global variable
  SystemCoreClockUpdate();

  // Enable peripheral clocks
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
}

int main(void) {
  SystemInit();
  SystemClock_Config();

  Interface_Init();

  TraceInit();

  config_userbutton();
  leds_init();
  uart_init();
  
  LCD_Init();

  LCD_GotoXY(0, 0);
  LCD_WriteString("****************");
  LCD_GotoXY(1, 0);
  LCD_WriteString("*PROGTOMATA2000*");

  TRice(iD(5882), "info: 🐛 PROGTOMATA2000 System initialized\n");

  xSemaphorePlayback =
    xSemaphoreCreateBinaryStatic(&xSemaphorePlaybackStatic);
  if (xSemaphorePlayback == NULL) {
    // Handle error: Semaphore creation failed
    TRice(iD(3325), "error: Semaphore creation failed\n");
  }

  xSemaphoreModifyBuffer =
      xSemaphoreCreateBinaryStatic(&xSemaphoreModifyBufferStatic);
  if (xSemaphoreModifyBuffer == NULL) {
    // Handle error: Semaphore creation failed
    TRice(iD(5995), "error: Semaphore creation failed\n");
  }

  EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);

  if (EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 90, I2S_AudioFreq_44k) != 0) {
    TRice(iD(5761), "msg: Audio codec initialization failed\n");
  }

  memset(playbackBuffer, 0, BUFFERSIZE*sizeof(int16_t));

  TRice(iD(2233), "msg: Audio setup complete\n");

  sequencerTaskHandle = xTaskCreateStatic(
      vSequencerTask, "SequencerTask", SEQUENCER_TASK_STACK_SIZE, NULL,
      SEQUENCER_TASK_PRIORITY, sequencerTaskStack, &sequencerTaskBuffer);

  playbackTaskHandle = xTaskCreateStatic(
      vPlaybackTask, "PlaybackTask", PLAYBACK_TASK_STACK_SIZE, NULL,
      PLAYBACK_TASK_PRIORITY, playbackTaskStack, &playbackTaskBuffer);

  modifyBufferTaskHandle = xTaskCreateStatic(
      vModifyBufferTask, "ModifyBufferTask", MODIFYBUFFER_TASK_STACK_SIZE, NULL,
      MODIFYBUFFER_TASK_PRIORITY, modifyBufferTaskStack,
      &modifyBufferTaskBuffer);

  sampleButtonTaskHandle = xTaskCreateStatic(
      vButtonSampleTask, "SampleButtonTask", SAMPLE_BUTTON_TASK_STACK_SIZE,
      NULL, SAMPLE_BUTTON_TASK_PRIORITY, sampleButtonTaskStack,
      &sampleButtonTaskBuffer);

  stepButtonTaskHandle = xTaskCreateStatic(
      vButtonStepTask, "StepButtonTask", STEP_BUTTON_TASK_STACK_SIZE, NULL,
      STEP_BUTTON_TASK_PRIORITY, stepButtonTaskStack, &stepButtonTaskBuffer);

  // Create blink task
  blinkTaskHandle = xTaskCreateStatic(
      vBlinkTask, "BlinkTask", BLINK_TASK_STACK_SIZE, NULL,
      BLINK_TASK_PRIORITY, blinkTaskStack, &blinkTaskBuffer);

  vTaskStartScheduler();  // This shall never return

  for (;;) {
  }
}

void vSequencerTask(void *pvparameters) {
  TRice(iD(3448), "Sequencer started\n");
  while (1) {
    // Wait for a trigger to start the sequencer
    for (uint8_t step = 0; step < NUM_STEPS; step++) {
      currentStep = step;
      // TRice(iD(3035), "Processing step %d\n", step);

      if (stepGrid[0][step] || stepGrid[1][step]) {
        xSemaphoreGive(xSemaphoreModifyBuffer);
      }
      vTaskDelay((playback_delay/NUM_STEPS) / portTICK_RATE_MS);
    }
  }
}

void vPlaybackTask(void *pvparameters) {
  while (1) {
    // Wait for the semaphore to be given
    xSemaphoreTake(xSemaphorePlayback, portMAX_DELAY);

    if (playbackBuffer != NULL && BUFFERSIZE > 0) {
      EVAL_AUDIO_Play((uint16_t *)(playbackBuffer), BUFFERSIZE);
      status = 1;
    } else {
      TRice(iD(2182), "warning: Playback buffer is empty or invalid\n");
    }
  }
}

void vModifyBufferTask(void *pvparameters) {
  while (1) {
    xSemaphoreTake(xSemaphoreModifyBuffer, portMAX_DELAY);

    for (int i = 0; i < BUFFERSIZE; i++) {
      playbackBuffer[i] = 0;
      if ((stepGrid[0][currentStep]) && (i < SOUNDSIZE2*2)) {
        playbackBuffer[i] += (int16_t)kick_44100_stereo[i] / 2;
      }
      if ((stepGrid[1][currentStep]) && (i < SOUNDSIZE3)) {
        playbackBuffer[i] += (int16_t)openhat_22050_mono[i] / 2;
      }
    }
    xSemaphoreGive(xSemaphorePlayback);  // Signal playback task
  }
}

void vButtonSampleTask(void *p) {
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
      // TRice(iD(3068), "Reset blink to minimum delay\n");

      GPIO_SetBits(GPIOD,
                   GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

      kBlinkDelay = MIN_BLINK_DELAY;
      kBlinkStep = MIN_BLINK_DELAY;
    }
    prevStatePA0 = currentStatePA0;

    // TURN LEDs ON WHILE PRESSED, OFF WHILE NOT PRESSED ---
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
      sampleButtonState = 0x0000;  // Toggle step 1
      // Trigger playback task for Sound 1
      TRice(iD(7889), "Button 2 pressed: triggering playback for sound 2\n");
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
      sampleButtonState = 0x0001;  // Toggle step 2
      // Trigger playback task for Sound 2
      TRice(iD(2545), "Button 3 pressed: triggering playback for sound 3\n");
      // Turn on external LED on PD6 to indicate Button 2 action
      GPIO_SetBits(GPIOD, GPIO_Pin_6);
    }
    prevStatePD2 = currentStatePD2;

    // Add a debounce delay
    vTaskDelay(pdMS_TO_TICKS(25));
  }

  // Delete the task if it ever exits
  vTaskDelete(NULL);
}

void vButtonStepTask(void *pvParameters) {
  stepButtonState = 0;
  // wasPressed: 0 = released, 1 = pressed
  static uint8_t wasPressed = 0;

  // Create the semaphore before any button processing
  xButtonSemaphoreHandle = xSemaphoreCreateBinaryStatic(&xButtonSemaphore);

  // Initially, give the semaphore so our task doesn't block immediately
  xSemaphoreGive(xButtonSemaphoreHandle);

  for (;;) {
    // Wait a short time for our "event" semaphore (simulating an event trigger)
    if (xSemaphoreTake(xButtonSemaphoreHandle, pdMS_TO_TICKS(30)) == pdTRUE) {
      // Poll the button value
      stepButtonState = Interface_ReadButtonStep();

      // Process an event transitioning from idle to a valid state (0-3)
      if ((stepButtonState != STEPIDLE_VALUE) && (wasPressed == 0)) {
        TRice(iD(1844), "info: stepButtonState: %d\n", stepButtonState);

        if (stepButtonState <= NUM_STEPS && sampleButtonState < NUM_SAMPLES) {
          stepGrid[sampleButtonState][stepButtonState-1] =
            1 - stepGrid[sampleButtonState][stepButtonState-1];
          TRice(iD(3956), "info: Select step %d for sample %d\n", stepButtonState, sampleButtonState);
        }
        wasPressed = 1;  // Mark that this press has been processed
      }
      // When the button returns to idle, clear the flag
      if (stepButtonState == STEPIDLE_VALUE) {
        wasPressed = 0;
      }
    }

    // Delay for debouncing and to free CPU time for other tasks
    vTaskDelay(pdMS_TO_TICKS(50));

    // Re-give the semaphore to trigger the next poll cycle
    xSemaphoreGive(xButtonSemaphoreHandle);
  }
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

    if (kBlinkDelay >= MAX_BLINK_DELAY) {
      kBlinkStep -= MIN_BLINK_DELAY;  // Reverse step direction

    } else if (kBlinkDelay < MIN_BLINK_DELAY || kBlinkStep == 0) {
      kBlinkDelay = MIN_BLINK_DELAY;
      kBlinkStep = MIN_BLINK_DELAY;
    }

    // TRice(iD(7937), "att:🐁 Blink LEDs cycle: blinkStep=%d; blinkDelay=%d\n",
    //      kBlinkStep, kBlinkDelay);
  }

  vTaskDelete(NULL);
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
  TRice(iD(3770), "GetSampleCallBack\n");
  return 1;
}

/*
 * Called when buffer has been played out
 * Releases semaphore and wakes up task which modifies the buffer
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size) {
  TRice(iD(6478), "TransferComplete_CallBack. pBuffer: %d; Size: %d\n", pBuffer,
        Size);
  status = 0;
}

void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size) {
  TRice(iD(2248), "HalfTransfer_CallBack. pBuffer: %d; Size: %d\n", pBuffer,
  Size);
}

void EVAL_AUDIO_Error_CallBack(void *pData) {
  TRice(iD(6277), "error: Error_CallBack. Position: %d\n", pData);
}

uint32_t Codec_TIMEOUT_UserCallback(void) {
  TRice(iD(3396), "Codec_TIMEOUT_UserCallback\n");
  return 1;
}

/*
 * This FreeRTOS callback function gets called once per tick (default = 1000Hz)
 */
void vApplicationTickHook(void) { ++tickTime; }

/*
 * Continually send "silence" to the speaker when not playing
 */
void vApplicationIdleHook(void) {

  ++u64IdleTicksCnt;

  if (status == 0) {
    if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)) {
      SPI_I2S_SendData(CODEC_I2S, 0);
    }
  }
}
