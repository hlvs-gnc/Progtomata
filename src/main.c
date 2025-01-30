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
#include <stm32f4xx_pwr.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_flash.h>

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

// FreeRTOS Hook functions
#include <hooks.h>

// Mono Audio samples
#include <kick_22050_mono.h>
#include <openhat_22050_mono.h>

// Stereo Audio samples
#include <kick_44100_stereo.h>

// N Buttons' states - each bit indicate respective button's ON/OFF state
static uint16_t buttonState = 0;

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
  RCC_HCLKConfig(RCC_SYSCLK_Div1);   // AHB = SYSCLK/1 = 168 MHz
  RCC_PCLK2Config(RCC_HCLK_Div2);    // APB2 = AHB/2 = 84 MHz
  RCC_PCLK1Config(RCC_HCLK_Div4);    // APB1 = AHB/4 = 42 MHz

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

  // Enable peripheral clockS
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | 
                         RCC_AHB1Periph_GPIOC, ENABLE);
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

  TRice(iD(1158), "info: 🐛 PROGTOMATA2000 System initialized\n");

  xSemaphorePlayback = xSemaphoreCreateBinaryStatic(&xSemaphorePlaybackStatic);
  if (xSemaphorePlayback == NULL) {
    // Handle error: Semaphore creation failed
    TRice(iD(4558), "error: Semaphore creation failed\n");
  }

  xSemaphoreModifyBuffer = xSemaphoreCreateBinaryStatic(&xSemaphoreModifyBufferStatic);
  if (xSemaphoreModifyBuffer == NULL) {
    // Handle error: Semaphore creation failed
    TRice(iD(5830), "error: Semaphore creation failed\n");
  }

  EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);

  if (EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 90, I2S_AudioFreq_44k) != 0) {
    TRice(iD(5116), "msg: Audio codec initialization failed\n");
  }

  TRice(iD(1060), "msg: Audio setup complete\n");

/*
  // Create blink task
  blinkTaskHandle =
      xTaskCreateStatic(vBlinkTask, "BlinkTask",
                        BLINK_TASK_STACK_SIZE, NULL,
                        1, blinkTaskStack,
                        &blinkTaskBuffer);
*/
  playbackTaskHandle =
      xTaskCreateStatic(vPlaybackTask, "PlaybackTask",
                        PLAYBACK_TASK_STACK_SIZE, NULL,
                        1, playbackTaskStack,
                        &playbackTaskBuffer);

  modifyBufferTaskHandle =
      xTaskCreateStatic(vModifyBufferTask, "ModifyBufferTask",
                        MODIFYBUFFER_TASK_STACK_SIZE, NULL,
                        1, modifyBufferTaskStack,
                        &modifyBufferTaskBuffer);

  buttonTaskHandle =
      xTaskCreateStatic(vButtonTask, "ButtonTask",
                        BUTTON_TASK_STACK_SIZE, NULL,
                        1, buttonTaskStack,
                        &buttonTaskBuffer);

  sequencerTaskHandle =
      xTaskCreateStatic(vSequencerTask, "SequencerTask",
                        SEQUENCER_TASK_STACK_SIZE, NULL,
                        1, sequencerTaskStack,
                        &sequencerTaskBuffer);
                        
  vTaskStartScheduler();  // This shall never return

  for (;;) {
  }
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void) {
  TRice(iD(5526), "GetSampleCallBack\n");
  return 1;
}

void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size) {
  TRice(iD(6894), "HalfTransfer_CallBack. pBuffer: %d; Size: %d\n",
        pBuffer, Size);
}

/*
 * Called when buffer has been played out
 * Releases semaphore and wakes up task which modifies the buffer
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size) {
  TRice(iD(1983), "TransferComplete_CallBack. pBuffer: %d; Size: %d\n",
        pBuffer, Size);
  status = 0;
}

void EVAL_AUDIO_Error_CallBack(void *pData) {
  TRice(iD(2795), "error: Error_CallBack. Position: %d\n", pData);
}

uint32_t Codec_TIMEOUT_UserCallback(void) {
  TRice(iD(3625), "Codec_TIMEOUT_UserCallback\n");
  return 1;
}

void vPlaybackTask(void *pvparameters) {
  for (;;) {
    // Wait for the semaphore to be given
    while (xSemaphoreTake(xSemaphorePlayback, (portTickType)0xFF) == pdFALSE) {}
    
    status = 1;
    EVAL_AUDIO_Play((uint16_t *)(playbackBuffer), BUFFERSIZE);
    vTaskDelay(playback_delay / portTICK_RATE_MS);
  }
}

void vSequencerTask(void *pvparameters) {
  while (1) {
    if (buttonState & 0x0002) {
      xSemaphoreGive(xSemaphoreModifyBuffer);
    }

    if (buttonState & 0x0003) {
      xSemaphoreGive(xSemaphoreModifyBuffer);
    }
    vTaskDelay(playback_delay / portTICK_RATE_MS);
  }
}

void vModifyBufferTask(void *pvparameters) {

  while (1) {
    xSemaphoreTake(xSemaphoreModifyBuffer, portMAX_DELAY);
    memset(playbackBuffer, 0, BUFFERSIZE * sizeof(uint16_t));

    if (buttonState & 0x0002) {
      memcpy(playbackBuffer, kick_44100_stereo, SOUNDSIZE2 * sizeof(uint16_t));

    } else if (buttonState & 0x0003) {
      memcpy(playbackBuffer, openhat_22050_mono, SOUNDSIZE3 * sizeof(uint16_t));
    }

    xSemaphoreGive(xSemaphorePlayback);  // Signal Modify Buffer Task
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
      buttonState = 0x0001;
      TRice(iD(2345), "Onboard button pressed: triggering playback for sound 1\n");
      
      GPIO_SetBits(GPIOD,
                   GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

      kBlinkDelay = MIN_BLINK_DELAY;
      kBlinkStep = MIN_BLINK_DELAY;
      TRice(iD(4986), "Reset blink to minimum delay\n");

      xSemaphoreGive(xSemaphoreModifyBuffer);  // Signal Modify Buffer Task
    }
    prevStatePA0 = currentStatePA0;

    //TURN LEDs ON WHILE PRESSED, OFF WHILE NOT PRESSED ---
    // PD1 => PD5
    if (currentStatePD1 == Bit_RESET) { 
      // Button is pressed
      GPIO_SetBits(GPIOD, GPIO_Pin_5);
    } else {
      // Button is not pressed
      GPIO_ResetBits(GPIOD, GPIO_Pin_5);
    }

    // Handle PD1 (Trigger Playback for Sound 1)
    if (currentStatePD1 == Bit_RESET && prevStatePD1 == Bit_SET) {
      buttonState ^= 0x0002; // Toggle step 1
      // Trigger playback task for Sound 1
      TRice(iD(6754), "Button 2 pressed: triggering playback for sound 2\n");
      // Turn on external LED on PD5 to indicate Button 1 action
      GPIO_SetBits(GPIOD, GPIO_Pin_5);

      xSemaphoreGive(xSemaphoreModifyBuffer);  // Signal Modify Buffer Task
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
      buttonState ^= 0x0003; // Toggle step 2
      // Trigger playback task for Sound 2
      TRice(iD(4252), "Button 3 pressed: triggering playback for sound 3\n");
      // Turn on external LED on PD6 to indicate Button 2 action
      GPIO_SetBits(GPIOD, GPIO_Pin_6);

      xSemaphoreGive(xSemaphoreModifyBuffer);  // Signal Modify Buffer Task
    }
    prevStatePD2 = currentStatePD2;

    // Add a debounce delay
    vTaskDelay(pdMS_TO_TICKS(10));
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

    if (kBlinkDelay >= MAX_BLINK_DELAY) {
      kBlinkStep -= MIN_BLINK_DELAY;  // Reverse step direction

    } else if (kBlinkDelay < MIN_BLINK_DELAY || kBlinkStep == 0) {
      kBlinkDelay = MIN_BLINK_DELAY;
      kBlinkStep = MIN_BLINK_DELAY;
    }

    TRice(iD(5046), "att:🐁 Blink LEDs cycle: blinkStep=%d; blinkDelay=%d\n",
          kBlinkStep, kBlinkDelay);
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

/*
 * This FreeRTOS callback function gets called once per tick (default = 1000Hz)
 */
void vApplicationTickHook(void) { ++tickTime; }

/*
 * Continually send "silence" to the speaker when not playing
 */
void vApplicationIdleHook(void) {
  uint8_t i = 0;

  ++u64IdleTicksCnt;

  if (status == 0) {
    if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)) {
      SPI_I2S_SendData(CODEC_I2S, i);
      if (i > 16) {
        i = 0;
      }
    }
  }
}
