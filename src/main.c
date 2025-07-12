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
 *
 * Dependencies:
 * - FreeRTOS Kernel
 * - STM32F4xx Standard Peripheral Library
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
#include <stm32f4xx_dma.h>

// STM32F4 Discovery
#include <stm32f4_discovery.h>
#include <stm32f4_discovery_audio_codec.h>

// Displays
#include <interface.h>
#include <oled.h>

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

static void Sequencer_SetBpm(uint16_t bpm) {
  if (bpm < MIN_BPM) bpm = MIN_BPM;
  if (bpm > MAX_BPM) bpm = MAX_BPM;

  taskENTER_CRITICAL(); // protect against DMA callbacks
  currBpm = bpm;
  nbrSamplesStep = (SAMPLE_RATE * 60U) / bpm;
  taskEXIT_CRITICAL();
}

static void mixSegment(uint32_t dst, uint32_t cnt, uint32_t ofs) {
  for (uint8_t s = 0; s < NUM_SAMPLES; ++s) {
    if (!stepGrid[s][stepIndex]) {
      continue;
    }

    const int16_t *src = sampleData[s] + ofs;
    uint32_t       len = sampleLen[s] - ofs;
    if (len > cnt) {
      len = cnt;
    }

    for (uint32_t i = 0; i < len; ++i) {
      int32_t v = playbackBuffer[dst + i] + (src[i] >> 1);
      playbackBuffer[dst + i] = CLIP16(v);
    }
  }
}

/* Render two consecutive steps starting at 'firstStep' into half 'base' */
static void renderHalf(uint32_t base) {
  const uint32_t halfSamples = BUFFERSIZE / 2;
  uint32_t todo = halfSamples; /* samples still to generate     */

  while (todo) {
    uint32_t stepSamples  = nbrSamplesStep;
    uint32_t leftInStep   = stepSamples - stepPos;
    uint32_t chunk        = (leftInStep < todo) ? leftInStep : todo;

    mixSegment(base + (halfSamples - todo), chunk, stepPos);

    stepPos  += chunk;
    todo     -= chunk;

    if (stepPos == stepSamples) { /* reached next step border ? */
      stepPos = 0;
      stepIndex = (stepIndex + 1) % NUM_STEPS;
    }
  }
}

/**
 * @brief Configure the system clock to 168 MHz (for STM32F4)
 */
static void SystemClock_Config(void) {
  // Enable the power interface clock and configure voltage regulator
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_MainRegulatorModeConfig(PWR_Regulator_Voltage_Scale1);

  // Enable HSE
  RCC_HSEConfig(RCC_HSE_ON);
  while (RCC_WaitForHSEStartUp() == ERROR)
    ;

  // Configure Flash wait states
  FLASH_SetLatency(FLASH_Latency_1);
  FLASH_PrefetchBufferCmd(ENABLE);

  // Configure AHB, APB1, and APB2 prescalers
  // AHB @ 168 MHz, APB2 @ 84 MHz, APB1 @ 42 MHz
  RCC_HCLKConfig(RCC_SYSCLK_Div1); // AHB = SYSCLK/1 = 168 MHz
  RCC_PCLK2Config(RCC_HCLK_Div2);  // APB2 = AHB/2 = 84 MHz
  RCC_PCLK1Config(RCC_HCLK_Div4);  // APB1 = AHB/4 = 42 MHz

  // Set up main PLL to 168 MHz system clock
  // (HSE=8 MHz, VCO=336 MHz, /2 => 168 MHz, /7 => 48 MHz USB)
  RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
  RCC_PLLCmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    ;

  // Select the main PLL as system clock source
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  while (RCC_GetSYSCLKSource() != 0x08)
    ; // 0x08 = PLL used as sysclk

  // Configure the I2S PLL for a proper I2S clock
  RCC_PLLI2SConfig(271, 2);
  RCC_PLLI2SCmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY) == RESET)
    ;

  // Update the SystemCoreClock global variable
  SystemCoreClockUpdate();

  // Enable peripheral clocks
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
}

/**
 * @brief Configures the user button GPIO (PA0) as an input.
 *
 * Initializes GPIO settings, enabling input mode without pull-up or
 * pull-down resistors. Prepares the pin to detect user button presses.
 */
static void config_userbutton(void) {
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
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // Enable internal pull-up
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**
 * @brief Initializes GPIO pins connected to LEDs.
 *
 * Prepares GPIOD Pins 12, 13, 14, and 15 for output mode. Ensures
 * proper configuration for controlling LED states.
 */
static void leds_init(void) {
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

int main(void) {
  SystemInit();
  SystemClock_Config();

  // Initialize OLED display
  OLED_Init();

  // Clear the display buffer
  OLED_Clear();

  // Text width: 16 characters * 6 pixels = 96 pixels
  // Starting X position: (132 - 96) / 2 = 18
  OLED_DrawString(18, 16, "*PROGTOMATA2000*");

  // Update the display to show the content
  OLED_UpdateScreen();

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
  
  // Create the semaphore before any button processing
  xButtonSemaphoreHandle = xSemaphoreCreateBinaryStatic(&xButtonSemaphoreStatic);
  if (xButtonSemaphoreHandle == NULL) {
    TRice(iD(7573), "error: Button semaphore creation failed\n");
  }

  EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);

  if (EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 65, I2S_AudioFreq_22k) != 0) {
    TRice(iD(2873), "msg: Audio codec initialization failed\n");
  }

  Sequencer_SetBpm(120);
  
  memset(playbackBuffer, 0, BUFFERSIZE * sizeof(int16_t));

  // Start audio playback
  if (EVAL_AUDIO_Play((uint16_t *)playbackBuffer, BUFFERSIZE) != 0) {
    TRice(iD(3630), "error: Failed to start audio playback\n");
  }

  TRice(iD(1375), "msg: Audio setup complete\n");

  sampleButtonTaskHandle = xTaskCreateStatic(
      vButtonSampleTask, "SampleButtonTask", SAMPLE_BUTTON_TASK_STACK_SIZE, NULL,
      SAMPLE_BUTTON_TASK_PRIORITY, sampleButtonTaskStack,
      &sampleButtonTaskBuffer);

  stepButtonTaskHandle = xTaskCreateStatic(
      vButtonStepTask, "StepButtonTask", STEP_BUTTON_TASK_STACK_SIZE, NULL,
      STEP_BUTTON_TASK_PRIORITY, stepButtonTaskStack,
      &stepButtonTaskBuffer);

  blinkTaskHandle = xTaskCreateStatic(
      vBlinkTask, "BlinkTask", BLINK_TASK_STACK_SIZE, NULL,
      BLINK_TASK_PRIORITY, blinkTaskStack,
      &blinkTaskBuffer);

  animationTaskHandle = xTaskCreateStatic(
      vOledAnimationTask, "OledAnimationTask", ANIMATION_TASK_STACK_SIZE, NULL,
      ANIMATION_TASK_PRIORITY, animationTaskStack,
      &animationTaskBuffer);

  TRice(iD(7392), "info: 🐛 PROGTOMATA2000 System initialized\n");

  vTaskStartScheduler(); // This shall never return

  for (;;) {
  }
}

static void vButtonSampleTask(void *p) {
  uint8_t prevStatePA0 = Bit_RESET; // Previous state for PA0
  uint8_t prevStatePD1 = Bit_RESET; // Previous state for PD1
  uint8_t prevStatePD2 = Bit_RESET; // Previous state for PD2

  while (1) {
    // Read current states
    uint8_t currentStatePA0 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
    uint8_t currentStatePD1 = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1);
    uint8_t currentStatePD2 = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2);

    // Handle PA0 (onboard button)
    if (currentStatePA0 == Bit_SET && prevStatePA0 == Bit_RESET) {
      GPIO_SetBits(GPIOD,
                   GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

      kBlinkDelay = MIN_BLINK_DELAY;
      kBlinkStep = MIN_BLINK_DELAY;
    }
    prevStatePA0 = currentStatePA0;

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
      sampleButtonState = 0x0000; // Toggle step 1
      TRice(iD(7916), "Button 2 pressed: set playback for sound 2\n");
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
      sampleButtonState = 0x0001; // Toggle step 2
      TRice(iD(7521), "Button 3 pressed: set playback for sound 3\n");
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

  // Initially, give the semaphore so the task doesn't block immediately
  xSemaphoreGive(xButtonSemaphoreHandle);

  for (;;) {
    // Wait a short time for the event semaphore
    if (xSemaphoreTake(xButtonSemaphoreHandle, pdMS_TO_TICKS(30)) == pdTRUE) {
      // Poll the button value
      stepButtonState = Interface_ReadButtonStep();

      // Process an event transitioning from idle to a valid state (0-3)
      if ((stepButtonState != STEPIDLE_VALUE) && (wasPressed == 0)) {
        TRice(iD(7943), "info: stepButtonState: %d\n", stepButtonState);

        if (stepButtonState <= NUM_STEPS && sampleButtonState < NUM_SAMPLES) {
          stepGrid[sampleButtonState][stepButtonState - 1] =
              1 - stepGrid[sampleButtonState][stepButtonState - 1];
          TRice(iD(5589), "info: Select step %d for sample %d\n",
                stepButtonState, sampleButtonState);
        }
        wasPressed = 1; // Mark that this press has been processed
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
      kBlinkStep -= MIN_BLINK_DELAY; // Reverse step direction

    } else if (kBlinkDelay < MIN_BLINK_DELAY || kBlinkStep == 0) {
      kBlinkDelay = MIN_BLINK_DELAY;
      kBlinkStep = MIN_BLINK_DELAY;
    }
#ifdef BLINK
    TRice(iD(3768), "att:🐁 Blink LEDs cycle: blinkStep=%d; blinkDelay=%d\n",
          kBlinkStep, kBlinkDelay);
#endif
  }

  vTaskDelete(NULL);
}

void vOledAnimationTask(void *pvParameters) {
  int16_t x = 64;
  int16_t y = 32;
  int16_t radius = 10;

  int16_t vx = 3; // initial velocity X
  int16_t vy = 3; // initial velocity Y

  while (1) {
    // Clear previous frame
    OLED_Clear();

    // Draw circle at new position
    OLED_DrawCircle(x, y, radius, true);

    // Update screen
    OLED_UpdateScreen();

    // Move the circle
    x += vx;
    y += vy;

    // Check for collisions with frame borders
    if ((x - radius <= 0) || (x + radius >= 128)) {
      vx = -vx;
      // Clamp position inside the frame to avoid getting stuck
      if (x - radius <= 0) {
        x = radius + 1;
      }
      
      if (x + radius >= 128) {
        x = 128 - radius - 1;
      }
    }

    if ((y - radius <= 0) || (y + radius >= 64)) {
      vy = -vy;
      if (y - radius <= 0) {
        y = radius + 1;
      }
      if (y + radius >= 64) {
        y = 64 - radius - 1;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void) {
  TRice(iD(6204), "GetSample\n");
  return 1;
}

void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size) {
  memset(&playbackBuffer[0], 0, (BUFFERSIZE/2)*sizeof(int16_t));
  renderHalf(0);                 /* renders first half-buffer            */
  playHeadStep = stepIndex;      /* UI can flash LEDs exactly on beat    */

#ifdef DEBUG
  TRice(iD(4918), "HalfTransfer. pBuffer: %x; Size: %d\n", pBuffer, Size);
#endif
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size) {
  memset(&playbackBuffer[BUFFERSIZE/2], 0, (BUFFERSIZE/2)*sizeof(int16_t));
  renderHalf(BUFFERSIZE / 2);    /* renders second half-buffer           */
  playHeadStep = stepIndex;

#ifdef DEBUG
  TRice(iD(7634), "TransferComplete. pBuffer: %x; Size: %d\n", pBuffer, Size);
#endif
}

void EVAL_AUDIO_Error_CallBack(void *pData) {
  TRice(iD(4389), "error: Error. Position: %x\n", pData);
}

uint32_t Codec_TIMEOUT_UserCallback(void) {
  TRice(iD(1479), "Codec_TIMEOUT_User\n");
  return 1;
}

void vApplicationTickHook(void) {
  ++tickTime;
}

void vApplicationIdleHook(void) {
  ++u64IdleTicksCnt;
}
