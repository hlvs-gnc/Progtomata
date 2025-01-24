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

int main(void) {
  SystemInit();

  config_userbutton();
  leds_init();
  uart_init();

  LCD_Init();

  LCD_GotoXY(0, 0);
  LCD_WriteString("****************");
  LCD_GotoXY(1, 0);
  LCD_WriteString("*PROGTOMATA2000*");

  TraceInit();
  
	TRICE32(Id(1342), "info:PROGTOMATA2000\n");

  // uart_print("System initialized\r\n");

  // Create button task
  xTaskCreateStatic(vButtonTask, "ButtonTask", BUTTON_TASK_STACK_SIZE, NULL, 1,
                    buttonTaskStack, &buttonTaskBuffer);

  // Create blink task
  xTaskCreateStatic(vBlinkTask, "BlinkTask", BLINK_TASK_STACK_SIZE, NULL, 1,
                    blinkTaskStack, &blinkTaskBuffer);
  
  vTaskStartScheduler();

  // This shall never return
  for (;;) {
  }
}

void vButtonTask(void *p) {
  uint8_t prevStatePA0 = Bit_RESET;  // Previous state for PA0
  uint8_t prevStatePD1 = Bit_RESET;  // Previous state for PD1
  uint8_t prevStatePD2 = Bit_RESET;  // Previous state for PD2
  bool led_state[2] = {false, false};

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
      // uart_print("Onboard button pressed\r\n");
      // uart_print("Reset to minimum delay\r\n\n");
      TRICE32(Id(4696), "Onboard button pressed");
      TRICE32(Id(5918), "Reset to minimum delay");
    }
    prevStatePA0 = currentStatePA0;

    // Handle PD1 (Button 1)
    if (currentStatePD1 == Bit_RESET && prevStatePD1 == Bit_SET &&
        !led_state[0]) {
      GPIO_SetBits(GPIOD, GPIO_Pin_5);  // Turn ON LED1 (PD5)
      led_state[0] = true;
      // uart_print("LED1 ON\r\n");
      TRICE32(Id(7563), "LED1 ON");
    } else if (currentStatePD1 == Bit_RESET && led_state[0]) {
      GPIO_ResetBits(GPIOD, GPIO_Pin_5);  // Turn OFF LED1 (PD5)
      led_state[0] = false;
      // uart_print("LED1 OFF\r\n");
      TRICE32(Id(6359), "LED1 OFF");
    }
    prevStatePD1 = currentStatePD1;

    // Handle PD2 (Button 2)
    if (currentStatePD2 == Bit_RESET && prevStatePD2 == Bit_SET &&
        !led_state[1]) {
      GPIO_SetBits(GPIOD, GPIO_Pin_6);  // Turn ON LED2 (PD6)
      led_state[1] = true;
      // uart_print("LED2 ON\r\n");
      TRICE32(Id(1808), "LED2 OFF");

    } else if (currentStatePD2 == Bit_RESET && led_state[1]) {
      GPIO_ResetBits(GPIOD, GPIO_Pin_6);  // Turn OFF LED1 (PD5)
      led_state[1] = false;
      //uart_print("LED2 OFF\r\n");
      TRICE32(Id(4237), "LED2 OFF");
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

    TRICE32(Id(7664), "Blink LEDs cycle");
    // uart_print("BlinkDelay: %d, BlinkStep: %d\r\n\n", kBlinkDelay, kBlinkStep);
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
