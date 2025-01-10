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

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	gpio_init();
	lcd_screen_init();

	delay_ms(100);
	lcd_commandSerial(CLEAR);
	write_to_screen_string("LCD Test");
	delay_ms(500);

  config_userbutton();
  leds_init();
  uart_init();

  // Create button task
  xTaskCreateStatic(vButtonTask, "ButtonTask", BUTTON_TASK_STACK_SIZE, NULL, 1,
                    buttonTaskStack, &buttonTaskBuffer);

  // Create blink task
  xTaskCreateStatic(vBlinkTask, "BlinkTask", BLINK_TASK_STACK_SIZE, NULL, 1,
                    blinkTaskStack, &blinkTaskBuffer);

  uart_send_string("System initialized\r\n");

  vTaskStartScheduler();

  // This shall never return
  for (;;) {
  }
}

void vButtonTask(void *p) {
  // Button task to handle user input
  uint8_t prevState = Bit_RESET;

  while (1) {
    uint8_t currentState = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
    // Read state of push button and save it in "state" variable
    // If state is high, turn on LEDs
    if (currentState == Bit_SET && prevState == Bit_RESET) {
      GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
      // If button state changes to pressed
      kBlinkDelay = MIN_DELAY; // Reset to minimum delay
    }

    prevState = currentState;
  }

  vTaskDelete(NULL);
}

void vBlinkTask(void *p) {

  while (1) {
    STM_EVAL_LEDOn(LED3);
    vTaskDelay(kBlinkDelay);

    STM_EVAL_LEDOn(LED4);
    vTaskDelay(kBlinkDelay);

    STM_EVAL_LEDOn(LED5);
    vTaskDelay(kBlinkDelay);

    STM_EVAL_LEDOn(LED6);
    vTaskDelay(kBlinkDelay);

    // Adjust delay
    kBlinkDelay += kBlinkStep;

    if (kBlinkDelay >= MAX_DELAY || kBlinkDelay <= MIN_DELAY) {
      kBlinkStep = -kBlinkStep; // Reverse step direction
    }
  }

  vTaskDelete(NULL);
}

void config_userbutton(void) {
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
}

void leds_init(void) {
  // Initialize LEDs
  STM_EVAL_LEDInit(LED3);

  STM_EVAL_LEDInit(LED4);

  STM_EVAL_LEDInit(LED5);

  STM_EVAL_LEDInit(LED6);
}
