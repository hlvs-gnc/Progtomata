/**
 * @file lcd.h
 *
 * @brief Header file for LCD interface functions.
 *
 * @details This file defines initialization and control functions for
 * managing an LCD screen using STM32F4 microcontrollers.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons 
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef LCD_H_
#define LCD_H_

#include <string.h>

#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>

/**
 * @brief LCD command for initial power-up.
 */
#define POWER_UP 0x33

/**
 * @brief LCD command to set 4-bit mode.
 */
#define FOURBIT_MODE 0x32

/**
 * @brief LCD command for 8-bit mode and two-line mode.
 */
#define EIGHTBIT_MODE 0x38

/**
 * @brief LCD display on command.
 */
#define DISPLAY 0xF

/**
 * @brief LCD command for two-line mode.
 */
#define TWOLINE_MODE 0x28

/**
 * @brief LCD command to set up cursor without blinking.
 */
#define SETUP_CURSOR 0x0C

/**
 * @brief LCD command to set up cursor with blinking.
 */
#define SETUP_CURSOR_BLINKING 0xF

/**
 * @brief LCD command to clear the display.
 */
#define CLEAR 0x01

/**
 * @brief LCD command to move the cursor to the home position.
 */
#define CURSOR_HOME 0x02

/**
 * @brief LCD command to move the cursor to the second line.
 */
#define LINE_TWO 0xC0

/**
 * @brief Initializes GPIO pins required for the LCD interface.
 */
void gpio_init(void);

/**
 * @brief Delays execution for a specified number of milliseconds.
 *
 * @param milli Number of milliseconds to delay.
 */
void delay_ms(int milli);

/**
 * @brief Toggles the Enable pin to signal the LCD.
 */
void pulse_enable(void);

/**
 * @brief Sends a command to the LCD.
 *
 * @param command The command to send.
 */
void lcd_command(unsigned char command);

/**
 * @brief Sends a command to the LCD using a serial interface.
 *
 * @param command The command to send.
 */
void lcd_commandSerial(unsigned char command);

/**
 * @brief Initializes the LCD screen with the required settings.
 */
void lcd_screen_init(void);

/**
 * @brief Writes a single character to the LCD screen.
 *
 * @param character The character to write.
 */
void write_to_screen_characters(unsigned char character);

/**
 * @brief Writes a string to the LCD screen.
 *
 * @param instring The string to write.
 */
void write_to_screen_string(const char *instring);

/**
 * @brief Displays a numeric value on the LCD screen.
 *
 * @param data_in The numeric value to display.
 */
void print_double_to_screen(uint16_t data_in);

/**
 * @brief Displays potentiometer readings on the LCD screen.
 *
 * @param data_in The potentiometer reading to display.
 */
void print_pot_to_screen(uint16_t data_in);

#endif  // LCD_H_
