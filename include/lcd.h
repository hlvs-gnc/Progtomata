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

//*****************************************************************************
//* CONNECTIONS: *
//*---------------------------------------------------------------------------*
//* LCD PINS   | NAME                     | CONNECTED TO STM32F4 Discovery Port
//*---------------------------------------------------------------------------*
//* 1............VSS.......................GND                       *
//* 2............VDD.......................+5V                       *
//* 3............CONTRAST..................POT 5K                    *
//Potentiometer Pins: pin 1 to V+, wiper (2nd pin) to pin 3 of LCD, pin 3-> GND
//* 4............RS  - Register Select.....PE3                       *
//* 5............RW  - Read/Write..........GND                       *
//* 6............E   - Enable..............PE5                       *
//* 7............DB0  - Data line 0........GND                       *
//* 8............DB1  - Data line 1........GND                       *
//* 9............DB2  - Data line 2........GND                       *
//* 10...........DB3  - Data line 3........GND                       *
//* 11...........DB4  - Data line 4........PE10                      *
//* 12...........DB5  - Data line 5........PE11                      *
//* 13...........DB6  - Data line 6........PE12                      *
//* 14...........DB7  - Data line 7........PE13                      *
//* 15...........BACKLIGHT POSITIVE........VDD                       *
//* 16...........BACKLIGHT NEGATIVE........VSS                       *
//*****************************************************************************

// Pin definitions
/// @brief GPIO Port for LCD
#define LCD_PORT            GPIOE
/// @brief GPIO Pin for LCD Register Select
#define LCD_RS_PIN          GPIO_Pin_3
/// @brief GPIO Pin for LCD Enable
#define LCD_E_PIN           GPIO_Pin_5
/// @brief GPIO Pin for LCD Data Bit 4
#define LCD_D4_PIN          GPIO_Pin_10
/// @brief GPIO Pin for LCD Data Bit 5
#define LCD_D5_PIN          GPIO_Pin_11
/// @brief GPIO Pin for LCD Data Bit 6
#define LCD_D6_PIN          GPIO_Pin_12
/// @brief GPIO Pin for LCD Data Bit 7
#define LCD_D7_PIN          GPIO_Pin_13

// Macros for setting/clearing bits
/// @brief Set RS pin high
#define LCD_RS_HIGH()       GPIO_SetBits(LCD_PORT, LCD_RS_PIN)
/// @brief Set RS pin low
#define LCD_RS_LOW()        GPIO_ResetBits(LCD_PORT, LCD_RS_PIN)

/// @brief Set E pin high
#define LCD_E_HIGH()        GPIO_SetBits(LCD_PORT, LCD_E_PIN)
/// @brief Set E pin low
#define LCD_E_LOW()         GPIO_ResetBits(LCD_PORT, LCD_E_PIN)

// Helper macros to set data bits
/// @brief Set D4 pin high
#define LCD_D4_HIGH()       GPIO_SetBits(LCD_PORT, LCD_D4_PIN)
/// @brief Set D4 pin low
#define LCD_D4_LOW()        GPIO_ResetBits(LCD_PORT, LCD_D4_PIN)
/// @brief Set D5 pin high
#define LCD_D5_HIGH()       GPIO_SetBits(LCD_PORT, LCD_D5_PIN)
/// @brief Set D5 pin low
#define LCD_D5_LOW()        GPIO_ResetBits(LCD_PORT, LCD_D5_PIN)
/// @brief Set D6 pin high
#define LCD_D6_HIGH()       GPIO_SetBits(LCD_PORT, LCD_D6_PIN)
/// @brief Set D6 pin low
#define LCD_D6_LOW()        GPIO_ResetBits(LCD_PORT, LCD_D6_PIN)
/// @brief Set D7 pin high
#define LCD_D7_HIGH()       GPIO_SetBits(LCD_PORT, LCD_D7_PIN)
/// @brief Set D7 pin low
#define LCD_D7_LOW()        GPIO_ResetBits(LCD_PORT, LCD_D7_PIN)

// Function prototypes

/**
 * @brief Initializes the LCD.
 *
 * Initializes the LCD in 4-bit mode, sets the display to be on with
 * a blinking cursor, and clears the display.
 */
void LCD_Init(void);

/**
 * @brief Clears the LCD display.
 *
 * Clears the display and moves the cursor to home.
 */
void LCD_Clear(void);

/**
 * @brief Prints a string to the LCD.
 *
 * Prints the string to the LCD, wrapping to the next line if necessary.
 *
 * @param[in] str The string to print.
 */
void LCD_WriteString(const char *str);

/**
 * @brief Moves the LCD cursor to the specified row and column.
 *
 * @param[in] row The row to move to.
 * @param[in] col The column to move to.
 */
void LCD_GotoXY(uint8_t row, uint8_t col);

#endif  // LCD_H_
