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

// --- Pin definitions ---
#define LCD_PORT            GPIOE
#define LCD_RS_PIN          GPIO_Pin_3
#define LCD_E_PIN           GPIO_Pin_5
#define LCD_D4_PIN          GPIO_Pin_10
#define LCD_D5_PIN          GPIO_Pin_11
#define LCD_D6_PIN          GPIO_Pin_12
#define LCD_D7_PIN          GPIO_Pin_13

// --- Macros for setting/clearing bits ---
#define LCD_RS_HIGH()       GPIO_SetBits(LCD_PORT, LCD_RS_PIN)
#define LCD_RS_LOW()        GPIO_ResetBits(LCD_PORT, LCD_RS_PIN)

#define LCD_E_HIGH()        GPIO_SetBits(LCD_PORT, LCD_E_PIN)
#define LCD_E_LOW()         GPIO_ResetBits(LCD_PORT, LCD_E_PIN)

// Helper macros to set data bits
#define LCD_D4_HIGH()       GPIO_SetBits(LCD_PORT, LCD_D4_PIN)
#define LCD_D4_LOW()        GPIO_ResetBits(LCD_PORT, LCD_D4_PIN)
#define LCD_D5_HIGH()       GPIO_SetBits(LCD_PORT, LCD_D5_PIN)
#define LCD_D5_LOW()        GPIO_ResetBits(LCD_PORT, LCD_D5_PIN)
#define LCD_D6_HIGH()       GPIO_SetBits(LCD_PORT, LCD_D6_PIN)
#define LCD_D6_LOW()        GPIO_ResetBits(LCD_PORT, LCD_D6_PIN)
#define LCD_D7_HIGH()       GPIO_SetBits(LCD_PORT, LCD_D7_PIN)
#define LCD_D7_LOW()        GPIO_ResetBits(LCD_PORT, LCD_D7_PIN)

// Function prototypes
void LCD_GPIO_Setup(void);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_WriteString(char *str);
void LCD_GotoXY(uint8_t row, uint8_t col);

#endif  // LCD_H_
