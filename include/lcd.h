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

#define    POWER_UP        0x33
#define    FOURBIT_MODE    0X32

#define    EIGHTBIT_MODE   0x38  // as well as two line mode
#define    DISPLAY         0xF

#define    TWOLINE_MODE    0x28
#define    SETUP_CURSOR    0x0C
#define    SETUP_CURSOR_BLINKING    0xF

#define    CLEAR           0x01
#define    CURSOR_HOME     0x02
#define    LINE_TWO        0xC0

void gpio_init(void);

void delay_ms(int milli);
void pulse_enable(void);

void lcd_command(unsigned char command);
void lcd_commandSerial(unsigned char command);
void lcd_screen_init(void);

void write_to_screen_characters(unsigned char character);
void write_to_screen_string(const char *instring);

void print_double_to_screen(uint16_t data_in);
void print_pot_to_screen(uint16_t data_in);

#endif  // LCD_H_
