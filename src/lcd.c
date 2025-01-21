/**
 * @file lcd.c
 *
 * @brief Source file for LCD interface implementation.
 *
 * @details This file provides the implementation of the functions required to
 * initialize and control an LCD screen using STM32F4 microcontrollers. It
 * includes GPIO configuration, delays, and data transfer methods.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#include "lcd.h"

//********************************************************************************************
//* CONNECTIONS: *
//*------------------------------------------------------------------------------------------*
//* LCD PINS   | NAME                      | CONNECTED TO STM32F4 Discovery Port
//*------------------------------------------------------------------------------------------*
//* 1............VSS.......................GND                       *
//* 2............VDD.......................+5V                       *
//* 3............CONTRAST..................POT 5K                    *
//  Potentiometer Pins: pin 1 to V+, wiper (2nd pin) to pin 3 of LCD, pin 3 to
//  GND
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
//********************************************************************

uint16_t Register_Select = GPIO_Pin_3;
uint16_t Enable = GPIO_Pin_5;

uint16_t DB4 = GPIO_Pin_10;
uint16_t DB5 = GPIO_Pin_11;
uint16_t DB6 = GPIO_Pin_12;
uint16_t DB7 = GPIO_Pin_13;

static void LCD_WriteNibble(uint8_t nibble) {
  // nibble is assumed to be in the low 4 bits

  // D4
  if (nibble & 0x01)
    LCD_D4_HIGH();
  else
    LCD_D4_LOW();

  // D5
  if (nibble & 0x02)
    LCD_D5_HIGH();
  else
    LCD_D5_LOW();

  // D6
  if (nibble & 0x04)
    LCD_D6_HIGH();
  else
    LCD_D6_LOW();

  // D7
  if (nibble & 0x08)
    LCD_D7_HIGH();
  else
    LCD_D7_LOW();

  // Pulse Enable
  LCD_E_HIGH();
  // A short delay so LCD can latch the data
  // Use a simple for-loop or a microsecond delay if you have it
  for (volatile int i = 0; i < 500; i++);
  LCD_E_LOW();

  for (volatile int i = 0; i < 500; i++);
}

static void LCD_Send(uint8_t value, uint8_t isData) {
  if (isData) {
    LCD_RS_HIGH();
  } else {
    LCD_RS_LOW();
  }

  // Send high nibble (bits 7..4)
  LCD_WriteNibble(value >> 4);

  // Send low nibble (bits 3..0)
  LCD_WriteNibble(value & 0x0F);
}

static void LCD_Command(uint8_t cmd) {
  LCD_Send(cmd, 0 /* isData = false */);
  // Command writes often need extra delay
  for (volatile int i = 0; i < 4000; i++);
}

static void LCD_Data(uint8_t data) {
  LCD_Send(data, 1 /* isData = true */);
  // Data writes typically need less delay
  for (volatile int i = 0; i < 2000; i++);
}

void LCD_GPIO_Setup(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 |  // RS, E
                             GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 |
                             GPIO_Pin_13;         // D4..D7
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;      // Output
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // Speed
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // Push-pull
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // No pull resistors
  GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_ResetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 |
                            GPIO_Pin_12 | GPIO_Pin_13);
}

void LCD_Init(void) {
  LCD_GPIO_Setup();
  // Let the LCD power up
  for (volatile int i = 0; i < 500000; i++);  // ~some ms delay

  // Step 1: We are still in 8-bit mode from power-on,
  //         send 0x3 a few times to ensure it sees 8-bit commands
  LCD_RS_LOW();
  LCD_WriteNibble(0x03);
  for (volatile int i = 0; i < 30000; i++);
  LCD_WriteNibble(0x03);
  for (volatile int i = 0; i < 30000; i++);
  LCD_WriteNibble(0x03);
  for (volatile int i = 0; i < 30000; i++);

  // Step 2: Switch to 4-bit mode
  LCD_WriteNibble(0x02);
  for (volatile int i = 0; i < 30000; i++);

  // Now we can use the full commands
  LCD_Command(0x28);  // 4-bit, 2 lines, 5x8 font
  LCD_Command(0x0C);  // Display on, cursor off
  LCD_Command(0x06);  // Entry mode, auto-increment cursor
  LCD_Command(0x01);  // Clear display
  for (volatile int i = 0; i < 30000; i++);
}

void LCD_Clear(void) {
  LCD_Command(0x01);  // Clear display
  for (volatile int i = 0; i < 30000; i++);
}

void LCD_WriteString(char *str) {
  while (*str) {
    LCD_Data((uint8_t)*str++);
  }
}

void LCD_GotoXY(uint8_t row, uint8_t col) {
  // For a typical 16x2:
  // row 0 address = 0x00, row 1 address = 0x40
  // If you have a 20x4 or different geometry, addresses differ
  uint8_t address = (row == 0) ? 0x00 : 0x40;
  address += col;
  LCD_Command(0x80 | address);
}
