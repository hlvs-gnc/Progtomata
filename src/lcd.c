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

#include <lcd.h>

uint16_t Register_Select = GPIO_Pin_3;
uint16_t Enable = GPIO_Pin_5;

uint16_t DB4 = GPIO_Pin_10;
uint16_t DB5 = GPIO_Pin_11;
uint16_t DB6 = GPIO_Pin_12;
uint16_t DB7 = GPIO_Pin_13;

static void LCD_WriteNibble(uint8_t nibble)
{
  // nibble is assumed to be in the low 4 bits

  // D4
  if (nibble & 0x01) {
    LCD_D4_HIGH();
  } else {
    LCD_D4_LOW();
  }

  // D5
  if (nibble & 0x02) {
    LCD_D5_HIGH();
  } else {
    LCD_D5_LOW();
  }

  // D6
  if (nibble & 0x04) {
    LCD_D6_HIGH();
  } else {
    LCD_D6_LOW();
  }

  // D7
  if (nibble & 0x08) {
    LCD_D7_HIGH();
  } else {
    LCD_D7_LOW();
  }

  // Pulse Enable
  LCD_E_HIGH();
  // A short delay so LCD can latch the data
  // Use a simple for-loop or a microsecond delay if you have it
  for (volatile int i = 0; i < 500; i++)
    ;

  LCD_E_LOW();

  for (volatile int i = 0; i < 500; i++)
    ;
}

static void LCD_Send(uint8_t value, uint8_t isData)
{
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

static void LCD_Command(uint8_t cmd)
{
  LCD_Send(cmd, 0 /* isData = false */);
  // Command writes often need extra delay
  for (volatile int i = 0; i < 4000; i++)
    ;
}

static void LCD_Data(uint8_t data)
{
  LCD_Send(data, 1 /* isData = true */);
  // Data writes typically need less delay
  for (volatile int i = 0; i < 2000; i++)
    ;
}

/**
 * @brief Configures GPIO pins connected to the LCD.
 *
 * Prepares pins for output mode and sets them low. This is
 * necessary before calling @ref LCD_Init().
 */
static void LCD_GPIO_Setup(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | // RS, E
                             GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 |
                             GPIO_Pin_13;        // D4..D7
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;     // Output
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // Speed
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;    // Push-pull
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;  // No pull resistors
  GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_ResetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 |
                            GPIO_Pin_12 | GPIO_Pin_13);
}

void LCD_Init(void)
{
  LCD_GPIO_Setup();

  // Let the LCD power up
  for (volatile int i = 0; i < 500000; i++)
    ; // ~some ms delay

  // State is in 8-bit mode from power-on,
  // send 0x3 a few times to ensure it sees 8-bit commands
  LCD_RS_LOW();
  LCD_WriteNibble(0x03);
  for (volatile int i = 0; i < 30000; i++)
    ;
  LCD_WriteNibble(0x03);
  for (volatile int i = 0; i < 30000; i++)
    ;
  LCD_WriteNibble(0x03);
  for (volatile int i = 0; i < 30000; i++)
    ;

  // Switch to 4-bit mode
  LCD_WriteNibble(0x02);
  for (volatile int i = 0; i < 30000; i++)
    ;

  // Use the full commands
  LCD_Command(0x28); // 4-bit, 2 lines, 5x8 font
  LCD_Command(0x0C); // Display on, cursor off
  LCD_Command(0x06); // Entry mode, auto-increment cursor
  LCD_Command(0x01); // Clear display
  for (volatile int i = 0; i < 30000; i++)
    ;
}

void LCD_Clear(void)
{
  LCD_Command(0x01); // Clear display
  for (volatile int i = 0; i < 30000; i++)
    ;
}

void LCD_WriteString(const char *str)
{
  while (*str) {
    LCD_Data((uint8_t)*str++);
  }
}

void LCD_GotoXY(uint8_t row, uint8_t col)
{
  // For a typical 16x2:
  // row 0 address = 0x00, row 1 address = 0x40
  uint8_t address = (row == 0) ? 0x00 : 0x40;
  address += col;
  LCD_Command(0x80 | address);
}
