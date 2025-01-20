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
//  Potentiometer Pins: pin 1 to V+, wiper (2nd pin) to pin 3 of LCD, pin 3 to GND
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

void gpio_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // configure 6 pins (defined above)
  GPIO_InitStruct.GPIO_Pin = Register_Select | Enable | DB4 | DB5 | DB6 | DB7; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // pins are output
  // this sets the GPIO modules clock speed
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  // this sets the pin type to push/ pull
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  // Sets the pullup / pulldown resistors to be inactive
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  // this finally passes all the values to the GPIO_Init function which takes care
  // of setting the corresponding bits
  GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void delay_ms(int milli) {
  // approximate loops per ms at 168 MHz, Debug config
  int delay = milli * 17612;
  for (; delay != 0; delay--) {};
}

void pulse_enable(void) {
  delay_ms(1);
  GPIOE->ODR ^= (1 << 5); // Sets enable high

  delay_ms(1);
  GPIOE->ODR ^= (1 << 5); // Sets enable low
}

void lcd_commandSerial(unsigned char command) {

  GPIO_ResetBits(GPIOE, Register_Select); // Register Select line low
  // Put upper nibble (DB7 to DB4 bits) on data lines
  unsigned char upper_nibble = command & 0xF0; 
  GPIOE->ODR = upper_nibble << 6;
  pulse_enable();

  // Put lower nibble (DB3 to DB0 bits) on data lines
  unsigned char lower_nibble = command & 0xF; 
  GPIOE->ODR = lower_nibble << 10;
  pulse_enable();
}

void lcd_screen_init(void) {

  delay_ms(100);
  // Power up initialization for the lcd 'final note recommendation'
  lcd_commandSerial(POWER_UP);
  lcd_commandSerial(FOURBIT_MODE); // Set LCD into 4 bit mode
  lcd_commandSerial(SETUP_CURSOR_BLINKING); // Turn display on and set up cursor
  lcd_commandSerial(TWOLINE_MODE);          // Set up 2 lines and character size
  lcd_commandSerial(CLEAR);                 // Clear display
}

void write_to_screen_characters(unsigned char character) {
  // Put upper nibble (DB7 to DB4 bits) on data lines
  unsigned char upper_nibble = character & 0xF0; 
  GPIOE->ODR = upper_nibble << 6;

  // Use this pin as Register Select, needs to be high
  GPIO_SetBits(GPIOE, Register_Select); 

  pulse_enable();
  // Put lower nibble (DB3 to DB0 bits) on data lines
  unsigned char lower_nibble = character & 0xF; 
  GPIOE->ODR = lower_nibble << 10;

  // Use this pin as Register Select, needs to be high
  GPIO_SetBits(GPIOE, Register_Select); 

  pulse_enable();
}

void write_to_screen_string(const char *instring) {
  unsigned char count = 0;
  int length = 0;

  length = (unsigned int) strlen(instring);

  // Write each character to LCD
  while (instring[count]) {
    if (count > 15) {
      GPIO_ResetBits(GPIOE, Register_Select);

      lcd_commandSerial(0xC0);
      for (count = 0; count < length; count++) {
        write_to_screen_characters(instring[count]);
      }
    } else {
      // Write each character to LCD
      write_to_screen_characters(instring[count]);
      count++;
    }
  }
}

void print_double_to_screen(uint16_t data_in) {
  unsigned char characters[4] = {0};

  int array_length = sizeof(characters) / sizeof(characters[0]);
  int base = 1000;

  uint16_t temp = 0;

  int j;

  for (j = 0; j < array_length; j++) {
    characters[j] = data_in / base + 48;
    temp = data_in / base;
    data_in = data_in - (temp * base);
    base = base / 10;
  }

  int i;

  for (i = 0; i < array_length; ++i) {
    write_to_screen_characters(characters[i]);
  }
}

void print_pot_to_screen(uint16_t data_in) {
  unsigned char characters[3] = {0};

  int array_length = sizeof(characters) / sizeof(characters[0]);
  int base = 1000;

  uint16_t temp = 0;

  int j;

  for (j = 0; j < array_length; j++) {
    characters[j] = data_in / base + 48;
    temp = data_in / base;
    data_in = data_in - (temp * base);
    base = base / 10;
  }

  int i;

  for (i = 0; i < array_length; ++i) {
    write_to_screen_characters(characters[i]);
  }
}
