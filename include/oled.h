/**
 * @file oled.h
 *
 * @brief Header file for OLED interface functions.
 *
 * @details This file defines initialization and control functions for
 * managing an OLED display using STM32F4 microcontrollers.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef OLED_H_
#define OLED_H_

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"

/* OLED Display Configuration */
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES 8
#define OLED_I2C_ADDR 0x78  // 0x3C << 1

/* Error handling */
typedef enum {
  OLED_OK = 0,
  OLED_ERROR,
  OLED_TIMEOUT,
  OLED_BUSY
} OLED_Status;

/* I2C Configuration */
#define OLED_I2C I2C1
#define OLED_I2C_CLK RCC_APB1Periph_I2C1
#define OLED_I2C_GPIO_CLK RCC_AHB1Periph_GPIOB
#define OLED_I2C_GPIO GPIOB
#define OLED_I2C_SCL_PIN GPIO_Pin_8
#define OLED_I2C_SDA_PIN GPIO_Pin_9
#define OLED_I2C_SCL_SOURCE GPIO_PinSource8
#define OLED_I2C_SDA_SOURCE GPIO_PinSource9
#define OLED_I2C_AF GPIO_AF_I2C1

/* OLED Commands */
#define OLED_CMD_DISPLAY_OFF 0xAE
#define OLED_CMD_DISPLAY_ON 0xAF
#define OLED_CMD_SET_DISPLAY_CLOCK 0xD5
#define OLED_CMD_SET_MULTIPLEX 0xA8
#define OLED_CMD_SET_DISPLAY_OFFSET 0xD3
#define OLED_CMD_SET_START_LINE 0x40
#define OLED_CMD_CHARGE_PUMP 0x8D
#define OLED_CMD_MEMORY_MODE 0x20
#define OLED_CMD_SEG_REMAP 0xA0
#define OLED_CMD_COM_SCAN_DEC 0xC8
#define OLED_CMD_SET_COM_PINS 0xDA
#define OLED_CMD_SET_CONTRAST 0x81
#define OLED_CMD_SET_PRECHARGE 0xD9
#define OLED_CMD_SET_VCOM_DETECT 0xDB
#define OLED_CMD_DISPLAY_ALL_ON_RESUME 0xA4
#define OLED_CMD_NORMAL_DISPLAY 0xA6
#define OLED_CMD_COLUMN_ADDR 0x21
#define OLED_CMD_PAGE_ADDR 0x22
#define OLED_CMD_INVERT_DISPLAY 0xA7
#define OLED_CMD_ACTIVATE_SCROLL 0x2F
#define OLED_CMD_DEACTIVATE_SCROLL 0x2E
#define OLED_CMD_SET_VERTICAL_SCROLL 0xA3
#define OLED_CMD_RIGHT_HORIZONTAL_SCROLL 0x26
#define OLED_CMD_LEFT_HORIZONTAL_SCROLL 0x27

/* Control byte */
#define OLED_CONTROL_BYTE_CMD_SINGLE 0x80
#define OLED_CONTROL_BYTE_CMD_STREAM 0x00
#define OLED_CONTROL_BYTE_DATA_STREAM 0x40

/* Basic 5x7 font */
static const uint8_t Font5x7[][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00},  // Space
  {0x00, 0x00, 0x5F, 0x00, 0x00},  // !
  {0x00, 0x07, 0x00, 0x07, 0x00},  // "
  {0x14, 0x7F, 0x14, 0x7F, 0x14},  // #
  {0x24, 0x2A, 0x7F, 0x2A, 0x12},  // $
  {0x23, 0x13, 0x08, 0x64, 0x62},  // %
  {0x36, 0x49, 0x55, 0x22, 0x50},  // &
  {0x00, 0x05, 0x03, 0x00, 0x00},  // '
  {0x00, 0x1C, 0x22, 0x41, 0x00},  // (
  {0x00, 0x41, 0x22, 0x1C, 0x00},  // )
  {0x14, 0x08, 0x3E, 0x08, 0x14},  // *
  {0x08, 0x08, 0x3E, 0x08, 0x08},  // +
  {0x00, 0x50, 0x30, 0x00, 0x00},  // ,
  {0x08, 0x08, 0x08, 0x08, 0x08},  // -
  {0x00, 0x60, 0x60, 0x00, 0x00},  // .
  {0x20, 0x10, 0x08, 0x04, 0x02},  // /
  {0x3E, 0x51, 0x49, 0x45, 0x3E},  // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00},  // 1
  {0x42, 0x61, 0x51, 0x49, 0x46},  // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31},  // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10},  // 4
  {0x27, 0x45, 0x45, 0x45, 0x39},  // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30},  // 6
  {0x01, 0x71, 0x09, 0x05, 0x03},  // 7
  {0x36, 0x49, 0x49, 0x49, 0x36},  // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E},  // 9
  {0x00, 0x36, 0x36, 0x00, 0x00},  // :
  {0x00, 0x56, 0x36, 0x00, 0x00},  // ;
  {0x08, 0x14, 0x22, 0x41, 0x00},  // <
  {0x14, 0x14, 0x14, 0x14, 0x14},  // =
  {0x00, 0x41, 0x22, 0x14, 0x08},  // >
  {0x02, 0x01, 0x51, 0x09, 0x06},  // ?
  {0x32, 0x49, 0x79, 0x41, 0x3E},  // @
  {0x7E, 0x11, 0x11, 0x11, 0x7E},  // A
  {0x7F, 0x49, 0x49, 0x49, 0x36},  // B
  {0x3E, 0x41, 0x41, 0x41, 0x22},  // C
  {0x7F, 0x41, 0x41, 0x22, 0x1C},  // D
  {0x7F, 0x49, 0x49, 0x49, 0x41},  // E
  {0x7F, 0x09, 0x09, 0x09, 0x01},  // F
  {0x3E, 0x41, 0x49, 0x49, 0x7A},  // G
  {0x7F, 0x08, 0x08, 0x08, 0x7F},  // H
  {0x00, 0x41, 0x7F, 0x41, 0x00},  // I
  {0x20, 0x40, 0x41, 0x3F, 0x01},  // J
  {0x7F, 0x08, 0x14, 0x22, 0x41},  // K
  {0x7F, 0x40, 0x40, 0x40, 0x40},  // L
  {0x7F, 0x02, 0x0C, 0x02, 0x7F},  // M
  {0x7F, 0x04, 0x08, 0x10, 0x7F},  // N
  {0x3E, 0x41, 0x41, 0x41, 0x3E},  // O
  {0x7F, 0x09, 0x09, 0x09, 0x06},  // P
  {0x3E, 0x41, 0x51, 0x21, 0x5E},  // Q
  {0x7F, 0x09, 0x19, 0x29, 0x46},  // R
  {0x46, 0x49, 0x49, 0x49, 0x31},  // S
  {0x01, 0x01, 0x7F, 0x01, 0x01},  // T
  {0x3F, 0x40, 0x40, 0x40, 0x3F},  // U
  {0x1F, 0x20, 0x40, 0x20, 0x1F},  // V
  {0x3F, 0x40, 0x38, 0x40, 0x3F},  // W
  {0x63, 0x14, 0x08, 0x14, 0x63},  // X
  {0x07, 0x08, 0x70, 0x08, 0x07},  // Y
  {0x61, 0x51, 0x49, 0x45, 0x43},  // Z
  {0x00, 0x7F, 0x41, 0x41, 0x00},  // [
  {0x02, 0x04, 0x08, 0x10, 0x20},  // backslash
  {0x00, 0x41, 0x41, 0x7F, 0x00},  // ]
  {0x04, 0x02, 0x01, 0x02, 0x04},  // ^
  {0x40, 0x40, 0x40, 0x40, 0x40},  // _
  {0x00, 0x01, 0x02, 0x04, 0x00},  // `
  {0x20, 0x54, 0x54, 0x54, 0x78},  // a
  {0x7F, 0x48, 0x44, 0x44, 0x38},  // b
  {0x38, 0x44, 0x44, 0x44, 0x20},  // c
  {0x38, 0x44, 0x44, 0x48, 0x7F},  // d
  {0x38, 0x54, 0x54, 0x54, 0x18},  // e
  {0x08, 0x7E, 0x09, 0x01, 0x02},  // f
  {0x0C, 0x52, 0x52, 0x52, 0x3E},  // g
  {0x7F, 0x08, 0x04, 0x04, 0x78},  // h
  {0x00, 0x44, 0x7D, 0x40, 0x00},  // i
  {0x20, 0x40, 0x44, 0x3D, 0x00},  // j
  {0x7F, 0x10, 0x28, 0x44, 0x00},  // k
  {0x00, 0x41, 0x7F, 0x40, 0x00},  // l
  {0x7C, 0x04, 0x18, 0x04, 0x78},  // m
  {0x7C, 0x08, 0x04, 0x04, 0x78},  // n
  {0x38, 0x44, 0x44, 0x44, 0x38},  // o
  {0x7C, 0x14, 0x14, 0x14, 0x08},  // p
  {0x08, 0x14, 0x14, 0x18, 0x7C},  // q
  {0x7C, 0x08, 0x04, 0x04, 0x08},  // r
  {0x48, 0x54, 0x54, 0x54, 0x20},  // s
  {0x04, 0x3F, 0x44, 0x40, 0x20},  // t
  {0x3C, 0x40, 0x40, 0x20, 0x7C},  // u
  {0x1C, 0x20, 0x40, 0x20, 0x1C},  // v
  {0x3C, 0x40, 0x30, 0x40, 0x3C},  // w
  {0x44, 0x28, 0x10, 0x28, 0x44},  // x
  {0x0C, 0x50, 0x50, 0x50, 0x3C},  // y
  {0x44, 0x64, 0x54, 0x4C, 0x44},  // z
};

/* Function Prototypes */
/**
 * @brief Initializes the OLED display.
 *
 * This function initializes the OLED display and sets it up for use. It must
 * be called before any other function in this module.
 */
void OLED_Init(void);

/**
 * @brief Initializes the OLED display.
 *
 * This function initializes the OLED display and sets it up for use. It must
 * be called before any other function in this module.
 */
void OLED_Init(void);

/**
 * @brief Writes a single command to the OLED display.
 *
 * @param cmd the command to write
 */
void OLED_WriteCommand(uint8_t cmd);

/**
 * @brief Writes a single data byte to the OLED display.
 *
 * @param data the data to write
 */
void OLED_WriteData(uint8_t data);

/**
 * @brief Writes multiple data bytes to the OLED display.
 *
 * @param data the data to write
 * @param size the number of bytes to write
 */
void OLED_WriteMultipleData(uint8_t* data, uint16_t size);

/**
 * @brief Sets the cursor position of the OLED display.
 *
 * @param x the x-coordinate of the position
 * @param y the y-coordinate of the position
 */
void OLED_SetCursor(uint8_t x, uint8_t y);

/**
 * @brief Clears the OLED display.
 */
void OLED_Clear(void);

/**
 * @brief Turns the OLED display on.
 */
void OLED_DisplayOn(void);

/**
 * @brief Turns the OLED display off.
 */
void OLED_DisplayOff(void);

/**
 * @brief Sets the contrast of the OLED display.
 *
 * @param contrast the contrast value, valid range is 0-255
 */
void OLED_SetContrast(uint8_t contrast);

/**
 * @brief Inverts the display of the OLED display.
 *
 * @param invert if true, the display is inverted
 */
void OLED_InvertDisplay(bool invert);

/**
 * @brief Draws a single pixel on the OLED display.
 *
 * @param x the x-coordinate of the pixel
 * @param y the y-coordinate of the pixel
 * @param color the color of the pixel, true for white, false for black
 */
void OLED_DrawPixel(uint8_t x, uint8_t y, bool color);

/**
 * @brief Draws a single character on the OLED display.
 *
 * @param x the x-coordinate of the position
 * @param y the y-coordinate of the position
 * @param c the character to draw
 */
void OLED_DrawChar(uint8_t x, uint8_t y, char c);

/**
 * @brief Draws a string on the OLED display.
 *
 * @param x the x-coordinate of the position
 * @param y the y-coordinate of the position
 * @param str the string to draw
 */
void OLED_DrawString(uint8_t x, uint8_t y, const char* str);

/**
 * @brief Draws a single line on the OLED display.
 *
 * @param x0 the x-coordinate of the start of the line
 * @param y0 the y-coordinate of the start of the line
 * @param x1 the x-coordinate of the end of the line
 * @param y1 the y-coordinate of the end of the line
 * @param color the color of the line, true for white, false for black
 */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool color);

/**
 * @brief Draws a rectangle on the OLED display.
 *
 * @param x the x-coordinate of the top-left of the rectangle
 * @param y the y-coordinate of the top-left of the rectangle
 * @param width the width of the rectangle
 * @param height the height of the rectangle
 * @param color the color of the rectangle, true for white, false for black
 */
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                        bool color);

/**
 * @brief Fills a rectangle on the OLED display.
 *
 * @param x the x-coordinate of the top-left of the rectangle
 * @param y the y-coordinate of the top-left of the rectangle
 * @param width the width of the rectangle
 * @param height the height of the rectangle
 * @param color the color of the rectangle, true for white, false for black
 */
void OLED_FillRectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                        bool color);

/**
 * @brief Draws a circle on the OLED display.
 *
 * @param x0 the x-coordinate of the center of the circle
 * @param y0 the y-coordinate of the center of the circle
 * @param radius the radius of the circle
 * @param color the color of the circle, true for white, false for black
 */
void OLED_DrawCircle(uint8_t x0, uint8_t y0, uint8_t radius, bool color);

/**
 * @brief Fills a circle on the OLED display.
 *
 * @param x0 the x-coordinate of the center of the circle
 * @param y0 the y-coordinate of the center of the circle
 * @param radius the radius of the circle
 * @param color the color of the circle, true for white, false for black
 */
void OLED_FillCircle(uint8_t x0, uint8_t y0, uint8_t radius, bool color);

/**
 * @brief Updates the OLED display.
 */
void OLED_UpdateScreen(void);

/**
 * @brief Scrolls the OLED display to the right.
 *
 * @param start_page the starting page number
 * @param end_page the ending page number
 * @param speed the scroll speed
 */
void OLED_ScrollRight(uint8_t start_page, uint8_t end_page, uint8_t speed);

/**
 * @brief Scrolls the OLED display to the left.
 *
 * @param start_page the starting page number
 * @param end_page the ending page number
 * @param speed the scroll speed
 */
void OLED_ScrollLeft(uint8_t start_page, uint8_t end_page, uint8_t speed);

/**
 * @brief Stops scrolling the OLED display.
 */
void OLED_StopScroll(void);

/**
 * @brief Draws a bitmap on the OLED display.
 *
 * @param x the x-coordinate of the top-left of the bitmap
 * @param y the y-coordinate of the top-left of the bitmap
 * @param bitmap the bitmap data
 * @param width the width of the bitmap
 * @param height the height of the bitmap
 */
void OLED_DrawBitmap(uint8_t x, uint8_t y, const uint8_t* bitmap, uint8_t width,
                     uint8_t height);

#endif  // OLED_H_
