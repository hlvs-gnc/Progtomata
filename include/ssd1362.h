/**
 * @file ssd1362.h
 *
 * @brief Header file for SSD1362 interface functions.
 *
 * @details This file defines initialization and control functions for
 * managing a 256x64, 16 gray-scale SSD1362 OLED display.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#ifndef SSD1362_H_
#define SSD1362_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_rcc.h>

/* Interface selection: define SSD1362_INTERFACE as SSD1362_IFACE_I2C before
 * including this header to use I2C instead of the default 4-wire SPI. */
#define SSD1362_IFACE_SPI 0
#define SSD1362_IFACE_I2C 1
#ifndef SSD1362_INTERFACE
#define SSD1362_INTERFACE SSD1362_IFACE_SPI
#endif

/* SSD1362 Display Configuration */
#define SSD1362_WIDTH  256
#define SSD1362_HEIGHT 64
/* Each GDDRAM byte packs 2 horizontally adjacent 4-bit gray-scale pixels */
#define SSD1362_BUF_COLS (SSD1362_WIDTH / 2)

/* Error handling */
typedef enum { SSD1362_OK = 0, SSD1362_ERROR, SSD1362_TIMEOUT } SSD1362_Status;

/* Control pins, common to both SPI and I2C wiring (freed from the removed
 * parallel LCD driver, hence GPIOE). PE0-PE2 are used by the encoder. */
#define SSD1362_CTRL_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SSD1362_CTRL_GPIO     GPIOE
#define SSD1362_CS_PIN        GPIO_Pin_3 // Tie to VSS externally when in I2C mode
#define SSD1362_DC_PIN        GPIO_Pin_4 // D/C# in SPI mode, SA0 in I2C mode
#define SSD1362_RES_PIN       GPIO_Pin_5

#if SSD1362_INTERFACE == SSD1362_IFACE_SPI
/* Bit-banged 4-wire SPI pins (D0 = SCLK, D1 = SDIN) */
#define SSD1362_SCLK_PIN GPIO_Pin_6
#define SSD1362_SDIN_PIN GPIO_Pin_7
#else
/* Hardware I2C peripheral (separate bus from the SSD1306, which uses I2C1) */
#define SSD1362_I2C                      I2C2
#define SSD1362_I2C_CLK                  RCC_APB1Periph_I2C2
#define SSD1362_I2C_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define SSD1362_I2C_GPIO                 GPIOB
#define SSD1362_I2C_SCL_PIN              GPIO_Pin_10
#define SSD1362_I2C_SDA_PIN              GPIO_Pin_11
#define SSD1362_I2C_SCL_SOURCE           GPIO_PinSource10
#define SSD1362_I2C_SDA_SOURCE           GPIO_PinSource11
#define SSD1362_I2C_AF                   GPIO_AF_I2C2
/* Slave address for SA0 (DC pin) tied LOW; use 0x7A if wired HIGH */
#define SSD1362_I2C_ADDR                 0x78

/* I2C control byte (Co, D/C#, 000000) */
#define SSD1362_CONTROL_BYTE_CMD_STREAM  0x00
#define SSD1362_CONTROL_BYTE_DATA_STREAM 0x40
#endif

/* Set Re-map (A0h) argument: 0x43 (SEG odd/even split + Column
 * Address/Nibble remap) was confirmed correct for both text and the
 * original wizard GIF (centered, no cuts, no overlap). The "2 copies"
 * overlap seen later traced back to the source art's aspect ratio
 * changing drastically (not this remap value), so leave this as-is
 * unless the panel wiring genuinely changes. Add 0x10 to flip vertically
 * (COM remap). See datasheet Table 9-1. */
#ifndef SSD1362_REMAP
#define SSD1362_REMAP 0x43 // SEG odd/even split + Column Address/Nibble remap
#endif

/* SSD1362 Commands (see datasheet section 8/9) */
#define SSD1362_CMD_SET_COLUMN_ADDR     0x15
#define SSD1362_CMD_SET_ROW_ADDR        0x75
#define SSD1362_CMD_SET_CONTRAST        0x81
#define SSD1362_CMD_SET_REMAP           0xA0
#define SSD1362_CMD_SET_START_LINE      0xA1
#define SSD1362_CMD_SET_DISPLAY_OFFSET  0xA2
#define SSD1362_CMD_SET_VERTICAL_SCROLL 0xA3
#define SSD1362_CMD_DISPLAY_NORMAL      0xA4
#define SSD1362_CMD_DISPLAY_ALL_ON      0xA5
#define SSD1362_CMD_DISPLAY_ALL_OFF     0xA6
#define SSD1362_CMD_DISPLAY_INVERSE     0xA7
#define SSD1362_CMD_SET_MUX_RATIO       0xA8
#define SSD1362_CMD_FUNCTION_SELECT_A   0xAB
#define SSD1362_CMD_IREF_SELECTION      0xAD
#define SSD1362_CMD_DISPLAY_OFF         0xAE
#define SSD1362_CMD_DISPLAY_ON          0xAF
#define SSD1362_CMD_SET_PHASE_LENGTH    0xB1
#define SSD1362_CMD_SET_CLOCK_DIV       0xB3
#define SSD1362_CMD_SET_GPIO            0xB5
#define SSD1362_CMD_SET_PRECHARGE2      0xB6
#define SSD1362_CMD_SET_GRAY_TABLE      0xB8
#define SSD1362_CMD_SET_LINEAR_LUT      0xB9
#define SSD1362_CMD_SET_PRECHARGE_VOLT  0xBC
#define SSD1362_CMD_PRECHARGE_CAP_SEL   0xBD
#define SSD1362_CMD_SET_VCOMH           0xBE
#define SSD1362_CMD_SET_COMMAND_LOCK    0xFD
#define SSD1362_CMD_SET_FADE_BLINK      0x23

/* Basic 5x7 font, shared column layout with the SSD1306 driver */
static const uint8_t SSD1362_Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
};

/**
 * @brief Initializes the SSD1362 display.
 *
 * Configures the control/interface pins and peripheral, resets the panel
 * and applies the default gray-scale display configuration. Must be called
 * before any other function in this module.
 */
void SSD1362_Init(void);

/**
 * @brief Clears the SSD1362 display buffer (does not push it to the panel).
 */
void SSD1362_Clear(void);

/**
 * @brief Turns the SSD1362 display on.
 */
void SSD1362_DisplayOn(void);

/**
 * @brief Turns the SSD1362 display off (sleep mode).
 */
void SSD1362_DisplayOff(void);

/**
 * @brief Sets the contrast of the SSD1362 display.
 *
 * @param contrast the contrast value, valid range is 0-255
 */
void SSD1362_SetContrast(uint8_t contrast);

/**
 * @brief Inverts the display of the SSD1362 display.
 *
 * @param invert if true, the display is inverted
 */
void SSD1362_InvertDisplay(bool invert);

/**
 * @brief Pushes the display buffer to the panel's GDDRAM.
 */
void SSD1362_UpdateScreen(void);

/**
 * @brief Sets a single pixel in the display buffer.
 *
 * @param x the x-coordinate of the pixel, 0-255
 * @param y the y-coordinate of the pixel, 0-63
 * @param gray the gray-scale level of the pixel, 0 (off) - 15 (brightest)
 */
void SSD1362_DrawPixel(uint16_t x, uint8_t y, uint8_t gray);

/**
 * @brief Draws a string on the SSD1362 display.
 *
 * @param x the x-coordinate of the position
 * @param y the y-coordinate of the position
 * @param str the string to draw
 * @param gray the gray-scale level to draw with, 0-15
 */
void SSD1362_DrawString(uint16_t x, uint8_t y, const char *str, uint8_t gray);

/**
 * @brief Draws a 4-bit-per-pixel packed bitmap on the SSD1362 display.
 *
 * @param x the x-coordinate of the top-left of the bitmap
 * @param y the y-coordinate of the top-left of the bitmap
 * @param bitmap the bitmap data, 2 pixels per byte (high nibble first)
 * @param width the width of the bitmap
 * @param height the height of the bitmap
 */
void SSD1362_DrawBitmap4(uint16_t x, uint8_t y, const uint8_t *bitmap,
                         uint16_t width, uint8_t height);

/**
 * @brief Draws a line on the SSD1362 display.
 *
 * @param x0 the x-coordinate of the starting point
 * @param y0 the y-coordinate of the starting point
 * @param x1 the x-coordinate of the ending point
 * @param y1 the y-coordinate of the ending point
 * @param gray the gray-scale level to draw with, 0-15
 */
void SSD1362_DrawLine(uint16_t x0, uint8_t y0, uint16_t x1, uint8_t y1,
                      uint8_t gray);

/**
 * @brief Draws a waveform visualization on the SSD1362 display.
 *
 * @param audioBuffer pointer to the audio buffer to visualize
 * @param bufferSize size of the audio buffer
 * @param startX starting x-coordinate for the waveform
 * @param width width of the waveform display area
 * @param gray the gray-scale level to draw with, 0-15
 */
void SSD1362_DrawWaveform(const int16_t *audioBuffer, uint16_t bufferSize,
                          uint16_t startX, uint16_t width, uint8_t gray);

#endif // SSD1362_H_
