/**
 * @file oled.c
 *
 * @brief Source file for OLED interface implementation.
 *
 * @details This file provides the implementation of the functions required to
 * initialize and control an OLED screen using STM32F4 microcontrollers. It
 * includes I2C configuration, delays and data transfer methods.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#include "oled.h"

/* Private variables */
static uint8_t OLED_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];
static uint32_t OLED_Timeout = 10000;

/* Private function prototypes */
static void OLED_DelayMs(uint32_t ms);
static OLED_Status OLED_I2C_Write(uint8_t addr, uint8_t reg, uint8_t data);
static OLED_Status OLED_I2C_WriteMulti(uint8_t addr, uint8_t reg, uint8_t *data,
                                       uint16_t count);

/* Delay function */
static void OLED_DelayMs(uint32_t ms) {
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  nCount = (RCC_Clocks.HCLK_Frequency / 10000) * ms;
  for (; nCount != 0; nCount--)
    ;
}

/* I2C Write Single Byte */
static OLED_Status OLED_I2C_Write(uint8_t addr, uint8_t reg, uint8_t data) {
  uint32_t timeout = OLED_Timeout;
  uint32_t status = OLED_OK;

  /* Wait while I2C busy */
  while (I2C_GetFlagStatus(OLED_I2C, I2C_FLAG_BUSY)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send START condition */
  I2C_GenerateSTART(OLED_I2C, ENABLE);

  /* Wait for EV5 */
  timeout = OLED_Timeout;
  while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send device address for write */
  I2C_Send7bitAddress(OLED_I2C, addr, I2C_Direction_Transmitter);

  /* Wait for EV6 */
  timeout = OLED_Timeout;
  while (
      !I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send register */
  I2C_SendData(OLED_I2C, reg);

  /* Wait for EV8 */
  timeout = OLED_Timeout;
  while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send data */
  I2C_SendData(OLED_I2C, data);

  /* Wait for EV8_2 */
  timeout = OLED_Timeout;
  while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(OLED_I2C, ENABLE);

  return status;
}

/* I2C Write Multiple Bytes */
static OLED_Status OLED_I2C_WriteMulti(uint8_t addr, uint8_t reg, uint8_t *data,
                                       uint16_t count) {
  uint32_t timeout = OLED_Timeout;
  uint32_t status = OLED_OK;

  /* Wait while I2C busy */
  while (I2C_GetFlagStatus(OLED_I2C, I2C_FLAG_BUSY)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send START condition */
  I2C_GenerateSTART(OLED_I2C, ENABLE);

  /* Wait for EV5 */
  timeout = OLED_Timeout;
  while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send device address for write */
  I2C_Send7bitAddress(OLED_I2C, addr, I2C_Direction_Transmitter);

  /* Wait for EV6 */
  timeout = OLED_Timeout;
  while (
      !I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send register */
  I2C_SendData(OLED_I2C, reg);

  /* Wait for EV8 */
  timeout = OLED_Timeout;
  while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if ((timeout--) == 0) {
      status = OLED_TIMEOUT;
    }
  }

  /* Send data */
  while (count--) {
    I2C_SendData(OLED_I2C, *data++);

    /* Wait for EV8 */
    timeout = OLED_Timeout;
    while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
      if ((timeout--) == 0) {
        status = OLED_TIMEOUT;
      }
    }
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(OLED_I2C, ENABLE);

  return status;
}

/* Initialize OLED Display */
void OLED_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  I2C_InitTypeDef I2C_InitStruct;

  /* Enable clocks */
  RCC_AHB1PeriphClockCmd(OLED_I2C_GPIO_CLK, ENABLE);
  RCC_APB1PeriphClockCmd(OLED_I2C_CLK, ENABLE);

  /* Configure I2C pins */
  GPIO_InitStruct.GPIO_Pin = OLED_I2C_SCL_PIN | OLED_I2C_SDA_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(OLED_I2C_GPIO, &GPIO_InitStruct);

  /* Connect pins to I2C */
  GPIO_PinAFConfig(OLED_I2C_GPIO, OLED_I2C_SCL_SOURCE, OLED_I2C_AF);
  GPIO_PinAFConfig(OLED_I2C_GPIO, OLED_I2C_SDA_SOURCE, OLED_I2C_AF);

/* Special handling for I2C3 if using PA8/PC9 configuration */
#ifdef OLED_I2C_GPIO2
  GPIO_InitStruct.GPIO_Pin = OLED_I2C_SDA_PIN;
  GPIO_Init(OLED_I2C_GPIO2, &GPIO_InitStruct);
  GPIO_PinAFConfig(OLED_I2C_GPIO2, OLED_I2C_SDA_SOURCE, OLED_I2C_AF);
#endif

  /* Configure I2C */
  I2C_InitStruct.I2C_ClockSpeed = 400000;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0x00;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(OLED_I2C, &I2C_InitStruct);

  /* Enable I2C */
  I2C_Cmd(OLED_I2C, ENABLE);

  /* Wait for initialization */
  OLED_DelayMs(100);

  /* Initialize display */
  OLED_WriteCommand(OLED_CMD_DISPLAY_OFF);
  OLED_WriteCommand(OLED_CMD_SET_DISPLAY_CLOCK);
  OLED_WriteCommand(0x80);
  OLED_WriteCommand(OLED_CMD_SET_MULTIPLEX);
  OLED_WriteCommand(0x3F);
  OLED_WriteCommand(OLED_CMD_SET_DISPLAY_OFFSET);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(OLED_CMD_SET_START_LINE | 0x00);
  OLED_WriteCommand(OLED_CMD_CHARGE_PUMP);
  OLED_WriteCommand(0x14);
  OLED_WriteCommand(OLED_CMD_MEMORY_MODE);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(OLED_CMD_SEG_REMAP | 0x01);
  OLED_WriteCommand(OLED_CMD_COM_SCAN_DEC);
  OLED_WriteCommand(OLED_CMD_SET_COM_PINS);
  OLED_WriteCommand(0x12);
  OLED_WriteCommand(OLED_CMD_SET_CONTRAST);
  OLED_WriteCommand(0xCF);
  OLED_WriteCommand(OLED_CMD_SET_PRECHARGE);
  OLED_WriteCommand(0xF1);
  OLED_WriteCommand(OLED_CMD_SET_VCOM_DETECT);
  OLED_WriteCommand(0x40);
  OLED_WriteCommand(OLED_CMD_DISPLAY_ALL_ON_RESUME);
  OLED_WriteCommand(OLED_CMD_NORMAL_DISPLAY);
  OLED_WriteCommand(OLED_CMD_DISPLAY_ON);

  /* Clear screen */
  OLED_Clear();
  OLED_UpdateScreen();
}

/* Write command to OLED */
void OLED_WriteCommand(uint8_t cmd) {
  OLED_I2C_Write(OLED_I2C_ADDR, OLED_CONTROL_BYTE_CMD_SINGLE, cmd);
}

/* Write data to OLED */
void OLED_WriteData(uint8_t data) {
  OLED_I2C_Write(OLED_I2C_ADDR, OLED_CONTROL_BYTE_DATA_STREAM, data);
}

/* Write multiple data to OLED */
void OLED_WriteMultipleData(uint8_t *data, uint16_t size) {
  OLED_I2C_WriteMulti(OLED_I2C_ADDR, OLED_CONTROL_BYTE_DATA_STREAM, data, size);
}

/* Set cursor position */
void OLED_SetCursor(uint8_t x, uint8_t y) {
  OLED_WriteCommand(OLED_CMD_COLUMN_ADDR);
  OLED_WriteCommand(x);
  OLED_WriteCommand(OLED_WIDTH - 1);
  OLED_WriteCommand(OLED_CMD_PAGE_ADDR);
  OLED_WriteCommand(y);
  OLED_WriteCommand(OLED_PAGES - 1);
}

/* Clear screen buffer */
void OLED_Clear(void) { memset(OLED_Buffer, 0x00, sizeof(OLED_Buffer)); }

/* Turn display on */
void OLED_DisplayOn(void) { OLED_WriteCommand(OLED_CMD_DISPLAY_ON); }

/* Turn display off */
void OLED_DisplayOff(void) { OLED_WriteCommand(OLED_CMD_DISPLAY_OFF); }

/* Set display contrast */
void OLED_SetContrast(uint8_t contrast) {
  OLED_WriteCommand(OLED_CMD_SET_CONTRAST);
  OLED_WriteCommand(contrast);
}

/* Invert display */
void OLED_InvertDisplay(bool invert) {
  if (invert) {
    OLED_WriteCommand(OLED_CMD_INVERT_DISPLAY);
  } else {
    OLED_WriteCommand(OLED_CMD_NORMAL_DISPLAY);
  }
}

/* Draw pixel in buffer */
void OLED_DrawPixel(uint8_t x, uint8_t y, bool color) {
  if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
    return;
  }

  if (color) {
    OLED_Buffer[x + (y / 8) * OLED_WIDTH] |= 1 << (y % 8);
  } else {
    OLED_Buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
  }
}

/* Draw character */
void OLED_DrawChar(uint8_t x, uint8_t y, char c) {
  uint8_t i, j;

  if (c < 32 || c > 122) {
    c = 32; // Replace with space
  }

  for (i = 0; i < 5; i++) {
    uint8_t line = Font5x7[c - 32][i];
    for (j = 0; j < 8; j++) {
      if (line & (1 << j)) {
        OLED_DrawPixel(x + i, y + j, true);
      }
    }
  }
}

/* Draw string */
void OLED_DrawString(uint8_t x, uint8_t y, const char *str) {
  while (*str) {
    OLED_DrawChar(x, y, *str);
    x += 6; // 5 pixels wide + 1 pixel space
    if (x + 5 >= OLED_WIDTH) {
      x = 0;
      y += 8;
      if (y >= OLED_HEIGHT) {
        break;
      }
    }
    str++;
  }
}

/* Draw line using Bresenham's algorithm */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool color) {
  int16_t dx = abs(x1 - x0);
  int16_t dy = abs(y1 - y0);
  int16_t sx = x0 < x1 ? 1 : -1;
  int16_t sy = y0 < y1 ? 1 : -1;
  int16_t err = dx - dy;

  while (1) {
    OLED_DrawPixel(x0, y0, color);

    if (x0 == x1 && y0 == y1)
      break;

    int16_t e2 = 2 * err;

    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }

    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

/* Draw rectangle */
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                        bool color) {
  OLED_DrawLine(x, y, x + width - 1, y, color);
  OLED_DrawLine(x, y + height - 1, x + width - 1, y + height - 1, color);
  OLED_DrawLine(x, y, x, y + height - 1, color);
  OLED_DrawLine(x + width - 1, y, x + width - 1, y + height - 1, color);
}

/* Fill rectangle */
void OLED_FillRectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height,
                        bool color) {
  uint8_t i, j;

  for (i = x; i < x + width; i++) {
    for (j = y; j < y + height; j++) {
      OLED_DrawPixel(i, j, color);
    }
  }
}

/* Draw circle using Midpoint Circle Algorithm */
void OLED_DrawCircle(uint8_t x0, uint8_t y0, uint8_t radius, bool color) {
  int16_t x = radius;
  int16_t y = 0;
  int16_t err = 0;

  while (x >= y) {
    OLED_DrawPixel(x0 + x, y0 + y, color);
    OLED_DrawPixel(x0 + y, y0 + x, color);
    OLED_DrawPixel(x0 - y, y0 + x, color);
    OLED_DrawPixel(x0 - x, y0 + y, color);
    OLED_DrawPixel(x0 - x, y0 - y, color);
    OLED_DrawPixel(x0 - y, y0 - x, color);
    OLED_DrawPixel(x0 + y, y0 - x, color);
    OLED_DrawPixel(x0 + x, y0 - y, color);

    if (err <= 0) {
      y += 1;
      err += 2 * y + 1;
    }

    if (err > 0) {
      x -= 1;
      err -= 2 * x + 1;
    }
  }
}

/* Fill circle */
void OLED_FillCircle(uint8_t x0, uint8_t y0, uint8_t radius, bool color) {
  int16_t x = radius;
  int16_t y = 0;
  int16_t err = 0;

  while (x >= y) {
    OLED_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
    OLED_DrawLine(x0 - x, y0 - y, x0 + x, y0 - y, color);
    OLED_DrawLine(x0 - y, y0 + x, x0 + y, y0 + x, color);
    OLED_DrawLine(x0 - y, y0 - x, x0 + y, y0 - x, color);

    if (err <= 0) {
      y += 1;
      err += 2 * y + 1;
    }

    if (err > 0) {
      x -= 1;
      err -= 2 * x + 1;
    }
  }
}

/* Update screen with buffer content */
void OLED_UpdateScreen(void) {
  uint8_t i;

  for (i = 0; i < 8; i++) {
    OLED_WriteCommand(0xB0 + i); // Set page address
    OLED_WriteCommand(0x00);     // Set lower column address
    OLED_WriteCommand(0x10);     // Set higher column address

    OLED_WriteMultipleData(&OLED_Buffer[OLED_WIDTH * i], OLED_WIDTH);
  }
}

/* Scroll right */
void OLED_ScrollRight(uint8_t start_page, uint8_t end_page, uint8_t speed) {
  OLED_WriteCommand(OLED_CMD_RIGHT_HORIZONTAL_SCROLL);
  OLED_WriteCommand(0x00);       // Dummy byte
  OLED_WriteCommand(start_page); // Start page
  OLED_WriteCommand(speed);      // Time interval
  OLED_WriteCommand(end_page);   // End page
  OLED_WriteCommand(0x00);       // Dummy byte
  OLED_WriteCommand(0xFF);       // Dummy byte
  OLED_WriteCommand(OLED_CMD_ACTIVATE_SCROLL);
}

/* Scroll left */
void OLED_ScrollLeft(uint8_t start_page, uint8_t end_page, uint8_t speed) {
  OLED_WriteCommand(OLED_CMD_LEFT_HORIZONTAL_SCROLL);
  OLED_WriteCommand(0x00);       // Dummy byte
  OLED_WriteCommand(start_page); // Start page
  OLED_WriteCommand(speed);      // Time interval
  OLED_WriteCommand(end_page);   // End page
  OLED_WriteCommand(0x00);       // Dummy byte
  OLED_WriteCommand(0xFF);       // Dummy byte
  OLED_WriteCommand(OLED_CMD_ACTIVATE_SCROLL);
}

/* Stop scrolling */
void OLED_StopScroll(void) { OLED_WriteCommand(OLED_CMD_DEACTIVATE_SCROLL); }

/* Draw bitmap */
void OLED_DrawBitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width,
                     uint8_t height) {
  uint8_t i, j;
  uint8_t byte_width = (width + 7) / 8;

  for (j = 0; j < height; j++) {
    for (i = 0; i < width; i++) {
      uint8_t byte = bitmap[j * byte_width + i / 8];
      uint8_t bit = byte & (0x80 >> (i % 8));
      OLED_DrawPixel(x + i, y + j, bit != 0);
    }
  }
}
