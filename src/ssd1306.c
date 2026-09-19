/**
 * @file ssd1306.c
 *
 * @brief Source file for SSD1306 interface implementation.
 *
 * @details This file provides the implementation of the functions required to
 * initialize and control an SSD1306 screen. Includes I2C configuration
 * and data transfer methods.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#include <ssd1306.h>

// Private variables
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static uint32_t SSD1306_Timeout = 10000;

// Function prototypes
static void SSD1306_DelayMs(uint32_t ms);
static SSD1306_Status SSD1306_I2C_Write(uint8_t addr, uint8_t reg,
                                        uint8_t data);
static SSD1306_Status SSD1306_I2C_WriteMulti(uint8_t addr, uint8_t reg,
                                             uint8_t *data, uint16_t count);

static void SSD1306_DelayMs(uint32_t ms) {
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  nCount = (RCC_Clocks.HCLK_Frequency / 10000) * ms;
  for (; nCount != 0; nCount--)
    ;
}

static SSD1306_Status SSD1306_I2C_Write(uint8_t addr, uint8_t reg,
                                        uint8_t data) {
  uint32_t timeout = SSD1306_Timeout;
  uint32_t status = SSD1306_OK;

  /* Wait while I2C busy */
  while (I2C_GetFlagStatus(SSD1306_I2C, I2C_FLAG_BUSY)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send START condition */
  I2C_GenerateSTART(SSD1306_I2C, ENABLE);

  /* Wait for EV5 */
  timeout = SSD1306_Timeout;
  while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send device address for write */
  I2C_Send7bitAddress(SSD1306_I2C, addr, I2C_Direction_Transmitter);

  /* Wait for EV6 */
  timeout = SSD1306_Timeout;
  while (!I2C_CheckEvent(SSD1306_I2C,
                         I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send register */
  I2C_SendData(SSD1306_I2C, reg);

  /* Wait for EV8 */
  timeout = SSD1306_Timeout;
  while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send data */
  I2C_SendData(SSD1306_I2C, data);

  /* Wait for EV8_2 */
  timeout = SSD1306_Timeout;
  while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(SSD1306_I2C, ENABLE);

  return status;
}

static SSD1306_Status SSD1306_I2C_WriteMulti(uint8_t addr, uint8_t reg,
                                             uint8_t *data, uint16_t count) {
  uint32_t timeout = SSD1306_Timeout;
  uint32_t status = SSD1306_OK;

  /* Wait while I2C busy */
  while (I2C_GetFlagStatus(SSD1306_I2C, I2C_FLAG_BUSY)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send START condition */
  I2C_GenerateSTART(SSD1306_I2C, ENABLE);

  /* Wait for EV5 */
  timeout = SSD1306_Timeout;
  while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send device address for write */
  I2C_Send7bitAddress(SSD1306_I2C, addr, I2C_Direction_Transmitter);

  /* Wait for EV6 */
  timeout = SSD1306_Timeout;
  while (!I2C_CheckEvent(SSD1306_I2C,
                         I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send register */
  I2C_SendData(SSD1306_I2C, reg);

  /* Wait for EV8 */
  timeout = SSD1306_Timeout;
  while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if ((timeout--) == 0) {
      status = SSD1306_TIMEOUT;
    }
  }

  /* Send data */
  while (count--) {
    I2C_SendData(SSD1306_I2C, *data++);

    /* Wait for EV8 */
    timeout = SSD1306_Timeout;
    while (!I2C_CheckEvent(SSD1306_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
      if ((timeout--) == 0) {
        status = SSD1306_TIMEOUT;
      }
    }
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(SSD1306_I2C, ENABLE);

  return status;
}

static void SSD1306_WriteCommand(uint8_t cmd) {
  SSD1306_I2C_Write(SSD1306_I2C_ADDR, SSD1306_CONTROL_BYTE_CMD_SINGLE, cmd);
}

void SSD1306_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  I2C_InitTypeDef I2C_InitStruct;

  /* Enable clocks */
  RCC_AHB1PeriphClockCmd(SSD1306_I2C_GPIO_CLK, ENABLE);
  RCC_APB1PeriphClockCmd(SSD1306_I2C_CLK, ENABLE);

  /* Configure I2C pins */
  GPIO_InitStruct.GPIO_Pin = SSD1306_I2C_SCL_PIN | SSD1306_I2C_SDA_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SSD1306_I2C_GPIO, &GPIO_InitStruct);

  /* Connect pins to I2C */
  GPIO_PinAFConfig(SSD1306_I2C_GPIO, SSD1306_I2C_SCL_SOURCE, SSD1306_I2C_AF);
  GPIO_PinAFConfig(SSD1306_I2C_GPIO, SSD1306_I2C_SDA_SOURCE, SSD1306_I2C_AF);

  /* Special handling for I2C3 if using PA8/PC9 configuration */
#ifdef SSD1306_I2C_GPIO2
  GPIO_InitStruct.GPIO_Pin = SSD1306_I2C_SDA_PIN;
  GPIO_Init(SSD1306_I2C_GPIO2, &GPIO_InitStruct);
  GPIO_PinAFConfig(SSD1306_I2C_GPIO2, SSD1306_I2C_SDA_SOURCE, SSD1306_I2C_AF);
#endif

  /* Configure I2C */
  I2C_InitStruct.I2C_ClockSpeed = 400000;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0x00;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(SSD1306_I2C, &I2C_InitStruct);

  /* Enable I2C */
  I2C_Cmd(SSD1306_I2C, ENABLE);

  /* Wait for initialization */
  SSD1306_DelayMs(100);

  /* Initialize display */
  SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_OFF);
  SSD1306_WriteCommand(SSD1306_CMD_SET_DISPLAY_CLOCK);
  SSD1306_WriteCommand(0x80);
  SSD1306_WriteCommand(SSD1306_CMD_SET_MULTIPLEX);
  SSD1306_WriteCommand(0x3F);
  SSD1306_WriteCommand(SSD1306_CMD_SET_DISPLAY_OFFSET);
  SSD1306_WriteCommand(0x00);
  SSD1306_WriteCommand(SSD1306_CMD_SET_START_LINE);
  SSD1306_WriteCommand(SSD1306_CMD_CHARGE_PUMP);
  SSD1306_WriteCommand(0x14);
  SSD1306_WriteCommand(SSD1306_CMD_MEMORY_MODE);
  SSD1306_WriteCommand(0x00);
  SSD1306_WriteCommand(SSD1306_CMD_SEG_REMAP | 0x01);
  SSD1306_WriteCommand(SSD1306_CMD_COM_SCAN_DEC);
  SSD1306_WriteCommand(SSD1306_CMD_SET_COM_PINS);
  SSD1306_WriteCommand(0x12);
  SSD1306_WriteCommand(SSD1306_CMD_SET_CONTRAST);
  SSD1306_WriteCommand(0xCF);
  SSD1306_WriteCommand(SSD1306_CMD_SET_PRECHARGE);
  SSD1306_WriteCommand(0xF1);
  SSD1306_WriteCommand(SSD1306_CMD_SET_VCOM_DETECT);
  SSD1306_WriteCommand(0x40);
  SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_ALL_ON_RESUME);
  SSD1306_WriteCommand(SSD1306_CMD_NORMAL_DISPLAY);
  SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_ON);

  /* Clear screen */
  SSD1306_Clear();
  SSD1306_UpdateScreen();
}

void SSD1306_WriteData(uint8_t data) {
  SSD1306_I2C_Write(SSD1306_I2C_ADDR, SSD1306_CONTROL_BYTE_DATA_STREAM, data);
}

static void SSD1306_WriteMultipleData(uint8_t *data, uint16_t size) {
  SSD1306_I2C_WriteMulti(SSD1306_I2C_ADDR, SSD1306_CONTROL_BYTE_DATA_STREAM,
                         data, size);
}

static void SSD1306_DrawPixel(uint8_t x, uint8_t y, bool color) {
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
    return;
  }

  if (color) {
    SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
  } else {
    SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
  }
}

static void SSD1306_DrawChar(uint8_t x, uint8_t y, char c) {
  uint8_t i, j;

  if (c < 32 || c > 122) {
    c = 32; // Replace with space
  }

  for (i = 0; i < 5; i++) {
    uint8_t line = Font5x7[c - 32][i];
    for (j = 0; j < 8; j++) {
      if (line & (1 << j)) {
        SSD1306_DrawPixel(x + i, y + j, true);
      }
    }
  }
}

void SSD1306_SetCursor(uint8_t x, uint8_t y) {
  SSD1306_WriteCommand(SSD1306_CMD_COLUMN_ADDR);
  SSD1306_WriteCommand(x);
  SSD1306_WriteCommand(SSD1306_WIDTH - 1);
  SSD1306_WriteCommand(SSD1306_CMD_PAGE_ADDR);
  SSD1306_WriteCommand(y);
  SSD1306_WriteCommand(SSD1306_PAGES - 1);
}

void SSD1306_Clear(void) {
  memset(SSD1306_Buffer, 0x00, sizeof(SSD1306_Buffer));
}

void SSD1306_DisplayOn(void) {
  SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_ON);
}

void SSD1306_DisplayOff(void) {
  SSD1306_WriteCommand(SSD1306_CMD_DISPLAY_OFF);
}

void SSD1306_SetContrast(uint8_t contrast) {
  SSD1306_WriteCommand(SSD1306_CMD_SET_CONTRAST);
  SSD1306_WriteCommand(contrast);
}

void SSD1306_InvertDisplay(bool invert) {
  if (invert) {
    SSD1306_WriteCommand(SSD1306_CMD_INVERT_DISPLAY);
  } else {
    SSD1306_WriteCommand(SSD1306_CMD_NORMAL_DISPLAY);
  }
}

void SSD1306_UpdateScreen(void) {
  uint8_t i;

  for (i = 0; i < 8; i++) {
    SSD1306_WriteCommand(0xB0 + i); // Set page address
    SSD1306_WriteCommand(0x00);     // Set lower column address
    SSD1306_WriteCommand(0x10);     // Set higher column address

    SSD1306_WriteMultipleData(&SSD1306_Buffer[SSD1306_WIDTH * i],
                              SSD1306_WIDTH);
  }
}

void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str) {
  while (*str) {
    SSD1306_DrawChar(x, y, *str);
    x += 6; // 5 pixels wide + 1 pixel space
    if (x + 5 >= SSD1306_WIDTH) {
      x = 0;
      y += 8;
      if (y >= SSD1306_HEIGHT) {
        break;
      }
    }
    str++;
  }
}

void SSD1306_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,
                      bool color) {
  int16_t dx = x1 > x0 ? x1 - x0 : x0 - x1;
  int16_t dy = y1 > y0 ? y1 - y0 : y0 - y1;
  int16_t sx = x0 < x1 ? 1 : -1;
  int16_t sy = y0 < y1 ? 1 : -1;
  int16_t err = dx - dy;
  int16_t e2;

  while (1) {
    SSD1306_DrawPixel(x0, y0, color);

    if (x0 == x1 && y0 == y1) {
      break;
    }

    e2 = 2 * err;
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

void SSD1306_DrawBitmap(uint8_t x, uint8_t y, const uint8_t *bitmap,
                        uint8_t width, uint8_t height) {
  uint8_t i, j;
  uint8_t byte_width = (width + 7) / 8;

  for (j = 0; j < height; j++) {
    for (i = 0; i < width; i++) {
      uint8_t byte = bitmap[j * byte_width + i / 8];
      uint8_t bit = byte & (0x80 >> (i % 8));
      SSD1306_DrawPixel(x + i, y + j, bit != 0);
    }
  }
}

void SSD1306_DrawWaveform(const int16_t *audioBuffer, uint16_t bufferSize,
                          uint8_t startX, uint8_t width) {
  if (audioBuffer == NULL || bufferSize == 0 || width == 0) {
    return;
  }

  // Calculate sampling interval to fit width
  uint16_t sampleStep = bufferSize / width;
  if (sampleStep == 0) {
    sampleStep = 1;
  }

  // Draw waveform
  uint8_t prevY = 32;
  for (uint8_t i = 0; i < width; i++) {
    // Get sample from buffer
    uint16_t sampleIndex = i * sampleStep;
    if (sampleIndex >= bufferSize) {
      sampleIndex = bufferSize - 1;
    }

    // Convert 16-bit audio sample to Y coordinate (0-63)
    // Audio range: -32768 to 32767
    // Map to display range: 0 to 63, center at 32
    int16_t sample = audioBuffer[sampleIndex];
    int16_t scaledSample = (int16_t)((int32_t)sample * 28 / 32768);
    uint8_t y = 32 - scaledSample;

    // Clamp to display bounds
    if (y > 63) {
      y = 63;
    }

    // Draw line from previous point to current point
    if (i > 0) {
      SSD1306_DrawLine(startX + i - 1, prevY, startX + i, y, true);
    }

    prevY = y;
  }
}
