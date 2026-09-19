/**
 * @file ssd1362.c
 *
 * @brief Source file for SSD1362 interface implementation.
 *
 * @details This file provides the implementation of the functions required
 * to initialize and control a 256x64 16 gray-scale SSD1362 OLED.
 * Supports both 4-wire SPI and I2C.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#include <ssd1362.h>

/* 4-bit gray-scale frame buffer, 2 pixels packed per byte */
static uint8_t SSD1362_Buffer[SSD1362_HEIGHT * SSD1362_BUF_COLS];

static void SSD1362_DelayMs(uint32_t ms) {
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  nCount = (RCC_Clocks.HCLK_Frequency / 10000) * ms;
  for (; nCount != 0; nCount--)
    ;
}

static void SSD1362_Reset(void) {
  GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_RES_PIN);
  SSD1362_DelayMs(10);
  GPIO_SetBits(SSD1362_CTRL_GPIO, SSD1362_RES_PIN);
  SSD1362_DelayMs(10);
}

#if SSD1362_INTERFACE == SSD1362_IFACE_SPI

static void SSD1362_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_AHB1PeriphClockCmd(SSD1362_CTRL_GPIO_CLK, ENABLE);

  GPIO_InitStruct.GPIO_Pin = SSD1362_CS_PIN | SSD1362_DC_PIN | SSD1362_RES_PIN |
                             SSD1362_SCLK_PIN | SSD1362_SDIN_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SSD1362_CTRL_GPIO, &GPIO_InitStruct);

  GPIO_SetBits(SSD1362_CTRL_GPIO, SSD1362_CS_PIN | SSD1362_RES_PIN);
  GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_SCLK_PIN | SSD1362_SDIN_PIN);
}

/* SDIN is sampled by the panel on the rising edge of SCLK, MSB first */
static void SSD1362_SPI_WriteByte(uint8_t byte) {
  for (uint8_t i = 0; i < 8; i++) {
    GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_SCLK_PIN);

    if (byte & 0x80) {
      GPIO_SetBits(SSD1362_CTRL_GPIO, SSD1362_SDIN_PIN);
    } else {
      GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_SDIN_PIN);
    }
    byte <<= 1;

    GPIO_SetBits(SSD1362_CTRL_GPIO, SSD1362_SCLK_PIN);
  }
  GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_SCLK_PIN);
}

static void SSD1362_WriteCommand(uint8_t cmd) {
  GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_CS_PIN);
  GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_DC_PIN);
  SSD1362_SPI_WriteByte(cmd);
  GPIO_SetBits(SSD1362_CTRL_GPIO, SSD1362_CS_PIN);
}

static void SSD1362_WriteData(const uint8_t *data, uint16_t count) {
  GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_CS_PIN);
  GPIO_SetBits(SSD1362_CTRL_GPIO, SSD1362_DC_PIN);
  while (count--) {
    SSD1362_SPI_WriteByte(*data++);
  }
  GPIO_SetBits(SSD1362_CTRL_GPIO, SSD1362_CS_PIN);
}

#else /* SSD1362_IFACE_I2C */

static uint32_t SSD1362_Timeout = 10000;

static void SSD1362_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_AHB1PeriphClockCmd(SSD1362_CTRL_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(SSD1362_I2C_GPIO_CLK, ENABLE);
  RCC_APB1PeriphClockCmd(SSD1362_I2C_CLK, ENABLE);

  /* CS# tied LOW and DC# used as the fixed SA0 slave address bit */
  GPIO_InitStruct.GPIO_Pin = SSD1362_CS_PIN | SSD1362_DC_PIN | SSD1362_RES_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SSD1362_CTRL_GPIO, &GPIO_InitStruct);

  GPIO_ResetBits(SSD1362_CTRL_GPIO, SSD1362_CS_PIN | SSD1362_DC_PIN);
  GPIO_SetBits(SSD1362_CTRL_GPIO, SSD1362_RES_PIN);

  GPIO_InitStruct.GPIO_Pin = SSD1362_I2C_SCL_PIN | SSD1362_I2C_SDA_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SSD1362_I2C_GPIO, &GPIO_InitStruct);

  GPIO_PinAFConfig(SSD1362_I2C_GPIO, SSD1362_I2C_SCL_SOURCE, SSD1362_I2C_AF);
  GPIO_PinAFConfig(SSD1362_I2C_GPIO, SSD1362_I2C_SDA_SOURCE, SSD1362_I2C_AF);

  I2C_InitTypeDef I2C_InitStruct;
  I2C_InitStruct.I2C_ClockSpeed = 400000;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0x00;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(SSD1362_I2C, &I2C_InitStruct);

  I2C_Cmd(SSD1362_I2C, ENABLE);
}

static SSD1362_Status SSD1362_I2C_Write(uint8_t control, const uint8_t *data,
                                        uint16_t count) {
  uint32_t timeout = SSD1362_Timeout;
  SSD1362_Status status = SSD1362_OK;

  while (I2C_GetFlagStatus(SSD1362_I2C, I2C_FLAG_BUSY)) {
    if ((timeout--) == 0) {
      return SSD1362_TIMEOUT;
    }
  }

  I2C_GenerateSTART(SSD1362_I2C, ENABLE);
  timeout = SSD1362_Timeout;
  while (!I2C_CheckEvent(SSD1362_I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
    if ((timeout--) == 0) {
      return SSD1362_TIMEOUT;
    }
  }

  I2C_Send7bitAddress(SSD1362_I2C, SSD1362_I2C_ADDR, I2C_Direction_Transmitter);
  timeout = SSD1362_Timeout;
  while (!I2C_CheckEvent(SSD1362_I2C,
                         I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    if ((timeout--) == 0) {
      return SSD1362_TIMEOUT;
    }
  }

  I2C_SendData(SSD1362_I2C, control);
  timeout = SSD1362_Timeout;
  while (!I2C_CheckEvent(SSD1362_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    if ((timeout--) == 0) {
      return SSD1362_TIMEOUT;
    }
  }

  while (count--) {
    I2C_SendData(SSD1362_I2C, *data++);
    timeout = SSD1362_Timeout;
    while (!I2C_CheckEvent(SSD1362_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
      if ((timeout--) == 0) {
        status = SSD1362_TIMEOUT;
        break;
      }
    }
  }

  I2C_GenerateSTOP(SSD1362_I2C, ENABLE);
  return status;
}

static void SSD1362_WriteCommand(uint8_t cmd) {
  SSD1362_I2C_Write(SSD1362_CONTROL_BYTE_CMD_STREAM, &cmd, 1);
}

static void SSD1362_WriteData(const uint8_t *data, uint16_t count) {
  SSD1362_I2C_Write(SSD1362_CONTROL_BYTE_DATA_STREAM, data, count);
}

#endif

void SSD1362_Init(void) {
  SSD1362_GPIO_Init();
  SSD1362_Reset();

  SSD1362_WriteCommand(SSD1362_CMD_SET_COMMAND_LOCK);
  SSD1362_WriteCommand(0x12); // Unlock

  SSD1362_WriteCommand(SSD1362_CMD_DISPLAY_OFF);

  SSD1362_WriteCommand(SSD1362_CMD_SET_COLUMN_ADDR);
  SSD1362_WriteCommand(0x00);
  SSD1362_WriteCommand(SSD1362_BUF_COLS - 1);

  SSD1362_WriteCommand(SSD1362_CMD_SET_ROW_ADDR);
  SSD1362_WriteCommand(0x00);
  SSD1362_WriteCommand(SSD1362_HEIGHT - 1);

  SSD1362_WriteCommand(SSD1362_CMD_SET_CONTRAST);
  SSD1362_WriteCommand(0x7F);

  SSD1362_WriteCommand(SSD1362_CMD_SET_REMAP);
  SSD1362_WriteCommand(SSD1362_REMAP);

  SSD1362_WriteCommand(SSD1362_CMD_SET_START_LINE);
  SSD1362_WriteCommand(0x00);

  SSD1362_WriteCommand(SSD1362_CMD_SET_DISPLAY_OFFSET);
  SSD1362_WriteCommand(0x00);

  SSD1362_WriteCommand(SSD1362_CMD_SET_MUX_RATIO);
  SSD1362_WriteCommand(0x3F); // 64 MUX

  SSD1362_WriteCommand(SSD1362_CMD_FUNCTION_SELECT_A);
  SSD1362_WriteCommand(0x01); // Enable internal VDD regulator

  SSD1362_WriteCommand(SSD1362_CMD_SET_PHASE_LENGTH);
  SSD1362_WriteCommand(0x82);

  SSD1362_WriteCommand(SSD1362_CMD_SET_CLOCK_DIV);
  SSD1362_WriteCommand(0xA1);

  SSD1362_WriteCommand(SSD1362_CMD_SET_PRECHARGE2);
  SSD1362_WriteCommand(0x04);

  SSD1362_WriteCommand(SSD1362_CMD_SET_PRECHARGE_VOLT);
  SSD1362_WriteCommand(0x04);

  SSD1362_WriteCommand(SSD1362_CMD_SET_VCOMH);
  SSD1362_WriteCommand(0x05);

  SSD1362_WriteCommand(SSD1362_CMD_SET_LINEAR_LUT);

  SSD1362_WriteCommand(SSD1362_CMD_DISPLAY_NORMAL);

  SSD1362_Clear();
  SSD1362_UpdateScreen();

  SSD1362_WriteCommand(SSD1362_CMD_DISPLAY_ON);
}

void SSD1362_Clear(void) {
  memset(SSD1362_Buffer, 0x00, sizeof(SSD1362_Buffer));
}

void SSD1362_DisplayOn(void) {
  SSD1362_WriteCommand(SSD1362_CMD_DISPLAY_ON);
}

void SSD1362_DisplayOff(void) {
  SSD1362_WriteCommand(SSD1362_CMD_DISPLAY_OFF);
}

void SSD1362_SetContrast(uint8_t contrast) {
  SSD1362_WriteCommand(SSD1362_CMD_SET_CONTRAST);
  SSD1362_WriteCommand(contrast);
}

void SSD1362_InvertDisplay(bool invert) {
  SSD1362_WriteCommand(invert ? SSD1362_CMD_DISPLAY_INVERSE
                              : SSD1362_CMD_DISPLAY_NORMAL);
}

void SSD1362_UpdateScreen(void) {
  SSD1362_WriteCommand(SSD1362_CMD_SET_COLUMN_ADDR);
  SSD1362_WriteCommand(0x00);
  SSD1362_WriteCommand(SSD1362_BUF_COLS - 1);

  SSD1362_WriteCommand(SSD1362_CMD_SET_ROW_ADDR);
  SSD1362_WriteCommand(0x00);
  SSD1362_WriteCommand(SSD1362_HEIGHT - 1);

  SSD1362_WriteData(SSD1362_Buffer, sizeof(SSD1362_Buffer));
}

void SSD1362_DrawPixel(uint16_t x, uint8_t y, uint8_t gray) {
  if (x >= SSD1362_WIDTH || y >= SSD1362_HEIGHT) {
    return;
  }

  uint16_t index = y * SSD1362_BUF_COLS + x / 2;
  gray &= 0x0F;

  if (x & 1) {
    SSD1362_Buffer[index] = (SSD1362_Buffer[index] & 0x0F) | (gray << 4);
  } else {
    SSD1362_Buffer[index] = (SSD1362_Buffer[index] & 0xF0) | gray;
  }
}

static void SSD1362_DrawChar(uint16_t x, uint8_t y, char c, uint8_t gray) {
  if (c < 32 || c > 122) {
    c = 32; // Replace with space
  }

  for (uint8_t i = 0; i < 5; i++) {
    uint8_t line = SSD1362_Font5x7[c - 32][i];
    for (uint8_t j = 0; j < 8; j++) {
      if (line & (1 << j)) {
        SSD1362_DrawPixel(x + i, y + j, gray);
      }
    }
  }
}

void SSD1362_DrawString(uint16_t x, uint8_t y, const char *str, uint8_t gray) {
  while (*str) {
    SSD1362_DrawChar(x, y, *str, gray);
    x += 6; // 5 pixels wide + 1 pixel space
    if (x + 5 >= SSD1362_WIDTH) {
      x = 0;
      y += 8;
      if (y >= SSD1362_HEIGHT) {
        break;
      }
    }
    str++;
  }
}

void SSD1362_DrawBitmap4(uint16_t x, uint8_t y, const uint8_t *bitmap,
                         uint16_t width, uint8_t height) {
  uint16_t byte_width = (width + 1) / 2;

  for (uint8_t j = 0; j < height; j++) {
    for (uint16_t i = 0; i < width; i++) {
      uint8_t byte = bitmap[j * byte_width + i / 2];
      uint8_t gray = (i & 1) ? (byte >> 4) : (byte & 0x0F);
      SSD1362_DrawPixel(x + i, y + j, gray);
    }
  }
}

void SSD1362_DrawLine(uint16_t x0, uint8_t y0, uint16_t x1, uint8_t y1,
                      uint8_t gray) {
  int16_t dx = x1 > x0 ? x1 - x0 : x0 - x1;
  int16_t dy = y1 > y0 ? y1 - y0 : y0 - y1;
  int16_t sx = x0 < x1 ? 1 : -1;
  int16_t sy = y0 < y1 ? 1 : -1;
  int16_t err = dx - dy;
  int16_t e2;

  while (1) {
    SSD1362_DrawPixel(x0, y0, gray);

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

void SSD1362_DrawWaveform(const int16_t *audioBuffer, uint16_t bufferSize,
                          uint16_t startX, uint16_t width, uint8_t gray) {
  if (audioBuffer == NULL || bufferSize == 0 || width == 0) {
    return;
  }

  uint16_t sampleStep = bufferSize / width;
  if (sampleStep == 0) {
    sampleStep = 1;
  }

  uint8_t prevY = SSD1362_HEIGHT / 2;
  for (uint16_t i = 0; i < width; i++) {
    uint16_t sampleIndex = i * sampleStep;
    if (sampleIndex >= bufferSize) {
      sampleIndex = bufferSize - 1;
    }

    // Map 16-bit audio sample to a Y coordinate centered on the display
    int16_t sample = audioBuffer[sampleIndex];
    int16_t scaledSample =
        (int16_t)((int32_t)sample * (SSD1362_HEIGHT / 2 - 4) / 32768);
    uint8_t y = SSD1362_HEIGHT / 2 - scaledSample;

    if (y >= SSD1362_HEIGHT) {
      y = SSD1362_HEIGHT - 1;
    }

    if (i > 0) {
      SSD1362_DrawLine(startX + i - 1, prevY, startX + i, y, gray);
    }

    prevY = y;
  }
}
