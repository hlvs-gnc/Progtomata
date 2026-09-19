/**
 * @file gif_demo.c
 *
 * @brief Loops a pre-converted GIF animation on the SSD1362 display.
 *
 * @details Frame data is generated offline from a source GIF by
 * wizardry/gif_to_ssd1362.py into gif_demo_data.h (already packed into the
 * 4bpp GDDRAM layout expected by SSD1362_DrawBitmap4()).
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * @author Radar2000
 */

#include <FreeRTOS.h>
#include <gif_demo_data.h>
#include <ssd1362.h>
#include <task.h>

void vGifDemoTask(void *pvParameters) {
  (void)pvParameters;

  vTaskDelay(pdMS_TO_TICKS(100));

  while (1) {
    for (uint16_t i = 0; i < GIFDEMO_FRAME_COUNT; i++) {
      SSD1362_Clear();
      SSD1362_DrawBitmap4(0, 0, GifDemo_Frames[i], GIFDEMO_WIDTH, GIFDEMO_HEIGHT);
      SSD1362_UpdateScreen();

      vTaskDelay(pdMS_TO_TICKS(GifDemo_FrameDelayMs[i]));
    }
  }

  vTaskDelete(NULL);
}
