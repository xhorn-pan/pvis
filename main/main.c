#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "hal/lcd_types.h"
#include "hal/lv_hal_disp.h"
#include "hal/lv_hal_tick.h"
#include "lvgl.h"
#include "misc/lv_timer.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static const char *TAG = "pvis";

#define LCD_HOST SPI2_HOST

/// lcd pin def
#define PVIS_LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)
#define PVIS_LCD_BL_ON 1
#define PVIS_LCD_BL_OFF !PVIS_LCD_BL_ON
#define PVIS_LCD_PIN_SCLK 6
#define PVIS_LCD_PIN_MOSI 4 // sda
#define PVIS_LCD_PIN_DC 13
#define PVIS_LCD_PIN_CS 10
#define PVIS_LCD_PIN_RST 14
#define PVIS_LCD_PIN_BL 8
// screen pixel number
#define PVIS_LCD_RES_H 240
#define PVIS_LCD_RES_V 240
// bit number represent of command and parameter
#define PVIS_LCD_CMD_BITS 8
#define PVIS_LCD_PARAM_BITS 8
#define PVIS_LVGL_TICK_MS 2

extern void pvis_ui(lv_disp_t *disp);

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx) {
  lv_disp_drv_t *disp_drv = (lv_disp_drv_t *)user_ctx;
  lv_disp_flush_ready(disp_drv);
  return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                          lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_hdl = (esp_lcd_panel_handle_t)drv->user_data;
  int offset_x1 = area->x1;
  int offset_x2 = area->x2;
  int offset_y1 = area->y1;
  int offset_y2 = area->y2;

  // copy a buffer's content to specific area of the display
  esp_lcd_panel_draw_bitmap(panel_hdl, offset_x1, offset_y1, offset_x2 + 1,
                            offset_y2 + 1, color_map);
}

static void increase_lvgl_tick(void *arg) { lv_tick_inc(PVIS_LVGL_TICK_MS); }

void app_main(void) {
  static lv_disp_draw_buf_t disp_buf;
  static lv_disp_drv_t disp_drv;

  // backlight config
  gpio_config_t bl_config = {.mode = GPIO_MODE_OUTPUT,
                             .pin_bit_mask = 1ULL << PVIS_LCD_PIN_BL};
  ESP_ERROR_CHECK(gpio_config(&bl_config));
  // init spi bus
  spi_bus_config_t spi_bus_config = {
      .sclk_io_num = PVIS_LCD_PIN_SCLK,
      .mosi_io_num = PVIS_LCD_PIN_MOSI,
      .miso_io_num = GPIO_NUM_NC,
      .quadwp_io_num = GPIO_NUM_NC,
      .quadhd_io_num = GPIO_NUM_NC,
      .max_transfer_sz = PVIS_LCD_RES_H * 80 * sizeof(uint16_t),
  };
  ESP_ERROR_CHECK(
      spi_bus_initialize(LCD_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));

  // install panel io
  esp_lcd_panel_io_handle_t io_hdl = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = PVIS_LCD_PIN_DC,
      .cs_gpio_num = PVIS_LCD_PIN_CS,
      .pclk_hz = PVIS_LCD_PIXEL_CLOCK_HZ,
      .lcd_cmd_bits = PVIS_LCD_CMD_BITS,
      .lcd_param_bits = PVIS_LCD_PARAM_BITS,
      .spi_mode = 0,
      .trans_queue_depth = 10,
      .on_color_trans_done = notify_lvgl_flush_ready,
      .user_ctx = &disp_drv,
  };

  // attach lcd to spi bus
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &io_hdl));

  esp_lcd_panel_handle_t panel_hdl = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PVIS_LCD_PIN_RST,
      .color_space = ESP_LCD_COLOR_SPACE_RGB,
      .bits_per_pixel = 16,
  };

  ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_hdl, &panel_config, &panel_hdl));

  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_hdl));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_hdl));

  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_hdl, true));

  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_hdl, true, false));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_hdl, true));

  // turn on backlight
  gpio_set_level(PVIS_LCD_PIN_BL, PVIS_LCD_BL_ON);

  lv_init();

  lv_color_t *buf1 = heap_caps_malloc(PVIS_LCD_RES_H * 20 * sizeof(lv_color_t),
                                      MALLOC_CAP_DMA);
  assert(buf1);
  lv_color_t *buf2 = heap_caps_malloc(PVIS_LCD_RES_H * 20 * sizeof(lv_color_t),
                                      MALLOC_CAP_DMA);
  assert(buf2);
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, PVIS_LCD_RES_H * 20);

  // register display drv to lvgl
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = PVIS_LCD_RES_H;
  disp_drv.ver_res = PVIS_LCD_RES_V;
  disp_drv.flush_cb = lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_hdl;

  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  const esp_timer_create_args_t tick_timer_args = {
      .callback = &increase_lvgl_tick, .name = "pvis_tick"};
  esp_timer_handle_t tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(tick_timer, PVIS_LVGL_TICK_MS * 1000));

  pvis_ui(disp);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
    lv_timer_handler();
  }
}
