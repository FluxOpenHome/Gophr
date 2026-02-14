#include "gophr_display.h"

#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_gc9a01.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"

static const char *TAG = "gophr_display";

/* M5Dial GC9A01 pin mapping */
#define LCD_PIN_MOSI    5
#define LCD_PIN_SCLK    6
#define LCD_PIN_CS      7
#define LCD_PIN_DC      4
#define LCD_PIN_RST     8
#define LCD_PIN_BL      9

#define LCD_H_RES       240
#define LCD_V_RES       240
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_SPI_FREQ_HZ (40 * 1000 * 1000) /* 40 MHz */
#define LCD_CMD_BITS    8
#define LCD_PARAM_BITS  8

/* LVGL draw buffer: 20 lines at a time (no PSRAM, must fit in internal RAM) */
#define LVGL_BUF_LINES  20
#define LVGL_BUF_SIZE   (LCD_H_RES * LVGL_BUF_LINES * sizeof(lv_color16_t))

/* Backlight LEDC config */
#define BL_LEDC_TIMER   LEDC_TIMER_0
#define BL_LEDC_CHANNEL LEDC_CHANNEL_0
#define BL_LEDC_FREQ    5000

static lv_display_t *s_display = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;

/* Called by esp_lcd when DMA transfer is complete (ISR context).
 * This is the correct way to signal LVGL that the flush buffer is free. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                     esp_lcd_panel_io_event_data_t *edata,
                                     void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/* Swap bytes in RGB565 pixel data for SPI byte order.
 * SPI sends MSB first; LVGL stores RGB565 as native little-endian
 * but the GC9A01 expects big-endian (MSB first), so we swap each 2-byte pixel. */
static void swap_bytes_rgb565(uint8_t *buf, size_t len_bytes)
{
    uint16_t *px = (uint16_t *)buf;
    size_t count = len_bytes / 2;
    for (size_t i = 0; i < count; i++) {
        px[i] = (px[i] >> 8) | (px[i] << 8);
    }
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    int w = area->x2 - area->x1 + 1;
    int h = area->y2 - area->y1 + 1;
    swap_bytes_rgb565(px_map, w * h * 2);

    esp_lcd_panel_draw_bitmap(s_panel,
        area->x1, area->y1,
        area->x2 + 1, area->y2 + 1,
        px_map);
    /* flush_ready is called by notify_lvgl_flush_ready when DMA completes */
}

esp_err_t gophr_display_init(void)
{
    ESP_LOGI(TAG, "Initializing GC9A01 display");

    /* Initialize SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LVGL_BUF_SIZE,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* Create LCD panel IO (SPI) */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = LCD_SPI_FREQ_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = NULL, /* will be set after display is created */
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_cfg, &io_handle));

    /* Create GC9A01 panel */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_cfg, &s_panel));

    /* Reset and init the panel */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false)); /* Fix horizontal mirroring */
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    /* Initialize backlight via LEDC PWM */
    ledc_timer_config_t bl_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = BL_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = BL_LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&bl_timer));

    ledc_channel_config_t bl_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BL_LEDC_CHANNEL,
        .timer_sel = BL_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LCD_PIN_BL,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&bl_channel));

    /* Turn backlight on to 80% */
    gophr_display_set_backlight(80);

    /* Initialize LVGL */
    lv_init();

    /* Create LVGL display */
    s_display = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_color_format(s_display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(s_display, lvgl_flush_cb);

    /* Now set the user_ctx so the DMA callback can find the display.
     * We need to update the IO handle's user_ctx via the callback struct. */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, s_display);

    /* Allocate draw buffers from internal DMA-capable RAM */
    static uint8_t *buf1 = NULL;
    static uint8_t *buf2 = NULL;
    buf1 = heap_caps_malloc(LVGL_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    buf2 = heap_caps_malloc(LVGL_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!buf1 || !buf2) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers (%u bytes each)", (unsigned)LVGL_BUF_SIZE);
        return ESP_ERR_NO_MEM;
    }
    lv_display_set_buffers(s_display, buf1, buf2, LVGL_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

    ESP_LOGI(TAG, "Display initialized: %dx%d, buf=%u bytes x2", LCD_H_RES, LCD_V_RES, (unsigned)LVGL_BUF_SIZE);
    return ESP_OK;
}

void gophr_display_set_backlight(uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint32_t duty = (uint32_t)(percent * 255 / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
}

lv_display_t *gophr_display_get(void)
{
    return s_display;
}
