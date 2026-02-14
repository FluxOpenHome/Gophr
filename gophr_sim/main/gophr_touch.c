#include "gophr_touch.h"

#include "esp_log.h"
#include "esp_lcd_touch_ft5x06.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

static const char *TAG = "gophr_touch";

/* M5Dial FT3267 I2C pin mapping */
#define TOUCH_I2C_SDA   11
#define TOUCH_I2C_SCL   12
#define TOUCH_I2C_ADDR  0x38
#define TOUCH_INT_PIN   14
#define TOUCH_I2C_FREQ  100000  /* M5Dial has no external I2C pull-ups; 400kHz NACKs */

static esp_lcd_touch_handle_t s_touch = NULL;
static lv_indev_t *s_touch_indev = NULL;

/* LVGL touch read callback — suppress I2C error log spam */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t x[1], y[1];
    uint16_t strength[1];
    uint8_t count = 0;

    /* Temporarily suppress I2C error logs during touch polling */
    esp_log_level_t old_level = esp_log_level_get("i2c.master");
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    esp_lcd_touch_read_data(s_touch);

    esp_log_level_set("i2c.master", old_level);

    bool pressed = esp_lcd_touch_get_coordinates(s_touch, x, y, strength, &count, 1);

    if (pressed && count > 0) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

esp_err_t gophr_touch_init(void)
{
    ESP_LOGI(TAG, "Initializing FT3267 touch controller");

    /* Install I2C master bus */
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = TOUCH_I2C_SDA,
        .scl_io_num = TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

    /* FT3267 needs time after power-on before it responds to I2C */
    vTaskDelay(pdMS_TO_TICKS(300));

    /* Create touch panel handle */
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    tp_io_cfg.scl_speed_hz = TOUCH_I2C_FREQ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_cfg, &tp_io_handle));

    esp_lcd_touch_config_t touch_cfg = {
        .x_max = 240,
        .y_max = 240,
        .rst_gpio_num = -1,
        .int_gpio_num = TOUCH_INT_PIN,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &touch_cfg, &s_touch));

    /* Register LVGL touch indev */
    s_touch_indev = lv_indev_create();
    lv_indev_set_type(s_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_touch_indev, lvgl_touch_read_cb);

    ESP_LOGI(TAG, "Touch controller initialized");
    return ESP_OK;
}
