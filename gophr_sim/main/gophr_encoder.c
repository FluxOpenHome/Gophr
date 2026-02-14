#include "gophr_encoder.h"

#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gophr_encoder";

/* M5Dial rotary encoder + button pins */
#define ENCODER_PIN_A   40
#define ENCODER_PIN_B   41
#define BUTTON_PIN      42

/* PCNT limits */
#define PCNT_HIGH_LIMIT  1000
#define PCNT_LOW_LIMIT  -1000

pcnt_unit_handle_t s_pcnt_unit = NULL; /* non-static: accessed by gophr_ui.c */
static lv_indev_t *s_encoder_indev = NULL;
static int s_last_count = 0;

/* LVGL encoder read callback */
static void lvgl_encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    /* Read encoder count */
    int count = 0;
    pcnt_unit_get_count(s_pcnt_unit, &count);

    int diff = count - s_last_count;
    s_last_count = count;

    data->enc_diff = (int16_t)diff;

    /* Read button (active low with pull-up) */
    data->state = gpio_get_level(BUTTON_PIN) == 0
        ? LV_INDEV_STATE_PRESSED
        : LV_INDEV_STATE_RELEASED;
}

esp_err_t gophr_encoder_init(void)
{
    ESP_LOGI(TAG, "Initializing rotary encoder (PCNT) + button");

    /* Configure PCNT unit */
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_pcnt_unit));

    /* Set glitch filter */
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter_cfg));

    /* Configure channel 0 for quadrature decoding */
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = ENCODER_PIN_A,
        .level_gpio_num = ENCODER_PIN_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &chan_a_cfg, &chan_a));

    /* Phase A: count on edges, direction from phase B level */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Configure channel 1 for the other phase */
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = ENCODER_PIN_B,
        .level_gpio_num = ENCODER_PIN_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &chan_b_cfg, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Enable and start PCNT unit */
    ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_unit));

    /* Configure button GPIO42 as input with pull-up */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));

    /* Register LVGL encoder indev */
    s_encoder_indev = lv_indev_create();
    lv_indev_set_type(s_encoder_indev, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(s_encoder_indev, lvgl_encoder_read_cb);

    ESP_LOGI(TAG, "Encoder initialized (A=%d, B=%d, Btn=%d)",
             ENCODER_PIN_A, ENCODER_PIN_B, BUTTON_PIN);
    return ESP_OK;
}

lv_indev_t *gophr_encoder_get_indev(void)
{
    return s_encoder_indev;
}
