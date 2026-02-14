/*
 * Gophr Moisture Probe Simulator
 * M5Dial (ESP32-S3) | ESP-IDF | LVGL | MQTT
 *
 * Simulates a Gophr Zigbee moisture probe using Wi-Fi + MQTT.
 * Two modes: Instant (set and send) or Sweep (ramp over time).
 */

#include "gophr_display.h"
#include "gophr_touch.h"
#include "gophr_encoder.h"
#include "gophr_buzzer.h"
#include "gophr_wifi.h"
#include "gophr_mqtt.h"
#include "gophr_ui.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "lvgl.h"

static const char *TAG = "gophr_main";

/* Hold pin - must be set high to keep M5Dial powered */
#define HOLD_PIN    46

/* Global LVGL mutex (shared with gophr_ui.c) */
SemaphoreHandle_t g_lvgl_mutex = NULL;

/* ---------- LVGL Tick ---------- */

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(5);
}

/* ---------- LVGL Task ---------- */

static void lvgl_task(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(10))) {
            lv_timer_handler();
            xSemaphoreGive(g_lvgl_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ---------- UI Task ---------- */

static void ui_task(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(10))) {
            gophr_ui_run();
            xSemaphoreGive(g_lvgl_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); /* 50 Hz state machine */
    }
}

/* ---------- App Main ---------- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Gophr Moisture Simulator v1.0.0 ===");
    ESP_LOGI(TAG, "M5Dial | ESP32-S3 | Wi-Fi + MQTT");

    /* Set hold pin high to keep power on */
    gpio_config_t hold_cfg = {
        .pin_bit_mask = (1ULL << HOLD_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&hold_cfg);
    gpio_set_level(HOLD_PIN, 1);

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Create LVGL mutex */
    g_lvgl_mutex = xSemaphoreCreateMutex();
    assert(g_lvgl_mutex != NULL);

    /* Initialize hardware */
    ESP_ERROR_CHECK(gophr_display_init());
    ESP_ERROR_CHECK(gophr_touch_init());
    ESP_ERROR_CHECK(gophr_encoder_init());
    ESP_ERROR_CHECK(gophr_buzzer_init());

    /* Set up LVGL tick (5ms periodic timer) */
    const esp_timer_create_args_t tick_timer_args = {
        .callback = lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 5000)); /* 5ms */

    /* Set up LVGL encoder group for navigation */
    lv_group_t *group = lv_group_create();
    lv_group_set_default(group);
    lv_indev_set_group(gophr_encoder_get_indev(), group);

    /* Initialize UI (creates all screens) */
    if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(1000))) {
        ESP_ERROR_CHECK(gophr_ui_init());
        xSemaphoreGive(g_lvgl_mutex);
    }

    /* Start LVGL rendering task */
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 8192, NULL, 5, NULL, 0);

    /* Start UI state machine task */
    xTaskCreatePinnedToCore(ui_task, "ui", 4096, NULL, 4, NULL, 0);

    /* Initialize Wi-Fi subsystem (STA mode ready, does NOT auto-connect) */
    ESP_ERROR_CHECK(gophr_wifi_init());

    /* Initialize MQTT subsystem (does NOT auto-connect, UI handles provisioning) */
    ESP_ERROR_CHECK(gophr_mqtt_init());

    ESP_LOGI(TAG, "All systems initialized - provisioning handled by UI");
}
