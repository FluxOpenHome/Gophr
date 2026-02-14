#include "gophr_sleep.h"
#include "gophr_drivers.h"
#include "gophr_matter.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gophr_sleep";

static int s_sleep_duration_min;
static int s_min_awake_min;
static int s_max_awake_min;
static bool s_sleep_disabled;
static uint32_t s_awake_start_ms;
static bool s_sleep_sequence_active = false;

/* ---------- NVS Persistence ---------- */

static void load_config(void)
{
    nvs_handle_t nvs;
    if (nvs_open("gophr_sleep", NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGW(TAG, "No sleep config in NVS, using defaults");
        s_sleep_duration_min = GOPHR_DEFAULT_SLEEP_DURATION_MIN;
        s_min_awake_min = GOPHR_DEFAULT_MIN_AWAKE_MIN;
        s_max_awake_min = GOPHR_DEFAULT_MAX_AWAKE_MIN;
        s_sleep_disabled = GOPHR_DEFAULT_SLEEP_DISABLED;
        return;
    }

    int32_t val;
    uint8_t bval;

    if (nvs_get_i32(nvs, "duration", &val) == ESP_OK) s_sleep_duration_min = val;
    else s_sleep_duration_min = GOPHR_DEFAULT_SLEEP_DURATION_MIN;

    if (nvs_get_i32(nvs, "min_awake", &val) == ESP_OK) s_min_awake_min = val;
    else s_min_awake_min = GOPHR_DEFAULT_MIN_AWAKE_MIN;

    if (nvs_get_i32(nvs, "max_awake", &val) == ESP_OK) s_max_awake_min = val;
    else s_max_awake_min = GOPHR_DEFAULT_MAX_AWAKE_MIN;

    if (nvs_get_u8(nvs, "disabled", &bval) == ESP_OK) s_sleep_disabled = (bval != 0);
    else s_sleep_disabled = GOPHR_DEFAULT_SLEEP_DISABLED;

    nvs_close(nvs);
    ESP_LOGI(TAG, "Sleep config: duration=%dmin, min_awake=%dmin, max_awake=%dmin, disabled=%d",
             s_sleep_duration_min, s_min_awake_min, s_max_awake_min, s_sleep_disabled);
}

static void save_config(void)
{
    nvs_handle_t nvs;
    if (nvs_open("gophr_sleep", NVS_READWRITE, &nvs) != ESP_OK) return;

    nvs_set_i32(nvs, "duration", s_sleep_duration_min);
    nvs_set_i32(nvs, "min_awake", s_min_awake_min);
    nvs_set_i32(nvs, "max_awake", s_max_awake_min);
    nvs_set_u8(nvs, "disabled", s_sleep_disabled ? 1 : 0);
    nvs_commit(nvs);
    nvs_close(nvs);
}

/* ---------- Init ---------- */

esp_err_t gophr_sleep_init(void)
{
    s_awake_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_sleep_sequence_active = false;
    load_config();
    return ESP_OK;
}

/* ---------- Sleep Execution ---------- */

static void enter_deep_sleep(void)
{
    uint64_t sleep_us = (uint64_t)s_sleep_duration_min * 60ULL * 1000000ULL;
    ESP_LOGI(TAG, "Entering deep sleep for %d minutes", s_sleep_duration_min);

    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
    /* Device resets on wake - code below never executes */
}

static void sleep_sequence(void)
{
    s_sleep_sequence_active = true;
    ESP_LOGI(TAG, "Sleep sequence started");

    /* Power down sensors */
    gophr_sensor_power(false);
    gophr_aht20_power(false);

    /* Turn off LED */
    gophr_led_off();
    gophr_led_power(false);

    /* Wait 5 seconds for Matter attribute reports to propagate */
    ESP_LOGI(TAG, "Waiting 5s for attribute propagation...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Final check: only sleep if still on the network */
    if (!gophr_matter_is_connected()) {
        ESP_LOGW(TAG, "Lost network during sleep sequence, aborting");
        s_sleep_sequence_active = false;
        gophr_sensor_power(true);
        gophr_aht20_power(true);
        gophr_led_power(true);
        gophr_led_set_color(0, 76, 0); /* Green */
        return;
    }

    enter_deep_sleep();
}

/* ---------- Periodic Check ---------- */

void gophr_sleep_check(void)
{
    if (s_sleep_sequence_active) return;
    if (s_sleep_disabled) return;
    if (!gophr_matter_is_connected()) return;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t awake_ms = now_ms - s_awake_start_ms;
    uint32_t max_awake_ms = (uint32_t)s_max_awake_min * 60U * 1000U;
    uint32_t min_awake_ms = (uint32_t)s_min_awake_min * 60U * 1000U;

    bool should_sleep = false;

    if (awake_ms > max_awake_ms) {
        ESP_LOGI(TAG, "Max awake time exceeded (%d min) - forcing sleep", s_max_awake_min);
        should_sleep = true;
    } else if (awake_ms > min_awake_ms) {
        ESP_LOGI(TAG, "Min awake reached (%d min) - going to sleep", s_min_awake_min);
        should_sleep = true;
    }

    if (should_sleep) {
        sleep_sequence();
    }
}

void gophr_sleep_now(void)
{
    if (!gophr_matter_is_connected()) {
        ESP_LOGW(TAG, "Refusing manual sleep: not on network");
        return;
    }
    s_sleep_disabled = false;
    sleep_sequence();
}

/* ---------- Getters/Setters ---------- */

int gophr_sleep_get_duration(void) { return s_sleep_duration_min; }

void gophr_sleep_set_duration(int minutes)
{
    if (minutes < 1 || minutes > 1440) return;
    s_sleep_duration_min = minutes;
    save_config();
    ESP_LOGI(TAG, "Sleep duration set to %d min", minutes);
}

int gophr_sleep_get_min_awake(void) { return s_min_awake_min; }

void gophr_sleep_set_min_awake(int minutes)
{
    if (minutes < 0 || minutes > 15) return;
    s_min_awake_min = minutes;
    save_config();
    ESP_LOGI(TAG, "Min awake set to %d min", minutes);
}

int gophr_sleep_get_max_awake(void) { return s_max_awake_min; }

void gophr_sleep_set_max_awake(int minutes)
{
    if (minutes < 10 || minutes > 1440) return;
    s_max_awake_min = minutes;
    save_config();
    ESP_LOGI(TAG, "Max awake set to %d min", minutes);
}

bool gophr_sleep_is_disabled(void) { return s_sleep_disabled; }

void gophr_sleep_set_disabled(bool disabled)
{
    s_sleep_disabled = disabled;
    if (disabled) {
        s_awake_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    save_config();
    ESP_LOGI(TAG, "Sleep %s", disabled ? "disabled" : "enabled");
}

uint32_t gophr_sleep_get_awake_seconds(void)
{
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (now_ms - s_awake_start_ms) / 1000;
}

bool gophr_sleep_sequence_active(void)
{
    return s_sleep_sequence_active;
}
