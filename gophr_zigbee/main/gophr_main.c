#include "gophr_zigbee.h"
#include "gophr_drivers.h"
#include "gophr_sensors.h"
#include "gophr_sleep.h"

#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "platform/esp_zigbee_platform.h"

static const char *TAG = "gophr_main";

/* Forward declaration */
static void sensor_task(void *arg);

/* ---------- Zigbee Signal Handler (called by stack) ---------- */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    gophr_zigbee_signal_handler(signal_struct);
}

/* ---------- Zigbee Task ---------- */

static void zigbee_task(void *arg)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = GOPHR_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create all endpoints and clusters */
    ESP_ERROR_CHECK(gophr_zigbee_create_device());

    /* Set channel mask */
    esp_zb_set_primary_network_channel_set(GOPHR_CHANNEL_MASK);

    ESP_LOGI(TAG, "Zigbee stack initialized, starting...");
    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Run the Zigbee main loop (this blocks forever) */
    esp_zb_stack_main_loop();
}

/* ---------- Boot Sequence ---------- */

static void boot_sequence(void)
{
    ESP_LOGI(TAG, "=== GOPHR Boot Sequence ===");

    /* Step 1: Initialize GPIO power controls */
    ESP_ERROR_CHECK(gophr_gpio_init());

    /* Step 2: Power on sensor rail and wait for stabilization */
    ESP_LOGI(TAG, "Powering on sensor rail (GPIO6)...");
    gophr_sensor_power(true);
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Step 3: Initialize ADC */
    ESP_ERROR_CHECK(gophr_adc_init());

    /* Step 4: Power on AHT20 and LED */
    ESP_LOGI(TAG, "Powering on AHT20 (GPIO7) and LED (GPIO18)...");
    gophr_aht20_power(true);
    gophr_led_power(true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Step 5: Initialize I2C / AHT20 */
    ESP_ERROR_CHECK(gophr_i2c_init());

    /* Step 6: Initialize WS2812B LED */
    ESP_ERROR_CHECK(gophr_led_init());

    /* Step 7: Blue pulse to indicate boot */
    gophr_led_set_color(0, 0, 76); /* Blue = booting */
    ESP_LOGI(TAG, "LED: Blue (booting)");

    /* Step 8: Initialize sensor subsystem (loads calibration from NVS) */
    ESP_ERROR_CHECK(gophr_sensors_init());

    /* Step 9: Initialize sleep subsystem */
    ESP_ERROR_CHECK(gophr_sleep_init());

    ESP_LOGI(TAG, "=== Boot Sequence Complete ===");
}

/* ---------- Sensor Task ---------- */

static void sensor_task(void *arg)
{
    int cycle = 0;

    /* Wait for Zigbee to initialize before reading sensors */
    vTaskDelay(pdMS_TO_TICKS(10000));

    ESP_LOGI(TAG, "Sensor task started");

    while (1) {
        /* Read moisture sensors every cycle (5 seconds) */
        gophr_sensors_read_moisture();

        /* Read power every cycle */
        gophr_sensors_read_power();

        /* Read AHT20 every 12th cycle (~60 seconds) */
        if (cycle % 12 == 0) {
            gophr_sensors_read_aht20();
        }

        /* Update Zigbee attributes if joined */
        if (gophr_zigbee_is_joined()) {
            const sensor_readings_t *r = gophr_sensors_get_readings();

            gophr_zigbee_update_temperature(r->temperature);
            gophr_zigbee_update_humidity(r->humidity);

            for (int i = 0; i < MOISTURE_SENSOR_COUNT; i++) {
                gophr_zigbee_update_moisture(i, r->moisture_percent[i]);
            }

            gophr_zigbee_update_battery(r->battery_voltage, r->battery_percent);

            /* Force report on first reading after join, then every 12 cycles */
            if (cycle == 0 || cycle % 12 == 0) {
                gophr_zigbee_report_all();
            }

            ESP_LOGI(TAG, "Sensors: temp=%.1fC, hum=%.1f%%, bat=%.2fV(%.0f%%), "
                     "m1=%.0f%%, m2=%.0f%%, m3=%.0f%%",
                     r->temperature, r->humidity,
                     r->battery_voltage, r->battery_percent,
                     r->moisture_percent[0], r->moisture_percent[1], r->moisture_percent[2]);
        }

        /* Check if we should go to sleep (every 6th cycle = ~30 seconds) */
        if (cycle % 6 == 0) {
            gophr_sleep_check();
        }

        cycle++;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ---------- Entry Point ---------- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Gophr Zigbee Sensor v1.0 ===");

    /* Initialize NVS (required for Zigbee and calibration storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash erase and re-init");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    /* Run hardware boot sequence */
    boot_sequence();

    /* Configure Zigbee platform (native radio, no host connection) */
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee task (runs on its own stack) */
    xTaskCreate(zigbee_task, "zigbee_task", 4096, NULL, 5, NULL);

    /* Start sensor reading task */
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "All tasks started");
}
