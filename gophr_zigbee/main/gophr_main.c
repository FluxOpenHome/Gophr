/*
 * Gophr - Solar-Powered Zigbee Moisture Sensor
 * ESP32-C6 / ESP-IDF / Zigbee 3.0 End Device
 *
 * Converted from ESPHome YAML (gophr.yaml) to native ESP-IDF.
 * Uses same GPIO pin mapping as original ESP32-C3 PCB design.
 */

#include "gophr_drivers.h"
#include "gophr_sensors.h"
#include "gophr_zigbee.h"
#include "gophr_sleep.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"

static const char *TAG = "gophr_main";

/* Track boot phase for LED feedback */
static bool s_boot_complete = false;

/* ---------- Zigbee App Signal Handler (required by stack) ---------- */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    gophr_zigbee_signal_handler(signal_struct);
}

/* ---------- Sensor Reading Task ---------- */

static void sensor_task(void *pvParameters)
{
    /* Boot sequence: match original ESPHome behavior */
    ESP_LOGI(TAG, "Boot sequence: powering on sensors...");
    gophr_sensor_power(true);
    vTaskDelay(pdMS_TO_TICKS(30000)); /* Wait 30s for sensors to stabilize */

    gophr_aht20_power(true);
    gophr_led_power(true);
    vTaskDelay(pdMS_TO_TICKS(15000)); /* Wait 15s */

    /* Init LED and set blue pulse (boot indicator) */
    gophr_led_init();
    gophr_led_set_color(0, 0, 128); /* Blue ~50% */

    /* Init I2C for AHT20 (after power enabled) */
    gophr_i2c_init();

    /* Wait for moisture sensors to read > 0.9V (30s timeout) */
    ESP_LOGI(TAG, "Waiting for moisture sensors to stabilize...");
    uint32_t wait_start = xTaskGetTickCount();
    bool sensors_ready = false;

    while (!sensors_ready && ((xTaskGetTickCount() - wait_start) * portTICK_PERIOD_MS < 30000)) {
        gophr_sensors_read_moisture();
        const sensor_readings_t *r = gophr_sensors_get_readings();

        sensors_ready = true;
        for (int i = 0; i < MOISTURE_SENSOR_COUNT; i++) {
            if (r->moisture_voltage[i] < 0.9f) {
                sensors_ready = false;
                break;
            }
        }

        if (!sensors_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (sensors_ready) {
        ESP_LOGI(TAG, "All moisture sensors ready");
    } else {
        ESP_LOGW(TAG, "Moisture sensor timeout - continuing anyway");
    }

    s_boot_complete = true;
    ESP_LOGI(TAG, "Boot complete");

    /* ---------- Main sensor loop ---------- */
    uint32_t loop_count = 0;
    const uint32_t MOISTURE_INTERVAL_MS = 5000;   /* Read moisture every 5s */
    const uint32_t AHT20_INTERVAL_LOOPS = 12;     /* Read AHT20 every 60s (12 * 5s) */
    const uint32_t POWER_INTERVAL_LOOPS = 6;      /* Read power every 30s (6 * 5s) */
    const uint32_t SLEEP_CHECK_LOOPS = 6;         /* Check sleep every 30s */

    while (1) {
        /* Read moisture sensors every loop (5s) */
        gophr_sensors_read_moisture();
        const sensor_readings_t *readings = gophr_sensors_get_readings();

        /* Update Zigbee moisture attributes */
        for (int i = 0; i < MOISTURE_SENSOR_COUNT; i++) {
            gophr_zigbee_update_moisture(i, readings->moisture_percent[i]);
        }

        /* Read AHT20 every 60s */
        if (loop_count % AHT20_INTERVAL_LOOPS == 0) {
            if (gophr_sensors_read_aht20() == ESP_OK) {
                gophr_zigbee_update_temperature(readings->temperature);
                gophr_zigbee_update_humidity(readings->humidity);
            }
        }

        /* Read power every 30s */
        if (loop_count % POWER_INTERVAL_LOOPS == 0) {
            gophr_sensors_read_power();
            gophr_zigbee_update_battery(readings->battery_voltage, readings->battery_percent);
        }

        /* Check sleep every 30s */
        if (loop_count % SLEEP_CHECK_LOOPS == 0) {
            gophr_sleep_check();
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(MOISTURE_INTERVAL_MS));
    }
}

/* ---------- Zigbee Task ---------- */

static void zigbee_task(void *pvParameters)
{
    /* Initialize Zigbee stack as End Device */
    esp_zb_cfg_t zb_nwk_cfg = GOPHR_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create all endpoints and register device */
    gophr_zigbee_create_device();

    /* Set channel mask */
    esp_zb_set_primary_network_channel_set(GOPHR_CHANNEL_MASK);

    /* Start Zigbee stack */
    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Enter Zigbee main loop (does not return) */
    esp_zb_stack_main_loop();
}

/* ---------- App Main ---------- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Gophr Zigbee Sensor v1.0.0 ===");
    ESP_LOGI(TAG, "ESP32-C6 | Zigbee End Device");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize platform config for Zigbee radio */
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Initialize hardware */
    ESP_ERROR_CHECK(gophr_gpio_init());
    ESP_ERROR_CHECK(gophr_adc_init());

    /* Initialize sensor subsystem (loads calibration from NVS) */
    ESP_ERROR_CHECK(gophr_sensors_init());

    /* Initialize sleep subsystem (loads config from NVS) */
    ESP_ERROR_CHECK(gophr_sleep_init());

    /* Start Zigbee task (high priority, runs the stack main loop) */
    xTaskCreate(zigbee_task, "zigbee_main", 4096, NULL, 5, NULL);

    /* Start sensor reading task */
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "All tasks started");
}
