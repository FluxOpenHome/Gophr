#include "gophr_sensors.h"
#include "gophr_drivers.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

static const char *TAG = "gophr_sensors";

static moisture_cal_t s_calibration[MOISTURE_SENSOR_COUNT];
static sensor_readings_t s_readings;

/* Median filter buffers */
static float s_moisture_buf[MOISTURE_SENSOR_COUNT][MEDIAN_FILTER_WINDOW];
static int s_moisture_buf_idx[MOISTURE_SENSOR_COUNT];
static int s_moisture_buf_count[MOISTURE_SENSOR_COUNT];

/* GPIO pin for each moisture sensor */
static const int s_moisture_gpio[MOISTURE_SENSOR_COUNT] = {
    GPIO_MOISTURE_1, GPIO_MOISTURE_2, GPIO_MOISTURE_3
};

/* ---------- Median Filter ---------- */

static int float_compare(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

static float median_filter(float *buf, int count)
{
    if (count == 0) return NAN;

    float sorted[MEDIAN_FILTER_WINDOW];
    memcpy(sorted, buf, count * sizeof(float));
    qsort(sorted, count, sizeof(float), float_compare);

    return sorted[count / 2];
}

/* ---------- Calibration NVS ---------- */

esp_err_t gophr_sensors_load_calibration(void)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("gophr_cal", NVS_READONLY, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No calibration in NVS, using factory defaults");
        gophr_sensors_factory_reset_calibration();
        return ESP_OK;
    }

    for (int i = 0; i < MOISTURE_SENSOR_COUNT; i++) {
        char key[16];

        snprintf(key, sizeof(key), "s%d_dry", i);
        size_t sz = sizeof(float);
        if (nvs_get_blob(nvs, key, &s_calibration[i].dry_value, &sz) != ESP_OK) {
            ESP_LOGW(TAG, "Missing cal key %s", key);
        }

        snprintf(key, sizeof(key), "s%d_wet", i);
        sz = sizeof(float);
        if (nvs_get_blob(nvs, key, &s_calibration[i].wet_value, &sz) != ESP_OK) {
            ESP_LOGW(TAG, "Missing cal key %s", key);
        }

        snprintf(key, sizeof(key), "s%d_dts", i);
        sz = sizeof(s_calibration[i].dry_timestamp);
        if (nvs_get_str(nvs, key, s_calibration[i].dry_timestamp, &sz) != ESP_OK) {
            strcpy(s_calibration[i].dry_timestamp, "Factory");
        }

        snprintf(key, sizeof(key), "s%d_wts", i);
        sz = sizeof(s_calibration[i].wet_timestamp);
        if (nvs_get_str(nvs, key, s_calibration[i].wet_timestamp, &sz) != ESP_OK) {
            strcpy(s_calibration[i].wet_timestamp, "Factory");
        }

        ESP_LOGI(TAG, "Sensor %d cal: dry=%.3fV (%s), wet=%.3fV (%s)",
                 i + 1, s_calibration[i].dry_value, s_calibration[i].dry_timestamp,
                 s_calibration[i].wet_value, s_calibration[i].wet_timestamp);
    }

    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t gophr_sensors_save_calibration(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open("gophr_cal", NVS_READWRITE, &nvs), TAG, "NVS open failed");

    for (int i = 0; i < MOISTURE_SENSOR_COUNT; i++) {
        char key[16];

        snprintf(key, sizeof(key), "s%d_dry", i);
        nvs_set_blob(nvs, key, &s_calibration[i].dry_value, sizeof(float));

        snprintf(key, sizeof(key), "s%d_wet", i);
        nvs_set_blob(nvs, key, &s_calibration[i].wet_value, sizeof(float));

        snprintf(key, sizeof(key), "s%d_dts", i);
        nvs_set_str(nvs, key, s_calibration[i].dry_timestamp);

        snprintf(key, sizeof(key), "s%d_wts", i);
        nvs_set_str(nvs, key, s_calibration[i].wet_timestamp);
    }

    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Calibration saved to NVS");
    return ESP_OK;
}

/* ---------- Init ---------- */

esp_err_t gophr_sensors_init(void)
{
    memset(&s_readings, 0, sizeof(s_readings));
    memset(s_moisture_buf, 0, sizeof(s_moisture_buf));
    memset(s_moisture_buf_idx, 0, sizeof(s_moisture_buf_idx));
    memset(s_moisture_buf_count, 0, sizeof(s_moisture_buf_count));

    /* Load calibration from NVS */
    gophr_sensors_load_calibration();

    ESP_LOGI(TAG, "Sensor subsystem initialized");
    return ESP_OK;
}

/* ---------- Moisture Reading ---------- */

esp_err_t gophr_sensors_read_moisture(void)
{
    for (int i = 0; i < MOISTURE_SENSOR_COUNT; i++) {
        float voltage = gophr_adc_read_voltage(s_moisture_gpio[i]);

        /* Drop NaN or out-of-range readings */
        if (isnan(voltage) || voltage < 0.0f || voltage > 3.3f) {
            ESP_LOGW(TAG, "Moisture %d: invalid reading %.3fV, dropped", i + 1, voltage);
            continue;
        }

        /* Add to median filter buffer */
        int idx = s_moisture_buf_idx[i];
        s_moisture_buf[i][idx] = voltage;
        s_moisture_buf_idx[i] = (idx + 1) % MEDIAN_FILTER_WINDOW;
        if (s_moisture_buf_count[i] < MEDIAN_FILTER_WINDOW) {
            s_moisture_buf_count[i]++;
        }

        /* Apply median filter */
        float filtered = median_filter(s_moisture_buf[i], s_moisture_buf_count[i]);
        s_readings.moisture_voltage[i] = filtered;

        /* Calculate moisture percentage */
        float dry = s_calibration[i].dry_value;
        float wet = s_calibration[i].wet_value;

        if (fabsf(dry - wet) < 0.001f) {
            s_readings.moisture_percent[i] = 0.0f;
            continue;
        }

        float pct = ((dry - filtered) / (dry - wet)) * 100.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;

        /* Round to nearest 5% */
        pct = roundf(pct / 5.0f) * 5.0f;
        s_readings.moisture_percent[i] = pct;

        ESP_LOGD(TAG, "Moisture %d: raw=%.3fV, filtered=%.3fV, pct=%.0f%%",
                 i + 1, voltage, filtered, pct);
    }

    return ESP_OK;
}

/* ---------- Power Reading ---------- */

esp_err_t gophr_sensors_read_power(void)
{
    /* Battery voltage (with voltage divider correction) */
    float bat_raw = gophr_adc_read_voltage(GPIO_BATTERY_VOLTAGE);
    if (!isnan(bat_raw)) {
        s_readings.battery_voltage = bat_raw * BATTERY_DIVIDER_RATIO;

        /* Calculate battery percentage */
        float pct = ((s_readings.battery_voltage - BATTERY_VOLTAGE_MIN) /
                     (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) * 100.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        s_readings.battery_percent = pct;
    }

    /* Solar voltage (with voltage divider correction) */
    float sol_raw = gophr_adc_read_voltage(GPIO_SOLAR_VOLTAGE);
    if (!isnan(sol_raw)) {
        s_readings.solar_voltage = sol_raw * BATTERY_DIVIDER_RATIO;
        s_readings.solar_charging = s_readings.solar_voltage > SOLAR_CHARGING_THRESHOLD;
    }

    ESP_LOGD(TAG, "Battery: %.2fV (%.0f%%), Solar: %.2fV (%s)",
             s_readings.battery_voltage, s_readings.battery_percent,
             s_readings.solar_voltage, s_readings.solar_charging ? "charging" : "not charging");
    return ESP_OK;
}

/* ---------- AHT20 Reading ---------- */

esp_err_t gophr_sensors_read_aht20(void)
{
    float temp, hum;
    esp_err_t ret = gophr_aht20_read(&temp, &hum);
    if (ret == ESP_OK) {
        s_readings.temperature = temp;
        s_readings.humidity = hum;
        ESP_LOGD(TAG, "AHT20: temp=%.1f°C, humidity=%.1f%%", temp, hum);
    }
    return ret;
}

/* ---------- Getters ---------- */

const sensor_readings_t *gophr_sensors_get_readings(void)
{
    return &s_readings;
}

const moisture_cal_t *gophr_sensors_get_calibration(int sensor_index)
{
    if (sensor_index < 0 || sensor_index >= MOISTURE_SENSOR_COUNT) return NULL;
    return &s_calibration[sensor_index];
}

/* ---------- Calibration ---------- */

esp_err_t gophr_sensors_calibrate_dry(int sensor_index)
{
    if (sensor_index < 0 || sensor_index >= MOISTURE_SENSOR_COUNT) return ESP_ERR_INVALID_ARG;

    s_calibration[sensor_index].dry_value = s_readings.moisture_voltage[sensor_index];
    snprintf(s_calibration[sensor_index].dry_timestamp, sizeof(s_calibration[sensor_index].dry_timestamp),
             "Calibrated");

    ESP_LOGI(TAG, "Sensor %d dry calibrated: %.3fV", sensor_index + 1,
             s_calibration[sensor_index].dry_value);

    return gophr_sensors_save_calibration();
}

esp_err_t gophr_sensors_calibrate_wet(int sensor_index)
{
    if (sensor_index < 0 || sensor_index >= MOISTURE_SENSOR_COUNT) return ESP_ERR_INVALID_ARG;

    s_calibration[sensor_index].wet_value = s_readings.moisture_voltage[sensor_index];
    snprintf(s_calibration[sensor_index].wet_timestamp, sizeof(s_calibration[sensor_index].wet_timestamp),
             "Calibrated");

    ESP_LOGI(TAG, "Sensor %d wet calibrated: %.3fV", sensor_index + 1,
             s_calibration[sensor_index].wet_value);

    return gophr_sensors_save_calibration();
}

esp_err_t gophr_sensors_factory_reset_calibration(void)
{
    const float defaults[MOISTURE_SENSOR_COUNT][2] = {
        {FACTORY_S1_DRY, FACTORY_S1_WET},
        {FACTORY_S2_DRY, FACTORY_S2_WET},
        {FACTORY_S3_DRY, FACTORY_S3_WET},
    };

    for (int i = 0; i < MOISTURE_SENSOR_COUNT; i++) {
        s_calibration[i].dry_value = defaults[i][0];
        s_calibration[i].wet_value = defaults[i][1];
        strcpy(s_calibration[i].dry_timestamp, "Factory");
        strcpy(s_calibration[i].wet_timestamp, "Factory");
    }

    ESP_LOGI(TAG, "Calibration reset to factory defaults");
    return gophr_sensors_save_calibration();
}
