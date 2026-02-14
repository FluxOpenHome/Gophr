#pragma once

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdbool.h>

/* ---------- Endpoint IDs ---------- */
#define GOPHR_EP_TEMP           1   /* Temperature + Basic + Power Config */
#define GOPHR_EP_HUMIDITY       2   /* AHT20 Humidity */
#define GOPHR_EP_MOISTURE_1     3   /* Soil Moisture 1 (as humidity %) */
#define GOPHR_EP_MOISTURE_2     4   /* Soil Moisture 2 (as humidity %) */
#define GOPHR_EP_MOISTURE_3     5   /* Soil Moisture 3 (as humidity %) */

/* ---------- Device Info ---------- */
#define GOPHR_MANUFACTURER_NAME "\x05""GOPHR"       /* ZCL string: length-prefixed */
#define GOPHR_MODEL_IDENTIFIER  "\x08""Gophr-C6"

/* ---------- ZED Configuration ---------- */
#define GOPHR_ZED_TIMEOUT       ESP_ZB_ZED_TIMEOUT_64MIN
#define GOPHR_ZED_KEEP_ALIVE    3000  /* ms */

/* ---------- Reporting Intervals ---------- */
#define GOPHR_REPORT_MIN_INTERVAL   1     /* seconds */
#define GOPHR_REPORT_MAX_INTERVAL   0     /* 0 = report on change only */
#define GOPHR_TEMP_REPORT_DELTA     50    /* 0.5°C change triggers report */
#define GOPHR_HUMIDITY_REPORT_DELTA 100   /* 1.0% change triggers report */

/* ---------- Zigbee Channel ---------- */
#define GOPHR_CHANNEL_MASK      ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* ---------- ZED Config Macro ---------- */
#define GOPHR_ZB_ZED_CONFIG()                                   \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                   \
        .install_code_policy = false,                           \
        .nwk_cfg.zed_cfg = {                                    \
            .ed_timeout = GOPHR_ZED_TIMEOUT,                    \
            .keep_alive = GOPHR_ZED_KEEP_ALIVE,                 \
        },                                                      \
    }

/* ---------- API ---------- */

/* Create all Zigbee endpoints and clusters, register the device */
esp_err_t gophr_zigbee_create_device(void);

/* Update temperature attribute (Celsius) */
void gophr_zigbee_update_temperature(float celsius);

/* Update AHT20 humidity attribute (%) */
void gophr_zigbee_update_humidity(float percent);

/* Update soil moisture attribute (%) for sensor 0-2 */
void gophr_zigbee_update_moisture(int sensor_index, float percent);

/* Update battery attributes */
void gophr_zigbee_update_battery(float voltage, float percent);

/* Send report for all attributes */
void gophr_zigbee_report_all(void);

/* Zigbee signal handler (called by stack) */
void gophr_zigbee_signal_handler(esp_zb_app_signal_t *signal_struct);

/* Check if device is joined to a network */
bool gophr_zigbee_is_joined(void);
