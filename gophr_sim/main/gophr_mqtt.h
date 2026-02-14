#pragma once

#include "esp_err.h"
#include <stdbool.h>

#define MQTT_URI_MAX_LEN    128
#define MQTT_USER_MAX_LEN   64
#define MQTT_PASS_MAX_LEN   64

/* Initialize MQTT client using provided or NVS-saved credentials */
esp_err_t gophr_mqtt_init(void);

/* Start MQTT connection with explicit credentials. Saves to NVS. */
esp_err_t gophr_mqtt_connect(const char *broker_uri, const char *username, const char *password);

/* Try to connect using saved NVS credentials. Returns true if creds exist. */
bool gophr_mqtt_connect_saved(void);

/* Check if MQTT is connected */
bool gophr_mqtt_is_connected(void);

/* Check if saved MQTT credentials exist in NVS */
bool gophr_mqtt_has_saved_creds(void);

/* Publish HA auto-discovery configs for all 3 moisture sensors */
esp_err_t gophr_mqtt_publish_discovery(void);

/* Publish a single moisture sensor value (index 0-2, percent 0-100) */
esp_err_t gophr_mqtt_publish_moisture(int index, int percent);

/* Publish all 3 moisture values at once */
esp_err_t gophr_mqtt_publish_all(int m1, int m2, int m3);
