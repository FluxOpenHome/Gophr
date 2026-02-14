#pragma once

#include "esp_err.h"
#include "lvgl.h"
#include <stdbool.h>

/* UI states */
typedef enum {
    UI_STATE_SPLASH,
    /* Provisioning */
    UI_STATE_WIFI_SCANNING,
    UI_STATE_WIFI_SELECT,
    UI_STATE_WIFI_PASSWORD,
    UI_STATE_WIFI_CONNECTING,
    UI_STATE_MQTT_SETUP,
    UI_STATE_MQTT_CONNECTING,
    /* Main */
    UI_STATE_MODE_SELECT,
    /* Instant mode */
    UI_STATE_SENSOR_0,
    UI_STATE_SENSOR_1,
    UI_STATE_SENSOR_2,
    UI_STATE_SUMMARY,
    UI_STATE_SENDING,
    UI_STATE_CONFIRMATION,
    /* Sweep mode */
    UI_STATE_SWEEP_SENSOR_0,
    UI_STATE_SWEEP_SENSOR_1,
    UI_STATE_SWEEP_SENSOR_2,
    UI_STATE_SWEEP_SUMMARY,
    UI_STATE_SWEEP_RUNNING,
    UI_STATE_SWEEP_DONE,
} ui_state_t;

/* Initialize all UI screens and styles */
esp_err_t gophr_ui_init(void);

/* Run the UI state machine (called from ui_task) */
void gophr_ui_run(void);

/* Update connection status indicators */
void gophr_ui_set_wifi_status(bool connected);
void gophr_ui_set_mqtt_status(bool connected);
