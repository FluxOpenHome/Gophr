#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* Number of moisture sensor channels */
#define MOISTURE_SENSOR_COUNT   3

/* Median filter window size */
#define MEDIAN_FILTER_WINDOW    5

/* Battery voltage range for percentage calculation */
#define BATTERY_VOLTAGE_MIN     3.0f   /* 0% */
#define BATTERY_VOLTAGE_MAX     4.2f   /* 100% */
#define BATTERY_DIVIDER_RATIO   2.5f   /* Voltage divider multiplier */

/* Solar charging threshold */
#define SOLAR_CHARGING_THRESHOLD 1.0f  /* Volts (after divider correction) */

/* Factory default calibration values */
#define FACTORY_S1_DRY          1.979f
#define FACTORY_S1_WET          1.388f
#define FACTORY_S2_DRY          1.979f
#define FACTORY_S2_WET          1.388f
#define FACTORY_S3_DRY          2.046f
#define FACTORY_S3_WET          1.391f

/* Calibration data for one moisture sensor */
typedef struct {
    float dry_value;
    float wet_value;
    char dry_timestamp[32];
    char wet_timestamp[32];
} moisture_cal_t;

/* All sensor readings */
typedef struct {
    float moisture_voltage[MOISTURE_SENSOR_COUNT];
    float moisture_percent[MOISTURE_SENSOR_COUNT];
    float battery_voltage;
    float battery_percent;
    float solar_voltage;
    bool solar_charging;
    float temperature;
    float humidity;
} sensor_readings_t;

/* Initialize sensor subsystem, load calibration from NVS */
esp_err_t gophr_sensors_init(void);

/* Read all moisture sensors (applies median filter) */
esp_err_t gophr_sensors_read_moisture(void);

/* Read battery and solar voltages */
esp_err_t gophr_sensors_read_power(void);

/* Read AHT20 temperature and humidity */
esp_err_t gophr_sensors_read_aht20(void);

/* Get current sensor readings */
const sensor_readings_t *gophr_sensors_get_readings(void);

/* Calibration */
esp_err_t gophr_sensors_calibrate_dry(int sensor_index);
esp_err_t gophr_sensors_calibrate_wet(int sensor_index);
esp_err_t gophr_sensors_factory_reset_calibration(void);
const moisture_cal_t *gophr_sensors_get_calibration(int sensor_index);

/* Save/load calibration to NVS */
esp_err_t gophr_sensors_save_calibration(void);
esp_err_t gophr_sensors_load_calibration(void);
