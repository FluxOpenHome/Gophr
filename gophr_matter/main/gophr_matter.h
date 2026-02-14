#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Endpoint IDs (assigned by Matter stack) ---------- */
/* These are stored after creation since Matter auto-assigns them */

/* ---------- Device Info ---------- */
#define GOPHR_VENDOR_NAME       "GOPHR"
#define GOPHR_PRODUCT_NAME      "Gophr-C6"

/* ---------- API ---------- */

/* Create the Matter node with all endpoints */
esp_err_t gophr_matter_init(void);

/* Update temperature attribute (Celsius) */
void gophr_matter_update_temperature(float celsius);

/* Update AHT20 humidity attribute (%) */
void gophr_matter_update_humidity(float percent);

/* Update soil moisture attribute (%) for sensor 0-2 */
void gophr_matter_update_moisture(int sensor_index, float percent);

/* Update battery attributes */
void gophr_matter_update_battery(float voltage, float percent);

/* Check if device is commissioned and on the network */
bool gophr_matter_is_connected(void);

#ifdef __cplusplus
}
#endif
