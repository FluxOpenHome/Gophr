#pragma once

#include "esp_err.h"

/* Initialize FT3267 touch controller via I2C + LVGL touch indev */
esp_err_t gophr_touch_init(void);
