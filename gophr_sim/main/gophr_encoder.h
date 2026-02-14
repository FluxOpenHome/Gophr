#pragma once

#include "esp_err.h"
#include "lvgl.h"

/* Initialize rotary encoder (PCNT) + button (GPIO42) + LVGL encoder indev */
esp_err_t gophr_encoder_init(void);

/* Get the LVGL encoder indev (for assigning to groups) */
lv_indev_t *gophr_encoder_get_indev(void);
