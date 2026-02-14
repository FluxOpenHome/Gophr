#pragma once

#include "esp_err.h"
#include "lvgl.h"

/* Initialize GC9A01 display via SPI + LVGL display driver */
esp_err_t gophr_display_init(void);

/* Set backlight brightness (0-100%) */
void gophr_display_set_backlight(uint8_t percent);

/* Get the LVGL display object */
lv_display_t *gophr_display_get(void);
