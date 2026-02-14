#pragma once

#include "lvgl.h"

/* Color constants */
#define GOPHR_COLOR_BG          lv_color_hex(0x0D0D0D)
#define GOPHR_COLOR_TEXT        lv_color_hex(0xFFFFFF)
#define GOPHR_COLOR_TEXT_DIM    lv_color_hex(0x888888)
#define GOPHR_COLOR_TEXT_HINT   lv_color_hex(0x555555)
#define GOPHR_COLOR_ACCENT      lv_color_hex(0x00E676)
#define GOPHR_COLOR_BRAND_GREEN lv_color_hex(0x6DAC39)  /* From gophr.svg logo */
#define GOPHR_COLOR_ARC_TRACK   lv_color_hex(0x1A1A1A)
#define GOPHR_COLOR_DOT_INACTIVE lv_color_hex(0x333333)
#define GOPHR_COLOR_DRY         lv_color_hex(0xFF3D00)
#define GOPHR_COLOR_MID_RANGE   lv_color_hex(0xFFAB00)
#define GOPHR_COLOR_TEAL        lv_color_hex(0x00BFA5)
#define GOPHR_COLOR_WET         lv_color_hex(0x2979FF)
#define GOPHR_COLOR_ERROR       lv_color_hex(0xFF3D00)

/* Initialize shared styles */
void gophr_styles_init(void);

/* Map moisture 0-100% to gradient color (red -> yellow -> teal -> blue) */
lv_color_t gophr_moisture_color(int percent);
