#include "gophr_ui_styles.h"

void gophr_styles_init(void)
{
    /* Styles are applied inline using the color constants from the header.
     * This function is reserved for any global style objects if needed later. */
}

/* Interpolate between two colors by a fraction (0.0 - 1.0) */
static lv_color_t color_lerp(lv_color_t c1, lv_color_t c2, float t)
{
    uint8_t r1 = c1.red;
    uint8_t g1 = c1.green;
    uint8_t b1 = c1.blue;
    uint8_t r2 = c2.red;
    uint8_t g2 = c2.green;
    uint8_t b2 = c2.blue;

    uint8_t r = (uint8_t)(r1 + (r2 - r1) * t);
    uint8_t g = (uint8_t)(g1 + (g2 - g1) * t);
    uint8_t b = (uint8_t)(b1 + (b2 - b1) * t);

    return lv_color_make(r, g, b);
}

lv_color_t gophr_moisture_color(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    if (percent <= 33) {
        /* Red (#FF3D00) -> Yellow (#FFAB00) */
        float t = (float)percent / 33.0f;
        return color_lerp(GOPHR_COLOR_DRY, GOPHR_COLOR_MID_RANGE, t);
    } else if (percent <= 66) {
        /* Yellow (#FFAB00) -> Teal (#00BFA5) */
        float t = (float)(percent - 33) / 33.0f;
        return color_lerp(GOPHR_COLOR_MID_RANGE, GOPHR_COLOR_TEAL, t);
    } else {
        /* Teal (#00BFA5) -> Blue (#2979FF) */
        float t = (float)(percent - 66) / 34.0f;
        return color_lerp(GOPHR_COLOR_TEAL, GOPHR_COLOR_WET, t);
    }
}
