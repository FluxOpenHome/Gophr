#include "gophr_ui.h"
#include "gophr_ui_styles.h"
#include "gophr_encoder.h"
#include "gophr_buzzer.h"
#include "gophr_mqtt.h"
#include "gophr_wifi.h"
#include "gophr_logo.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "gophr_ui";

/* ---------- Constants ---------- */

#define SCREEN_SIZE         240
#define ARC_RADIUS          108
#define ARC_WIDTH           14
#define ARC_START_ANGLE     135
#define ARC_END_ANGLE       45
#define CENTER_X            120
#define CENTER_Y            120

static const char *SENSOR_NAMES[] = {"SHALLOW", "MID", "DEEP"};

/* Sweep duration presets in seconds */
static const int SWEEP_DURATIONS[] = {30, 60, 300, 900, 1800, 3600};
static const char *SWEEP_DURATION_LABELS[] = {"30s", "1m", "5m", "15m", "30m", "1h"};
#define SWEEP_DURATION_COUNT 6

/* Character set for text entry */
static const char CHARSET[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*()-_=+[]{}|;:',.<>/?` ~";
#define CHARSET_LEN (sizeof(CHARSET) - 1)

/* ---------- State ---------- */

static ui_state_t s_state = UI_STATE_SPLASH;
static int s_moisture[3] = {50, 50, 50};
static bool s_wifi_connected = false;
static bool s_mqtt_connected = false;

/* Sweep mode state */
static int s_sweep_target[3] = {50, 50, 50};
static int s_sweep_duration_idx[3] = {1, 1, 1}; /* Default 1m */
static int s_sweep_start_values[3] = {0, 0, 0};
static int64_t s_sweep_start_time = 0;
static bool s_sweep_active = false;
static TaskHandle_t s_sweep_task = NULL;

/* Sub-state for sweep setup: 0 = adjusting target, 1 = adjusting duration */
static int s_sweep_setup_field = 0;

/* Wi-Fi provisioning state */
static wifi_scan_entry_t s_scan_results[WIFI_SCAN_MAX_AP];
static int s_scan_count = 0;
static int s_wifi_selected_idx = 0;
static char s_wifi_password[WIFI_PASS_MAX_LEN] = {0};
static int s_wifi_pass_cursor = 0;
static int s_wifi_pass_char_idx = 0; /* Index into CHARSET */

/* MQTT provisioning state */
static char s_mqtt_uri[MQTT_URI_MAX_LEN] = "mqtt://";
static char s_mqtt_user[MQTT_USER_MAX_LEN] = {0};
static char s_mqtt_pass[MQTT_PASS_MAX_LEN] = {0};
static int s_mqtt_field = 0;       /* 0=uri, 1=user, 2=pass */
static int s_mqtt_cursor = 7;      /* cursor position in current field */
static int s_mqtt_char_idx = 0;    /* Index into CHARSET */

/* LVGL mutex (shared with main) */
extern SemaphoreHandle_t g_lvgl_mutex;

/* ---------- Screen Objects ---------- */

/* Splash */
static lv_obj_t *scr_splash = NULL;
static lv_obj_t *splash_arc = NULL;

/* Wi-Fi scanning */
static lv_obj_t *scr_wifi_scan = NULL;
static lv_obj_t *wifi_scan_arc = NULL;
static lv_obj_t *wifi_scan_label = NULL;

/* Wi-Fi select */
static lv_obj_t *scr_wifi_select = NULL;
static lv_obj_t *wifi_select_list = NULL;
static lv_obj_t *wifi_select_title = NULL;

/* Wi-Fi password */
static lv_obj_t *scr_wifi_pass = NULL;
static lv_obj_t *wifi_pass_ssid_label = NULL;
static lv_obj_t *wifi_pass_input_label = NULL;
static lv_obj_t *wifi_pass_char_label = NULL;
static lv_obj_t *wifi_pass_hint_label = NULL;
static lv_obj_t *wifi_pass_cursor_line = NULL;

/* Wi-Fi connecting */
static lv_obj_t *scr_wifi_connecting = NULL;
static lv_obj_t *wifi_connecting_arc = NULL;
static lv_obj_t *wifi_connecting_label = NULL;
static lv_obj_t *wifi_connecting_ssid_label = NULL;

/* MQTT setup */
static lv_obj_t *scr_mqtt_setup = NULL;
static lv_obj_t *mqtt_field_title = NULL;
static lv_obj_t *mqtt_input_label = NULL;
static lv_obj_t *mqtt_char_label = NULL;
static lv_obj_t *mqtt_hint_label = NULL;
static lv_obj_t *mqtt_dots[3] = {NULL};

/* MQTT connecting */
static lv_obj_t *scr_mqtt_connecting = NULL;
static lv_obj_t *mqtt_connecting_arc = NULL;
static lv_obj_t *mqtt_connecting_label = NULL;

/* Mode select */
static lv_obj_t *scr_mode = NULL;
static lv_obj_t *mode_btn_instant = NULL;
static lv_obj_t *mode_btn_sweep = NULL;

/* Sensor adjust (instant mode) */
static lv_obj_t *scr_sensor = NULL;
static lv_obj_t *sensor_arc = NULL;
static lv_obj_t *sensor_value_label = NULL;
static lv_obj_t *sensor_name_label = NULL;
static lv_obj_t *sensor_dots[3] = {NULL};
static lv_obj_t *sensor_hint_label = NULL;
static lv_obj_t *sensor_wifi_icon = NULL;
static lv_obj_t *sensor_mqtt_icon = NULL;
static int s_current_sensor = 0;

/* Summary (instant mode) */
static lv_obj_t *scr_summary = NULL;
static lv_obj_t *summary_bars[3] = {NULL};
static lv_obj_t *summary_value_labels[3] = {NULL};
static lv_obj_t *summary_send_btn = NULL;
static lv_obj_t *summary_pulse_arc = NULL;

/* Confirmation */
static lv_obj_t *scr_confirm = NULL;
static lv_obj_t *confirm_icon_label = NULL;
static lv_obj_t *confirm_text_label = NULL;
static lv_obj_t *confirm_sub_label = NULL;
static lv_obj_t *confirm_arc = NULL;

/* Sweep setup */
static lv_obj_t *scr_sweep_setup = NULL;
static lv_obj_t *sweep_setup_arc = NULL;
static lv_obj_t *sweep_setup_value_label = NULL;
static lv_obj_t *sweep_setup_name_label = NULL;
static lv_obj_t *sweep_setup_duration_label = NULL;
static lv_obj_t *sweep_setup_field_label = NULL;
static lv_obj_t *sweep_setup_dots[3] = {NULL};
static int s_sweep_current_sensor = 0;

/* Sweep summary */
static lv_obj_t *scr_sweep_summary = NULL;
static lv_obj_t *sweep_summary_rows[3] = {NULL};
static lv_obj_t *sweep_summary_btn = NULL;

/* Sweep running */
static lv_obj_t *scr_sweep_running = NULL;
static lv_obj_t *sweep_arcs[3] = {NULL};
static lv_obj_t *sweep_time_label = NULL;
static lv_obj_t *sweep_cancel_label = NULL;

/* ---------- Helpers ---------- */

static void set_screen_bg(lv_obj_t *scr)
{
    lv_obj_set_style_bg_color(scr, GOPHR_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t *create_label(lv_obj_t *parent, const char *text,
                               const lv_font_t *font, lv_color_t color,
                               lv_align_t align, int x_ofs, int y_ofs)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_align(label, align, x_ofs, y_ofs);
    return label;
}

/* Update arc color to match moisture % */
static void update_arc_color(lv_obj_t *arc, int percent)
{
    lv_color_t c = gophr_moisture_color(percent);
    lv_obj_set_style_arc_color(arc, c, LV_PART_INDICATOR);
}

/* Get a visible representation of the character at cursor */
static void get_password_display(char *buf, size_t buf_len, const char *password, int cursor, int pass_len)
{
    int display_start = 0;
    int max_display = 18; /* Max chars that fit on screen */

    if (cursor > max_display - 1) {
        display_start = cursor - max_display + 1;
    }

    int pos = 0;
    for (int i = display_start; i < pass_len && pos < (int)buf_len - 2; i++) {
        if (i == cursor) {
            buf[pos++] = password[i]; /* Show current char */
        } else {
            buf[pos++] = '*';
        }
    }
    /* Show cursor position for new char */
    if (cursor == pass_len && pos < (int)buf_len - 2) {
        buf[pos++] = '_';
    }
    buf[pos] = '\0';
}

/* ---------- Screen Builders ---------- */

static void build_splash_screen(void)
{
    scr_splash = lv_obj_create(NULL);
    set_screen_bg(scr_splash);

    /* Gophr logo image (white text + green underbar on transparent bg) */
    lv_obj_t *logo_img = lv_image_create(scr_splash);
    lv_image_set_src(logo_img, &gophr_logo);
    lv_obj_align(logo_img, LV_ALIGN_CENTER, 0, -20);

    create_label(scr_splash, "Moisture Simulator", &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_DIM, LV_ALIGN_CENTER, 0, 35);

    /* Loading arc ring */
    splash_arc = lv_arc_create(scr_splash);
    lv_obj_set_size(splash_arc, 220, 220);
    lv_obj_align(splash_arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation(splash_arc, 270);
    lv_arc_set_bg_angles(splash_arc, 0, 360);
    lv_arc_set_range(splash_arc, 0, 360);
    lv_arc_set_value(splash_arc, 0);
    lv_obj_remove_style(splash_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(splash_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(splash_arc, 2, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(splash_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(splash_arc, 2, LV_PART_MAIN);
    lv_obj_set_style_arc_color(splash_arc, GOPHR_COLOR_ARC_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(splash_arc, LV_OPA_60, LV_PART_MAIN);
}

/* ---------- Wi-Fi Provisioning Screens ---------- */

static void build_wifi_scan_screen(void)
{
    scr_wifi_scan = lv_obj_create(NULL);
    set_screen_bg(scr_wifi_scan);

    /* Spinning arc */
    wifi_scan_arc = lv_arc_create(scr_wifi_scan);
    lv_obj_set_size(wifi_scan_arc, 100, 100);
    lv_obj_align(wifi_scan_arc, LV_ALIGN_CENTER, 0, -15);
    lv_arc_set_rotation(wifi_scan_arc, 0);
    lv_arc_set_bg_angles(wifi_scan_arc, 0, 360);
    lv_arc_set_range(wifi_scan_arc, 0, 360);
    lv_arc_set_value(wifi_scan_arc, 90);
    lv_obj_remove_style(wifi_scan_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(wifi_scan_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(wifi_scan_arc, 4, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(wifi_scan_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(wifi_scan_arc, 4, LV_PART_MAIN);
    lv_obj_set_style_arc_color(wifi_scan_arc, GOPHR_COLOR_ARC_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(wifi_scan_arc, LV_OPA_40, LV_PART_MAIN);

    /* Wi-Fi symbol in center */
    create_label(scr_wifi_scan, LV_SYMBOL_WIFI, &lv_font_montserrat_20,
                 GOPHR_COLOR_ACCENT, LV_ALIGN_CENTER, 0, -15);

    wifi_scan_label = create_label(scr_wifi_scan, "Scanning...", &lv_font_montserrat_14,
                                    GOPHR_COLOR_TEXT_DIM, LV_ALIGN_CENTER, 0, 40);
}

static void build_wifi_select_screen(void)
{
    scr_wifi_select = lv_obj_create(NULL);
    set_screen_bg(scr_wifi_select);

    wifi_select_title = create_label(scr_wifi_select, "SELECT NETWORK", &lv_font_montserrat_14,
                                      GOPHR_COLOR_TEXT_DIM, LV_ALIGN_TOP_MID, 0, 20);

    /* Scrollable list area */
    wifi_select_list = lv_obj_create(scr_wifi_select);
    lv_obj_set_size(wifi_select_list, 200, 160);
    lv_obj_align(wifi_select_list, LV_ALIGN_CENTER, 0, 15);
    lv_obj_set_style_bg_opa(wifi_select_list, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wifi_select_list, 0, 0);
    lv_obj_set_style_pad_all(wifi_select_list, 0, 0);
    lv_obj_set_style_pad_row(wifi_select_list, 4, 0);
    lv_obj_set_flex_flow(wifi_select_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(wifi_select_list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(wifi_select_list, LV_DIR_VER);

    /* Hint at bottom */
    create_label(scr_wifi_select, "Rotate | Press", &lv_font_montserrat_12,
                 GOPHR_COLOR_TEXT_HINT, LV_ALIGN_BOTTOM_MID, 0, -12);
}

static void populate_wifi_list(void)
{
    /* Clear existing children */
    lv_obj_clean(wifi_select_list);

    for (int i = 0; i < s_scan_count; i++) {
        lv_obj_t *row = lv_obj_create(wifi_select_list);
        lv_obj_set_size(row, 196, 30);
        lv_obj_set_style_bg_color(row, i == s_wifi_selected_idx ?
            GOPHR_COLOR_ACCENT : GOPHR_COLOR_ARC_TRACK, 0);
        lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(row, 8, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_left(row, 10, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        /* SSID text */
        lv_obj_t *ssid_lbl = lv_label_create(row);
        lv_label_set_text(ssid_lbl, s_scan_results[i].ssid);
        lv_obj_set_style_text_font(ssid_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(ssid_lbl, i == s_wifi_selected_idx ?
            GOPHR_COLOR_BG : GOPHR_COLOR_TEXT, 0);
        lv_obj_align(ssid_lbl, LV_ALIGN_LEFT_MID, 0, 0);
        lv_label_set_long_mode(ssid_lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(ssid_lbl, 140);

        /* RSSI indicator */
        lv_obj_t *rssi_lbl = lv_label_create(row);
        int8_t rssi = s_scan_results[i].rssi;
        const char *icon = rssi > -50 ? LV_SYMBOL_WIFI :
                           rssi > -70 ? LV_SYMBOL_WIFI : LV_SYMBOL_WIFI;
        lv_label_set_text(rssi_lbl, icon);
        lv_obj_set_style_text_font(rssi_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(rssi_lbl, i == s_wifi_selected_idx ?
            GOPHR_COLOR_BG : GOPHR_COLOR_TEXT_DIM, 0);
        lv_obj_set_style_text_opa(rssi_lbl, rssi > -70 ? LV_OPA_COVER : LV_OPA_50, 0);
        lv_obj_align(rssi_lbl, LV_ALIGN_RIGHT_MID, -5, 0);
    }

    if (s_scan_count == 0) {
        create_label(wifi_select_list, "No networks found", &lv_font_montserrat_12,
                     GOPHR_COLOR_TEXT_DIM, LV_ALIGN_CENTER, 0, 0);
    }
}

static void build_wifi_password_screen(void)
{
    scr_wifi_pass = lv_obj_create(NULL);
    set_screen_bg(scr_wifi_pass);

    /* Network name at top */
    wifi_pass_ssid_label = create_label(scr_wifi_pass, "", &lv_font_montserrat_12,
                                         GOPHR_COLOR_ACCENT, LV_ALIGN_TOP_MID, 0, 25);

    create_label(scr_wifi_pass, "PASSWORD", &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_DIM, LV_ALIGN_TOP_MID, 0, 42);

    /* Current character selector (large) */
    wifi_pass_char_label = create_label(scr_wifi_pass, "a", &lv_font_montserrat_48,
                                         GOPHR_COLOR_ACCENT, LV_ALIGN_CENTER, 0, -10);

    /* Arrow indicators */
    create_label(scr_wifi_pass, LV_SYMBOL_LEFT, &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_HINT, LV_ALIGN_CENTER, -50, -10);
    create_label(scr_wifi_pass, LV_SYMBOL_RIGHT, &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_HINT, LV_ALIGN_CENTER, 50, -10);

    /* Password entered so far (masked) */
    wifi_pass_input_label = create_label(scr_wifi_pass, "_", &lv_font_montserrat_14,
                                          GOPHR_COLOR_TEXT, LV_ALIGN_CENTER, 0, 40);

    /* Hint */
    wifi_pass_hint_label = create_label(scr_wifi_pass, "Rotate=char  Press=add  Hold=done",
                                         &lv_font_montserrat_12, GOPHR_COLOR_TEXT_HINT,
                                         LV_ALIGN_BOTTOM_MID, 0, -12);
}

static void update_wifi_password_display(void)
{
    /* Show current character in large font */
    char ch_buf[2] = {CHARSET[s_wifi_pass_char_idx], '\0'};
    lv_label_set_text(wifi_pass_char_label, ch_buf);

    /* Show password entered so far */
    int len = strlen(s_wifi_password);
    char display[32];
    get_password_display(display, sizeof(display), s_wifi_password, len, len);
    lv_label_set_text(wifi_pass_input_label, display);
}

static void build_wifi_connecting_screen(void)
{
    scr_wifi_connecting = lv_obj_create(NULL);
    set_screen_bg(scr_wifi_connecting);

    /* Spinning arc */
    wifi_connecting_arc = lv_arc_create(scr_wifi_connecting);
    lv_obj_set_size(wifi_connecting_arc, 120, 120);
    lv_obj_align(wifi_connecting_arc, LV_ALIGN_CENTER, 0, -15);
    lv_arc_set_rotation(wifi_connecting_arc, 0);
    lv_arc_set_bg_angles(wifi_connecting_arc, 0, 360);
    lv_arc_set_range(wifi_connecting_arc, 0, 360);
    lv_arc_set_value(wifi_connecting_arc, 90);
    lv_obj_remove_style(wifi_connecting_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(wifi_connecting_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(wifi_connecting_arc, 4, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(wifi_connecting_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(wifi_connecting_arc, 4, LV_PART_MAIN);
    lv_obj_set_style_arc_color(wifi_connecting_arc, GOPHR_COLOR_ARC_TRACK, LV_PART_MAIN);

    wifi_connecting_label = create_label(scr_wifi_connecting, "Connecting...", &lv_font_montserrat_14,
                                          GOPHR_COLOR_TEXT_DIM, LV_ALIGN_CENTER, 0, 45);

    wifi_connecting_ssid_label = create_label(scr_wifi_connecting, "", &lv_font_montserrat_12,
                                               GOPHR_COLOR_ACCENT, LV_ALIGN_CENTER, 0, -15);
}

/* ---------- MQTT Provisioning Screens ---------- */

static void build_mqtt_setup_screen(void)
{
    scr_mqtt_setup = lv_obj_create(NULL);
    set_screen_bg(scr_mqtt_setup);

    /* Field title */
    mqtt_field_title = create_label(scr_mqtt_setup, "BROKER URI", &lv_font_montserrat_14,
                                     GOPHR_COLOR_TEXT_DIM, LV_ALIGN_TOP_MID, 0, 25);

    /* Current character selector */
    mqtt_char_label = create_label(scr_mqtt_setup, "a", &lv_font_montserrat_48,
                                    GOPHR_COLOR_ACCENT, LV_ALIGN_CENTER, 0, -15);

    /* Arrow indicators */
    create_label(scr_mqtt_setup, LV_SYMBOL_LEFT, &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_HINT, LV_ALIGN_CENTER, -50, -15);
    create_label(scr_mqtt_setup, LV_SYMBOL_RIGHT, &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_HINT, LV_ALIGN_CENTER, 50, -15);

    /* Current value display */
    mqtt_input_label = create_label(scr_mqtt_setup, "mqtt://", &lv_font_montserrat_12,
                                     GOPHR_COLOR_TEXT, LV_ALIGN_CENTER, 0, 35);
    lv_label_set_long_mode(mqtt_input_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(mqtt_input_label, 200);
    lv_obj_set_style_text_align(mqtt_input_label, LV_TEXT_ALIGN_CENTER, 0);

    /* Page dots for fields */
    for (int i = 0; i < 3; i++) {
        mqtt_dots[i] = lv_obj_create(scr_mqtt_setup);
        lv_obj_set_size(mqtt_dots[i], 8, 8);
        lv_obj_set_style_radius(mqtt_dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(mqtt_dots[i], 0, 0);
        lv_obj_set_style_bg_color(mqtt_dots[i],
            i == 0 ? GOPHR_COLOR_ACCENT : GOPHR_COLOR_DOT_INACTIVE, 0);
        lv_obj_align(mqtt_dots[i], LV_ALIGN_BOTTOM_MID, (i - 1) * 16, -30);
        lv_obj_clear_flag(mqtt_dots[i], LV_OBJ_FLAG_SCROLLABLE);
    }

    /* Hint */
    mqtt_hint_label = create_label(scr_mqtt_setup, "Rotate=char  Press=add  Hold=next",
                                    &lv_font_montserrat_12, GOPHR_COLOR_TEXT_HINT,
                                    LV_ALIGN_BOTTOM_MID, 0, -12);
}

static void update_mqtt_setup_display(void)
{
    /* Update field title */
    const char *titles[] = {"BROKER URI", "USERNAME", "PASSWORD"};
    lv_label_set_text(mqtt_field_title, titles[s_mqtt_field]);

    /* Update dots */
    for (int i = 0; i < 3; i++) {
        lv_obj_set_style_bg_color(mqtt_dots[i],
            i == s_mqtt_field ? GOPHR_COLOR_ACCENT : GOPHR_COLOR_DOT_INACTIVE, 0);
    }

    /* Show current character */
    char ch_buf[2] = {CHARSET[s_mqtt_char_idx], '\0'};
    lv_label_set_text(mqtt_char_label, ch_buf);

    /* Show current field value */
    char *field_val = NULL;
    switch (s_mqtt_field) {
    case 0: field_val = s_mqtt_uri; break;
    case 1: field_val = s_mqtt_user; break;
    case 2: field_val = s_mqtt_pass; break;
    }

    if (field_val) {
        int len = strlen(field_val);
        /* For password, mask it */
        if (s_mqtt_field == 2 && len > 0) {
            char masked[MQTT_PASS_MAX_LEN];
            for (int i = 0; i < len; i++) {
                masked[i] = '*';
            }
            masked[len] = '_';
            masked[len + 1] = '\0';
            lv_label_set_text(mqtt_input_label, masked);
        } else {
            char display[MQTT_URI_MAX_LEN + 2];
            snprintf(display, sizeof(display), "%s_", field_val);
            lv_label_set_text(mqtt_input_label, display);
        }
    }

    /* Update hint for last field */
    if (s_mqtt_field == 2) {
        lv_label_set_text(mqtt_hint_label, "Rotate=char  Press=add  Hold=connect");
    } else {
        lv_label_set_text(mqtt_hint_label, "Rotate=char  Press=add  Hold=next");
    }
}

static void build_mqtt_connecting_screen(void)
{
    scr_mqtt_connecting = lv_obj_create(NULL);
    set_screen_bg(scr_mqtt_connecting);

    mqtt_connecting_arc = lv_arc_create(scr_mqtt_connecting);
    lv_obj_set_size(mqtt_connecting_arc, 120, 120);
    lv_obj_align(mqtt_connecting_arc, LV_ALIGN_CENTER, 0, -15);
    lv_arc_set_rotation(mqtt_connecting_arc, 0);
    lv_arc_set_bg_angles(mqtt_connecting_arc, 0, 360);
    lv_arc_set_range(mqtt_connecting_arc, 0, 360);
    lv_arc_set_value(mqtt_connecting_arc, 90);
    lv_obj_remove_style(mqtt_connecting_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(mqtt_connecting_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(mqtt_connecting_arc, 4, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(mqtt_connecting_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(mqtt_connecting_arc, 4, LV_PART_MAIN);
    lv_obj_set_style_arc_color(mqtt_connecting_arc, GOPHR_COLOR_ARC_TRACK, LV_PART_MAIN);

    mqtt_connecting_label = create_label(scr_mqtt_connecting, "Connecting to broker...",
                                          &lv_font_montserrat_14, GOPHR_COLOR_TEXT_DIM,
                                          LV_ALIGN_CENTER, 0, 45);
}

/* ---------- Original Screens ---------- */

static void build_mode_select_screen(void)
{
    scr_mode = lv_obj_create(NULL);
    set_screen_bg(scr_mode);

    create_label(scr_mode, "SELECT MODE", &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_DIM, LV_ALIGN_TOP_MID, 0, 30);

    /* Instant button */
    mode_btn_instant = lv_btn_create(scr_mode);
    lv_obj_set_size(mode_btn_instant, 160, 50);
    lv_obj_align(mode_btn_instant, LV_ALIGN_CENTER, 0, -25);
    lv_obj_set_style_bg_color(mode_btn_instant, GOPHR_COLOR_ACCENT, 0);
    lv_obj_set_style_radius(mode_btn_instant, 25, 0);
    lv_obj_set_style_shadow_width(mode_btn_instant, 0, 0);
    lv_obj_t *lbl1 = lv_label_create(mode_btn_instant);
    lv_label_set_text(lbl1, "INSTANT");
    lv_obj_set_style_text_font(lbl1, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl1, GOPHR_COLOR_BG, 0);
    lv_obj_center(lbl1);

    /* Sweep button */
    mode_btn_sweep = lv_btn_create(scr_mode);
    lv_obj_set_size(mode_btn_sweep, 160, 50);
    lv_obj_align(mode_btn_sweep, LV_ALIGN_CENTER, 0, 40);
    lv_obj_set_style_bg_color(mode_btn_sweep, GOPHR_COLOR_ARC_TRACK, 0);
    lv_obj_set_style_bg_color(mode_btn_sweep, GOPHR_COLOR_ACCENT, LV_STATE_FOCUSED);
    lv_obj_set_style_radius(mode_btn_sweep, 25, 0);
    lv_obj_set_style_shadow_width(mode_btn_sweep, 0, 0);
    lv_obj_set_style_border_width(mode_btn_sweep, 2, 0);
    lv_obj_set_style_border_color(mode_btn_sweep, GOPHR_COLOR_ACCENT, 0);
    lv_obj_t *lbl2 = lv_label_create(mode_btn_sweep);
    lv_label_set_text(lbl2, "SWEEP");
    lv_obj_set_style_text_font(lbl2, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl2, GOPHR_COLOR_ACCENT, 0);
    lv_obj_center(lbl2);
}

static void build_sensor_screen(void)
{
    scr_sensor = lv_obj_create(NULL);
    set_screen_bg(scr_sensor);

    /* Main gauge arc */
    sensor_arc = lv_arc_create(scr_sensor);
    lv_obj_set_size(sensor_arc, ARC_RADIUS * 2, ARC_RADIUS * 2);
    lv_obj_align(sensor_arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation(sensor_arc, ARC_START_ANGLE);
    lv_arc_set_bg_angles(sensor_arc, 0, 270);
    lv_arc_set_range(sensor_arc, 0, 100);
    lv_arc_set_value(sensor_arc, 50);
    lv_obj_remove_style(sensor_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(sensor_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(sensor_arc, ARC_WIDTH, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(sensor_arc, ARC_WIDTH, LV_PART_MAIN);
    lv_obj_set_style_arc_color(sensor_arc, GOPHR_COLOR_ARC_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(sensor_arc, true, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(sensor_arc, true, LV_PART_MAIN);
    update_arc_color(sensor_arc, 50);

    /* Center value label */
    sensor_value_label = create_label(scr_sensor, "50%", &lv_font_montserrat_48,
                                       GOPHR_COLOR_TEXT, LV_ALIGN_CENTER, 0, -5);

    /* Sensor name label */
    sensor_name_label = create_label(scr_sensor, "SHALLOW", &lv_font_montserrat_14,
                                      GOPHR_COLOR_TEXT_DIM, LV_ALIGN_CENTER, 0, -45);

    /* Page dots at bottom */
    for (int i = 0; i < 3; i++) {
        sensor_dots[i] = lv_obj_create(scr_sensor);
        lv_obj_set_size(sensor_dots[i], 8, 8);
        lv_obj_set_style_radius(sensor_dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(sensor_dots[i], 0, 0);
        lv_obj_set_style_bg_color(sensor_dots[i],
            i == 0 ? GOPHR_COLOR_ACCENT : GOPHR_COLOR_DOT_INACTIVE, 0);
        lv_obj_align(sensor_dots[i], LV_ALIGN_BOTTOM_MID, (i - 1) * 16, -22);
        lv_obj_clear_flag(sensor_dots[i], LV_OBJ_FLAG_SCROLLABLE);
    }

    /* Hint label */
    sensor_hint_label = create_label(scr_sensor, "Rotate | Press", &lv_font_montserrat_12,
                                      GOPHR_COLOR_TEXT_HINT, LV_ALIGN_BOTTOM_MID, 0, -8);

    /* Wi-Fi / MQTT status dots (top corners) */
    sensor_wifi_icon = lv_obj_create(scr_sensor);
    lv_obj_set_size(sensor_wifi_icon, 6, 6);
    lv_obj_set_style_radius(sensor_wifi_icon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(sensor_wifi_icon, 0, 0);
    lv_obj_set_style_bg_color(sensor_wifi_icon, GOPHR_COLOR_ERROR, 0);
    lv_obj_align(sensor_wifi_icon, LV_ALIGN_TOP_LEFT, 30, 20);
    lv_obj_clear_flag(sensor_wifi_icon, LV_OBJ_FLAG_SCROLLABLE);

    sensor_mqtt_icon = lv_obj_create(scr_sensor);
    lv_obj_set_size(sensor_mqtt_icon, 6, 6);
    lv_obj_set_style_radius(sensor_mqtt_icon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(sensor_mqtt_icon, 0, 0);
    lv_obj_set_style_bg_color(sensor_mqtt_icon, GOPHR_COLOR_ERROR, 0);
    lv_obj_align(sensor_mqtt_icon, LV_ALIGN_TOP_RIGHT, -30, 20);
    lv_obj_clear_flag(sensor_mqtt_icon, LV_OBJ_FLAG_SCROLLABLE);
}

static void update_sensor_screen(int index)
{
    if (index < 0 || index > 2) return;
    s_current_sensor = index;

    lv_arc_set_value(sensor_arc, s_moisture[index]);
    update_arc_color(sensor_arc, s_moisture[index]);

    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", s_moisture[index]);
    lv_label_set_text(sensor_value_label, buf);
    lv_obj_set_style_text_color(sensor_value_label,
        gophr_moisture_color(s_moisture[index]), 0);

    lv_label_set_text(sensor_name_label, SENSOR_NAMES[index]);

    for (int i = 0; i < 3; i++) {
        lv_obj_set_style_bg_color(sensor_dots[i],
            i == index ? GOPHR_COLOR_ACCENT : GOPHR_COLOR_DOT_INACTIVE, 0);
    }
}

static void build_summary_screen(void)
{
    scr_summary = lv_obj_create(NULL);
    set_screen_bg(scr_summary);

    create_label(scr_summary, "SUMMARY", &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_DIM, LV_ALIGN_TOP_MID, 0, 25);

    /* Three sensor bars */
    for (int i = 0; i < 3; i++) {
        int y_pos = 60 + i * 38;

        /* Name */
        char name[16];
        snprintf(name, sizeof(name), "S%d %s", i + 1, SENSOR_NAMES[i]);
        lv_obj_t *name_lbl = lv_label_create(scr_summary);
        lv_label_set_text(name_lbl, name);
        lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(name_lbl, GOPHR_COLOR_TEXT_DIM, 0);
        lv_obj_set_pos(name_lbl, 45, y_pos);

        /* Value */
        summary_value_labels[i] = lv_label_create(scr_summary);
        lv_label_set_text(summary_value_labels[i], "50%");
        lv_obj_set_style_text_font(summary_value_labels[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(summary_value_labels[i], GOPHR_COLOR_TEXT, 0);
        lv_obj_set_pos(summary_value_labels[i], 175, y_pos - 2);

        /* Bar */
        summary_bars[i] = lv_bar_create(scr_summary);
        lv_obj_set_size(summary_bars[i], 150, 6);
        lv_obj_set_pos(summary_bars[i], 45, y_pos + 18);
        lv_bar_set_range(summary_bars[i], 0, 100);
        lv_bar_set_value(summary_bars[i], 50, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(summary_bars[i], GOPHR_COLOR_ARC_TRACK, LV_PART_MAIN);
        lv_obj_set_style_radius(summary_bars[i], 3, LV_PART_MAIN);
        lv_obj_set_style_radius(summary_bars[i], 3, LV_PART_INDICATOR);
    }

    /* Send button */
    summary_send_btn = lv_btn_create(scr_summary);
    lv_obj_set_size(summary_send_btn, 100, 40);
    lv_obj_align(summary_send_btn, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_bg_color(summary_send_btn, GOPHR_COLOR_ACCENT, 0);
    lv_obj_set_style_radius(summary_send_btn, 20, 0);
    lv_obj_set_style_shadow_width(summary_send_btn, 0, 0);
    lv_obj_t *send_lbl = lv_label_create(summary_send_btn);
    lv_label_set_text(send_lbl, "SEND");
    lv_obj_set_style_text_font(send_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(send_lbl, GOPHR_COLOR_BG, 0);
    lv_obj_center(send_lbl);

    /* Pulsing outer ring */
    summary_pulse_arc = lv_arc_create(scr_summary);
    lv_obj_set_size(summary_pulse_arc, 230, 230);
    lv_obj_align(summary_pulse_arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation(summary_pulse_arc, 0);
    lv_arc_set_bg_angles(summary_pulse_arc, 0, 360);
    lv_arc_set_range(summary_pulse_arc, 0, 100);
    lv_arc_set_value(summary_pulse_arc, 100);
    lv_obj_remove_style(summary_pulse_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(summary_pulse_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(summary_pulse_arc, 2, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(summary_pulse_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(summary_pulse_arc, LV_OPA_40, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(summary_pulse_arc, 0, LV_PART_MAIN);
}

static void update_summary_screen(void)
{
    for (int i = 0; i < 3; i++) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", s_moisture[i]);
        lv_label_set_text(summary_value_labels[i], buf);
        lv_obj_set_style_text_color(summary_value_labels[i],
            gophr_moisture_color(s_moisture[i]), 0);

        lv_bar_set_value(summary_bars[i], s_moisture[i], LV_ANIM_ON);
        lv_obj_set_style_bg_color(summary_bars[i],
            gophr_moisture_color(s_moisture[i]), LV_PART_INDICATOR);
    }
}

static void build_confirmation_screen(void)
{
    scr_confirm = lv_obj_create(NULL);
    set_screen_bg(scr_confirm);

    /* Checkmark / X icon */
    confirm_icon_label = create_label(scr_confirm, LV_SYMBOL_OK, &lv_font_montserrat_48,
                                       GOPHR_COLOR_ACCENT, LV_ALIGN_CENTER, 0, -25);

    /* Result text */
    confirm_text_label = create_label(scr_confirm, "SENT!", &lv_font_montserrat_20,
                                       GOPHR_COLOR_ACCENT, LV_ALIGN_CENTER, 0, 15);

    /* Sub text */
    confirm_sub_label = create_label(scr_confirm, "Values published", &lv_font_montserrat_12,
                                      GOPHR_COLOR_TEXT_DIM, LV_ALIGN_CENTER, 0, 40);

    /* Sweep arc animation */
    confirm_arc = lv_arc_create(scr_confirm);
    lv_obj_set_size(confirm_arc, 200, 200);
    lv_obj_align(confirm_arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation(confirm_arc, 270);
    lv_arc_set_bg_angles(confirm_arc, 0, 360);
    lv_arc_set_range(confirm_arc, 0, 360);
    lv_arc_set_value(confirm_arc, 0);
    lv_obj_remove_style(confirm_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(confirm_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(confirm_arc, 3, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(confirm_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(confirm_arc, 0, LV_PART_MAIN);
}

static void show_confirmation(bool success)
{
    if (success) {
        lv_label_set_text(confirm_icon_label, LV_SYMBOL_OK);
        lv_obj_set_style_text_color(confirm_icon_label, GOPHR_COLOR_ACCENT, 0);
        lv_label_set_text(confirm_text_label, "SENT!");
        lv_obj_set_style_text_color(confirm_text_label, GOPHR_COLOR_ACCENT, 0);
        lv_label_set_text(confirm_sub_label, "Values published");
        lv_obj_set_style_arc_color(confirm_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
    } else {
        lv_label_set_text(confirm_icon_label, LV_SYMBOL_CLOSE);
        lv_obj_set_style_text_color(confirm_icon_label, GOPHR_COLOR_ERROR, 0);
        lv_label_set_text(confirm_text_label, "FAILED");
        lv_obj_set_style_text_color(confirm_text_label, GOPHR_COLOR_ERROR, 0);
        lv_label_set_text(confirm_sub_label, "Check connection");
        lv_obj_set_style_arc_color(confirm_arc, GOPHR_COLOR_ERROR, LV_PART_INDICATOR);
    }
    lv_arc_set_value(confirm_arc, 0);
}

/* ---------- Sweep Mode Screens ---------- */

static void build_sweep_setup_screen(void)
{
    scr_sweep_setup = lv_obj_create(NULL);
    set_screen_bg(scr_sweep_setup);

    /* Sensor name */
    sweep_setup_name_label = create_label(scr_sweep_setup, "SHALLOW", &lv_font_montserrat_14,
                                           GOPHR_COLOR_TEXT_DIM, LV_ALIGN_TOP_MID, 0, 22);

    /* Field label (TARGET / DURATION) */
    sweep_setup_field_label = create_label(scr_sweep_setup, "TARGET", &lv_font_montserrat_12,
                                            GOPHR_COLOR_ACCENT, LV_ALIGN_TOP_MID, 0, 40);

    /* Arc for target value */
    sweep_setup_arc = lv_arc_create(scr_sweep_setup);
    lv_obj_set_size(sweep_setup_arc, 180, 180);
    lv_obj_align(sweep_setup_arc, LV_ALIGN_CENTER, 0, -8);
    lv_arc_set_rotation(sweep_setup_arc, ARC_START_ANGLE);
    lv_arc_set_bg_angles(sweep_setup_arc, 0, 270);
    lv_arc_set_range(sweep_setup_arc, 0, 100);
    lv_arc_set_value(sweep_setup_arc, 50);
    lv_obj_remove_style(sweep_setup_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(sweep_setup_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(sweep_setup_arc, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(sweep_setup_arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(sweep_setup_arc, GOPHR_COLOR_ARC_TRACK, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(sweep_setup_arc, true, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(sweep_setup_arc, true, LV_PART_MAIN);
    update_arc_color(sweep_setup_arc, 50);

    /* Value label (shows % or duration) */
    sweep_setup_value_label = create_label(scr_sweep_setup, "50%", &lv_font_montserrat_32,
                                            GOPHR_COLOR_TEXT, LV_ALIGN_CENTER, 0, -8);

    /* Duration label below the arc */
    sweep_setup_duration_label = create_label(scr_sweep_setup, "Duration: 1m",
                                               &lv_font_montserrat_12,
                                               GOPHR_COLOR_TEXT_DIM, LV_ALIGN_BOTTOM_MID, 0, -38);

    /* Page dots */
    for (int i = 0; i < 3; i++) {
        sweep_setup_dots[i] = lv_obj_create(scr_sweep_setup);
        lv_obj_set_size(sweep_setup_dots[i], 8, 8);
        lv_obj_set_style_radius(sweep_setup_dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(sweep_setup_dots[i], 0, 0);
        lv_obj_set_style_bg_color(sweep_setup_dots[i],
            i == 0 ? GOPHR_COLOR_ACCENT : GOPHR_COLOR_DOT_INACTIVE, 0);
        lv_obj_align(sweep_setup_dots[i], LV_ALIGN_BOTTOM_MID, (i - 1) * 16, -22);
        lv_obj_clear_flag(sweep_setup_dots[i], LV_OBJ_FLAG_SCROLLABLE);
    }

    /* Hint */
    create_label(scr_sweep_setup, "Rotate | Press to toggle",
                 &lv_font_montserrat_12, GOPHR_COLOR_TEXT_HINT,
                 LV_ALIGN_BOTTOM_MID, 0, -8);
}

static void update_sweep_setup_screen(int sensor_idx)
{
    s_sweep_current_sensor = sensor_idx;

    lv_label_set_text(sweep_setup_name_label, SENSOR_NAMES[sensor_idx]);

    for (int i = 0; i < 3; i++) {
        lv_obj_set_style_bg_color(sweep_setup_dots[i],
            i == sensor_idx ? GOPHR_COLOR_ACCENT : GOPHR_COLOR_DOT_INACTIVE, 0);
    }

    if (s_sweep_setup_field == 0) {
        /* Target mode */
        lv_label_set_text(sweep_setup_field_label, "TARGET");
        lv_arc_set_value(sweep_setup_arc, s_sweep_target[sensor_idx]);
        update_arc_color(sweep_setup_arc, s_sweep_target[sensor_idx]);

        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", s_sweep_target[sensor_idx]);
        lv_label_set_text(sweep_setup_value_label, buf);
        lv_obj_set_style_text_color(sweep_setup_value_label,
            gophr_moisture_color(s_sweep_target[sensor_idx]), 0);

        lv_obj_clear_flag(sweep_setup_arc, LV_OBJ_FLAG_HIDDEN);
    } else {
        /* Duration mode */
        lv_label_set_text(sweep_setup_field_label, "DURATION");

        lv_label_set_text(sweep_setup_value_label,
            SWEEP_DURATION_LABELS[s_sweep_duration_idx[sensor_idx]]);
        lv_obj_set_style_text_color(sweep_setup_value_label, GOPHR_COLOR_ACCENT, 0);

        /* Hide the arc in duration mode, show just the value */
        lv_arc_set_value(sweep_setup_arc, 0);
        lv_obj_set_style_arc_color(sweep_setup_arc, GOPHR_COLOR_ARC_TRACK, LV_PART_INDICATOR);
    }

    char dur_buf[24];
    snprintf(dur_buf, sizeof(dur_buf), "Duration: %s",
             SWEEP_DURATION_LABELS[s_sweep_duration_idx[sensor_idx]]);
    lv_label_set_text(sweep_setup_duration_label, dur_buf);
}

static void build_sweep_summary_screen(void)
{
    scr_sweep_summary = lv_obj_create(NULL);
    set_screen_bg(scr_sweep_summary);

    create_label(scr_sweep_summary, "SWEEP SETUP", &lv_font_montserrat_14,
                 GOPHR_COLOR_TEXT_DIM, LV_ALIGN_TOP_MID, 0, 22);

    for (int i = 0; i < 3; i++) {
        int y_pos = 55 + i * 40;

        sweep_summary_rows[i] = lv_label_create(scr_sweep_summary);
        lv_obj_set_style_text_font(sweep_summary_rows[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(sweep_summary_rows[i], GOPHR_COLOR_TEXT, 0);
        lv_obj_set_pos(sweep_summary_rows[i], 35, y_pos);
    }

    /* Start button */
    sweep_summary_btn = lv_btn_create(scr_sweep_summary);
    lv_obj_set_size(sweep_summary_btn, 130, 40);
    lv_obj_align(sweep_summary_btn, LV_ALIGN_BOTTOM_MID, 0, -28);
    lv_obj_set_style_bg_color(sweep_summary_btn, GOPHR_COLOR_ACCENT, 0);
    lv_obj_set_style_radius(sweep_summary_btn, 20, 0);
    lv_obj_set_style_shadow_width(sweep_summary_btn, 0, 0);
    lv_obj_t *start_lbl = lv_label_create(sweep_summary_btn);
    lv_label_set_text(start_lbl, "START");
    lv_obj_set_style_text_font(start_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(start_lbl, GOPHR_COLOR_BG, 0);
    lv_obj_center(start_lbl);
}

static void update_sweep_summary_screen(void)
{
    for (int i = 0; i < 3; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s: %d%% -> %d%%  (%s)",
                 SENSOR_NAMES[i], s_moisture[i], s_sweep_target[i],
                 SWEEP_DURATION_LABELS[s_sweep_duration_idx[i]]);
        lv_label_set_text(sweep_summary_rows[i], buf);
        lv_obj_set_style_text_color(sweep_summary_rows[i],
            gophr_moisture_color(s_sweep_target[i]), 0);
    }
}

static void build_sweep_running_screen(void)
{
    scr_sweep_running = lv_obj_create(NULL);
    set_screen_bg(scr_sweep_running);

    /* 3 concentric arcs: Shallow=outer, Mid=mid, Deep=inner */
    int sizes[] = {210, 170, 130};
    int widths[] = {12, 12, 12};

    for (int i = 0; i < 3; i++) {
        sweep_arcs[i] = lv_arc_create(scr_sweep_running);
        lv_obj_set_size(sweep_arcs[i], sizes[i], sizes[i]);
        lv_obj_align(sweep_arcs[i], LV_ALIGN_CENTER, 0, 0);
        lv_arc_set_rotation(sweep_arcs[i], ARC_START_ANGLE);
        lv_arc_set_bg_angles(sweep_arcs[i], 0, 270);
        lv_arc_set_range(sweep_arcs[i], 0, 100);
        lv_arc_set_value(sweep_arcs[i], 0);
        lv_obj_remove_style(sweep_arcs[i], NULL, LV_PART_KNOB);
        lv_obj_clear_flag(sweep_arcs[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_arc_width(sweep_arcs[i], widths[i], LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(sweep_arcs[i], widths[i], LV_PART_MAIN);
        lv_obj_set_style_arc_color(sweep_arcs[i], GOPHR_COLOR_ARC_TRACK, LV_PART_MAIN);
        lv_obj_set_style_arc_rounded(sweep_arcs[i], true, LV_PART_INDICATOR);
        lv_obj_set_style_arc_rounded(sweep_arcs[i], true, LV_PART_MAIN);
    }

    /* Time label in center */
    sweep_time_label = create_label(scr_sweep_running, "0:00", &lv_font_montserrat_32,
                                     GOPHR_COLOR_TEXT, LV_ALIGN_CENTER, 0, -5);

    /* Cancel hint */
    sweep_cancel_label = create_label(scr_sweep_running, "Press to cancel",
                                       &lv_font_montserrat_12, GOPHR_COLOR_TEXT_HINT,
                                       LV_ALIGN_CENTER, 0, 25);
}

/* ---------- Sweep Task ---------- */

static void sweep_task_func(void *pvParameters)
{
    ESP_LOGI(TAG, "Sweep started");
    s_sweep_active = true;

    /* Record start values */
    for (int i = 0; i < 3; i++) {
        s_sweep_start_values[i] = s_moisture[i];
    }
    s_sweep_start_time = esp_timer_get_time();

    /* Find the longest duration */
    int max_duration_s = 0;
    for (int i = 0; i < 3; i++) {
        int dur = SWEEP_DURATIONS[s_sweep_duration_idx[i]];
        if (dur > max_duration_s) max_duration_s = dur;
    }

    while (s_sweep_active) {
        int64_t now = esp_timer_get_time();
        float elapsed_s = (float)(now - s_sweep_start_time) / 1000000.0f;

        bool all_done = true;

        for (int i = 0; i < 3; i++) {
            int duration = SWEEP_DURATIONS[s_sweep_duration_idx[i]];
            float t = elapsed_s / (float)duration;
            if (t > 1.0f) t = 1.0f;
            else all_done = false;

            int value = s_sweep_start_values[i] +
                (int)((s_sweep_target[i] - s_sweep_start_values[i]) * t);
            s_moisture[i] = value;
        }

        /* Publish current values via MQTT */
        gophr_mqtt_publish_all(s_moisture[0], s_moisture[1], s_moisture[2]);

        /* Update UI (need LVGL mutex) */
        if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(50))) {
            for (int i = 0; i < 3; i++) {
                lv_arc_set_value(sweep_arcs[i], s_moisture[i]);
                update_arc_color(sweep_arcs[i], s_moisture[i]);
            }

            /* Update time display */
            int elapsed_int = (int)elapsed_s;
            int remaining = max_duration_s - elapsed_int;
            if (remaining < 0) remaining = 0;
            int min = remaining / 60;
            int sec = remaining % 60;
            char time_buf[16];
            snprintf(time_buf, sizeof(time_buf), "%d:%02d", min, sec);
            lv_label_set_text(sweep_time_label, time_buf);

            xSemaphoreGive(g_lvgl_mutex);
        }

        if (all_done) break;

        vTaskDelay(pdMS_TO_TICKS(2000)); /* Update every 2 seconds */
    }

    s_sweep_active = false;
    ESP_LOGI(TAG, "Sweep complete");

    /* Transition to done state */
    s_state = UI_STATE_SWEEP_DONE;
    s_sweep_task = NULL;
    vTaskDelete(NULL);
}

/* ---------- Wi-Fi / MQTT Background Tasks ---------- */

static TaskHandle_t s_wifi_scan_task = NULL;
static TaskHandle_t s_wifi_connect_task = NULL;
static TaskHandle_t s_mqtt_connect_task = NULL;

static void wifi_scan_task_func(void *pvParameters)
{
    s_scan_count = gophr_wifi_scan(s_scan_results, WIFI_SCAN_MAX_AP);
    ESP_LOGI(TAG, "Wi-Fi scan complete: %d networks", s_scan_count);

    /* Transition to select screen */
    if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(100))) {
        s_wifi_selected_idx = 0;
        populate_wifi_list();
        s_state = UI_STATE_WIFI_SELECT;
        lv_scr_load_anim(scr_wifi_select, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
        xSemaphoreGive(g_lvgl_mutex);
    }

    s_wifi_scan_task = NULL;
    vTaskDelete(NULL);
}

static void wifi_connect_saved_task_func(void *pvParameters)
{
    /* Try connecting with NVS-saved credentials */
    bool had_creds = gophr_wifi_connect_saved();
    esp_err_t ret = (had_creds && gophr_wifi_is_connected()) ? ESP_OK : ESP_FAIL;

    if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(100))) {
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi auto-connected, checking MQTT");
            if (gophr_mqtt_has_saved_creds()) {
                lv_label_set_text(mqtt_connecting_label, "Connecting to broker...");
                s_state = UI_STATE_MQTT_CONNECTING;
                lv_scr_load_anim(scr_mqtt_connecting, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                xSemaphoreGive(g_lvgl_mutex);

                gophr_mqtt_connect_saved();
                for (int i = 0; i < 30; i++) {
                    vTaskDelay(pdMS_TO_TICKS(200));
                    if (gophr_mqtt_is_connected()) break;
                }

                if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(100))) {
                    if (gophr_mqtt_is_connected()) {
                        s_state = UI_STATE_MODE_SELECT;
                        lv_scr_load_anim(scr_mode, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                    } else {
                        s_mqtt_field = 0;
                        s_mqtt_cursor = strlen(s_mqtt_uri);
                        s_mqtt_char_idx = 0;
                        update_mqtt_setup_display();
                        s_state = UI_STATE_MQTT_SETUP;
                        lv_scr_load_anim(scr_mqtt_setup, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                    }
                    xSemaphoreGive(g_lvgl_mutex);
                }
            } else {
                s_mqtt_field = 0;
                s_mqtt_cursor = strlen(s_mqtt_uri);
                s_mqtt_char_idx = 0;
                update_mqtt_setup_display();
                s_state = UI_STATE_MQTT_SETUP;
                lv_scr_load_anim(scr_mqtt_setup, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                xSemaphoreGive(g_lvgl_mutex);
            }
        } else {
            /* Saved Wi-Fi failed, go to scan */
            s_state = UI_STATE_WIFI_SCANNING;
            lv_label_set_text(wifi_scan_label, "Scanning...");
            lv_scr_load_anim(scr_wifi_scan, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);
            xSemaphoreGive(g_lvgl_mutex);
            xTaskCreate(wifi_scan_task_func, "wifi_scan", 4096, NULL, 3, &s_wifi_scan_task);
        }
    }

    s_wifi_connect_task = NULL;
    vTaskDelete(NULL);
}

static void wifi_connect_task_func(void *pvParameters)
{
    esp_err_t ret = gophr_wifi_connect(
        s_scan_results[s_wifi_selected_idx].ssid, s_wifi_password);

    if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(100))) {
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi connected, proceeding to MQTT setup");
            /* Check if we have saved MQTT creds */
            if (gophr_mqtt_has_saved_creds()) {
                /* Try auto-connect to MQTT */
                lv_label_set_text(mqtt_connecting_label, "Connecting to broker...");
                s_state = UI_STATE_MQTT_CONNECTING;
                lv_scr_load_anim(scr_mqtt_connecting, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                xSemaphoreGive(g_lvgl_mutex);

                /* Start MQTT auto-connect in this task */
                gophr_mqtt_connect_saved();
                /* Give it a few seconds to connect */
                for (int i = 0; i < 30; i++) {
                    vTaskDelay(pdMS_TO_TICKS(200));
                    if (gophr_mqtt_is_connected()) break;
                }

                if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(100))) {
                    if (gophr_mqtt_is_connected()) {
                        s_state = UI_STATE_MODE_SELECT;
                        lv_scr_load_anim(scr_mode, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                    } else {
                        /* MQTT auto-connect failed, show setup screen */
                        s_mqtt_field = 0;
                        s_mqtt_cursor = strlen(s_mqtt_uri);
                        s_mqtt_char_idx = 0;
                        update_mqtt_setup_display();
                        s_state = UI_STATE_MQTT_SETUP;
                        lv_scr_load_anim(scr_mqtt_setup, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                    }
                    xSemaphoreGive(g_lvgl_mutex);
                }
            } else {
                /* No saved MQTT creds, show setup */
                s_mqtt_field = 0;
                s_mqtt_cursor = strlen(s_mqtt_uri);
                s_mqtt_char_idx = 0;
                update_mqtt_setup_display();
                s_state = UI_STATE_MQTT_SETUP;
                lv_scr_load_anim(scr_mqtt_setup, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                xSemaphoreGive(g_lvgl_mutex);
            }
        } else {
            /* Wi-Fi connection failed */
            lv_label_set_text(wifi_connecting_label, "Failed! Press to retry");
            lv_obj_set_style_text_color(wifi_connecting_label, GOPHR_COLOR_ERROR, 0);
            lv_obj_set_style_arc_color(wifi_connecting_arc, GOPHR_COLOR_ERROR, LV_PART_INDICATOR);
            /* Stay on connecting screen, user presses to go back to scan */
            xSemaphoreGive(g_lvgl_mutex);
        }
    }

    s_wifi_connect_task = NULL;
    vTaskDelete(NULL);
}

static void mqtt_connect_task_func(void *pvParameters)
{
    esp_err_t ret = gophr_mqtt_connect(s_mqtt_uri, s_mqtt_user, s_mqtt_pass);

    /* Give broker a few seconds to establish connection */
    if (ret == ESP_OK) {
        for (int i = 0; i < 30; i++) {
            vTaskDelay(pdMS_TO_TICKS(200));
            if (gophr_mqtt_is_connected()) break;
        }
    }

    if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(100))) {
        if (gophr_mqtt_is_connected()) {
            ESP_LOGI(TAG, "MQTT connected, going to mode select");
            s_state = UI_STATE_MODE_SELECT;
            lv_scr_load_anim(scr_mode, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
        } else {
            lv_label_set_text(mqtt_connecting_label, "Failed! Press to retry");
            lv_obj_set_style_text_color(mqtt_connecting_label, GOPHR_COLOR_ERROR, 0);
            lv_obj_set_style_arc_color(mqtt_connecting_arc, GOPHR_COLOR_ERROR, LV_PART_INDICATOR);
        }
        xSemaphoreGive(g_lvgl_mutex);
    }

    s_mqtt_connect_task = NULL;
    vTaskDelete(NULL);
}

/* ---------- Public API ---------- */

esp_err_t gophr_ui_init(void)
{
    ESP_LOGI(TAG, "Initializing UI");

    gophr_styles_init();

    build_splash_screen();
    build_wifi_scan_screen();
    build_wifi_select_screen();
    build_wifi_password_screen();
    build_wifi_connecting_screen();
    build_mqtt_setup_screen();
    build_mqtt_connecting_screen();
    build_mode_select_screen();
    build_sensor_screen();
    build_summary_screen();
    build_confirmation_screen();
    build_sweep_setup_screen();
    build_sweep_summary_screen();
    build_sweep_running_screen();

    /* Show splash screen */
    lv_scr_load(scr_splash);
    s_state = UI_STATE_SPLASH;

    ESP_LOGI(TAG, "UI initialized with all screens");
    return ESP_OK;
}

void gophr_ui_set_wifi_status(bool connected)
{
    s_wifi_connected = connected;
    if (sensor_wifi_icon) {
        lv_obj_set_style_bg_color(sensor_wifi_icon,
            connected ? GOPHR_COLOR_ACCENT : GOPHR_COLOR_ERROR, 0);
    }
}

void gophr_ui_set_mqtt_status(bool connected)
{
    s_mqtt_connected = connected;
    if (sensor_mqtt_icon) {
        lv_obj_set_style_bg_color(sensor_mqtt_icon,
            connected ? GOPHR_COLOR_ACCENT : GOPHR_COLOR_ERROR, 0);
    }
}

/* ---------- UI State Machine (called every 20ms from ui_task) ---------- */

static uint32_t s_state_timer = 0;
static bool s_btn_was_pressed = false;
static int s_btn_hold_count = 0; /* Track how long button held */

/* Debounced button press detection (returns true once per press) */
static bool button_pressed(lv_indev_t *enc_indev)
{
    lv_indev_data_t data;
    lv_indev_read(enc_indev, &data);

    bool pressed = (data.state == LV_INDEV_STATE_PRESSED);
    bool edge = pressed && !s_btn_was_pressed;

    if (pressed) {
        s_btn_hold_count++;
    } else {
        s_btn_hold_count = 0;
    }

    s_btn_was_pressed = pressed;
    return edge;
}

/* Check if button has been held for a duration (in ticks at 50Hz) */
static bool button_held(int ticks)
{
    return s_btn_hold_count >= ticks;
}

/* Check for button release (returns true once on release) */
static bool button_released(void)
{
    /* A "short press" is when the button was pressed and released
       without being held long enough for a hold action */
    return !s_btn_was_pressed && s_btn_hold_count == 0;
}

/* Get encoder rotation delta */
static int encoder_delta(lv_indev_t *enc_indev)
{
    lv_indev_data_t data;
    lv_indev_read(enc_indev, &data);
    return data.enc_diff;
}

void gophr_ui_run(void)
{
    lv_indev_t *enc = gophr_encoder_get_indev();
    if (!enc) return;

    bool btn = button_pressed(enc);
    int delta = encoder_delta(enc);
    bool held = button_held(25); /* ~500ms hold */

    /* Update status indicators */
    gophr_ui_set_wifi_status(gophr_wifi_is_connected());
    gophr_ui_set_mqtt_status(gophr_mqtt_is_connected());

    switch (s_state) {
    case UI_STATE_SPLASH:
        s_state_timer++;
        /* Animate splash arc (2 seconds at 50Hz = 100 ticks) */
        if (s_state_timer <= 100) {
            int angle = (s_state_timer * 360) / 100;
            lv_arc_set_value(splash_arc, angle);
        }
        if (s_state_timer >= 100) {
            s_state_timer = 0;

            /* Check for saved Wi-Fi credentials */
            if (gophr_wifi_has_saved_creds()) {
                /* Try auto-connect in background */
                lv_label_set_text(wifi_connecting_ssid_label, "Saved network");
                lv_label_set_text(wifi_connecting_label, "Connecting...");
                lv_obj_set_style_text_color(wifi_connecting_label, GOPHR_COLOR_TEXT_DIM, 0);
                lv_obj_set_style_arc_color(wifi_connecting_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
                s_state = UI_STATE_WIFI_CONNECTING;
                lv_scr_load_anim(scr_wifi_connecting, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);

                /* Connect using saved creds in background task */
                xTaskCreate(wifi_connect_saved_task_func, "wifi_conn", 4096, NULL, 3, &s_wifi_connect_task);
            } else {
                /* No saved creds, start Wi-Fi scan */
                s_state = UI_STATE_WIFI_SCANNING;
                lv_scr_load_anim(scr_wifi_scan, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);
                xTaskCreate(wifi_scan_task_func, "wifi_scan", 4096, NULL, 3, &s_wifi_scan_task);
            }
        }
        break;

    /* ---------- Wi-Fi Provisioning ---------- */
    case UI_STATE_WIFI_SCANNING:
        /* Animate the scanning spinner */
        s_state_timer++;
        {
            int rotation = (s_state_timer * 6) % 360;
            lv_arc_set_rotation(wifi_scan_arc, rotation);
        }
        break;

    case UI_STATE_WIFI_SELECT:
        if (delta != 0) {
            s_wifi_selected_idx += (delta > 0) ? 1 : -1;
            if (s_wifi_selected_idx < 0) s_wifi_selected_idx = 0;
            if (s_wifi_selected_idx >= s_scan_count)
                s_wifi_selected_idx = s_scan_count - 1;
            populate_wifi_list();

            /* Scroll to selected item */
            lv_obj_t *child = lv_obj_get_child(wifi_select_list, s_wifi_selected_idx);
            if (child) {
                lv_obj_scroll_to_view(child, LV_ANIM_ON);
            }
            gophr_buzzer_click();
        }

        if (btn && s_scan_count > 0) {
            gophr_buzzer_confirm();

            /* Check if the selected network is open */
            if (s_scan_results[s_wifi_selected_idx].authmode == WIFI_AUTH_OPEN) {
                /* Open network, connect directly */
                memset(s_wifi_password, 0, sizeof(s_wifi_password));
                lv_label_set_text(wifi_connecting_ssid_label,
                    s_scan_results[s_wifi_selected_idx].ssid);
                lv_label_set_text(wifi_connecting_label, "Connecting...");
                lv_obj_set_style_text_color(wifi_connecting_label, GOPHR_COLOR_TEXT_DIM, 0);
                lv_obj_set_style_arc_color(wifi_connecting_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
                s_state = UI_STATE_WIFI_CONNECTING;
                lv_scr_load_anim(scr_wifi_connecting, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                xTaskCreate(wifi_connect_task_func, "wifi_conn", 4096, NULL, 3, &s_wifi_connect_task);
            } else {
                /* Need password */
                memset(s_wifi_password, 0, sizeof(s_wifi_password));
                s_wifi_pass_cursor = 0;
                s_wifi_pass_char_idx = 0;
                lv_label_set_text(wifi_pass_ssid_label,
                    s_scan_results[s_wifi_selected_idx].ssid);
                update_wifi_password_display();
                s_state = UI_STATE_WIFI_PASSWORD;
                lv_scr_load_anim(scr_wifi_pass, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            }
        }
        break;

    case UI_STATE_WIFI_PASSWORD:
        if (delta != 0) {
            /* Rotate through characters */
            s_wifi_pass_char_idx += delta;
            if (s_wifi_pass_char_idx < 0) s_wifi_pass_char_idx = CHARSET_LEN - 1;
            if (s_wifi_pass_char_idx >= (int)CHARSET_LEN) s_wifi_pass_char_idx = 0;
            update_wifi_password_display();
        }

        if (btn) {
            /* Add current character to password */
            int len = strlen(s_wifi_password);
            if (len < WIFI_PASS_MAX_LEN - 1) {
                s_wifi_password[len] = CHARSET[s_wifi_pass_char_idx];
                s_wifi_password[len + 1] = '\0';
                gophr_buzzer_click();
                update_wifi_password_display();
            }
        }

        if (held) {
            /* Hold = done entering password (or backspace if short hold) */
            int len = strlen(s_wifi_password);
            if (len > 0) {
                gophr_buzzer_confirm();
                /* Start connecting */
                lv_label_set_text(wifi_connecting_ssid_label,
                    s_scan_results[s_wifi_selected_idx].ssid);
                lv_label_set_text(wifi_connecting_label, "Connecting...");
                lv_obj_set_style_text_color(wifi_connecting_label, GOPHR_COLOR_TEXT_DIM, 0);
                lv_obj_set_style_arc_color(wifi_connecting_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
                s_state = UI_STATE_WIFI_CONNECTING;
                lv_scr_load_anim(scr_wifi_connecting, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                xTaskCreate(wifi_connect_task_func, "wifi_conn", 4096, NULL, 3, &s_wifi_connect_task);
            }
            /* Reset hold counter so it doesn't trigger again */
            s_btn_hold_count = 0;
        }
        break;

    case UI_STATE_WIFI_CONNECTING:
        /* Animate spinner */
        s_state_timer++;
        {
            int rotation = (s_state_timer * 6) % 360;
            lv_arc_set_rotation(wifi_connecting_arc, rotation);
        }

        /* If connection failed and user presses button, go back to scan */
        if (btn && s_wifi_connect_task == NULL) {
            gophr_buzzer_click();
            s_state_timer = 0;
            s_state = UI_STATE_WIFI_SCANNING;
            lv_label_set_text(wifi_scan_label, "Scanning...");
            lv_scr_load_anim(scr_wifi_scan, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
            xTaskCreate(wifi_scan_task_func, "wifi_scan", 4096, NULL, 3, &s_wifi_scan_task);
        }
        break;

    /* ---------- MQTT Provisioning ---------- */
    case UI_STATE_MQTT_SETUP:
        if (delta != 0) {
            /* Rotate through characters */
            s_mqtt_char_idx += delta;
            if (s_mqtt_char_idx < 0) s_mqtt_char_idx = CHARSET_LEN - 1;
            if (s_mqtt_char_idx >= (int)CHARSET_LEN) s_mqtt_char_idx = 0;
            update_mqtt_setup_display();
        }

        if (btn) {
            /* Add character to current field */
            char *field = NULL;
            int max_len = 0;
            switch (s_mqtt_field) {
            case 0: field = s_mqtt_uri; max_len = MQTT_URI_MAX_LEN - 1; break;
            case 1: field = s_mqtt_user; max_len = MQTT_USER_MAX_LEN - 1; break;
            case 2: field = s_mqtt_pass; max_len = MQTT_PASS_MAX_LEN - 1; break;
            }
            if (field) {
                int len = strlen(field);
                if (len < max_len) {
                    field[len] = CHARSET[s_mqtt_char_idx];
                    field[len + 1] = '\0';
                    gophr_buzzer_click();
                    update_mqtt_setup_display();
                }
            }
        }

        if (held) {
            if (s_mqtt_field < 2) {
                /* Advance to next field */
                s_mqtt_field++;
                s_mqtt_char_idx = 0;
                gophr_buzzer_confirm();
                update_mqtt_setup_display();
            } else {
                /* Last field (password) - start MQTT connection */
                gophr_buzzer_confirm();
                lv_label_set_text(mqtt_connecting_label, "Connecting to broker...");
                lv_obj_set_style_text_color(mqtt_connecting_label, GOPHR_COLOR_TEXT_DIM, 0);
                lv_obj_set_style_arc_color(mqtt_connecting_arc, GOPHR_COLOR_ACCENT, LV_PART_INDICATOR);
                s_state = UI_STATE_MQTT_CONNECTING;
                s_state_timer = 0;
                lv_scr_load_anim(scr_mqtt_connecting, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                xTaskCreate(mqtt_connect_task_func, "mqtt_conn", 4096, NULL, 3, &s_mqtt_connect_task);
            }
            s_btn_hold_count = 0;
        }
        break;

    case UI_STATE_MQTT_CONNECTING:
        /* Animate spinner */
        s_state_timer++;
        {
            int rotation = (s_state_timer * 6) % 360;
            lv_arc_set_rotation(mqtt_connecting_arc, rotation);
        }

        /* If connection failed and user presses, go back to setup */
        if (btn && s_mqtt_connect_task == NULL) {
            gophr_buzzer_click();
            s_state_timer = 0;
            s_mqtt_field = 0;
            s_mqtt_char_idx = 0;
            update_mqtt_setup_display();
            s_state = UI_STATE_MQTT_SETUP;
            lv_scr_load_anim(scr_mqtt_setup, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        }
        break;

    /* ---------- Mode Select ---------- */
    case UI_STATE_MODE_SELECT:
        if (btn) {
            /* Check which button is focused */
            lv_obj_t *focused = lv_group_get_focused(lv_group_get_default());
            if (focused == mode_btn_sweep) {
                gophr_buzzer_confirm();
                s_sweep_setup_field = 0;
                s_sweep_current_sensor = 0;
                update_sweep_setup_screen(0);
                s_state = UI_STATE_SWEEP_SENSOR_0;
                lv_scr_load_anim(scr_sweep_setup, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            } else {
                gophr_buzzer_confirm();
                s_current_sensor = 0;
                update_sensor_screen(0);
                s_state = UI_STATE_SENSOR_0;
                lv_scr_load_anim(scr_sensor, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            }
        }
        break;

    /* ---------- Instant Mode ---------- */
    case UI_STATE_SENSOR_0:
    case UI_STATE_SENSOR_1:
    case UI_STATE_SENSOR_2:
    {
        int idx = s_state - UI_STATE_SENSOR_0;

        if (delta != 0) {
            s_moisture[idx] += delta;
            if (s_moisture[idx] < 0) s_moisture[idx] = 0;
            if (s_moisture[idx] > 100) s_moisture[idx] = 100;
            update_sensor_screen(idx);
        }

        if (btn) {
            gophr_buzzer_confirm();
            if (idx < 2) {
                /* Advance to next sensor */
                s_state = (ui_state_t)(UI_STATE_SENSOR_0 + idx + 1);
                update_sensor_screen(idx + 1);
            } else {
                /* Go to summary */
                update_summary_screen();
                s_state = UI_STATE_SUMMARY;
                lv_scr_load_anim(scr_summary, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            }
        }
        break;
    }

    case UI_STATE_SUMMARY:
        if (btn) {
            gophr_buzzer_send();
            s_state = UI_STATE_SENDING;
        }
        if (delta < 0) {
            /* Go back to last sensor */
            s_state = UI_STATE_SENSOR_2;
            update_sensor_screen(2);
            lv_scr_load_anim(scr_sensor, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        }
        break;

    case UI_STATE_SENDING:
    {
        esp_err_t ret = gophr_mqtt_publish_all(s_moisture[0], s_moisture[1], s_moisture[2]);
        bool success = (ret == ESP_OK);
        show_confirmation(success);

        if (success) {
            gophr_buzzer_send();
        } else {
            gophr_buzzer_error();
        }

        s_state = UI_STATE_CONFIRMATION;
        s_state_timer = 0;
        lv_scr_load_anim(scr_confirm, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
        break;
    }

    case UI_STATE_CONFIRMATION:
        s_state_timer++;
        /* Animate confirmation arc */
        if (s_state_timer <= 25) {
            int angle = (s_state_timer * 360) / 25;
            lv_arc_set_value(confirm_arc, angle);
        }
        /* Return to mode select after 2s (100 ticks at 50Hz) */
        if (s_state_timer >= 100) {
            s_state_timer = 0;
            s_state = UI_STATE_MODE_SELECT;
            lv_scr_load_anim(scr_mode, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);
        }
        break;

    /* ---------- Sweep Mode ---------- */
    case UI_STATE_SWEEP_SENSOR_0:
    case UI_STATE_SWEEP_SENSOR_1:
    case UI_STATE_SWEEP_SENSOR_2:
    {
        int idx = s_state - UI_STATE_SWEEP_SENSOR_0;

        if (delta != 0) {
            if (s_sweep_setup_field == 0) {
                /* Adjusting target moisture */
                s_sweep_target[idx] += delta;
                if (s_sweep_target[idx] < 0) s_sweep_target[idx] = 0;
                if (s_sweep_target[idx] > 100) s_sweep_target[idx] = 100;
            } else {
                /* Adjusting duration */
                s_sweep_duration_idx[idx] += (delta > 0 ? 1 : -1);
                if (s_sweep_duration_idx[idx] < 0) s_sweep_duration_idx[idx] = 0;
                if (s_sweep_duration_idx[idx] >= SWEEP_DURATION_COUNT)
                    s_sweep_duration_idx[idx] = SWEEP_DURATION_COUNT - 1;
            }
            update_sweep_setup_screen(idx);
        }

        if (btn) {
            if (s_sweep_setup_field == 0) {
                /* Switch from target to duration field */
                s_sweep_setup_field = 1;
                gophr_buzzer_click();
                update_sweep_setup_screen(idx);
            } else {
                /* Confirm this sensor, advance */
                s_sweep_setup_field = 0;
                gophr_buzzer_confirm();

                if (idx < 2) {
                    s_state = (ui_state_t)(UI_STATE_SWEEP_SENSOR_0 + idx + 1);
                    update_sweep_setup_screen(idx + 1);
                } else {
                    /* Go to sweep summary */
                    update_sweep_summary_screen();
                    s_state = UI_STATE_SWEEP_SUMMARY;
                    lv_scr_load_anim(scr_sweep_summary, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
                }
            }
        }
        break;
    }

    case UI_STATE_SWEEP_SUMMARY:
        if (btn) {
            gophr_buzzer_confirm();
            /* Start the sweep */
            for (int i = 0; i < 3; i++) {
                lv_arc_set_value(sweep_arcs[i], s_moisture[i]);
                update_arc_color(sweep_arcs[i], s_moisture[i]);
            }
            s_state = UI_STATE_SWEEP_RUNNING;
            lv_scr_load_anim(scr_sweep_running, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);

            /* Launch sweep task */
            xTaskCreate(sweep_task_func, "sweep", 4096, NULL, 3, &s_sweep_task);
        }
        if (delta < 0) {
            /* Go back */
            s_state = UI_STATE_SWEEP_SENSOR_2;
            s_sweep_setup_field = 0;
            update_sweep_setup_screen(2);
            lv_scr_load_anim(scr_sweep_setup, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        }
        break;

    case UI_STATE_SWEEP_RUNNING:
        if (btn) {
            /* Cancel sweep */
            s_sweep_active = false;
            gophr_buzzer_error();
            s_state = UI_STATE_MODE_SELECT;
            lv_scr_load_anim(scr_mode, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, false);
        }
        break;

    case UI_STATE_SWEEP_DONE:
        /* Show success confirmation, then return to mode select */
        show_confirmation(true);
        gophr_buzzer_send();
        s_state = UI_STATE_CONFIRMATION;
        s_state_timer = 0;
        lv_scr_load_anim(scr_confirm, LV_SCR_LOAD_ANIM_FADE_IN, 200, 0, false);
        break;
    }
}
