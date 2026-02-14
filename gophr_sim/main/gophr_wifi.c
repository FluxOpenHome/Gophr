#include "gophr_wifi.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <string.h>

static const char *TAG = "gophr_wifi";

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRY      5
#define NVS_NAMESPACE       "gophr_wifi"

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_count = 0;
static bool s_connected = false;
static bool s_connecting = false;
static char s_current_ssid[WIFI_SSID_MAX_LEN] = {0};

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (s_connecting) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        if (s_connecting && s_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGI(TAG, "Retry connection (%d/%d)", s_retry_count, WIFI_MAX_RETRY);
        } else {
            s_connecting = false;
            if (s_wifi_event_group) {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        s_connected = true;
        s_connecting = false;
        if (s_wifi_event_group) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

/* Save Wi-Fi credentials to NVS */
static esp_err_t save_creds_to_nvs(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    nvs_set_str(handle, "ssid", ssid);
    nvs_set_str(handle, "pass", password);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Wi-Fi credentials saved to NVS");
    return ESP_OK;
}

/* Load Wi-Fi credentials from NVS */
static bool load_creds_from_nvs(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return false;

    ret = nvs_get_str(handle, "ssid", ssid, &ssid_len);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    ret = nvs_get_str(handle, "pass", pass, &pass_len);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    return strlen(ssid) > 0;
}

esp_err_t gophr_wifi_init(void)
{
    ESP_LOGI(TAG, "Initializing Wi-Fi subsystem");

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi subsystem ready (not connected)");
    return ESP_OK;
}

int gophr_wifi_scan(wifi_scan_entry_t *results, int max_results)
{
    ESP_LOGI(TAG, "Starting Wi-Fi scan...");

    wifi_scan_config_t scan_cfg = {
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    esp_err_t ret = esp_wifi_scan_start(&scan_cfg, true); /* Blocking scan */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scan failed: %s", esp_err_to_name(ret));
        return 0;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count == 0) {
        ESP_LOGI(TAG, "No APs found");
        return 0;
    }

    uint16_t fetch_count = ap_count;
    if (fetch_count > max_results) fetch_count = max_results;

    wifi_ap_record_t *ap_records = calloc(fetch_count, sizeof(wifi_ap_record_t));
    if (!ap_records) return 0;

    esp_wifi_scan_get_ap_records(&fetch_count, ap_records);

    /* Copy to our simpler struct, dedup by SSID */
    int count = 0;
    for (int i = 0; i < fetch_count && count < max_results; i++) {
        if (strlen((char *)ap_records[i].ssid) == 0) continue;

        /* Check for duplicate SSID */
        bool dup = false;
        for (int j = 0; j < count; j++) {
            if (strcmp(results[j].ssid, (char *)ap_records[i].ssid) == 0) {
                dup = true;
                break;
            }
        }
        if (dup) continue;

        strncpy(results[count].ssid, (char *)ap_records[i].ssid, WIFI_SSID_MAX_LEN - 1);
        results[count].rssi = ap_records[i].rssi;
        results[count].authmode = ap_records[i].authmode;
        count++;
    }

    free(ap_records);
    ESP_LOGI(TAG, "Scan complete: %d unique networks found", count);
    return count;
}

esp_err_t gophr_wifi_connect(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Connecting to '%s'...", ssid);

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    /* If password is empty, allow open networks */
    if (strlen(password) == 0) {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    s_retry_count = 0;
    s_connecting = true;
    s_connected = false;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    /* Disconnect first if already connected */
    esp_wifi_disconnect();
    esp_wifi_connect();

    /* Wait for connection result (10 second timeout) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));

    if (bits & WIFI_CONNECTED_BIT) {
        strncpy(s_current_ssid, ssid, WIFI_SSID_MAX_LEN - 1);
        save_creds_to_nvs(ssid, password);
        ESP_LOGI(TAG, "Connected to '%s'", ssid);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Failed to connect to '%s'", ssid);
    s_connecting = false;
    return ESP_FAIL;
}

bool gophr_wifi_connect_saved(void)
{
    char ssid[WIFI_SSID_MAX_LEN] = {0};
    char pass[WIFI_PASS_MAX_LEN] = {0};

    if (!load_creds_from_nvs(ssid, sizeof(ssid), pass, sizeof(pass))) {
        ESP_LOGI(TAG, "No saved Wi-Fi credentials");
        return false;
    }

    ESP_LOGI(TAG, "Found saved credentials for '%s', connecting...", ssid);
    gophr_wifi_connect(ssid, pass);
    return true;
}

bool gophr_wifi_is_connected(void)
{
    return s_connected;
}

void gophr_wifi_get_ssid(char *buf, size_t len)
{
    strncpy(buf, s_current_ssid, len - 1);
    buf[len - 1] = '\0';
}

bool gophr_wifi_has_saved_creds(void)
{
    char ssid[WIFI_SSID_MAX_LEN] = {0};
    char pass[WIFI_PASS_MAX_LEN] = {0};
    return load_creds_from_nvs(ssid, sizeof(ssid), pass, sizeof(pass));
}
