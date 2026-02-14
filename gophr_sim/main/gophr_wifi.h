#pragma once

#include "esp_err.h"
#include "esp_wifi_types.h"
#include <stdbool.h>

#define WIFI_SCAN_MAX_AP    16
#define WIFI_SSID_MAX_LEN   33
#define WIFI_PASS_MAX_LEN   65

/* Scan result entry */
typedef struct {
    char ssid[WIFI_SSID_MAX_LEN];
    int8_t rssi;
    wifi_auth_mode_t authmode;
} wifi_scan_entry_t;

/* Initialize Wi-Fi subsystem (STA mode, does NOT auto-connect) */
esp_err_t gophr_wifi_init(void);

/* Scan for available networks. Returns number of APs found. */
int gophr_wifi_scan(wifi_scan_entry_t *results, int max_results);

/* Connect to a network. Saves credentials to NVS on success. */
esp_err_t gophr_wifi_connect(const char *ssid, const char *password);

/* Try to connect using saved NVS credentials. Returns true if creds exist. */
bool gophr_wifi_connect_saved(void);

/* Check if connected to AP with IP */
bool gophr_wifi_is_connected(void);

/* Get the currently connected SSID (empty string if not connected) */
void gophr_wifi_get_ssid(char *buf, size_t len);

/* Check if saved credentials exist in NVS */
bool gophr_wifi_has_saved_creds(void);
