#include "gophr_mqtt.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "gophr_wifi.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "gophr_mqtt";

#define NVS_NAMESPACE   "gophr_mqtt"

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;

/* HA MQTT discovery payload template */
static const char *DISCOVERY_TEMPLATE =
    "{"
    "\"name\":\"Moisture %d Percentage\","
    "\"unique_id\":\"gophr_sim_moisture_%d\","
    "\"state_topic\":\"gophr_sim/sensor/moisture_%d/state\","
    "\"availability_topic\":\"gophr_sim/status\","
    "\"unit_of_measurement\":\"%%\","
    "\"device_class\":\"humidity\","
    "\"state_class\":\"measurement\","
    "\"icon\":\"mdi:water-percent\","
    "\"device\":{"
        "\"identifiers\":[\"gophr_sim\"],"
        "\"name\":\"Gophr Simulator\","
        "\"manufacturer\":\"GOPHR\","
        "\"model\":\"Gophr-Sim\","
        "\"sw_version\":\"1.0.0\""
    "}"
    "}";

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected to broker");
        s_connected = true;
        esp_mqtt_client_publish(s_client, "gophr_sim/status", "online", 0, 1, 1);
        gophr_mqtt_publish_discovery();
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_connected = false;
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

/* Save MQTT credentials to NVS */
static esp_err_t save_creds_to_nvs(const char *uri, const char *user, const char *pass)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    nvs_set_str(handle, "uri", uri);
    nvs_set_str(handle, "user", user);
    nvs_set_str(handle, "pass", pass);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "MQTT credentials saved to NVS");
    return ESP_OK;
}

/* Load MQTT credentials from NVS */
static bool load_creds_from_nvs(char *uri, size_t uri_len,
                                char *user, size_t user_len,
                                char *pass, size_t pass_len)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return false;

    ret = nvs_get_str(handle, "uri", uri, &uri_len);
    if (ret != ESP_OK || strlen(uri) == 0) {
        nvs_close(handle);
        return false;
    }

    /* User and pass can be empty */
    nvs_get_str(handle, "user", user, &user_len);
    nvs_get_str(handle, "pass", pass, &pass_len);

    nvs_close(handle);
    return true;
}

static esp_err_t start_client(const char *uri, const char *user, const char *pass)
{
    /* Stop existing client if any */
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        s_connected = false;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
        .session.last_will = {
            .topic = "gophr_sim/status",
            .msg = "offline",
            .msg_len = 7,
            .qos = 1,
            .retain = 1,
        },
    };

    if (user && strlen(user) > 0) {
        mqtt_cfg.credentials.username = user;
    }
    if (pass && strlen(pass) > 0) {
        mqtt_cfg.credentials.authentication.password = pass;
    }

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_client) return ESP_FAIL;

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
        mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_client));

    ESP_LOGI(TAG, "MQTT client started, broker: %s", uri);
    return ESP_OK;
}

esp_err_t gophr_mqtt_init(void)
{
    ESP_LOGI(TAG, "MQTT subsystem initialized (not connected)");
    return ESP_OK;
}

esp_err_t gophr_mqtt_connect(const char *broker_uri, const char *username, const char *password)
{
    ESP_LOGI(TAG, "Connecting to MQTT broker: %s", broker_uri);

    esp_err_t ret = start_client(broker_uri, username, password);
    if (ret == ESP_OK) {
        save_creds_to_nvs(broker_uri, username ? username : "", password ? password : "");
    }
    return ret;
}

bool gophr_mqtt_connect_saved(void)
{
    char uri[MQTT_URI_MAX_LEN] = {0};
    char user[MQTT_USER_MAX_LEN] = {0};
    char pass[MQTT_PASS_MAX_LEN] = {0};

    if (!load_creds_from_nvs(uri, sizeof(uri), user, sizeof(user), pass, sizeof(pass))) {
        ESP_LOGI(TAG, "No saved MQTT credentials");
        return false;
    }

    ESP_LOGI(TAG, "Found saved MQTT config for '%s', connecting...", uri);
    start_client(uri, user, pass);
    return true;
}

bool gophr_mqtt_is_connected(void)
{
    return s_connected;
}

bool gophr_mqtt_has_saved_creds(void)
{
    char uri[MQTT_URI_MAX_LEN] = {0};
    char user[MQTT_USER_MAX_LEN] = {0};
    char pass[MQTT_PASS_MAX_LEN] = {0};
    return load_creds_from_nvs(uri, sizeof(uri), user, sizeof(user), pass, sizeof(pass));
}

esp_err_t gophr_mqtt_publish_discovery(void)
{
    if (!s_connected) return ESP_ERR_INVALID_STATE;

    char topic[80];
    char payload[512];

    for (int i = 1; i <= 3; i++) {
        snprintf(topic, sizeof(topic),
                 "homeassistant/sensor/gophr_sim_moisture_%d/config", i);
        snprintf(payload, sizeof(payload), DISCOVERY_TEMPLATE, i, i, i);

        int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
        if (msg_id < 0) {
            ESP_LOGE(TAG, "Failed to publish discovery for sensor %d", i);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Published HA discovery for moisture_%d", i);
    }

    return ESP_OK;
}

esp_err_t gophr_mqtt_publish_moisture(int index, int percent)
{
    if (!s_connected) return ESP_ERR_INVALID_STATE;
    if (index < 0 || index > 2) return ESP_ERR_INVALID_ARG;

    char topic[64];
    char value[8];

    snprintf(topic, sizeof(topic), "gophr_sim/sensor/moisture_%d/state", index + 1);
    snprintf(value, sizeof(value), "%d", percent);

    int msg_id = esp_mqtt_client_publish(s_client, topic, value, 0, 0, 0);
    if (msg_id < 0) return ESP_FAIL;

    ESP_LOGI(TAG, "Published moisture_%d = %d%%", index + 1, percent);
    return ESP_OK;
}

esp_err_t gophr_mqtt_publish_all(int m1, int m2, int m3)
{
    esp_err_t ret = ESP_OK;
    int values[] = {m1, m2, m3};

    for (int i = 0; i < 3; i++) {
        esp_err_t r = gophr_mqtt_publish_moisture(i, values[i]);
        if (r != ESP_OK) ret = r;
    }

    return ret;
}
