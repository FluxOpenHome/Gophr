#include "gophr_matter.h"
#include "gophr_drivers.h"

#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <esp_matter_attribute.h>

#include <app/server/Server.h>
#include <platform/CHIPDeviceLayer.h>
#include <app/clusters/temperature-measurement-server/temperature-measurement-server.h>

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace chip::app::Clusters;

static const char *TAG = "gophr_matter";

/* Stored endpoint IDs (assigned at creation) */
static uint16_t s_temp_ep_id = 0;
static uint16_t s_humidity_ep_id = 0;
static uint16_t s_moisture_ep_ids[3] = {0};
static bool s_connected = false;

/* ---------- Matter Event Callback ---------- */

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        s_connected = true;
        gophr_led_set_color(0, 76, 0); /* Green = connected */
        break;
    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
        if (event->ThreadConnectivityChange.Result == chip::DeviceLayer::kConnectivity_Established) {
            ESP_LOGI(TAG, "Thread network connected");
            s_connected = true;
            gophr_led_set_color(0, 76, 0);
        } else {
            ESP_LOGW(TAG, "Thread network disconnected");
            s_connected = false;
            gophr_led_set_color(0, 0, 128); /* Blue = disconnected */
        }
        break;
    default:
        break;
    }
}

/* ---------- Attribute Update Callback ---------- */

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type,
                                          uint16_t endpoint_id,
                                          uint32_t cluster_id,
                                          uint32_t attribute_id,
                                          esp_matter_attr_val_t *val,
                                          void *priv_data)
{
    /* Read-only sensor device — no writable attributes from controller */
    return ESP_OK;
}

/* ---------- Init ---------- */

esp_err_t gophr_matter_init(void)
{
    /* Create the Matter node */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, NULL);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return ESP_FAIL;
    }

    /* --- Endpoint 1: Temperature Sensor (AHT20 temp) --- */
    {
        temperature_sensor::config_t temp_config;
        temp_config.temperature_measurement.measured_value = nullable<int16_t>(2500);   /* 25.0°C */
        temp_config.temperature_measurement.min_measured_value = nullable<int16_t>(-1000); /* -10°C */
        temp_config.temperature_measurement.max_measured_value = nullable<int16_t>(8000);  /* 80°C */

        endpoint_t *ep = temperature_sensor::create(node, &temp_config, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create temperature endpoint");
            return ESP_FAIL;
        }
        s_temp_ep_id = endpoint::get_id(ep);

        /* Add Power Source cluster to temperature endpoint for battery info */
        power_source::config_t power_cfg;
        power_cfg.power_source.status = 1;  /* Active */
        power_cfg.power_source.order = 0;
        power_cfg.power_source.description = "Battery";
        power_cfg.power_source.battery.bat_charge_level = 0; /* OK */
        power_cfg.power_source.features.battery = true;
        power_cfg.power_source.features.rechargeable = true;

        cluster_t *power_cluster = power_source::create(ep, &power_cfg, CLUSTER_FLAG_SERVER);
        if (power_cluster) {
            /* Add battery percentage attribute */
            uint8_t bat_pct = 200; /* 100% in half-percent units */
            attribute::create(power_cluster, PowerSource::Attributes::BatPercentRemaining::Id,
                              ATTRIBUTE_FLAG_NULLABLE, esp_matter_nullable_uint8(bat_pct));

            /* Add battery voltage attribute (millivolts) */
            uint32_t bat_mv = 3700;
            attribute::create(power_cluster, PowerSource::Attributes::BatVoltage::Id,
                              ATTRIBUTE_FLAG_NULLABLE, esp_matter_nullable_uint32(bat_mv));
        }

        ESP_LOGI(TAG, "Temperature endpoint created (ID: %d)", s_temp_ep_id);
    }

    /* --- Endpoint 2: Humidity Sensor (AHT20 humidity) --- */
    {
        humidity_sensor::config_t hum_config;
        hum_config.relative_humidity_measurement.measured_value = nullable<uint16_t>(5000); /* 50.0% */
        hum_config.relative_humidity_measurement.min_measured_value = nullable<uint16_t>(0);
        hum_config.relative_humidity_measurement.max_measured_value = nullable<uint16_t>(10000); /* 100% */

        endpoint_t *ep = humidity_sensor::create(node, &hum_config, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create humidity endpoint");
            return ESP_FAIL;
        }
        s_humidity_ep_id = endpoint::get_id(ep);
        ESP_LOGI(TAG, "Humidity endpoint created (ID: %d)", s_humidity_ep_id);
    }

    /* --- Endpoints 3-5: Humidity Sensors (soil moisture 1-3) --- */
    for (int i = 0; i < 3; i++) {
        humidity_sensor::config_t moist_config;
        moist_config.relative_humidity_measurement.measured_value = nullable<uint16_t>(0);
        moist_config.relative_humidity_measurement.min_measured_value = nullable<uint16_t>(0);
        moist_config.relative_humidity_measurement.max_measured_value = nullable<uint16_t>(10000);

        endpoint_t *ep = humidity_sensor::create(node, &moist_config, ENDPOINT_FLAG_NONE, NULL);
        if (!ep) {
            ESP_LOGE(TAG, "Failed to create moisture %d endpoint", i + 1);
            return ESP_FAIL;
        }
        s_moisture_ep_ids[i] = endpoint::get_id(ep);
        ESP_LOGI(TAG, "Moisture %d endpoint created (ID: %d)", i + 1, s_moisture_ep_ids[i]);
    }

    /* Register device event callback */
    chip::DeviceLayer::PlatformMgrImpl().AddEventHandler(app_event_cb, 0);

    ESP_LOGI(TAG, "Matter device initialized with 5 endpoints");
    return ESP_OK;
}

/* ---------- Attribute Updates ---------- */

void gophr_matter_update_temperature(float celsius)
{
    int16_t matter_temp = (int16_t)(celsius * 100.0f);
    esp_matter_attr_val_t val = esp_matter_nullable_int16(matter_temp);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([val]() mutable {
        attribute::update(s_temp_ep_id,
                          TemperatureMeasurement::Id,
                          TemperatureMeasurement::Attributes::MeasuredValue::Id,
                          &val);
    });
}

void gophr_matter_update_humidity(float percent)
{
    uint16_t matter_hum = (uint16_t)(percent * 100.0f);
    esp_matter_attr_val_t val = esp_matter_nullable_uint16(matter_hum);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([val]() mutable {
        attribute::update(s_humidity_ep_id,
                          RelativeHumidityMeasurement::Id,
                          RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
                          &val);
    });
}

void gophr_matter_update_moisture(int sensor_index, float percent)
{
    if (sensor_index < 0 || sensor_index >= 3) return;

    uint16_t matter_hum = (uint16_t)(percent * 100.0f);
    uint16_t ep_id = s_moisture_ep_ids[sensor_index];
    esp_matter_attr_val_t val = esp_matter_nullable_uint16(matter_hum);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([ep_id, val]() mutable {
        attribute::update(ep_id,
                          RelativeHumidityMeasurement::Id,
                          RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
                          &val);
    });
}

void gophr_matter_update_battery(float voltage, float percent)
{
    /* Battery percentage in half-percent units (200 = 100%) */
    uint8_t bat_pct = (uint8_t)(percent * 2.0f);
    /* Battery voltage in millivolts */
    uint32_t bat_mv = (uint32_t)(voltage * 1000.0f);

    esp_matter_attr_val_t pct_val = esp_matter_nullable_uint8(bat_pct);
    esp_matter_attr_val_t mv_val = esp_matter_nullable_uint32(bat_mv);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([pct_val, mv_val]() mutable {
        attribute::update(s_temp_ep_id,
                          PowerSource::Id,
                          PowerSource::Attributes::BatPercentRemaining::Id,
                          &pct_val);
        attribute::update(s_temp_ep_id,
                          PowerSource::Id,
                          PowerSource::Attributes::BatVoltage::Id,
                          &mv_val);
    });
}

bool gophr_matter_is_connected(void)
{
    return s_connected;
}
