#include "gophr_zigbee.h"
#include "gophr_drivers.h"

#include "esp_log.h"
#include "esp_check.h"
#include "ha/esp_zigbee_ha_standard.h"

static const char *TAG = "gophr_zigbee";
static bool s_joined = false;

/* ---------- Helpers ---------- */

static int16_t celsius_to_zigbee(float temp)
{
    return (int16_t)(temp * 100.0f);
}

static uint16_t percent_to_zigbee_humidity(float pct)
{
    return (uint16_t)(pct * 100.0f);
}

/* ---------- Create Endpoints & Clusters ---------- */

static esp_zb_cluster_list_t *create_temperature_endpoint_clusters(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03, /* Battery */
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)GOPHR_MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)GOPHR_MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Identify cluster */
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list,
        esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Temperature measurement cluster */
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = celsius_to_zigbee(25.0f),
        .min_value = celsius_to_zigbee(-10.0f),
        .max_value = celsius_to_zigbee(80.0f),
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list,
        esp_zb_temperature_meas_cluster_create(&temp_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Power configuration cluster */
    esp_zb_power_config_cluster_cfg_t power_cfg = {0};
    esp_zb_attribute_list_t *power_cluster = esp_zb_power_config_cluster_create(&power_cfg);

    /* Add battery voltage attribute (uint8, units of 100mV) */
    uint8_t bat_voltage = 37; /* 3.7V default */
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_cluster,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &bat_voltage));

    /* Add battery percentage remaining (uint8, half-% units, 200=100%) */
    uint8_t bat_pct = 200;
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(power_cluster,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &bat_pct));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(cluster_list,
        power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}

static esp_zb_cluster_list_t *create_humidity_endpoint_clusters(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster (minimal) */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list,
        esp_zb_basic_cluster_create(&basic_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Identify cluster */
    esp_zb_identify_cluster_cfg_t identify_cfg = {.identify_time = 0};
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list,
        esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Humidity measurement cluster */
    esp_zb_humidity_meas_cluster_cfg_t hum_cfg = {
        .measured_value = percent_to_zigbee_humidity(50.0f),
        .min_value = percent_to_zigbee_humidity(0.0f),
        .max_value = percent_to_zigbee_humidity(100.0f),
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list,
        esp_zb_humidity_meas_cluster_create(&hum_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}

esp_err_t gophr_zigbee_create_device(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    /* Endpoint 1: Temperature + Power Config */
    esp_zb_endpoint_config_t temp_ep_cfg = {
        .endpoint = GOPHR_EP_TEMP,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, create_temperature_endpoint_clusters(), temp_ep_cfg);

    /* Endpoint 2: AHT20 Humidity */
    esp_zb_endpoint_config_t hum_ep_cfg = {
        .endpoint = GOPHR_EP_HUMIDITY,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, create_humidity_endpoint_clusters(), hum_ep_cfg);

    /* Endpoints 3-5: Soil Moisture (as humidity %) */
    uint8_t moisture_eps[] = {GOPHR_EP_MOISTURE_1, GOPHR_EP_MOISTURE_2, GOPHR_EP_MOISTURE_3};
    for (int i = 0; i < 3; i++) {
        esp_zb_endpoint_config_t moist_ep_cfg = {
            .endpoint = moisture_eps[i],
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, create_humidity_endpoint_clusters(), moist_ep_cfg);
    }

    /* Register device */
    esp_zb_device_register(ep_list);

    /* Configure reporting for temperature */
    esp_zb_zcl_reporting_info_t temp_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = GOPHR_EP_TEMP,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = GOPHR_REPORT_MIN_INTERVAL,
        .u.send_info.max_interval = GOPHR_REPORT_MAX_INTERVAL,
        .u.send_info.def_min_interval = GOPHR_REPORT_MIN_INTERVAL,
        .u.send_info.def_max_interval = GOPHR_REPORT_MAX_INTERVAL,
        .u.send_info.delta.u16 = GOPHR_TEMP_REPORT_DELTA,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&temp_report);

    /* Configure reporting for humidity (endpoint 2) */
    esp_zb_zcl_reporting_info_t hum_report = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = GOPHR_EP_HUMIDITY,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = GOPHR_REPORT_MIN_INTERVAL,
        .u.send_info.max_interval = GOPHR_REPORT_MAX_INTERVAL,
        .u.send_info.def_min_interval = GOPHR_REPORT_MIN_INTERVAL,
        .u.send_info.def_max_interval = GOPHR_REPORT_MAX_INTERVAL,
        .u.send_info.delta.u16 = GOPHR_HUMIDITY_REPORT_DELTA,
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&hum_report);

    /* Configure reporting for each moisture endpoint */
    for (int i = 0; i < 3; i++) {
        esp_zb_zcl_reporting_info_t moist_report = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .ep = moisture_eps[i],
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .u.send_info.min_interval = GOPHR_REPORT_MIN_INTERVAL,
            .u.send_info.max_interval = GOPHR_REPORT_MAX_INTERVAL,
            .u.send_info.def_min_interval = GOPHR_REPORT_MIN_INTERVAL,
            .u.send_info.def_max_interval = GOPHR_REPORT_MAX_INTERVAL,
            .u.send_info.delta.u16 = GOPHR_HUMIDITY_REPORT_DELTA,
            .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };
        esp_zb_zcl_update_reporting_info(&moist_report);
    }

    ESP_LOGI(TAG, "Zigbee device registered with 5 endpoints");
    return ESP_OK;
}

/* ---------- Attribute Updates ---------- */

void gophr_zigbee_update_temperature(float celsius)
{
    int16_t val = celsius_to_zigbee(celsius);
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(GOPHR_EP_TEMP,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &val, false);
    esp_zb_lock_release();
}

void gophr_zigbee_update_humidity(float percent)
{
    uint16_t val = percent_to_zigbee_humidity(percent);
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(GOPHR_EP_HUMIDITY,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        &val, false);
    esp_zb_lock_release();
}

void gophr_zigbee_update_moisture(int sensor_index, float percent)
{
    if (sensor_index < 0 || sensor_index >= 3) return;

    uint8_t ep = GOPHR_EP_MOISTURE_1 + sensor_index;
    uint16_t val = percent_to_zigbee_humidity(percent);

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(ep,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        &val, false);
    esp_zb_lock_release();
}

void gophr_zigbee_update_battery(float voltage, float percent)
{
    /* Battery voltage in 100mV units */
    uint8_t bat_v = (uint8_t)(voltage * 10.0f);
    /* Battery percentage in half-percent units (200 = 100%) */
    uint8_t bat_pct = (uint8_t)(percent * 2.0f);

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(GOPHR_EP_TEMP,
        ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
        &bat_v, false);
    esp_zb_zcl_set_attribute_val(GOPHR_EP_TEMP,
        ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        &bat_pct, false);
    esp_zb_lock_release();
}

void gophr_zigbee_report_all(void)
{
    /* Report temperature */
    esp_zb_zcl_report_attr_cmd_t report = {0};
    report.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;

    esp_zb_lock_acquire(portMAX_DELAY);

    /* Temperature */
    report.zcl_basic_cmd.src_endpoint = GOPHR_EP_TEMP;
    report.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    report.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
    esp_zb_zcl_report_attr_cmd_req(&report);

    /* Humidity */
    report.zcl_basic_cmd.src_endpoint = GOPHR_EP_HUMIDITY;
    report.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
    report.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
    esp_zb_zcl_report_attr_cmd_req(&report);

    /* Moisture 1-3 */
    for (int i = 0; i < 3; i++) {
        report.zcl_basic_cmd.src_endpoint = GOPHR_EP_MOISTURE_1 + i;
        report.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
        report.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
        esp_zb_zcl_report_attr_cmd_req(&report);
    }

    esp_zb_lock_release();
    ESP_LOGI(TAG, "Reported all attributes");
}

/* ---------- Network Signal Handler ---------- */

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

void gophr_zigbee_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode",
                     esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted, already on network");
                s_joined = true;
                /* Set LED green to indicate connected */
                gophr_led_set_color(0, 76, 0); /* ~30% green */
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s), retrying...",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, "
                     "PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            s_joined = true;
            gophr_led_set_color(0, 76, 0); /* Green = connected */
        } else {
            ESP_LOGI(TAG, "Network steering failed (status: %s), retrying...",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

bool gophr_zigbee_is_joined(void)
{
    return s_joined;
}
