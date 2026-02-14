#include "gophr_drivers.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/i2c_master.h"
#include "led_strip.h"
#include "esp_log.h"

#include <string.h>
#include <math.h>

static const char *TAG = "gophr_drivers";

/* ---------- ADC ---------- */

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;

/* Map GPIO number to ADC channel for ESP32-C6 ADC1 */
static adc_channel_t gpio_to_adc_channel(int gpio_num)
{
    switch (gpio_num) {
    case 0: return ADC_CHANNEL_0;
    case 1: return ADC_CHANNEL_1;
    case 2: return ADC_CHANNEL_2;
    case 3: return ADC_CHANNEL_3;
    case 4: return ADC_CHANNEL_4;
    default:
        ESP_LOGE(TAG, "Invalid ADC GPIO: %d", gpio_num);
        return ADC_CHANNEL_0;
    }
}

esp_err_t gophr_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &s_adc_handle), TAG, "ADC unit init failed");

    /* Configure all 5 ADC channels with 12dB attenuation (0-3.3V range) */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    int adc_gpios[] = {
        GPIO_MOISTURE_1, GPIO_MOISTURE_2, GPIO_MOISTURE_3,
        GPIO_BATTERY_VOLTAGE, GPIO_SOLAR_VOLTAGE
    };

    for (int i = 0; i < 5; i++) {
        adc_channel_t ch = gpio_to_adc_channel(adc_gpios[i]);
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, ch, &chan_cfg),
                            TAG, "ADC channel %d config failed", adc_gpios[i]);
    }

    /* Set up calibration */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration not available, using raw values");
        s_adc_cali_handle = NULL;
    }

    ESP_LOGI(TAG, "ADC initialized (5 channels, 12-bit, 12dB atten)");
    return ESP_OK;
}

int gophr_adc_read_raw(int gpio_num)
{
    if (!s_adc_handle) return -1;

    int raw = 0;
    adc_channel_t ch = gpio_to_adc_channel(gpio_num);
    esp_err_t ret = adc_oneshot_read(s_adc_handle, ch, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read GPIO%d failed: %s", gpio_num, esp_err_to_name(ret));
        return -1;
    }
    return raw;
}

float gophr_adc_read_voltage(int gpio_num)
{
    int raw = gophr_adc_read_raw(gpio_num);
    if (raw < 0) return NAN;

    if (s_adc_cali_handle) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_adc_cali_handle, raw, &mv) == ESP_OK) {
            return (float)mv / 1000.0f;
        }
    }

    /* Fallback: linear approximation for 12-bit, 12dB attenuation */
    return ((float)raw / 4095.0f) * 3.3f;
}

/* ---------- I2C / AHT20 ---------- */

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_aht20_dev = NULL;

esp_err_t gophr_i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "I2C bus init failed");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT20_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_aht20_dev),
                        TAG, "AHT20 device add failed");

    /* Send init command to AHT20 */
    vTaskDelay(pdMS_TO_TICKS(40));
    uint8_t init_cmd[] = {0xBE, 0x08, 0x00};
    esp_err_t ret = i2c_master_transmit(s_aht20_dev, init_cmd, sizeof(init_cmd), 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AHT20 init command failed (may already be initialized)");
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "I2C initialized (SDA=%d, SCL=%d), AHT20 at 0x%02X",
             GPIO_I2C_SDA, GPIO_I2C_SCL, AHT20_I2C_ADDR);
    return ESP_OK;
}

esp_err_t gophr_aht20_read(float *temperature, float *humidity)
{
    if (!s_aht20_dev) return ESP_ERR_INVALID_STATE;

    /* Trigger measurement */
    uint8_t trigger_cmd[] = {0xAC, 0x33, 0x00};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_aht20_dev, trigger_cmd, sizeof(trigger_cmd), 100),
                        TAG, "AHT20 trigger failed");

    /* Wait for measurement (AHT20 needs ~80ms) */
    vTaskDelay(pdMS_TO_TICKS(80));

    /* Read 7 bytes: status + 20-bit humidity + 20-bit temperature + CRC */
    uint8_t data[7] = {0};
    ESP_RETURN_ON_ERROR(i2c_master_receive(s_aht20_dev, data, sizeof(data), 100),
                        TAG, "AHT20 read failed");

    /* Check if busy */
    if (data[0] & 0x80) {
        ESP_LOGW(TAG, "AHT20 still busy");
        return ESP_ERR_NOT_FINISHED;
    }

    /* Parse humidity (20-bit) */
    uint32_t raw_hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((data[3] >> 4) & 0x0F);
    *humidity = ((float)raw_hum / 1048576.0f) * 100.0f;

    /* Parse temperature (20-bit) */
    uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    *temperature = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;

    ESP_LOGD(TAG, "AHT20: temp=%.1f°C, humidity=%.1f%%", *temperature, *humidity);
    return ESP_OK;
}

/* ---------- GPIO Power Control ---------- */

esp_err_t gophr_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_SENSOR_ENABLE) |
                        (1ULL << GPIO_AHT20_ENABLE) |
                        (1ULL << GPIO_LED_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "GPIO config failed");

    /* Start with all power rails off */
    gpio_set_level(GPIO_SENSOR_ENABLE, 0);
    gpio_set_level(GPIO_AHT20_ENABLE, 0);
    gpio_set_level(GPIO_LED_ENABLE, 0);

    ESP_LOGI(TAG, "GPIO power pins initialized (6, 7, 18)");
    return ESP_OK;
}

void gophr_sensor_power(bool enable)
{
    gpio_set_level(GPIO_SENSOR_ENABLE, enable ? 1 : 0);
    ESP_LOGI(TAG, "Sensor power %s", enable ? "ON" : "OFF");
}

void gophr_aht20_power(bool enable)
{
    gpio_set_level(GPIO_AHT20_ENABLE, enable ? 1 : 0);
    ESP_LOGI(TAG, "AHT20 power %s", enable ? "ON" : "OFF");
}

void gophr_led_power(bool enable)
{
    gpio_set_level(GPIO_LED_ENABLE, enable ? 1 : 0);
    ESP_LOGI(TAG, "LED power %s", enable ? "ON" : "OFF");
}

/* ---------- WS2812B LED ---------- */

static led_strip_handle_t s_led_strip = NULL;

esp_err_t gophr_led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = GPIO_STATUS_LED,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, /* 10 MHz */
        .flags.with_dma = false,
    };

    ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led_strip),
                        TAG, "LED strip init failed");

    led_strip_clear(s_led_strip);
    ESP_LOGI(TAG, "WS2812B LED initialized on GPIO%d", GPIO_STATUS_LED);
    return ESP_OK;
}

void gophr_led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led_strip) return;
    led_strip_set_pixel(s_led_strip, 0, r, g, b);
    led_strip_refresh(s_led_strip);
}

void gophr_led_off(void)
{
    if (!s_led_strip) return;
    led_strip_clear(s_led_strip);
}
