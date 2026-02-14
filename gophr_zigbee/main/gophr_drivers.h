#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* ---------- GPIO Pin Definitions (matching C3 PCB) ---------- */

/* ADC Inputs */
#define GPIO_MOISTURE_1         1
#define GPIO_MOISTURE_2         2
#define GPIO_MOISTURE_3         3
#define GPIO_BATTERY_VOLTAGE    0
#define GPIO_SOLAR_VOLTAGE      4

/* Digital Outputs - Power Enable */
#define GPIO_SENSOR_ENABLE      6
#define GPIO_AHT20_ENABLE       7
#define GPIO_LED_ENABLE         18

/* LED */
#define GPIO_STATUS_LED         10

/* I2C */
#define GPIO_I2C_SDA            19
#define GPIO_I2C_SCL            9

/* AHT20 */
#define AHT20_I2C_ADDR          0x38

/* ---------- ADC ---------- */

esp_err_t gophr_adc_init(void);
int gophr_adc_read_raw(int gpio_num);
float gophr_adc_read_voltage(int gpio_num);

/* ---------- I2C / AHT20 ---------- */

esp_err_t gophr_i2c_init(void);
esp_err_t gophr_aht20_read(float *temperature, float *humidity);

/* ---------- GPIO Power Control ---------- */

esp_err_t gophr_gpio_init(void);
void gophr_sensor_power(bool enable);
void gophr_aht20_power(bool enable);
void gophr_led_power(bool enable);

/* ---------- WS2812B LED ---------- */

esp_err_t gophr_led_init(void);
void gophr_led_set_color(uint8_t r, uint8_t g, uint8_t b);
void gophr_led_off(void);
