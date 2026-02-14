#pragma once

#include "esp_err.h"

/* Initialize buzzer (LEDC PWM on GPIO3) */
esp_err_t gophr_buzzer_init(void);

/* Short click sound (encoder detent) */
void gophr_buzzer_click(void);

/* Rising two-tone (screen advance / confirm) */
void gophr_buzzer_confirm(void);

/* Three quick chirps (MQTT sent) */
void gophr_buzzer_send(void);

/* Low descending tone (error) */
void gophr_buzzer_error(void);
