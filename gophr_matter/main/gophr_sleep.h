#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default sleep configuration */
#define GOPHR_DEFAULT_SLEEP_DURATION_MIN    60
#define GOPHR_DEFAULT_MIN_AWAKE_MIN         1
#define GOPHR_DEFAULT_MAX_AWAKE_MIN         120
#define GOPHR_DEFAULT_SLEEP_DISABLED        true

/* Initialize sleep subsystem, load config from NVS */
esp_err_t gophr_sleep_init(void);

/* Check if it's time to sleep (call periodically) */
void gophr_sleep_check(void);

/* Force immediate sleep */
void gophr_sleep_now(void);

/* Configuration getters/setters (persisted to NVS) */
int gophr_sleep_get_duration(void);
void gophr_sleep_set_duration(int minutes);

int gophr_sleep_get_min_awake(void);
void gophr_sleep_set_min_awake(int minutes);

int gophr_sleep_get_max_awake(void);
void gophr_sleep_set_max_awake(int minutes);

bool gophr_sleep_is_disabled(void);
void gophr_sleep_set_disabled(bool disabled);

/* Get time awake in seconds */
uint32_t gophr_sleep_get_awake_seconds(void);

/* Check if a sleep sequence is currently active */
bool gophr_sleep_sequence_active(void);

#ifdef __cplusplus
}
#endif
