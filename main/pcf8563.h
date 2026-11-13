#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t pcf8563_init(void);
esp_err_t pcf8563_read_local_epoch(uint32_t *epoch_seconds);
esp_err_t pcf8563_write_local_epoch(uint32_t epoch_seconds);
