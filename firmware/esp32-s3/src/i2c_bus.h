#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus_handle);
esp_err_t i2c_bus_add_device(i2c_master_bus_handle_t bus_handle,
                             uint16_t device_address,
                             i2c_master_dev_handle_t *dev_handle);

