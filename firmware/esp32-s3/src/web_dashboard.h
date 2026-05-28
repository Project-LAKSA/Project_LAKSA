#pragma once

/* Embedded WiFi dashboard.
 *
 * This layer connects the ESP32 to the configured WiFi network, serves the
 * browser UI, exposes JSON state, and receives servo commands. It uses the
 * already-initialized PCA9685 and BNO08x instances owned by main.
 */

#include "bno08x_adapter.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "pca9685_i2c.h"

typedef struct {
    pca9685_t *pca9685;
    bno08x_adapter_t *imu;
    SemaphoreHandle_t hardware_mutex;
    uint8_t servo13_angle;
    uint8_t servo15_angle;
} web_dashboard_t;

esp_err_t web_dashboard_start(web_dashboard_t *dashboard);
