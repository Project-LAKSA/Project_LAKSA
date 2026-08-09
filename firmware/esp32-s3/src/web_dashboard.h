#pragma once

/* Embedded WiFi dashboard.
 *
 * This layer connects the ESP32 to the configured WiFi network, serves the
 * browser UI, exposes JSON state, and receives steering and VESC commands.
 */

#include "bno08x_adapter.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "steering_control.h"
#include "vesc_uart.h"

typedef struct {
    steering_control_t *steering;
    vesc_uart_t *vesc;
    bno08x_adapter_t *imu;
    SemaphoreHandle_t hardware_mutex;
} web_dashboard_t;

esp_err_t web_dashboard_start(web_dashboard_t *dashboard);
