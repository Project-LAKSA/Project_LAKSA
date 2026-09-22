#pragma once

#include "bno08x_adapter.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "pca9685_i2c.h"
#include "steering_control.h"
#include "vesc_uart.h"

typedef struct {
    steering_control_t *steering;
    pca9685_t *pca9685;
    vesc_uart_t *vesc;
    bno08x_adapter_t *imu;
    SemaphoreHandle_t hardware_mutex;
} micro_ros_bridge_config_t;

esp_err_t micro_ros_bridge_start(const micro_ros_bridge_config_t *config);
