#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pca9685_i2c.h"

typedef struct {
    uint8_t target_angle_deg;
    uint8_t current_angle_deg;
    bool endpoint_relief_active;
} steering_snapshot_t;

typedef struct {
    pca9685_t *pca9685;
    SemaphoreHandle_t hardware_mutex;
    SemaphoreHandle_t state_lock;
    TaskHandle_t task;
    uint8_t target_angle_deg;
    uint8_t current_angle_deg;
    TickType_t endpoint_entered_tick;
    bool endpoint_timer_active;
    bool endpoint_relief_active;
    bool returning_to_center;
} steering_control_t;

esp_err_t steering_control_init(steering_control_t *steering,
                                pca9685_t *pca9685,
                                SemaphoreHandle_t hardware_mutex);
esp_err_t steering_control_set_target(steering_control_t *steering, uint8_t angle_deg);
esp_err_t steering_control_center(steering_control_t *steering);
esp_err_t steering_control_get_snapshot(steering_control_t *steering,
                                        steering_snapshot_t *snapshot);
