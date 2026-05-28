#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} pca9685_t;

esp_err_t pca9685_init(pca9685_t *pca9685, i2c_master_dev_handle_t i2c_dev, uint16_t pwm_freq_hz);
esp_err_t pca9685_set_pwm(pca9685_t *pca9685, uint8_t channel, uint16_t on_tick, uint16_t off_tick);
esp_err_t pca9685_set_servo_duty_cycle_us(pca9685_t *pca9685, uint8_t channel, uint16_t pulse_us);
esp_err_t pca9685_set_servo_angle(pca9685_t *pca9685, uint8_t channel, uint8_t angle);

