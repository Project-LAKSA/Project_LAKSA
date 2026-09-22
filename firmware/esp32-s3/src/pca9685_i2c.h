#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint16_t configured_pwm_freq_hz;
    int64_t last_successful_write_us;
    int64_t last_successful_read_us;
    uint32_t consecutive_i2c_errors;
    uint32_t total_i2c_errors;
    uint32_t reinitialization_count;
    esp_err_t last_i2c_error;
    uint16_t last_servo_pulse_us;
    uint16_t last_pwm_off_tick;
    uint8_t last_pwm_channel;
    uint8_t mode1;
    uint8_t prescale;
    bool initialized;
    bool responsive;
    bool configuration_matches;
} pca9685_t;

typedef struct {
    uint16_t configured_pwm_freq_hz;
    int64_t last_successful_write_us;
    int64_t last_successful_read_us;
    uint32_t consecutive_i2c_errors;
    uint32_t total_i2c_errors;
    uint32_t reinitialization_count;
    esp_err_t last_i2c_error;
    uint16_t last_servo_pulse_us;
    uint16_t last_pwm_off_tick;
    uint8_t last_pwm_channel;
    uint8_t mode1;
    uint8_t prescale;
    bool initialized;
    bool responsive;
    bool configuration_matches;
} pca9685_diagnostics_t;

esp_err_t pca9685_init(pca9685_t *pca9685, i2c_master_dev_handle_t i2c_dev, uint16_t pwm_freq_hz);
esp_err_t pca9685_set_pwm(pca9685_t *pca9685, uint8_t channel, uint16_t on_tick, uint16_t off_tick);
esp_err_t pca9685_set_servo_duty_cycle_us(pca9685_t *pca9685, uint8_t channel, uint16_t pulse_us);
esp_err_t pca9685_set_servo_angle(pca9685_t *pca9685, uint8_t channel, uint8_t angle);
esp_err_t pca9685_poll_diagnostics(pca9685_t *pca9685, pca9685_diagnostics_t *diagnostics);
