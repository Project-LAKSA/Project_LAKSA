#include "pca9685_i2c.h"
#include "esp32_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCA9685_MODE1_REG_ADDR 0x00
#define PCA9685_PRESCALE_REG_ADDR 0xFE
#define PCA9685_LED0_ON_L_REG_ADDR 0x06

/* MODE1 bit 7. After waking the PCA9685 from sleep, this asks the chip to
 * restart its PWM generator so the outputs run with the freshly configured
 * timing.
 */
#define PCA9685_MODE1_RESTART 0x80

/* MODE1 bit 5, Auto-Increment. With this enabled we can send the first PWM
 * register address plus four data bytes, and the PCA9685 automatically writes
 * them into ON_L, ON_H, OFF_L, and OFF_H in order.
 */
#define PCA9685_MODE1_AI 0x20

/* MODE1 bit 4. The datasheet expects the oscillator to be stopped before
 * changing PRESCALE, so we briefly put the chip to sleep, write PRESCALE, then
 * wake it and restart PWM generation.
 */
#define PCA9685_MODE1_SLEEP 0x10

/* PCA9685 defaults to a 25 MHz oscillator and uses a 12-bit PWM counter. */
#define PCA9685_OSC_CLOCK_HZ 25000000UL
#define PCA9685_PWM_RESOLUTION 4096UL
#define PCA9685_PWM_MAX_TICK (PCA9685_PWM_RESOLUTION - 1)
#define PCA9685_REGS_PER_CHANNEL 4
#define PCA9685_MAX_CHANNEL 15

static const char *TAG = "pca9685";

static esp_err_t pca9685_i2c_write_u8(pca9685_t *pca9685, uint8_t reg, uint8_t value)
{
    uint8_t data[] = {reg, value};
    return i2c_master_transmit(pca9685->i2c_dev, data, sizeof(data), I2C_TRANSMIT_TIMEOUT_MS);
}

static esp_err_t pca9685_i2c_read_u8(pca9685_t *pca9685, uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(
        pca9685->i2c_dev,
        &reg,
        1,
        value,
        1,
        I2C_TRANSMIT_TIMEOUT_MS);
}

static uint8_t pca9685_calculate_prescale(uint16_t freq_hz)
{
    uint32_t ticks_per_second = PCA9685_PWM_RESOLUTION * freq_hz;
    uint32_t rounded_prescale = (PCA9685_OSC_CLOCK_HZ + (ticks_per_second / 2)) / ticks_per_second;

    if (rounded_prescale == 0) {
        return 0;
    }

    return (uint8_t)(rounded_prescale - 1);
}

static esp_err_t pca9685_set_pwm_freq(pca9685_t *pca9685, uint16_t freq_hz)
{
    uint8_t old_mode = 0;

    /* Reading the current MODE1 register value */
    esp_err_t err = pca9685_i2c_read_u8(pca9685, PCA9685_MODE1_REG_ADDR, &old_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MODE1 read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Setting to sleep mode to change the PCA9685 clock frequency */
    uint8_t sleep_mode = (old_mode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    err = pca9685_i2c_write_u8(pca9685, PCA9685_MODE1_REG_ADDR, sleep_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sleep failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t prescale = pca9685_calculate_prescale(freq_hz);
    err = pca9685_i2c_write_u8(pca9685, PCA9685_PRESCALE_REG_ADDR, prescale);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "prescale failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Waking up the PCA9685 by setting the Auto-Increment bit before restarting */
    err = pca9685_i2c_write_u8(pca9685, PCA9685_MODE1_REG_ADDR, old_mode | PCA9685_MODE1_AI);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wake failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Wait for the oscillator to stabilize after waking up */
    vTaskDelay(pdMS_TO_TICKS(5));
    return pca9685_i2c_write_u8(
        pca9685,
        PCA9685_MODE1_REG_ADDR,
        old_mode | PCA9685_MODE1_AI | PCA9685_MODE1_RESTART);
}

esp_err_t pca9685_init(pca9685_t *pca9685, i2c_master_dev_handle_t i2c_dev, uint16_t pwm_freq_hz)
{
    pca9685->i2c_dev = i2c_dev;

    esp_err_t err = pca9685_i2c_write_u8(pca9685, PCA9685_MODE1_REG_ADDR, PCA9685_MODE1_AI);
    if (err != ESP_OK) {
        return err;
    }

    return pca9685_set_pwm_freq(pca9685, pwm_freq_hz);
}

esp_err_t pca9685_set_pwm(pca9685_t *pca9685, uint8_t channel, uint16_t on_tick, uint16_t off_tick)
{
    if (channel > PCA9685_MAX_CHANNEL || on_tick > PCA9685_PWM_MAX_TICK || off_tick > PCA9685_PWM_MAX_TICK) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = PCA9685_LED0_ON_L_REG_ADDR + (PCA9685_REGS_PER_CHANNEL * channel);
    uint8_t data[] = {
        reg,
        on_tick & 0xFF,  /* LEDn_ON_L byte  */
        on_tick >> 8,    /* LEDn_ON_H byte  */
        off_tick & 0xFF, /* LEDn_OFF_L byte */
        off_tick >> 8,   /* LEDn_OFF_H byte */
    };

    return i2c_master_transmit(pca9685->i2c_dev, data, sizeof(data), I2C_TRANSMIT_TIMEOUT_MS);
}

esp_err_t pca9685_set_servo_duty_cycle_us(pca9685_t *pca9685, uint8_t channel, uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US) {
        pulse_us = SERVO_MIN_US;
    } else if (pulse_us > SERVO_MAX_US) {
        pulse_us = SERVO_MAX_US;
    }

    uint32_t period_us = 1000000UL / SERVO_FREQ_HZ;
    uint16_t ticks = (uint16_t)((pulse_us * PCA9685_PWM_RESOLUTION + (period_us / 2)) / period_us);

    return pca9685_set_pwm(pca9685, channel, 0, ticks);
}

esp_err_t pca9685_set_servo_angle(pca9685_t *pca9685, uint8_t channel, uint8_t angle)
{
    if (angle > 180) {
        angle = 180;
    }

    uint16_t pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle / 180);
    return pca9685_set_servo_duty_cycle_us(pca9685, channel, pulse_us);
}
