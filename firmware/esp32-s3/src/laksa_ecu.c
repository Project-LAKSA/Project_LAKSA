#include "laksa_ecu.h"

#include <string.h>

static esp_err_t laksa_ecu_take(laksa_ecu_t *ecu)
{
    if (ecu == NULL || ecu->lock == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(ecu->lock, portMAX_DELAY);
    return ESP_OK;
}

static void laksa_ecu_give(laksa_ecu_t *ecu)
{
    xSemaphoreGive(ecu->lock);
}

static laksa_servo_state_t *laksa_ecu_servo_for_channel(laksa_ecu_t *ecu, uint8_t channel)
{
    if (channel == 13) {
        return &ecu->servo13;
    }
    if (channel == 15) {
        return &ecu->servo15;
    }

    return NULL;
}

esp_err_t laksa_ecu_init(laksa_ecu_t *ecu)
{
    if (ecu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ecu, 0, sizeof(*ecu));
    ecu->lock = xSemaphoreCreateMutex();
    if (ecu->lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t laksa_ecu_set_servo_target(laksa_ecu_t *ecu, uint8_t channel, uint8_t angle_deg)
{
    if (angle_deg > 180) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = laksa_ecu_take(ecu);
    if (err != ESP_OK) {
        return err;
    }

    laksa_servo_state_t *servo = laksa_ecu_servo_for_channel(ecu, channel);
    if (servo == NULL) {
        laksa_ecu_give(ecu);
        return ESP_ERR_INVALID_ARG;
    }

    servo->target_angle_deg = angle_deg;
    laksa_ecu_give(ecu);
    return ESP_OK;
}

esp_err_t laksa_ecu_set_servo_current(laksa_ecu_t *ecu, uint8_t channel, uint8_t angle_deg)
{
    if (angle_deg > 180) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = laksa_ecu_take(ecu);
    if (err != ESP_OK) {
        return err;
    }

    laksa_servo_state_t *servo = laksa_ecu_servo_for_channel(ecu, channel);
    if (servo == NULL) {
        laksa_ecu_give(ecu);
        return ESP_ERR_INVALID_ARG;
    }

    servo->current_angle_deg = angle_deg;
    laksa_ecu_give(ecu);
    return ESP_OK;
}

esp_err_t laksa_ecu_get_servo_targets(laksa_ecu_t *ecu, uint8_t *servo13_angle, uint8_t *servo15_angle)
{
    if (servo13_angle == NULL || servo15_angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = laksa_ecu_take(ecu);
    if (err != ESP_OK) {
        return err;
    }

    *servo13_angle = ecu->servo13.target_angle_deg;
    *servo15_angle = ecu->servo15.target_angle_deg;
    laksa_ecu_give(ecu);
    return ESP_OK;
}

esp_err_t laksa_ecu_set_imu(laksa_ecu_t *ecu, const laksa_imu_state_t *imu)
{
    if (imu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = laksa_ecu_take(ecu);
    if (err != ESP_OK) {
        return err;
    }

    ecu->imu = *imu;
    laksa_ecu_give(ecu);
    return ESP_OK;
}

esp_err_t laksa_ecu_get_snapshot(laksa_ecu_t *ecu, laksa_ecu_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = laksa_ecu_take(ecu);
    if (err != ESP_OK) {
        return err;
    }

    snapshot->servo13 = ecu->servo13;
    snapshot->servo15 = ecu->servo15;
    snapshot->imu = ecu->imu;
    laksa_ecu_give(ecu);
    return ESP_OK;
}
