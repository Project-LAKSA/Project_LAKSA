#pragma once

/* LAKSA ESP32 electronic control unit state.
 *
 * This is the central data store for values owned by the ESP32. Read data is
 * produced by sensor tasks. Write data is produced by interfaces such as the
 * web dashboard and consumed by actuator/control tasks.
 */

#include <stdbool.h>
#include <stdint.h>

#include "bno08x_adapter.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    uint8_t target_angle_deg;
    uint8_t current_angle_deg;
} laksa_servo_state_t;

typedef struct {
    bool has_accel;
    bool has_rotation;
    bool has_game_rotation;
    bno08x_adapter_vec3_t accel;
    bno08x_adapter_quat_t rotation;
    bno08x_adapter_quat_t game_rotation;
} laksa_imu_state_t;

typedef struct {
    laksa_servo_state_t servo13;
    laksa_servo_state_t servo15;
    laksa_imu_state_t imu;
    SemaphoreHandle_t lock;
} laksa_ecu_t;

typedef struct {
    laksa_servo_state_t servo13;
    laksa_servo_state_t servo15;
    laksa_imu_state_t imu;
} laksa_ecu_snapshot_t;

esp_err_t laksa_ecu_init(laksa_ecu_t *ecu);
esp_err_t laksa_ecu_set_servo_target(laksa_ecu_t *ecu, uint8_t channel, uint8_t angle_deg);
esp_err_t laksa_ecu_set_servo_current(laksa_ecu_t *ecu, uint8_t channel, uint8_t angle_deg);
esp_err_t laksa_ecu_get_servo_targets(laksa_ecu_t *ecu, uint8_t *servo13_angle, uint8_t *servo15_angle);
esp_err_t laksa_ecu_set_imu(laksa_ecu_t *ecu, const laksa_imu_state_t *imu);
esp_err_t laksa_ecu_get_snapshot(laksa_ecu_t *ecu, laksa_ecu_snapshot_t *snapshot);
