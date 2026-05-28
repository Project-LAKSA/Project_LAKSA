#pragma once

/* C wrapper around the esp32_BNO08x C++ component.
 *
 * The rest of the firmware stays in C. This adapter owns the C++ BNO08x
 * instance and exposes only the values used by the app.
 */

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} bno08x_adapter_vec3_t;

typedef struct {
    float i;
    float j;
    float k;
    float real;
    float accuracy;
} bno08x_adapter_quat_t;

typedef struct {
    void *driver;
    bool has_acceleration;
    bool has_rotation_vector;
    bool has_game_rotation_vector;
    bno08x_adapter_vec3_t acceleration;
    bno08x_adapter_quat_t rotation_vector;
    bno08x_adapter_quat_t game_rotation_vector;
} bno08x_adapter_t;

esp_err_t bno08x_adapter_init(bno08x_adapter_t *imu);
esp_err_t bno08x_adapter_update(bno08x_adapter_t *imu);
esp_err_t bno08x_adapter_get_acceleration(bno08x_adapter_t *imu, bno08x_adapter_vec3_t *acceleration);
esp_err_t bno08x_adapter_get_rotation_vector(bno08x_adapter_t *imu, bno08x_adapter_quat_t *quaternion);
esp_err_t bno08x_adapter_get_game_rotation_vector(bno08x_adapter_t *imu, bno08x_adapter_quat_t *quaternion);

#ifdef __cplusplus
}
#endif
