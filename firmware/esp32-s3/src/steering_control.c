#include "steering_control.h"

#include <string.h>

#include "esp32_config.h"
#include "esp_log.h"

#define STEERING_TASK_STACK_SIZE 3072
#define STEERING_TASK_PRIORITY 5
#define STEERING_ENDPOINT_NEAR_DEG 3

static const char *TAG = "steering";

static bool steering_is_near_left(uint8_t angle_deg)
{
    return angle_deg <= STEERING_LEFT_SAFE_DEG + STEERING_ENDPOINT_NEAR_DEG;
}

static bool steering_is_near_right(uint8_t angle_deg)
{
    return angle_deg >= STEERING_RIGHT_SAFE_DEG - STEERING_ENDPOINT_NEAR_DEG;
}

static bool steering_is_near_endpoint(uint8_t angle_deg)
{
    return steering_is_near_left(angle_deg) || steering_is_near_right(angle_deg);
}

static esp_err_t steering_write_angle(steering_control_t *steering, uint8_t angle_deg)
{
    if (steering->hardware_mutex != NULL) {
        xSemaphoreTake(steering->hardware_mutex, portMAX_DELAY);
    }

    esp_err_t err = pca9685_set_servo_angle(steering->pca9685, STEERING_PCA_CHANNEL, angle_deg);

    if (steering->hardware_mutex != NULL) {
        xSemaphoreGive(steering->hardware_mutex);
    }
    return err;
}

static void steering_apply_endpoint_protection(steering_control_t *steering, TickType_t now)
{
    if (!steering_is_near_endpoint(steering->target_angle_deg)) {
        steering->endpoint_timer_active = false;
        return;
    }

    if (!steering->endpoint_timer_active) {
        steering->endpoint_timer_active = true;
        steering->endpoint_entered_tick = now;
        return;
    }

    if (now - steering->endpoint_entered_tick < pdMS_TO_TICKS(STEERING_MAX_ENDPOINT_HOLD_MS)) {
        return;
    }

    if (steering_is_near_left(steering->target_angle_deg)) {
        steering->target_angle_deg = STEERING_LEFT_SAFE_DEG + STEERING_ENDPOINT_RELIEF_DEG;
    } else {
        steering->target_angle_deg = STEERING_RIGHT_SAFE_DEG - STEERING_ENDPOINT_RELIEF_DEG;
    }
    steering->endpoint_timer_active = false;
    steering->endpoint_relief_active = true;
    ESP_LOGI(TAG, "Endpoint relieved to %u degrees", steering->target_angle_deg);
}

static void steering_task(void *arg)
{
    steering_control_t *steering = (steering_control_t *)arg;

    while (true) {
        uint8_t next_angle;
        TickType_t now = xTaskGetTickCount();

        xSemaphoreTake(steering->state_lock, portMAX_DELAY);
        steering_apply_endpoint_protection(steering, now);

        uint8_t step = STEERING_NORMAL_STEP_DEG;
        if (steering->returning_to_center) {
            step = STEERING_RETURN_STEP_DEG;
        } else if (steering_is_near_endpoint(steering->target_angle_deg)) {
            step = STEERING_ENDPOINT_STEP_DEG;
        }

        if (steering->current_angle_deg < steering->target_angle_deg) {
            uint16_t candidate = steering->current_angle_deg + step;
            steering->current_angle_deg = candidate > steering->target_angle_deg
                                              ? steering->target_angle_deg
                                              : (uint8_t)candidate;
        } else if (steering->current_angle_deg > steering->target_angle_deg) {
            int candidate = steering->current_angle_deg - step;
            steering->current_angle_deg = candidate < steering->target_angle_deg
                                              ? steering->target_angle_deg
                                              : (uint8_t)candidate;
        }

        if (steering->current_angle_deg == steering->target_angle_deg) {
            steering->returning_to_center = false;
        }
        next_angle = steering->current_angle_deg;
        xSemaphoreGive(steering->state_lock);

        esp_err_t err = steering_write_angle(steering, next_angle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "PCA9685 write failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(STEERING_UPDATE_INTERVAL_MS));
    }
}

esp_err_t steering_control_init(steering_control_t *steering,
                                pca9685_t *pca9685,
                                SemaphoreHandle_t hardware_mutex)
{
    if (steering == NULL || pca9685 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(steering, 0, sizeof(*steering));
    steering->pca9685 = pca9685;
    steering->hardware_mutex = hardware_mutex;
    steering->target_angle_deg = STEERING_CENTER_DEG;
    steering->current_angle_deg = STEERING_CENTER_DEG;
    steering->state_lock = xSemaphoreCreateMutex();
    if (steering->state_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = steering_write_angle(steering, STEERING_CENTER_DEG);
    if (err != ESP_OK) {
        return err;
    }

    if (xTaskCreate(steering_task,
                    "steering",
                    STEERING_TASK_STACK_SIZE,
                    steering,
                    STEERING_TASK_PRIORITY,
                    &steering->task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "PCA9685 channel %d centered at %d degrees (safe range %d..%d)",
             STEERING_PCA_CHANNEL,
             STEERING_CENTER_DEG,
             STEERING_LEFT_SAFE_DEG,
             STEERING_RIGHT_SAFE_DEG);
    return ESP_OK;
}

esp_err_t steering_control_set_target(steering_control_t *steering, uint8_t angle_deg)
{
    if (steering == NULL || steering->state_lock == NULL ||
        angle_deg < STEERING_LEFT_SAFE_DEG || angle_deg > STEERING_RIGHT_SAFE_DEG) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(steering->state_lock, portMAX_DELAY);
    steering->target_angle_deg = angle_deg;
    steering->returning_to_center = angle_deg == STEERING_CENTER_DEG;
    steering->endpoint_relief_active = false;
    if (!steering_is_near_endpoint(angle_deg)) {
        steering->endpoint_timer_active = false;
    }
    xSemaphoreGive(steering->state_lock);
    return ESP_OK;
}

esp_err_t steering_control_center(steering_control_t *steering)
{
    return steering_control_set_target(steering, STEERING_CENTER_DEG);
}

esp_err_t steering_control_get_snapshot(steering_control_t *steering,
                                        steering_snapshot_t *snapshot)
{
    if (steering == NULL || steering->state_lock == NULL || snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(steering->state_lock, portMAX_DELAY);
    snapshot->target_angle_deg = steering->target_angle_deg;
    snapshot->current_angle_deg = steering->current_angle_deg;
    snapshot->endpoint_relief_active = steering->endpoint_relief_active;
    xSemaphoreGive(steering->state_lock);
    return ESP_OK;
}
