#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

typedef struct {
    int32_t requested_rpm;
    int32_t active_rpm;
    bool command_fresh;
    bool direction_change_pending;
    bool telemetry_fresh;
    float measured_rpm;
    float motor_current;
    float input_current;
    float duty_cycle;
    float input_voltage;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    float temp_mosfet;
    float temp_motor;
    float pid_position;
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t controller_id;
    uint8_t fault_code;
} vesc_uart_snapshot_t;

typedef struct {
    SemaphoreHandle_t lock;
    TaskHandle_t task;
    void *serial_port;
    void *driver;
    int32_t requested_rpm;
    int32_t active_rpm;
    TickType_t last_command_tick;
    TickType_t direction_change_tick;
    TickType_t last_telemetry_tick;
    bool has_received_command;
    bool direction_change_pending;
    bool has_received_telemetry;
    float measured_rpm;
    float motor_current;
    float input_current;
    float duty_cycle;
    float input_voltage;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    float temp_mosfet;
    float temp_motor;
    float pid_position;
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t controller_id;
    uint8_t fault_code;
} vesc_uart_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t vesc_uart_init(vesc_uart_t *vesc);
esp_err_t vesc_uart_set_target_rpm(vesc_uart_t *vesc, int32_t rpm);
esp_err_t vesc_uart_get_snapshot(vesc_uart_t *vesc, vesc_uart_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
