#include <stdio.h>

#include "bno08x_adapter.h"
#include "esp32_config.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "micro_ros_bridge.h"
#include "pca9685_i2c.h"
#include "steering_control.h"
#include "vesc_uart.h"
#if CONFIG_LAKSA_WEB_DASHBOARD
#include "web_dashboard.h"
#endif

#define IMU_UPDATE_TASK_INTERVAL_MS 10

typedef struct {
    bno08x_adapter_t *imu;
    SemaphoreHandle_t hardware_mutex;
} imu_update_task_context_t;

static void imu_update_task(void *arg)
{
    imu_update_task_context_t *context = (imu_update_task_context_t *)arg;
    esp_err_t last_error = ESP_OK;

    while (true) {
        xSemaphoreTake(context->hardware_mutex, portMAX_DELAY);
        esp_err_t err = bno08x_adapter_update(context->imu);
        xSemaphoreGive(context->hardware_mutex);

        if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
            if (err != last_error) {
                printf("BNO08x update failed: %s\n", esp_err_to_name(err));
            }
            last_error = err;
        } else if (err == ESP_OK) {
            last_error = ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_UPDATE_TASK_INTERVAL_MS));
    }
}

void app_main(void)
{
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_dev_handle_t pca9685_i2c_handle;

    pca9685_t pca9685;
    bno08x_adapter_t imu;
    static steering_control_t steering;
    static vesc_uart_t vesc;
#if CONFIG_LAKSA_WEB_DASHBOARD
    static web_dashboard_t dashboard;
#endif
    static imu_update_task_context_t imu_task_context;
    micro_ros_bridge_config_t ros_config;
    SemaphoreHandle_t hardware_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(hardware_mutex == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    ESP_ERROR_CHECK(i2c_bus_init(&i2c_bus_handle));
    ESP_ERROR_CHECK(i2c_bus_add_device(i2c_bus_handle, PCA9685_I2C_ADDR, &pca9685_i2c_handle));
    ESP_ERROR_CHECK(pca9685_init(&pca9685, pca9685_i2c_handle, SERVO_FREQ_HZ));
    ESP_ERROR_CHECK(steering_control_init(&steering, &pca9685, hardware_mutex));
    ESP_ERROR_CHECK(vesc_uart_init(&vesc));

    esp_err_t imu_err = bno08x_adapter_init(&imu);
    if (imu_err != ESP_OK) {
        printf("BNO08x init failed: %s; dashboard will continue without IMU data\n",
               esp_err_to_name(imu_err));
    } else {
        imu_task_context = (imu_update_task_context_t){
            .imu = &imu,
            .hardware_mutex = hardware_mutex,
        };
        xTaskCreate(imu_update_task, "imu_update", 4096, &imu_task_context, 5, NULL);
    }

#if CONFIG_LAKSA_WEB_DASHBOARD
    dashboard = (web_dashboard_t) {
        .steering = &steering,
        .vesc = &vesc,
        .imu = imu_err == ESP_OK ? &imu : NULL,
        .hardware_mutex = hardware_mutex,
    };
#endif

    ros_config = (micro_ros_bridge_config_t){
        .steering = &steering,
        .vesc = &vesc,
        .imu = imu_err == ESP_OK ? &imu : NULL,
        .hardware_mutex = hardware_mutex,
    };
    esp_err_t ros_err = micro_ros_bridge_start(&ros_config);
    if (ros_err != ESP_OK) {
        printf("micro-ROS start failed: %s\n", esp_err_to_name(ros_err));
    }

#if CONFIG_LAKSA_WEB_DASHBOARD
    esp_err_t dashboard_err = web_dashboard_start(&dashboard);
    if (dashboard_err != ESP_OK) {
        printf("Dashboard start failed: %s\n", esp_err_to_name(dashboard_err));
    }
#endif

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
