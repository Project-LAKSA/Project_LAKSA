#include <stdio.h>

#include "bn008x.h"
#if defined(BN008X_USE_I2C)
#include "bn008x_i2c.h"
#endif
#if defined(BN008X_USE_SPI)
#include "bn008x_spi.h"
#endif
#include "esp32_config.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "pca9685_i2c.h"

#define IMU_READ_ATTEMPTS 200
#define IMU_PRINT_INTERVAL_MS 1000

void app_main(void)
{
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_dev_handle_t pca9685_i2c_handle;
#if defined(BN008X_USE_I2C)
    i2c_master_dev_handle_t imu_i2c_handle;
#endif

    pca9685_t pca9685;
    bn008x_t imu;
#if defined(BN008X_USE_SPI)
    bn008x_spi_t imu_spi;
#endif

    ESP_ERROR_CHECK(i2c_bus_init(&i2c_bus_handle));
    ESP_ERROR_CHECK(i2c_bus_add_device(i2c_bus_handle, PCA9685_I2C_ADDR, &pca9685_i2c_handle));
    ESP_ERROR_CHECK(pca9685_init(&pca9685, pca9685_i2c_handle, SERVO_FREQ_HZ));

    for (uint8_t angle = 0; angle <= 180; angle += 10) {
        ESP_ERROR_CHECK(pca9685_set_servo_angle(&pca9685, 13, angle));
        ESP_ERROR_CHECK(pca9685_set_servo_angle(&pca9685, 15, angle));
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (uint8_t angle = 180; angle > 0; angle -= 10) {
        ESP_ERROR_CHECK(pca9685_set_servo_angle(&pca9685, 13, angle));
        ESP_ERROR_CHECK(pca9685_set_servo_angle(&pca9685, 15, angle));
        vTaskDelay(pdMS_TO_TICKS(50));
    }

#if defined(BN008X_USE_SPI)
    esp_err_t imu_err = bn008x_spi_bus_init();
    if (imu_err == ESP_OK) {
        imu_err = bn008x_spi_add_device(&imu_spi);
    }
    if (imu_err == ESP_OK) {
        imu_err = bn008x_init_spi(&imu,
                                  imu_spi.spi_dev,
                                  imu_spi.int_gpio,
                                  imu_spi.rst_gpio,
                                  imu_spi.wake_gpio);
    }
#elif defined(BN008X_USE_I2C)
    esp_err_t imu_err = i2c_bus_add_device(i2c_bus_handle, BN008X_I2C_ADDR, &imu_i2c_handle);
    if (imu_err == ESP_OK) {
        imu_err = bn008x_init_i2c(&imu, imu_i2c_handle);
    }
#else
#error "Select BN008X_USE_SPI or BN008X_USE_I2C in bn008x.h."
#endif

    if (imu_err == ESP_OK) {
        bn008x_product_id_t product_id = {0};
        esp_err_t product_err = bn008x_get_product_id(&imu, &product_id);
        if (product_err == ESP_OK) {
            printf("BNO08x Product ID: reset=%u part=%lu build=%lu patch=%u\n",
                   product_id.reset_cause,
                   (unsigned long)product_id.sw_part_number,
                   (unsigned long)product_id.sw_build_number,
                   product_id.sw_version_patch);
        } else {
            printf("BNO08x Product ID failed: %s\n", esp_err_to_name(product_err));
        }

        esp_err_t err = ESP_OK;
        for (uint16_t attempt = 0; attempt < IMU_READ_ATTEMPTS; attempt++) {
            err = bn008x_update(&imu);
            if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                printf("BNO08x update failed: %s\n", esp_err_to_name(err));
                break;
            }
            if (imu.has_acceleration &&
                imu.has_rotation_vector &&
                imu.has_game_rotation_vector) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        bn008x_vec3_t accel = {0};
        bn008x_quat_t rotation = {0};
        bn008x_quat_t game_rotation = {0};
        bool has_accel = bn008x_get_acceleration(&imu, &accel) == ESP_OK;
        bool has_rotation = bn008x_get_rotation_vector(&imu, &rotation) == ESP_OK;
        bool has_game_rotation = bn008x_get_game_rotation_vector(&imu, &game_rotation) == ESP_OK;

        while (true) {
            bool imu_updated = false;
            for (uint16_t elapsed_ms = 0; elapsed_ms < IMU_PRINT_INTERVAL_MS; elapsed_ms += 10) {
                err = bn008x_update(&imu);
                if (err == ESP_OK) {
                    imu_updated = true;
                } else if (err != ESP_ERR_NOT_FOUND) {
                    printf("BNO08x update failed: %s\n", esp_err_to_name(err));
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (!imu_updated) {
                printf("BNO08x no new data\n");
                continue;
            }

            has_accel = bn008x_get_acceleration(&imu, &accel) == ESP_OK;
            has_rotation = bn008x_get_rotation_vector(&imu, &rotation) == ESP_OK;
            has_game_rotation = bn008x_get_game_rotation_vector(&imu, &game_rotation) == ESP_OK;

            printf("ACC[%c] x=%0.3f y=%0.3f z=%0.3f | "
                   "RV[%c] i=%0.4f j=%0.4f k=%0.4f real=%0.4f acc=%0.4f | "
                   "GRV[%c] i=%0.4f j=%0.4f k=%0.4f real=%0.4f\n",
                   has_accel ? 'Y' : 'N',
                   accel.x,
                   accel.y,
                   accel.z,
                   has_rotation ? 'Y' : 'N',
                   rotation.i,
                   rotation.j,
                   rotation.k,
                   rotation.real,
                   rotation.accuracy,
                   has_game_rotation ? 'Y' : 'N',
                   game_rotation.i,
                   game_rotation.j,
                   game_rotation.k,
                   game_rotation.real);
        }
    } else {
        printf("BNO08x init failed: %s\n", esp_err_to_name(imu_err));
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
