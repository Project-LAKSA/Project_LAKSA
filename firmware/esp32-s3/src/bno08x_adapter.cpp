#include "bno08x_adapter.h"

#include <new>
#include <string.h>

#include "BNO08x.hpp"
#include "esp32_config.h"

#define BNO08X_REPORT_INTERVAL_US 50000UL

static bno08x_adapter_quat_t bno08x_adapter_from_quat(const bno08x_quat_t &quat)
{
    return {
        .i = quat.i,
        .j = quat.j,
        .k = quat.k,
        .real = quat.real,
        .accuracy = quat.rad_accuracy,
    };
}

static BNO08x *bno08x_adapter_driver(bno08x_adapter_t *imu)
{
    return static_cast<BNO08x *>(imu->driver);
}

extern "C" esp_err_t bno08x_adapter_init(bno08x_adapter_t *imu)
{
    if (imu == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(imu, 0, sizeof(*imu));

    bno08x_config_t config(
        SPI_MASTER_HOST,
        static_cast<gpio_num_t>(BNO08X_SPI_MOSI_IO),
        static_cast<gpio_num_t>(BNO08X_SPI_MISO_IO),
        static_cast<gpio_num_t>(BNO08X_SPI_SCLK_IO),
        static_cast<gpio_num_t>(BNO08X_SPI_CS_IO),
        static_cast<gpio_num_t>(BNO08X_SPI_INT_IO),
        static_cast<gpio_num_t>(BNO08X_SPI_RST_IO),
        BNO08X_SPI_FREQ_HZ);

    BNO08x *driver = new (std::nothrow) BNO08x(config);
    if (driver == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    if (!driver->initialize()) {
        delete driver;
        return ESP_FAIL;
    }

    bool enabled = true;
    enabled = driver->rpt.accelerometer.enable(BNO08X_REPORT_INTERVAL_US) && enabled;
    enabled = driver->rpt.rv.enable(BNO08X_REPORT_INTERVAL_US) && enabled;
    enabled = driver->rpt.rv_game.enable(BNO08X_REPORT_INTERVAL_US) && enabled;

    if (!enabled) {
        delete driver;
        return ESP_FAIL;
    }

    imu->driver = driver;
    return ESP_OK;
}

extern "C" esp_err_t bno08x_adapter_update(bno08x_adapter_t *imu)
{
    if (imu == nullptr || imu->driver == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    BNO08x *driver = bno08x_adapter_driver(imu);
    if (!driver->data_available()) {
        return ESP_ERR_NOT_FOUND;
    }

    if (driver->rpt.accelerometer.has_new_data()) {
        bno08x_accel_t accel = driver->rpt.accelerometer.get();
        imu->acceleration = {
            .x = accel.x,
            .y = accel.y,
            .z = accel.z,
        };
        imu->has_acceleration = true;
    }

    if (driver->rpt.rv.has_new_data()) {
        imu->rotation_vector = bno08x_adapter_from_quat(driver->rpt.rv.get_quat());
        imu->has_rotation_vector = true;
    }

    if (driver->rpt.rv_game.has_new_data()) {
        imu->game_rotation_vector = bno08x_adapter_from_quat(driver->rpt.rv_game.get_quat());
        imu->has_game_rotation_vector = true;
    }

    return ESP_OK;
}

extern "C" esp_err_t bno08x_adapter_get_acceleration(bno08x_adapter_t *imu, bno08x_adapter_vec3_t *acceleration)
{
    if (imu == nullptr || acceleration == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!imu->has_acceleration) {
        return ESP_ERR_NOT_FOUND;
    }

    *acceleration = imu->acceleration;
    return ESP_OK;
}

extern "C" esp_err_t bno08x_adapter_get_rotation_vector(bno08x_adapter_t *imu, bno08x_adapter_quat_t *quaternion)
{
    if (imu == nullptr || quaternion == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!imu->has_rotation_vector) {
        return ESP_ERR_NOT_FOUND;
    }

    *quaternion = imu->rotation_vector;
    return ESP_OK;
}

extern "C" esp_err_t bno08x_adapter_get_game_rotation_vector(bno08x_adapter_t *imu, bno08x_adapter_quat_t *quaternion)
{
    if (imu == nullptr || quaternion == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!imu->has_game_rotation_vector) {
        return ESP_ERR_NOT_FOUND;
    }

    *quaternion = imu->game_rotation_vector;
    return ESP_OK;
}
