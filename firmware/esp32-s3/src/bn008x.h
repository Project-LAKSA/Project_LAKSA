#pragma once

/* Common BNO08x SH-2/SHTP driver interface.
 *
 * This file owns sensor-level concepts: reports, parsed values, common packet
 * format, and report configuration. The board-level compile-time transport
 * selection lives in esp32_config.h.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp32_config.h"

#if defined(BN008X_USE_SPI) && defined(BN008X_USE_I2C)
#error "Select only one BNO08x transport."
#endif

#if defined(BN008X_USE_I2C)
#include "driver/i2c_master.h"
#elif defined(BN008X_USE_SPI)
#include "driver/gpio.h"
#include "driver/spi_master.h"
#else
#error "Select BN008X_USE_SPI or BN008X_USE_I2C."
#endif

#include "esp_err.h"

#define BN008X_SHTP_CHANNEL_COUNT 6
#define BN008X_SHTP_HEADER_LEN 4
#define BN008X_SHTP_MAX_PACKET_LEN 1024
#define BN008X_BOOT_DELAY_MS 300

#define BN008X_REPORT_INTERVAL_US 50000UL
#define BN008X_ENABLE_ACCELEROMETER 1
#define BN008X_ENABLE_ROTATION_VECTOR 1
#define BN008X_ENABLE_GAME_ROTATION_VECTOR 1
#define BN008X_ENABLE_ARVR_STABILIZED_ROTATION_VECTOR 1

typedef enum {
    BN008X_REPORT_ACCELEROMETER = 0x01,
    BN008X_REPORT_ROTATION_VECTOR = 0x05,
    BN008X_REPORT_GAME_ROTATION_VECTOR = 0x08,
    BN008X_REPORT_ARVR_STABILIZED_ROTATION_VECTOR = 0x28,
} bn008x_report_id_t;

typedef struct {
    float x;
    float y;
    float z;
} bn008x_vec3_t;

typedef struct {
    float i;
    float j;
    float k;
    float real;
    float accuracy;
} bn008x_quat_t;

typedef struct {
    uint8_t reset_cause;
    uint32_t sw_part_number;
    uint32_t sw_build_number;
    uint16_t sw_version_patch;
} bn008x_product_id_t;

typedef struct {
#if defined(BN008X_USE_I2C)
    i2c_master_dev_handle_t i2c_dev;
#endif
#if defined(BN008X_USE_SPI)
    spi_device_handle_t spi_dev;
    gpio_num_t spi_int_gpio;
    gpio_num_t spi_rst_gpio;
    gpio_num_t spi_wake_gpio;
#endif
    uint8_t tx_sequence[BN008X_SHTP_CHANNEL_COUNT];
    uint8_t rx_sequence[BN008X_SHTP_CHANNEL_COUNT];
    uint8_t command_sequence;
    bn008x_product_id_t product_id;
    bn008x_vec3_t acceleration;
    bn008x_quat_t rotation_vector;
    bn008x_quat_t game_rotation_vector;
    bool has_product_id;
    bool has_acceleration;
    bool has_rotation_vector;
    bool has_game_rotation_vector;
} bn008x_t;

typedef struct {
    uint16_t length;
    uint8_t channel;
    uint8_t sequence;
    uint8_t payload[BN008X_SHTP_MAX_PACKET_LEN - BN008X_SHTP_HEADER_LEN];
    uint16_t payload_len;
} bn008x_shtp_packet_t;

#if defined(BN008X_USE_I2C)
esp_err_t bn008x_init(bn008x_t *bn008x, i2c_master_dev_handle_t i2c_dev);
esp_err_t bn008x_init_i2c(bn008x_t *bn008x, i2c_master_dev_handle_t i2c_dev);
#endif

#if defined(BN008X_USE_SPI)
esp_err_t bn008x_init_spi(bn008x_t *bn008x,
                          spi_device_handle_t spi_dev,
                          gpio_num_t int_gpio,
                          gpio_num_t rst_gpio,
                          gpio_num_t wake_gpio);
#endif

esp_err_t bn008x_read_shtp_packet(bn008x_t *bn008x, bn008x_shtp_packet_t *packet);
esp_err_t bn008x_write_shtp_packet(bn008x_t *bn008x, const bn008x_shtp_packet_t *packet);
esp_err_t bn008x_soft_reset(bn008x_t *bn008x);
esp_err_t bn008x_get_product_id(bn008x_t *bn008x, bn008x_product_id_t *product_id);
esp_err_t bn008x_enable_report(bn008x_t *bn008x, bn008x_report_id_t report_id, uint32_t interval_us);
esp_err_t bn008x_update(bn008x_t *bn008x);
esp_err_t bn008x_get_acceleration(bn008x_t *bn008x, bn008x_vec3_t *acceleration);
esp_err_t bn008x_get_rotation_vector(bn008x_t *bn008x, bn008x_quat_t *quaternion);
esp_err_t bn008x_get_game_rotation_vector(bn008x_t *bn008x, bn008x_quat_t *quaternion);

void bn008x_clear_state(bn008x_t *bn008x);
esp_err_t bn008x_finish_init(bn008x_t *bn008x);
