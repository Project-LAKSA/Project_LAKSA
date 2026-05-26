/* Common BNO08x SH-2/SHTP logic.
 *
 * This file owns packet validation, sequence tracking, product ID requests,
 * report enabling, report parsing, and public getters. It delegates raw packet
 * I/O to exactly one transport selected at compile time.
 */

#include "bn008x.h"

#if defined(BN008X_USE_I2C)
#include "bn008x_i2c.h"
#elif defined(BN008X_USE_SPI)
#include "bn008x_spi.h"
#endif

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BN008X_INIT_PACKET_READ_ATTEMPTS 20
#define BN008X_PACKET_READ_RETRY_DELAY_MS 10

#define BN008X_SHTP_CHANNEL_CONTROL 2
#define BN008X_SHTP_CHANNEL_REPORTS 3

#define BN008X_REPORT_COMMAND_REQUEST 0xF2
#define BN008X_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define BN008X_REPORT_PRODUCT_ID_REQUEST 0xF9
#define BN008X_REPORT_BASE_TIMESTAMP 0xFB
#define BN008X_REPORT_SET_FEATURE_COMMAND 0xFD

#define BN008X_COMMAND_RESET 0x01

#define BN008X_SET_FEATURE_PAYLOAD_LEN 17
#define BN008X_COMMAND_REQUEST_PAYLOAD_LEN 12
#define BN008X_PRODUCT_ID_REQUEST_PAYLOAD_LEN 2
#define BN008X_PRODUCT_ID_RESPONSE_PAYLOAD_LEN 16

#define BN008X_ACCELEROMETER_Q_POINT 8
#define BN008X_ROTATION_VECTOR_Q_POINT 14

static const char *TAG = "bn008x";

static void bn008x_log_shtp_packet_debug(const bn008x_shtp_packet_t *packet);
static void bn008x_parse_packet(bn008x_t *bn008x, const bn008x_shtp_packet_t *packet);

static uint16_t bn008x_read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t bn008x_read_i16_le(const uint8_t *data)
{
    return (int16_t)bn008x_read_u16_le(data);
}

static uint32_t bn008x_read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void bn008x_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
    data[2] = (uint8_t)((value >> 16) & 0xFF);
    data[3] = (uint8_t)((value >> 24) & 0xFF);
}

static float bn008x_q_to_float(int16_t value, uint8_t q_point)
{
    return (float)value / (float)(1UL << q_point);
}

static esp_err_t bn008x_send_payload(bn008x_t *bn008x,
                                     uint8_t channel,
                                     const uint8_t *payload,
                                     uint16_t payload_len)
{
    bn008x_shtp_packet_t packet = {
        .channel = channel,
        .payload_len = payload_len,
    };

    if (payload_len > sizeof(packet.payload)) {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(packet.payload, payload, payload_len);
    return bn008x_write_shtp_packet(bn008x, &packet);
}

static bool bn008x_is_packet_not_ready(esp_err_t err)
{
    return err == ESP_ERR_NOT_FOUND ||
           err == ESP_ERR_INVALID_RESPONSE;
}

esp_err_t bn008x_read_shtp_packet(bn008x_t *bn008x, bn008x_shtp_packet_t *packet)
{
    if (bn008x == NULL || packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#if defined(BN008X_USE_I2C)
    return bn008x_i2c_read_shtp_packet(bn008x, packet);
#elif defined(BN008X_USE_SPI)
    return bn008x_spi_read_shtp_packet(bn008x, packet);
#endif
}

esp_err_t bn008x_drain_startup_packets(bn008x_t *bn008x)
{
    for (uint8_t attempt = 0; attempt < BN008X_INIT_PACKET_READ_ATTEMPTS; attempt++) {
        bn008x_shtp_packet_t packet = {0};
        esp_err_t err = bn008x_read_shtp_packet(bn008x, &packet);

        if (bn008x_is_packet_not_ready(err)) {
            ESP_LOGD(TAG, "No SHTP packet available yet: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(BN008X_PACKET_READ_RETRY_DELAY_MS));
            continue;
        }

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SHTP packet read failed: %s", esp_err_to_name(err));
            return err;
        }

        if (packet.channel < BN008X_SHTP_CHANNEL_COUNT) {
            bn008x->rx_sequence[packet.channel] = packet.sequence;
        }

        bn008x_log_shtp_packet_debug(&packet);
        bn008x_parse_packet(bn008x, &packet);
    }

    return ESP_OK;
}

static void bn008x_log_shtp_packet_debug(const bn008x_shtp_packet_t *packet)
{
    ESP_LOGD(
        TAG,
        "SHTP packet: length=%u channel=%u sequence=%u payload_len=%u",
        packet->length,
        packet->channel,
        packet->sequence,
        packet->payload_len);

    if (packet->payload_len > 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, packet->payload, packet->payload_len, ESP_LOG_DEBUG);
    }
}

esp_err_t bn008x_write_shtp_packet(bn008x_t *bn008x, const bn008x_shtp_packet_t *packet)
{
    if (bn008x == NULL || packet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (packet->channel >= BN008X_SHTP_CHANNEL_COUNT) {
        ESP_LOGE(TAG, "Invalid channel: %u", packet->channel);
        return ESP_ERR_INVALID_SIZE;
    }

    if (packet->payload_len > BN008X_SHTP_MAX_PACKET_LEN - BN008X_SHTP_HEADER_LEN) {
        ESP_LOGE(TAG, "Invalid payload length: %u", packet->payload_len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint16_t packet_len = BN008X_SHTP_HEADER_LEN + packet->payload_len;

    if (packet->length != 0 && packet->length != packet_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = ESP_OK;
#if defined(BN008X_USE_I2C)
    err = bn008x_i2c_write_shtp_packet(bn008x, packet);
#elif defined(BN008X_USE_SPI)
    err = bn008x_spi_write_shtp_packet(bn008x, packet);
#endif

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "TX SHTP: channel=%u sequence=%u payload0=0x%02X payload_len=%u",
                 packet->channel,
                 bn008x->tx_sequence[packet->channel],
                 packet->payload_len > 0 ? packet->payload[0] : 0,
                 packet->payload_len);
        bn008x->tx_sequence[packet->channel]++;
    }

    return err;
}

esp_err_t bn008x_soft_reset(bn008x_t *bn008x)
{
    if (bn008x == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[BN008X_COMMAND_REQUEST_PAYLOAD_LEN] = {0};
    payload[0] = BN008X_REPORT_COMMAND_REQUEST;
    payload[1] = bn008x->command_sequence++;
    payload[2] = BN008X_COMMAND_RESET;

    esp_err_t err = bn008x_send_payload(
        bn008x,
        BN008X_SHTP_CHANNEL_CONTROL,
        payload,
        sizeof(payload));
    if (err != ESP_OK) {
        return err;
    }

    memset(bn008x->tx_sequence, 0, sizeof(bn008x->tx_sequence));
    memset(bn008x->rx_sequence, 0, sizeof(bn008x->rx_sequence));
    vTaskDelay(pdMS_TO_TICKS(BN008X_BOOT_DELAY_MS));
    return bn008x_drain_startup_packets(bn008x);
}

esp_err_t bn008x_enable_report(bn008x_t *bn008x, bn008x_report_id_t report_id, uint32_t interval_us)
{
    if (bn008x == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[BN008X_SET_FEATURE_PAYLOAD_LEN] = {0};
    payload[0] = BN008X_REPORT_SET_FEATURE_COMMAND;
    payload[1] = (uint8_t)report_id;
    bn008x_write_u32_le(&payload[5], interval_us);

    return bn008x_send_payload(
        bn008x,
        BN008X_SHTP_CHANNEL_CONTROL,
        payload,
        sizeof(payload));
}

static esp_err_t bn008x_request_product_id(bn008x_t *bn008x)
{
    uint8_t payload[BN008X_PRODUCT_ID_REQUEST_PAYLOAD_LEN] = {
        BN008X_REPORT_PRODUCT_ID_REQUEST,
        0,
    };

    return bn008x_send_payload(
        bn008x,
        BN008X_SHTP_CHANNEL_CONTROL,
        payload,
        sizeof(payload));
}

esp_err_t bn008x_finish_init(bn008x_t *bn008x)
{
    esp_err_t err = bn008x_drain_startup_packets(bn008x);
    if (err != ESP_OK) {
        return err;
    }

    err = bn008x_request_product_id(bn008x);
    if (err == ESP_OK) {
        for (uint8_t attempt = 0; attempt < 100; attempt++) {
            err = bn008x_update(bn008x);
            if (bn008x->has_product_id) {
                break;
            }
            if (!bn008x_is_packet_not_ready(err) && err != ESP_OK) {
                return err;
            }
            vTaskDelay(pdMS_TO_TICKS(BN008X_PACKET_READ_RETRY_DELAY_MS));
        }
    }

#if BN008X_ENABLE_ACCELEROMETER
    err = bn008x_enable_report(bn008x, BN008X_REPORT_ACCELEROMETER, BN008X_REPORT_INTERVAL_US);
    if (err != ESP_OK) {
        return err;
    }
#endif

#if BN008X_ENABLE_ROTATION_VECTOR
    err = bn008x_enable_report(bn008x, BN008X_REPORT_ROTATION_VECTOR, BN008X_REPORT_INTERVAL_US);
    if (err != ESP_OK) {
        return err;
    }
#endif

#if BN008X_ENABLE_ARVR_STABILIZED_ROTATION_VECTOR
    err = bn008x_enable_report(bn008x,
                               BN008X_REPORT_ARVR_STABILIZED_ROTATION_VECTOR,
                               BN008X_REPORT_INTERVAL_US);
    if (err != ESP_OK) {
        return err;
    }
#endif

#if BN008X_ENABLE_GAME_ROTATION_VECTOR
    err = bn008x_enable_report(bn008x, BN008X_REPORT_GAME_ROTATION_VECTOR, BN008X_REPORT_INTERVAL_US);
    if (err != ESP_OK) {
        return err;
    }
#endif

    return ESP_OK;
}

static void bn008x_parse_product_id(bn008x_t *bn008x, const uint8_t *payload, uint16_t len)
{
    if (len < BN008X_PRODUCT_ID_RESPONSE_PAYLOAD_LEN || payload[0] != BN008X_REPORT_PRODUCT_ID_RESPONSE) {
        return;
    }

    bn008x->product_id.reset_cause = payload[1];
    bn008x->product_id.sw_part_number = bn008x_read_u32_le(&payload[4]);
    bn008x->product_id.sw_build_number = bn008x_read_u32_le(&payload[8]);
    bn008x->product_id.sw_version_patch = bn008x_read_u16_le(&payload[12]);
    bn008x->has_product_id = true;

    ESP_LOGI(
        TAG,
        "Product ID: reset=%u part=%" PRIu32 " build=%" PRIu32 " patch=%u",
        bn008x->product_id.reset_cause,
        bn008x->product_id.sw_part_number,
        bn008x->product_id.sw_build_number,
        bn008x->product_id.sw_version_patch);
}

static void bn008x_parse_accelerometer(bn008x_t *bn008x, const uint8_t *report, uint16_t len)
{
    if (len < 10) {
        return;
    }

    bn008x->acceleration.x = bn008x_q_to_float(bn008x_read_i16_le(&report[4]), BN008X_ACCELEROMETER_Q_POINT);
    bn008x->acceleration.y = bn008x_q_to_float(bn008x_read_i16_le(&report[6]), BN008X_ACCELEROMETER_Q_POINT);
    bn008x->acceleration.z = bn008x_q_to_float(bn008x_read_i16_le(&report[8]), BN008X_ACCELEROMETER_Q_POINT);
    bn008x->has_acceleration = true;
    ESP_LOGD(TAG,
             "Parsed accelerometer: x=%0.3f y=%0.3f z=%0.3f",
             bn008x->acceleration.x,
             bn008x->acceleration.y,
             bn008x->acceleration.z);
}

static void bn008x_parse_quaternion(bn008x_quat_t *quaternion, const uint8_t *report, uint16_t len)
{
    if (len < 12) {
        return;
    }

    quaternion->i = bn008x_q_to_float(bn008x_read_i16_le(&report[4]), BN008X_ROTATION_VECTOR_Q_POINT);
    quaternion->j = bn008x_q_to_float(bn008x_read_i16_le(&report[6]), BN008X_ROTATION_VECTOR_Q_POINT);
    quaternion->k = bn008x_q_to_float(bn008x_read_i16_le(&report[8]), BN008X_ROTATION_VECTOR_Q_POINT);
    quaternion->real = bn008x_q_to_float(bn008x_read_i16_le(&report[10]), BN008X_ROTATION_VECTOR_Q_POINT);
    quaternion->accuracy = 0.0f;

    if (len >= 14) {
        quaternion->accuracy = bn008x_q_to_float(bn008x_read_i16_le(&report[12]), BN008X_ROTATION_VECTOR_Q_POINT);
    }
}

static uint16_t bn008x_report_len(uint8_t report_id)
{
    switch (report_id) {
    case BN008X_REPORT_ACCELEROMETER:
        return 10;
    case BN008X_REPORT_ROTATION_VECTOR:
    case BN008X_REPORT_ARVR_STABILIZED_ROTATION_VECTOR:
        return 14;
    case BN008X_REPORT_GAME_ROTATION_VECTOR:
        return 12;
    case BN008X_REPORT_PRODUCT_ID_RESPONSE:
        return BN008X_PRODUCT_ID_RESPONSE_PAYLOAD_LEN;
    case BN008X_REPORT_BASE_TIMESTAMP:
        return 5;
    default:
        return 0;
    }
}

static void bn008x_parse_report(bn008x_t *bn008x, const uint8_t *report, uint16_t len)
{
    uint8_t report_id = report[0];

    switch (report_id) {
    case BN008X_REPORT_ACCELEROMETER:
        bn008x_parse_accelerometer(bn008x, report, len);
        break;
    case BN008X_REPORT_ROTATION_VECTOR:
    case BN008X_REPORT_ARVR_STABILIZED_ROTATION_VECTOR:
        bn008x_parse_quaternion(&bn008x->rotation_vector, report, len);
        bn008x->has_rotation_vector = true;
        ESP_LOGD(TAG,
                 "Parsed rotation vector: i=%0.4f j=%0.4f k=%0.4f real=%0.4f",
                 bn008x->rotation_vector.i,
                 bn008x->rotation_vector.j,
                 bn008x->rotation_vector.k,
                 bn008x->rotation_vector.real);
        break;
    case BN008X_REPORT_GAME_ROTATION_VECTOR:
        bn008x_parse_quaternion(&bn008x->game_rotation_vector, report, len);
        bn008x->has_game_rotation_vector = true;
        ESP_LOGD(TAG,
                 "Parsed game rotation vector: i=%0.4f j=%0.4f k=%0.4f real=%0.4f",
                 bn008x->game_rotation_vector.i,
                 bn008x->game_rotation_vector.j,
                 bn008x->game_rotation_vector.k,
                 bn008x->game_rotation_vector.real);
        break;
    case BN008X_REPORT_PRODUCT_ID_RESPONSE:
        bn008x_parse_product_id(bn008x, report, len);
        break;
    default:
        ESP_LOGD(TAG, "Unhandled SH-2 report id: 0x%02X", report_id);
        break;
    }
}

static void bn008x_parse_packet(bn008x_t *bn008x, const bn008x_shtp_packet_t *packet)
{
    if (packet->channel >= BN008X_SHTP_CHANNEL_COUNT) {
        return;
    }

    bn008x->rx_sequence[packet->channel] = packet->sequence;

    if (packet->channel != BN008X_SHTP_CHANNEL_CONTROL &&
        packet->channel != BN008X_SHTP_CHANNEL_REPORTS) {
        return;
    }

    ESP_LOGD(TAG, "RX parse channel=%u payload0=0x%02X payload_len=%u",
             packet->channel,
             packet->payload_len > 0 ? packet->payload[0] : 0,
             packet->payload_len);

    uint16_t offset = 0;
    while (offset < packet->payload_len) {
        uint8_t report_id = packet->payload[offset];
        uint16_t report_len = bn008x_report_len(report_id);

        if (report_len == 0 || offset + report_len > packet->payload_len) {
            ESP_LOGD(TAG, "Cannot parse report id 0x%02X len=%u offset=%u payload_len=%u",
                     report_id,
                     report_len,
                     offset,
                     packet->payload_len);
            return;
        }

        if (report_id != BN008X_REPORT_BASE_TIMESTAMP) {
            bn008x_parse_report(bn008x, &packet->payload[offset], report_len);
        }

        offset += report_len;
    }
}

esp_err_t bn008x_update(bn008x_t *bn008x)
{
    if (bn008x == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bn008x_shtp_packet_t packet = {0};
    esp_err_t err = bn008x_read_shtp_packet(bn008x, &packet);
    if (bn008x_is_packet_not_ready(err)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        return err;
    }

    bn008x_parse_packet(bn008x, &packet);
    return ESP_OK;
}

esp_err_t bn008x_get_product_id(bn008x_t *bn008x, bn008x_product_id_t *product_id)
{
    if (bn008x == NULL || product_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bn008x->has_product_id) {
        *product_id = bn008x->product_id;
        return ESP_OK;
    }

    esp_err_t err = bn008x_request_product_id(bn008x);
    if (err != ESP_OK) {
        return err;
    }

    for (uint8_t attempt = 0; attempt < 100; attempt++) {
        err = bn008x_update(bn008x);
        if (bn008x_is_packet_not_ready(err)) {
            vTaskDelay(pdMS_TO_TICKS(BN008X_PACKET_READ_RETRY_DELAY_MS));
            continue;
        }
        if (err != ESP_OK) {
            return err;
        }
        if (bn008x->has_product_id) {
            *product_id = bn008x->product_id;
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t bn008x_get_acceleration(bn008x_t *bn008x, bn008x_vec3_t *acceleration)
{
    if (bn008x == NULL || acceleration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!bn008x->has_acceleration) {
        return ESP_ERR_NOT_FOUND;
    }

    *acceleration = bn008x->acceleration;
    return ESP_OK;
}

esp_err_t bn008x_get_rotation_vector(bn008x_t *bn008x, bn008x_quat_t *quaternion)
{
    if (bn008x == NULL || quaternion == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!bn008x->has_rotation_vector) {
        return ESP_ERR_NOT_FOUND;
    }

    *quaternion = bn008x->rotation_vector;
    return ESP_OK;
}

esp_err_t bn008x_get_game_rotation_vector(bn008x_t *bn008x, bn008x_quat_t *quaternion)
{
    if (bn008x == NULL || quaternion == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!bn008x->has_game_rotation_vector) {
        return ESP_ERR_NOT_FOUND;
    }

    *quaternion = bn008x->game_rotation_vector;
    return ESP_OK;
}

void bn008x_clear_state(bn008x_t *bn008x)
{
    memset(bn008x->tx_sequence, 0, sizeof(bn008x->tx_sequence));
    memset(bn008x->rx_sequence, 0, sizeof(bn008x->rx_sequence));
    bn008x->command_sequence = 0;
    bn008x->has_product_id = false;
    bn008x->has_acceleration = false;
    bn008x->has_rotation_vector = false;
    bn008x->has_game_rotation_vector = false;
}
