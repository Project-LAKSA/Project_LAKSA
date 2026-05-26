/* BNO08x I2C transport implementation.
 *
 * This file turns SHTP packets into I2C transactions. It is intentionally
 * free of SPI pins, reset lines, or WAKE handling.
 */

#include "bn008x_i2c.h"

#if defined(BN008X_USE_I2C)

#include <string.h>

#include "esp32_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bn008x_i2c";

esp_err_t bn008x_i2c_read_shtp_packet(bn008x_t *bn008x, bn008x_shtp_packet_t *packet)
{
    uint8_t header[BN008X_SHTP_HEADER_LEN] = {0};

    esp_err_t err = i2c_master_receive(bn008x->i2c_dev,
                                       header,
                                       sizeof(header),
                                       I2C_TRANSMIT_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t packet_len = ((uint16_t)header[1] << 8) | (uint16_t)header[0];
    packet_len &= 0x7FFF;

    if (packet_len == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    if (packet_len < BN008X_SHTP_HEADER_LEN || packet_len > BN008X_SHTP_MAX_PACKET_LEN) {
        ESP_LOGE(TAG, "Invalid SHTP packet length: %u", packet_len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t data[BN008X_SHTP_MAX_PACKET_LEN] = {0};
    err = i2c_master_receive(bn008x->i2c_dev,
                             data,
                             packet_len,
                             I2C_TRANSMIT_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t received_len = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    received_len &= 0x7FFF;

    if (received_len != packet_len) {
        ESP_LOGE(TAG, "SHTP length changed while reading: header=%u packet=%u", packet_len, received_len);
        return ESP_ERR_INVALID_RESPONSE;
    }

    packet->length = received_len;
    packet->channel = data[2];
    packet->sequence = data[3];
    packet->payload_len = received_len - BN008X_SHTP_HEADER_LEN;

    if (packet->payload_len > 0) {
        memcpy(packet->payload, &data[BN008X_SHTP_HEADER_LEN], packet->payload_len);
    }

    return ESP_OK;
}

esp_err_t bn008x_i2c_write_shtp_packet(bn008x_t *bn008x, const bn008x_shtp_packet_t *packet)
{
    uint16_t packet_len = BN008X_SHTP_HEADER_LEN + packet->payload_len;
    uint8_t data[BN008X_SHTP_MAX_PACKET_LEN] = {0};

    data[0] = (uint8_t)(packet_len & 0x00FF);
    data[1] = (uint8_t)((packet_len >> 8) & 0x00FF);
    data[2] = packet->channel;
    data[3] = bn008x->tx_sequence[packet->channel];
    memcpy(&data[BN008X_SHTP_HEADER_LEN],
           packet->payload,
           packet->payload_len);

    return i2c_master_transmit(bn008x->i2c_dev,
                               data,
                               packet_len,
                               I2C_TRANSMIT_TIMEOUT_MS);
}

esp_err_t bn008x_init_i2c(bn008x_t *bn008x, i2c_master_dev_handle_t i2c_dev)
{
    if (bn008x == NULL || i2c_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(bn008x, 0, sizeof(*bn008x));
    bn008x->i2c_dev = i2c_dev;
    bn008x_clear_state(bn008x);

    vTaskDelay(pdMS_TO_TICKS(BN008X_BOOT_DELAY_MS));
    return bn008x_finish_init(bn008x);
}

esp_err_t bn008x_init(bn008x_t *bn008x, i2c_master_dev_handle_t i2c_dev)
{
    return bn008x_init_i2c(bn008x, i2c_dev);
}

#endif
