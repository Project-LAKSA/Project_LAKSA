/* BNO08x SPI transport implementation.
 *
 * This file owns the SPI bus transactions plus the SPI-only GPIOs: CS, INT,
 * RST, and PS0/WAKE. It is intentionally free of I2C address handling.
 */

#include "bn008x_spi.h"

#if defined(BN008X_USE_SPI)

#include <string.h>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bn008x_spi";

static uint8_t spi_header_debug_count;
static uint8_t spi_pin_debug_count;

static void bn008x_spi_log_pin_levels(const char *label)
{
    ESP_LOGD(TAG,
             "%s pins: CS=%d SCLK=%d MOSI=%d MISO=%d INT=%d RST=%d WAKE=%d",
             label,
             gpio_get_level(BN008X_SPI_CS_IO),
             gpio_get_level(BN008X_SPI_SCLK_IO),
             gpio_get_level(BN008X_SPI_MOSI_IO),
             gpio_get_level(BN008X_SPI_MISO_IO),
             gpio_get_level(BN008X_SPI_INT_IO),
             gpio_get_level(BN008X_SPI_RST_IO),
             gpio_get_level(BN008X_SPI_WAKE_IO));
}

static esp_err_t bn008x_spi_transmit(spi_device_handle_t spi_dev,
                                     const uint8_t *tx_data,
                                     uint8_t *rx_data,
                                     size_t len)
{
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.length = len * 8;
    transaction.tx_buffer = tx_data;
    transaction.rx_buffer = rx_data;

    return spi_device_polling_transmit(spi_dev, &transaction);
}

static esp_err_t bn008x_spi_wait_for_int(bn008x_t *bn008x)
{
    for (uint8_t attempt = 0; attempt < 125; attempt++) {
        if (gpio_get_level(bn008x->spi_int_gpio) == 0) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t bn008x_spi_bus_init(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = BN008X_SPI_MOSI_IO,
        .miso_io_num = BN008X_SPI_MISO_IO,
        .sclk_io_num = BN008X_SPI_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BN008X_SHTP_MAX_PACKET_LEN,
    };

    return spi_bus_initialize(BN008X_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
}

esp_err_t bn008x_spi_add_device(bn008x_spi_t *bn008x_spi)
{
    if (bn008x_spi == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(gpio_reset_pin(BN008X_SPI_CS_IO), TAG, "CS GPIO reset failed");
    ESP_RETURN_ON_ERROR(gpio_reset_pin(BN008X_SPI_RST_IO), TAG, "RST GPIO reset failed");
    ESP_RETURN_ON_ERROR(gpio_reset_pin(BN008X_SPI_WAKE_IO), TAG, "WAKE GPIO reset failed");
    ESP_RETURN_ON_ERROR(gpio_reset_pin(BN008X_SPI_INT_IO), TAG, "INT GPIO reset failed");

    gpio_config_t int_config = {
        .pin_bit_mask = 1ULL << BN008X_SPI_INT_IO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_config), TAG, "INT GPIO config failed");

    gpio_config_t cs_config = {
        .pin_bit_mask = 1ULL << BN008X_SPI_CS_IO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cs_config), TAG, "CS GPIO config failed");
    gpio_set_level(BN008X_SPI_CS_IO, 1);

    gpio_config_t rst_config = {
        .pin_bit_mask = 1ULL << BN008X_SPI_RST_IO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rst_config), TAG, "RST GPIO config failed");
    gpio_set_level(BN008X_SPI_RST_IO, 1);

    gpio_config_t wake_config = {
        .pin_bit_mask = 1ULL << BN008X_SPI_WAKE_IO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&wake_config), TAG, "WAKE GPIO config failed");
    gpio_set_level(BN008X_SPI_WAKE_IO, 1);

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = BN008X_SPI_FREQ_HZ,
        .mode = 3,
        .address_bits = 0,
        .command_bits = 0,
        .spics_io_num = -1,
        .queue_size = BN008X_SPI_QUEUE_SIZE,
    };

    ESP_RETURN_ON_ERROR(spi_bus_add_device(BN008X_SPI_HOST, &dev_config, &bn008x_spi->spi_dev),
                        TAG,
                        "SPI device add failed");

    bn008x_spi->int_gpio = BN008X_SPI_INT_IO;
    bn008x_spi->rst_gpio = BN008X_SPI_RST_IO;
    bn008x_spi->wake_gpio = BN008X_SPI_WAKE_IO;

    bn008x_spi_log_pin_levels("before reset");
    return bn008x_spi_reset(bn008x_spi);
}

esp_err_t bn008x_spi_reset(bn008x_spi_t *bn008x_spi)
{
    if (bn008x_spi == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level(bn008x_spi->wake_gpio, 1);
    gpio_set_level(bn008x_spi->rst_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(bn008x_spi->rst_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(bn008x_spi->rst_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(BN008X_BOOT_DELAY_MS));
    bn008x_spi_log_pin_levels("after reset");

    return ESP_OK;
}

esp_err_t bn008x_spi_read_shtp_packet(bn008x_t *bn008x, bn008x_shtp_packet_t *packet)
{
    if (gpio_get_level(bn008x->spi_int_gpio) != 0) {
        return ESP_ERR_NOT_FOUND;
    }

    static DMA_ATTR uint8_t tx_header[BN008X_SHTP_HEADER_LEN];
    static DMA_ATTR uint8_t rx_header[BN008X_SHTP_HEADER_LEN];
    static DMA_ATTR uint8_t tx_payload[BN008X_SHTP_MAX_PACKET_LEN - BN008X_SHTP_HEADER_LEN];
    static DMA_ATTR uint8_t rx_payload[BN008X_SHTP_MAX_PACKET_LEN - BN008X_SHTP_HEADER_LEN];

    memset(tx_header, 0x00, sizeof(tx_header));
    memset(rx_header, 0, sizeof(rx_header));

    gpio_set_level(BN008X_SPI_CS_IO, 0);
    esp_rom_delay_us(10);
    if (spi_pin_debug_count < 8) {
        bn008x_spi_log_pin_levels("during read");
        spi_pin_debug_count++;
    }

    esp_err_t err = bn008x_spi_transmit(bn008x->spi_dev, tx_header, rx_header, sizeof(tx_header));
    if (err != ESP_OK) {
        gpio_set_level(BN008X_SPI_CS_IO, 1);
        return err;
    }

    uint16_t raw_len = ((uint16_t)rx_header[1] << 8) | (uint16_t)rx_header[0];
    if (spi_header_debug_count < 20) {
        ESP_LOGD(TAG,
                 "SPI header: %02X %02X %02X %02X int=%d raw_len=0x%04X",
                 rx_header[0],
                 rx_header[1],
                 rx_header[2],
                 rx_header[3],
                 gpio_get_level(bn008x->spi_int_gpio),
                 raw_len);
        spi_header_debug_count++;
    }

    if (raw_len == 0xFFFF) {
        gpio_set_level(BN008X_SPI_CS_IO, 1);
        ESP_LOGD(TAG, "SPI returned reserved SHTP length 0xFFFF");
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t packet_len = raw_len & 0x7FFF;

    if (packet_len == 0) {
        gpio_set_level(BN008X_SPI_CS_IO, 1);
        return ESP_ERR_NOT_FOUND;
    }

    if (packet_len < BN008X_SHTP_HEADER_LEN || packet_len > BN008X_SHTP_MAX_PACKET_LEN) {
        gpio_set_level(BN008X_SPI_CS_IO, 1);
        ESP_LOGE(TAG, "Invalid SPI SHTP packet length: %u", packet_len);
        return ESP_ERR_INVALID_SIZE;
    }

    packet->length = packet_len;
    packet->channel = rx_header[2];
    packet->sequence = rx_header[3];
    packet->payload_len = packet_len - BN008X_SHTP_HEADER_LEN;

    if (packet->payload_len > 0) {
        memset(tx_payload, 0xFF, packet->payload_len);
        memset(rx_payload, 0, packet->payload_len);
        err = bn008x_spi_transmit(bn008x->spi_dev, tx_payload, rx_payload, packet->payload_len);
        if (err != ESP_OK) {
            gpio_set_level(BN008X_SPI_CS_IO, 1);
            return err;
        }
        memcpy(packet->payload, rx_payload, packet->payload_len);
    }

    gpio_set_level(BN008X_SPI_CS_IO, 1);
    return ESP_OK;
}

esp_err_t bn008x_spi_write_shtp_packet(bn008x_t *bn008x, const bn008x_shtp_packet_t *packet)
{
    uint16_t packet_len = BN008X_SHTP_HEADER_LEN + packet->payload_len;
    static DMA_ATTR uint8_t data[BN008X_SHTP_MAX_PACKET_LEN];
    memset(data, 0, sizeof(data));

    data[0] = (uint8_t)(packet_len & 0x00FF);
    data[1] = (uint8_t)((packet_len >> 8) & 0x00FF);
    data[2] = packet->channel;
    data[3] = bn008x->tx_sequence[packet->channel];
    memcpy(&data[BN008X_SHTP_HEADER_LEN],
           packet->payload,
           packet->payload_len);

    gpio_set_level(bn008x->spi_wake_gpio, 0);
    esp_rom_delay_us(1500);

    esp_err_t err = bn008x_spi_wait_for_int(bn008x);
    if (err != ESP_OK) {
        ESP_LOGD(TAG,
                 "INT did not assert before SPI write, sending packet anyway: %s",
                 esp_err_to_name(err));
    }

    gpio_set_level(BN008X_SPI_CS_IO, 0);
    esp_rom_delay_us(10);
    err = bn008x_spi_transmit(bn008x->spi_dev, data, NULL, packet_len);
    gpio_set_level(BN008X_SPI_CS_IO, 1);
    gpio_set_level(bn008x->spi_wake_gpio, 1);

    return err;
}

esp_err_t bn008x_init_spi(bn008x_t *bn008x,
                          spi_device_handle_t spi_dev,
                          gpio_num_t int_gpio,
                          gpio_num_t rst_gpio,
                          gpio_num_t wake_gpio)
{
    if (bn008x == NULL || spi_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(bn008x, 0, sizeof(*bn008x));
    bn008x->spi_dev = spi_dev;
    bn008x->spi_int_gpio = int_gpio;
    bn008x->spi_rst_gpio = rst_gpio;
    bn008x->spi_wake_gpio = wake_gpio;
    bn008x_clear_state(bn008x);

    return bn008x_finish_init(bn008x);
}

#endif
