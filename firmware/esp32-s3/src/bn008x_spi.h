#pragma once

/* BNO08x SPI transport.
 *
 * This file owns the ESP32 SPI host/pins and the helper used by main to add
 * the BNO08x as an SPI device. It is compiled only when BN008X_USE_SPI is
 * defined.
 */

#include "bn008x.h"

#if defined(BN008X_USE_SPI)

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#define BN008X_SPI_HOST SPI2_HOST
/* BNO08x SPI pin mapping with the board labels:
 * SCL = SCK, SDA = SO/MISO, AD0 = SI/MOSI.
 */
#define BN008X_SPI_MOSI_IO 6
#define BN008X_SPI_MISO_IO 7
#define BN008X_SPI_SCLK_IO 5
#define BN008X_SPI_CS_IO 4
#define BN008X_SPI_INT_IO 15
#define BN008X_SPI_RST_IO 16
/* PS0 must be high during reset to select SPI, then it becomes active-low WAKE. */
#define BN008X_SPI_WAKE_IO 17
#define BN008X_SPI_FREQ_HZ 1000000
#define BN008X_SPI_QUEUE_SIZE 1

typedef struct {
    spi_device_handle_t spi_dev;
    gpio_num_t int_gpio;
    gpio_num_t rst_gpio;
    gpio_num_t wake_gpio;
} bn008x_spi_t;

esp_err_t bn008x_spi_bus_init(void);
esp_err_t bn008x_spi_add_device(bn008x_spi_t *bn008x_spi);
esp_err_t bn008x_spi_reset(bn008x_spi_t *bn008x_spi);
esp_err_t bn008x_spi_read_shtp_packet(bn008x_t *bn008x, bn008x_shtp_packet_t *packet);
esp_err_t bn008x_spi_write_shtp_packet(bn008x_t *bn008x, const bn008x_shtp_packet_t *packet);

#endif
