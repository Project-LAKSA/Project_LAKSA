#pragma once

#include "driver/i2c_types.h"
#include "driver/spi_master.h"

/* WiFi station credentials for the embedded dashboard. */
#define WIFI_STA_SSID "LACASA128"
#define WIFI_STA_PASSWORD "Carrot1-Top-fan@"
#define WIFI_CONNECT_TIMEOUT_MS 15000

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_TRANSMIT_TIMEOUT_MS 100

#define SPI_MASTER_HOST SPI2_HOST
/* BNO08x SPI pin mapping with the board labels:
 * SCL = SCK, SDA = SO/MISO, AD0 = SI/MOSI.
 */
#define SPI_MASTER_MOSI_IO 6
#define SPI_MASTER_MISO_IO 7
#define SPI_MASTER_SCLK_IO 5
#define SPI_MASTER_MAX_TRANSFER_SIZE 1024
#define SPI_MASTER_DEVICE_FREQ_HZ 1000000
#define SPI_MASTER_DEVICE_MODE 3
#define SPI_MASTER_DEVICE_QUEUE_SIZE 1

#define BNO08X_SPI_MOSI_IO SPI_MASTER_MOSI_IO
#define BNO08X_SPI_MISO_IO SPI_MASTER_MISO_IO
#define BNO08X_SPI_SCLK_IO SPI_MASTER_SCLK_IO
#define BNO08X_SPI_CS_IO 4
#define BNO08X_SPI_INT_IO 15
#define BNO08X_SPI_RST_IO 16
#define BNO08X_SPI_FREQ_HZ 1000000

#define PCA9685_I2C_ADDR 0x40

#define SERVO_FREQ_HZ 50
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
