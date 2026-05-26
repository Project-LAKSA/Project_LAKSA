#pragma once

#include "driver/i2c_types.h"

/* Select exactly one BNO08x communication transport for this ESP32 board. */
//#define BN008X_USE_SPI
#define BN008X_USE_I2C

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_TRANSMIT_TIMEOUT_MS 100

#define PCA9685_I2C_ADDR 0x40

#define SERVO_FREQ_HZ 50
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
