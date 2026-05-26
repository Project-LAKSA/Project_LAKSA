#pragma once

/* BNO08x I2C transport.
 *
 * This file owns the BNO08x I2C address and the I2C packet read/write entry
 * points. It is compiled only when BN008X_USE_I2C is defined.
 */

#include "bn008x.h"

#if defined(BN008X_USE_I2C)

#define BN008X_I2C_ADDR 0x4B

esp_err_t bn008x_i2c_read_shtp_packet(bn008x_t *bn008x, bn008x_shtp_packet_t *packet);
esp_err_t bn008x_i2c_write_shtp_packet(bn008x_t *bn008x, const bn008x_shtp_packet_t *packet);

#endif
