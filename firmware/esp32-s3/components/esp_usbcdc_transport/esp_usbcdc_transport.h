#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/transport.h>

#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "The USB-CDC transport requires an ESP32-S2 or ESP32-S3"
#endif

bool esp_usbcdc_open(struct uxrCustomTransport *transport);
bool esp_usbcdc_close(struct uxrCustomTransport *transport);
size_t esp_usbcdc_write(struct uxrCustomTransport *transport,
                        const uint8_t *buffer,
                        size_t length,
                        uint8_t *error);
size_t esp_usbcdc_read(struct uxrCustomTransport *transport,
                       uint8_t *buffer,
                       size_t length,
                       int timeout_ms,
                       uint8_t *error);
