#include "esp_usbcdc_transport.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool tinyusb_driver_started;

bool esp_usbcdc_open(struct uxrCustomTransport *transport)
{
    if (transport == NULL || transport->args == NULL) {
        return false;
    }

    if (!tinyusb_driver_started) {
        const tinyusb_config_t config = {
            .device_descriptor = NULL,
            .string_descriptor = NULL,
            .external_phy = false,
            .configuration_descriptor = NULL,
        };
        if (tinyusb_driver_install(&config) != ESP_OK) {
            return false;
        }
        tinyusb_driver_started = true;
    }

    tinyusb_cdcacm_itf_t *cdc_port = (tinyusb_cdcacm_itf_t *)transport->args;
    const tinyusb_config_cdcacm_t acm_config = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = *cdc_port,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    return tusb_cdc_acm_init(&acm_config) == ESP_OK;
}

bool esp_usbcdc_close(struct uxrCustomTransport *transport)
{
    if (transport == NULL || transport->args == NULL) {
        return false;
    }
    tinyusb_cdcacm_itf_t *cdc_port = (tinyusb_cdcacm_itf_t *)transport->args;
    return tusb_cdc_acm_deinit(*cdc_port) == ESP_OK;
}

size_t esp_usbcdc_write(struct uxrCustomTransport *transport,
                        const uint8_t *buffer,
                        size_t length,
                        uint8_t *error)
{
    (void)error;
    tinyusb_cdcacm_itf_t *cdc_port = (tinyusb_cdcacm_itf_t *)transport->args;
    size_t written = tinyusb_cdcacm_write_queue(*cdc_port, buffer, length);
    (void)tinyusb_cdcacm_write_flush(*cdc_port, pdMS_TO_TICKS(10));
    return written;
}

size_t esp_usbcdc_read(struct uxrCustomTransport *transport,
                       uint8_t *buffer,
                       size_t length,
                       int timeout_ms,
                       uint8_t *error)
{
    (void)error;
    tinyusb_cdcacm_itf_t *cdc_port = (tinyusb_cdcacm_itf_t *)transport->args;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms > 0 ? timeout_ms : 0);

    do {
        size_t received = 0;
        if (tinyusb_cdcacm_read(*cdc_port, buffer, length, &received) == ESP_OK && received > 0) {
            return received;
        }
        if (timeout == 0) {
            break;
        }
        vTaskDelay(1);
    } while (xTaskGetTickCount() - start < timeout);

    return 0;
}
