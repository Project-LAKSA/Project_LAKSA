idf_component_register(SRCS "pca9685_i2c.c" "main.c"
                            "i2c_bus.c"
                            "bno08x_adapter.cpp"
                            "web_dashboard.c"
                            INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_spi esp_driver_gpio esp32_BNO08x
                                esp_wifi esp_http_server esp_netif esp_event nvs_flash)
